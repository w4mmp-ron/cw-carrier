/**
 * USB vendor opcode dispatch — logic port of usbvend01.c.
 * Stack-agnostic: TinyUSB/Cube call usb_vendor_setup / complete_out.
 */
#include "usb_vendor.h"
#include "usbvend.h"
#include "radio_state.h"
#include "proficio_config.h"
#include "control.h"
#include "system_boot.h"
#include "calibration.h"
#include "usb_device.h"
#include "hang_rec.h"
#include "loop_prof.h"
#include "keyer_boot.h"
#include "rx_resync.h"
#include <string.h>

static uint32_t s_result;
static int32_t  s_ppm_result;
static int32_t  s_ppm_temp;
static int32_t  s_proficio_temperature;
static int32_t  s_pa_temperature;
static int8_t   s_int_value;

/* Pending OUT targets (pointer into radio_state) */
static uint8_t  s_pending_cmd;
static uint8_t *s_pending_dst;
static uint16_t s_pending_len;

/* CMD_CALIBRATION write: payload lands here, wValue kept for complete_out */
static uint8_t  s_cal_buf[4];
static uint16_t s_cal_wvalue;

uint8_t emulated_register(void)
{
    uint8_t reg = 0;
    uint8_t st = Status_Read();
    if (TX_Request) {
        reg |= 0x10;
    }
    /* Match PSoC: STATUS_KEY_0 → 0x20, STATUS_KEY_1 → 0x02 */
    if (st & STATUS_KEY_0) {
        reg |= 0x20;
    }
    if (st & STATUS_KEY_1) {
        reg |= 0x02;
    }
    (void)E_key_down;
    return reg;
}

void usb_vendor_init(void)
{
    s_pending_cmd = 0;
    s_pending_dst = 0;
    s_pending_len = 0;
    E_dll_version = SI5351_DLL;
}

void usb_vendor_poll(void)
{
    /* Reserved for async USB stack service */
}

static usb_vendor_xfer_t xfer_in(uint8_t *src, uint16_t n, uint8_t *buf, uint16_t cap)
{
    usb_vendor_xfer_t x = {0};
    if (n > cap) {
        n = cap;
    }
    if (buf && src && n) {
        memcpy(buf, src, n);
    }
    x.data = buf;
    x.len = n;
    x.handled = 1;
    return x;
}

static usb_vendor_xfer_t xfer_out_prepare(uint8_t cmd, uint8_t *dst, uint16_t n)
{
    usb_vendor_xfer_t x = {0};
    s_pending_cmd = cmd;
    s_pending_dst = dst;
    s_pending_len = n;
    x.len = n;
    x.handled = 1;
    return x;
}

usb_vendor_xfer_t usb_vendor_setup(uint8_t bmRequestType, uint8_t bRequest,
                                   uint16_t wValue, uint16_t wIndex,
                                   uint16_t wLength, uint8_t *buf, uint16_t buf_cap)
{
    usb_vendor_xfer_t x = {0};
    (void)wLength;

    /* Device-to-host */
    if ((bmRequestType & USB_DIR_D2H) == USB_DIR_D2H) {
        switch (bRequest) {
        case CMD_GET_VERSION:
            s_result = FIRMWARE_VERSION;
            return xfer_in((uint8_t *)&s_result, 2, buf, buf_cap);

        case CMD_GET_PIN:
            s_result = emulated_register();
            s_result |= 0x08000000u;
            return xfer_in((uint8_t *)&s_result, 1, buf, buf_cap);

        case CMD_GET_FREQ:
            return xfer_in((uint8_t *)&Si570_LO, sizeof(Si570_LO), buf, buf_cap);

        case CMD_GET_STARTUP: {
            /* 56.32 MHz in host fixed-point style — stub */
            s_result = 0x713D0A07u;
            return xfer_in((uint8_t *)&s_result, 4, buf, buf_cap);
        }

        case CMD_GET_XTAL:
            E_dll_version = SI570_DLL;
            s_result = 0;
            return xfer_in((uint8_t *)&s_result, 4, buf, buf_cap);

        case CMD_SET_USRP1:
            if (wValue & 0x01u) {
                TX_Request = 1;
            } else {
                TX_Request = 0;
            }
            /* fall through — returns key */
            /* fallthrough */
        case CMD_GET_CW_KEY:
            s_result = emulated_register();
            return xfer_in((uint8_t *)&s_result, 1, buf, buf_cap);

        case CMD_SET_SI570:
            if ((wValue >> 8) == 0x87 && wIndex == 0x01) {
                s_result = 0;
                return xfer_in((uint8_t *)&s_result, 1, buf, buf_cap);
            }
            break;

        case CMD_GET_KEY_DOWN:
            return xfer_in((uint8_t *)&E_key_down, 1, buf, buf_cap);

        case CMD_GET_PTT:
            return xfer_in((uint8_t *)&E_PTT, 1, buf, buf_cap);

        /*
         * 3.10: these four go out little endian, as the host reads them (INT32 / INT16).
         * The PSoC (8051, big endian) swapped the bytes to get there; the pill copied the
         * swap, which on ARM sent them big endian: the temperature read 37 C as 620756992
         * (freq_cal.ini CALIBRATION_TEMPERATURE) and the 2-byte PPM reads gave the top
         * half (-46 -> -1, any positive value -> 0).
         */
        case CMD_GET_TRANSCEIVER_TEMP:
            /* Die / chip °C */
            s_proficio_temperature = E_transceiver_temp;
            return xfer_in((uint8_t *)&s_proficio_temperature, 4, buf, buf_cap);

        case CMD_GET_POTENTIA_TEMPERATURE:
            /* PA / board NTC °C (Solidus retired; opcode reused) */
            s_pa_temperature = E_pa_temp;
            return xfer_in((uint8_t *)&s_pa_temperature, 4, buf, buf_cap);

        case CMD_GET_PPM_INT:
            s_ppm_temp = ee_ppm_int;
            s_ppm_result = s_ppm_temp;
            return xfer_in((uint8_t *)&s_ppm_result, 2, buf, buf_cap);

        case CMD_GET_PPM_DEC:
            s_ppm_temp = ee_ppm_dec;
            s_ppm_result = s_ppm_temp;
            return xfer_in((uint8_t *)&s_ppm_result, 2, buf, buf_cap);

        case CMD_SET_XTAL_INT:
            s_int_value = ee_ppm_int;
            return xfer_in((uint8_t *)&s_int_value, 1, buf, buf_cap);

        case CMD_CALIBRATION:
            /* wValue: type << 8 | band. Bad type / band: not handled (stall) */
            x.len = cal_get((uint8_t)(wValue >> 8), (uint8_t)wValue, buf, buf_cap);
            if (x.len != 0u) {
                x.data = buf;
                x.handled = 1;
            }
            return x;

        case CMD_AUDIO_STATS:
            /* wValue 1 = also start a new min / max window after the read.
             * wIndex 1 = the hang record from before the last watchdog reset (3.13).
             * wIndex 2 = control transfer timing (3.17).
             * wIndex 3 = main loop timing (3.20).
             * wIndex 4 = receive slip recovery (pill 3.18, 2026-11-17); wValue 2 = fake a slip. */
            if (wIndex == 4u) {
                x.len = rx_resync_stats(buf, buf_cap, wValue);
            } else if (wIndex == 1u) {
                x.len = hang_rec_read(buf, buf_cap);
            } else if (wIndex == 2u) {
                x.len = usb_ctrl_stats(buf, buf_cap, (uint8_t)(wValue & 1u));
            } else if (wIndex == 3u) {
                x.len = loop_prof_stats(buf, buf_cap, (uint8_t)(wValue & 1u));
            } else {
                x.len = usb_audio_stats(buf, buf_cap, (uint8_t)(wValue & 1u));
            }
            if (x.len != 0u) {
                x.data = buf;
                x.handled = 1;
            }
            return x;

        case CMD_KEYER_BOOT:    /* 3.16: result of the last keyer I2C pass-through */
            x.len = keyer_boot_result(buf, buf_cap);
            if (x.len != 0u) {
                x.data = buf;
                x.handled = 1;
            }
            return x;

        default:
            break;
        }
        return x;
    }

    /* Host-to-device */
    switch (bRequest) {
    case CMD_SET_FREQ:
        return xfer_out_prepare(bRequest, (uint8_t *)&Si570_LO, sizeof(Si570_LO));

    case CMD_CALIBRATION: {
        uint8_t type = (uint8_t)(wValue >> 8);
        uint16_t n = cal_len(type);

        if (type == CAL_TYPE_SAVE) {    /* no save since build 10 (pill writes each set): stall */
            return x;
        }
        if (type == CAL_TYPE_QRO_VALID) {   /* 3.24: read only, a write stalls */
            return x;
        }
        /* exact length only, so a wrong payload is stalled, not cut off */
        if (n == 0u || n > sizeof(s_cal_buf) || (uint8_t)wValue >= CAL_BANDS || wLength != n) {
            return x;
        }
        s_cal_wvalue = wValue;
        return xfer_out_prepare(bRequest, s_cal_buf, n);
    }

    case CMD_SET_XTAL_INT:
        return xfer_out_prepare(bRequest, (uint8_t *)&E_calibration_int,
                                sizeof(E_calibration_int));

    case CMD_SET_XTAL_DEC:
        return xfer_out_prepare(bRequest, (uint8_t *)&E_calibration_dec,
                                sizeof(E_calibration_dec));

    case CMD_SET_PPM:
        return xfer_out_prepare(bRequest, (uint8_t *)&E_ppm, sizeof(E_ppm));

    case DLL_VERSION:
        return xfer_out_prepare(bRequest, &E_dll_version, 1);

    case CMD_SET_TRANSVERTER:
        return xfer_out_prepare(bRequest, &E_transverter, 1);

    case CMD_SET_PCB_VERSION:
        return xfer_out_prepare(bRequest, &E_pcb_version, 1);

    case CMD_SET_PA_BYPASS:
        return xfer_out_prepare(bRequest, &E_Amplifier, 1);

    case SET_CW_MODE:
        return xfer_out_prepare(bRequest, &E_host_mode, 1);

    case SET_QSK:
        return xfer_out_prepare(bRequest, &E_QSK, 1);

    case SET_TX_HOLD:
        return xfer_out_prepare(bRequest, &E_TX_Hold, 1);

    case CMD_SET_TRANSCEIVER_CW_PITCH:
        return xfer_out_prepare(bRequest, (uint8_t *)&E_cw_pitch, 1);

    case SET_KEYER_MODE:
        return xfer_out_prepare(bRequest, &E_keyer_mode, 1);

    case SET_CW_PADDLE:
        return xfer_out_prepare(bRequest, &E_paddle, 1);

    case SET_SPACING:
        return xfer_out_prepare(bRequest, &E_spacing, 1);

    case SET_MEM_TEXT_WPM:
        return xfer_out_prepare(bRequest, &E_mem_text_wpm, 1);

    case SET_PADDLE_FARNSWORTH:   /* 3.13 */
        return xfer_out_prepare(bRequest, &E_paddle_farns, 1);

    case SET_WEIGHT:
        return xfer_out_prepare(bRequest, &E_weight, 1);

    case SET_WPM:
        return xfer_out_prepare(bRequest, &E_wpm, 1);

    case SET_SIDE_TONE:
        return xfer_out_prepare(bRequest, &E_side_tone, 1);

    case SET_KEYER_INSTALLED:
        return xfer_out_prepare(bRequest, &E_keyer_installed, 1);

    case CMD_SET_KEYER_MEMORY:
        return xfer_out_prepare(bRequest, (uint8_t *)E_keyer_mem_pkt, 2);

    case CMD_KEYER_BOOT: {  /* 3.16: keyer I2C pass-through (keyer-flash.py) */
        uint8_t *dst = keyer_boot_setup(wValue, wIndex, wLength);
        if (!dst) {
            return x;       /* refused: stall */
        }
        if (wLength == 0u) {
            x.handled = 1;  /* queued already */
            x.len = 0;
            return x;
        }
        return xfer_out_prepare(bRequest, dst, wLength);
    }

    case CMD_ENTER_BOOTLOADER:
        /* Zero-length or any OUT: schedule ROM bootloader for CubeProgrammer */
        system_boot_request_reset();
        x.handled = 1;
        x.len = 0;
        return x;

    default:
        break;
    }
    return x;
}

void usb_vendor_complete_out(uint8_t bRequest, const uint8_t *buf, uint16_t len)
{
    if (!buf || !s_pending_dst) {
        return;
    }
    if (bRequest != s_pending_cmd) {
        return;
    }
    if (len > s_pending_len) {
        len = s_pending_len;
    }
    memcpy(s_pending_dst, buf, len);

    switch (bRequest) {
    case CMD_SET_FREQ:
        /* Host LO in Hz; update radio LO + CW LO */
        E_current_LO_freq = Si570_LO;
        CW_LO_Freq = Si570_LO;
        break;
    case CMD_SET_XTAL_INT:
        E_PPM_needs_updated |= PPM_INT_RECEIVED;
        break;
    case CMD_SET_XTAL_DEC:
        E_PPM_needs_updated |= PPM_DEC_RECEIVED;
        break;
    case SET_TX_HOLD:
        /* Hold time units: PSoC wrote period = hold * 100 timer ticks */
        break;
    case SET_PADDLE_FARNSWORTH:
        /* 3.14: pass every 0x9D to the PIC, even an unchanged one: the PIC keeps
         * it in RAM only, so an mscc restart puts it back after a PIC reset */
        E_paddle_farns_new = 1;
        break;
    case CMD_CALIBRATION:
        (void)cal_set((uint8_t)(s_cal_wvalue >> 8), (uint8_t)s_cal_wvalue, s_cal_buf, len);
        break;
    case CMD_KEYER_BOOT:
        keyer_boot_out_done(len);
        break;
    default:
        break;
    }

    s_pending_cmd = 0;
    s_pending_dst = 0;
    s_pending_len = 0;
}
