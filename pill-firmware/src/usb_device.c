/**
 * USB device — TinyUSB (Synopsys DWC2 OTG_FS) for Black Pill F411.
 *
 * Keep this path simple: aggressive SOF/re-enum recovery was killing a
 * healthy link (fast blink → slow blink). Host-reboot attach can be
 * revisited later without touching the steady-state enum path.
 */
#include "usb_device.h"
#include "usb_vendor.h"
#include "board.h"
#include "proficio_config.h"
#include "tusb.h"
#include "radio_state.h"
#include "control.h"
#include "stm32f4xx_hal.h"

static volatile uint8_t s_configured;

/*
 * Unplug handling. VBUS sensing is off (see usb_hw_init), so an unplug only
 * shows up as "bus idle" (suspend), the same as a sleeping host. Seen
 * 2026-10-04: after moving the cable from the Pi to a PC the app kept running
 * (LED O) but the new host did not see the rig until NRST. A pill started with
 * no host connected does accept a later plug-in, and restarting only the USB
 * core (dcd_init again) did not cure it. So on an unplug the processor is
 * reset, exactly what NRST does.
 *
 * The host is taken as gone, and the processor reset, on either of:
 *  - USBV+ on PA9 (daughter board divider, 10k / 23.5k -> ~3.5 V with a
 *    cable in) low for USB_VBUS_GONE_MS, like the PSoC watched VBUS: cable
 *    pulled, or the port lost power (PC shut down, motherboard port).
 *    Ignored until PA9 has read high once (a bare pill has no divider).
 *  - Bus idle for USB_RESTART_AFTER_MS: host restarted or shut down with
 *    USBV+ still up (powered hub), or any unplug on a bare pill. A sleeping
 *    host also lands here and resets the rig.
 * No reset loop: both need a host to have enumerated us since the last reset
 * (s_restart_armed), and a freshly reset pill with no host never gets there.
 */
#define USB_RESTART_AFTER_MS 500u
#define USB_VBUS_GONE_MS     100u
static volatile uint8_t s_suspended;
static volatile uint8_t s_restart_armed;
static volatile uint32_t s_suspend_ms;
static uint8_t  s_vbus_seen;   /* PA9 has read high: this board has the divider */
static uint8_t  s_vbus_low;    /* PA9 currently low, timing from s_vbus_low_ms */
static uint32_t s_vbus_low_ms;

static volatile uint32_t s_otg_irqs;   /* hang recorder (3.13) */

void OTG_FS_IRQHandler(void)
{
    s_otg_irqs++;
    tud_int_handler(0);
}

uint32_t usb_otg_irq_count(void)
{
    return s_otg_irqs;
}

/* Device mode: ignore pad VBUS so bare-pill USB-C still enumerates. */
static void usb_vbus_sense_off(void)
{
#ifdef USB_OTG_GCCFG_NOVBUSSENS
    USB_OTG_FS->GCCFG |= USB_OTG_GCCFG_NOVBUSSENS;
#endif
#ifdef USB_OTG_GCCFG_VBUSBSEN
    USB_OTG_FS->GCCFG &= ~USB_OTG_GCCFG_VBUSBSEN;
#endif
#ifdef USB_OTG_GCCFG_VBUSASEN
    USB_OTG_FS->GCCFG &= ~USB_OTG_GCCFG_VBUSASEN;
#endif
}

static void usb_hw_init(void)
{
    GPIO_InitTypeDef g = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_USB_OTG_FS_CLK_ENABLE();

    /* D-/D+ only. PA9 is GPIO USBV+ sense (divider) — not OTG VBUS pad. */
    g.Pin = GPIO_PIN_11 | GPIO_PIN_12;
    g.Mode = GPIO_MODE_AF_PP;
    g.Pull = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    g.Alternate = GPIO_AF10_OTG_FS;
    HAL_GPIO_Init(GPIOA, &g);

    usb_vbus_sense_off();

    HAL_NVIC_SetPriority(OTG_FS_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(OTG_FS_IRQn);
}


void usb_device_init(void)
{
    usb_vendor_init();
    board_delay_ms(50);
    usb_hw_init();
    (void)tusb_init();
}

void usb_device_poll(void)
{
    uint32_t now;
    uint8_t unplugged = 0;

    tud_task();
    now = board_millis();

    if (vbus_sense_present()) {
        s_vbus_seen = 1;
        s_vbus_low = 0;
    } else if (s_vbus_seen) {
        if (!s_vbus_low) {
            s_vbus_low = 1;
            s_vbus_low_ms = now;
        } else if ((now - s_vbus_low_ms) >= USB_VBUS_GONE_MS) {
            unplugged = 1;          /* USBV+ gone: cable pulled or port power off */
        }
    }
    if (s_suspended && (now - s_suspend_ms) >= USB_RESTART_AFTER_MS) {
        unplugged = 1;              /* bus idle: host gone, USBV+ may still be up */
    }

    if (unplugged && s_restart_armed) {
        /* On reset the RX and AMP pins float and the mother-board pull-ups
         * hold receive. */
        TX_Request = 0;
        NVIC_SystemReset();
    }
}

uint8_t usb_device_configured(void)
{
    return s_configured;
}

/*
 * Control transfer timing (3.17, evidence only): called from dcd_dwc2.c.
 * setup = SETUP received (interrupt); armed = first EP0 IN queued after it
 * (reply data, or status of a host-to-device request; main loop); done = the
 * host took that EP0 IN, or EP0 was stalled. A SETUP arriving before the last
 * one was done = the host gave up on it (abandoned).
 * setup->armed slow = the pill was late; armed->done slow = the host / bus.
 */
#define CTRL_SLOW_MS 100u
static volatile uint32_t s_ctrl_setups, s_ctrl_armed, s_ctrl_done, s_ctrl_abandoned;
static volatile uint32_t s_ctrl_slow, s_ctrl_last_slow_ms;
static volatile uint32_t s_ctrl_arm_max, s_ctrl_done_max;
static volatile uint32_t s_ctrl_t_setup, s_ctrl_t_armed;
static volatile uint8_t  s_ctrl_open, s_ctrl_is_armed;
/* 3.20: host-to-pill data arrived -> status queued (pill's own time) */
static volatile uint32_t s_ctrl_t_data, s_ctrl_data_max;
static volatile uint8_t  s_ctrl_has_data;

void proficio_ctrl_setup(void)
{
    s_ctrl_setups++;
    if (s_ctrl_open) {
        s_ctrl_abandoned++;
    }
    s_ctrl_t_setup = board_millis();
    s_ctrl_open = 1;
    s_ctrl_is_armed = 0;
    s_ctrl_has_data = 0;
}

void proficio_ctrl_data(void)
{
    if (s_ctrl_open && !s_ctrl_is_armed) {
        s_ctrl_t_data = board_millis();
        s_ctrl_has_data = 1;
    }
}

void proficio_ctrl_armed(void)
{
    uint32_t d;

    if (!s_ctrl_open || s_ctrl_is_armed) {
        return;
    }
    s_ctrl_t_armed = board_millis();
    s_ctrl_is_armed = 1;
    s_ctrl_armed++;
    d = s_ctrl_t_armed - s_ctrl_t_setup;
    if (d > s_ctrl_arm_max) {
        s_ctrl_arm_max = d;
    }
    if (s_ctrl_has_data) {
        d = s_ctrl_t_armed - s_ctrl_t_data;
        if (d > s_ctrl_data_max) {
            s_ctrl_data_max = d;
        }
    }
}

void proficio_ctrl_done(void)
{
    uint32_t now, d;

    if (!s_ctrl_open) {
        return;
    }
    now = board_millis();
    s_ctrl_open = 0;
    s_ctrl_done++;
    if (s_ctrl_is_armed) {
        d = now - s_ctrl_t_armed;
        if (d > s_ctrl_done_max) {
            s_ctrl_done_max = d;
        }
    }
    if ((now - s_ctrl_t_setup) >= CTRL_SLOW_MS) {
        s_ctrl_slow++;
        s_ctrl_last_slow_ms = now;
    }
}

uint16_t usb_ctrl_stats(uint8_t *buf, uint16_t cap, uint8_t clear)
{
    uint32_t w[USB_CTRL_STATS_WORDS];
    uint16_t i;

    if (cap < sizeof(w)) {
        return 0;
    }
    w[0] = 4u | (USB_CTRL_STATS_WORDS << 16);  /* layout version | word count */
    w[1] = board_millis();
    w[2] = s_ctrl_setups;
    w[3] = s_ctrl_armed;
    w[4] = s_ctrl_done;
    w[5] = s_ctrl_abandoned;
    w[6] = s_ctrl_arm_max;     /* ms, SETUP -> reply queued (pill) */
    w[7] = s_ctrl_done_max;    /* ms, reply queued -> taken (host / bus) */
    w[8] = s_ctrl_slow;        /* SETUP -> done >= CTRL_SLOW_MS */
    w[9] = s_ctrl_last_slow_ms;
    w[10] = proficio_ep_in_toc;  /* IN packets not ACKed by the host (3.18) */
    w[11] = proficio_ep0_in_abort;      /* old EP0 reply stopped at new SETUP (3.19) */
    w[12] = proficio_usb_wait_timeout;  /* USB core waits that ran out (3.19) */
    w[13] = s_ctrl_data_max;  /* ms, host data arrived -> status queued (3.20) */
    for (i = 0; i < USB_CTRL_STATS_WORDS; i++) {
        buf[i * 4u]      = (uint8_t)w[i];
        buf[i * 4u + 1u] = (uint8_t)(w[i] >> 8);
        buf[i * 4u + 2u] = (uint8_t)(w[i] >> 16);
        buf[i * 4u + 3u] = (uint8_t)(w[i] >> 24);
    }
    if (clear) {
        s_ctrl_arm_max = 0;
        s_ctrl_done_max = 0;
        s_ctrl_data_max = 0;
    }
    return (uint16_t)sizeof(w);
}

uint32_t usb_device_bus_resets(void)
{
    return 0;
}

uint32_t usb_device_setups(void)
{
    return 0;
}

void tud_mount_cb(void)
{
    usb_audio_streams_reset();
    s_suspended = 0;
    s_restart_armed = 1;
    s_configured = 1;
}

void tud_umount_cb(void)
{
    s_configured = 0;
}

/* Bus idle: unplugged or host asleep. Like the PSoC on losing USB power:
 * drop transmit and stop the USB audio streams. */
void tud_suspend_cb(bool remote_wakeup_en)
{
    (void)remote_wakeup_en;
    TX_Request = 0;
    s_configured = 0;
    usb_audio_streams_reset();
    s_suspend_ms = board_millis();
    s_suspended = 1;
}

void tud_resume_cb(void)
{
    s_suspended = 0;
    s_configured = (uint8_t)tud_mounted();
}

bool tud_vendor_control_xfer_cb(uint8_t rhport, uint8_t stage,
                                tusb_control_request_t const *request)
{
    static uint8_t buf[64];
    const uint8_t is_in = (uint8_t)((request->bmRequestType & 0x80u) != 0);

    if (stage == CONTROL_STAGE_SETUP) {
        /* 3.17: Windows asks for the Microsoft OS 2.0 set (WinUSB). Not a radio command. */
        if (is_in && request->bRequest == MS_OS_20_VENDOR_CODE &&
            request->wIndex == MS_OS_20_DESC_INDEX) {
            extern uint8_t const desc_ms_os_20[];
            return tud_control_xfer(rhport, request, (void *)(uintptr_t)desc_ms_os_20,
                                    MS_OS_20_DESC_LEN);
        }

        usb_vendor_xfer_t x = usb_vendor_setup(
            request->bmRequestType, request->bRequest, request->wValue,
            request->wIndex, request->wLength, buf, sizeof(buf));

        if (!x.handled) {
            return false;
        }

        if (is_in) {
            return tud_control_xfer(rhport, request, buf, x.len);
        }

        if (request->wLength > 0) {
            uint16_t n = request->wLength;
            if (n > sizeof(buf)) {
                n = sizeof(buf);
            }
            return tud_control_xfer(rhport, request, buf, n);
        }

        return tud_control_status(rhport, request);
    }

    if (stage == CONTROL_STAGE_DATA) {
        if (!is_in && request->wLength > 0) {
            uint16_t n = request->wLength;
            if (n > sizeof(buf)) {
                n = sizeof(buf);
            }
            usb_vendor_complete_out(request->bRequest, buf, n);
        }
        return true;
    }

    return true;
}
