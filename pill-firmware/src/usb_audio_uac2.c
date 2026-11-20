/**
 * TinyUSB UAC2 ↔ Proficio I2S audio bridge.
 * Does not touch USB bring-up (usb_device.c / DWC2 init).
 */
#include "tusb.h"
#include "usb_descriptors.h"
#include "audio.h"
#include "usb_device.h"
#include "board.h"
#include "app.h"
#include "stm32f4xx.h"
#include <string.h>

TU_VERIFY_STATIC(CFG_AUDIO_EP_OUT_SZ == CFG_TUD_AUDIO_FUNC_1_EP_OUT_SZ_MAX,
                 "audio.h CFG_AUDIO_EP_OUT_SZ must match the USB OUT packet size");

static uint8_t s_spk_alt;
static uint8_t s_mic_alt;
static uint32_t s_sample_rate = 96000;

static int8_t s_mute[CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX + 1];
static int16_t s_volume[CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX + 1];

uint8_t usb_audio_tx_enabled(void)
{
    return (uint8_t)(s_spk_alt != 0);
}

uint8_t usb_audio_rx_enabled(void)
{
    return (uint8_t)(s_mic_alt != 0);
}

void usb_audio_streams_reset(void)
{
    s_spk_alt = 0;
    s_mic_alt = 0;
}

/* ---- FIFO access for the I2S side (see usb_device.h) ---- */

uint16_t usb_audio_out_available(void)
{
    return tud_audio_available();
}

uint16_t usb_audio_out_depth(void)
{
    return CFG_TUD_AUDIO_FUNC_1_EP_OUT_SW_BUF_SZ;
}

uint16_t usb_audio_out_read(uint8_t *dst, uint16_t n)
{
    return tud_audio_read(dst, n);
}

void usb_audio_out_clear(void)
{
    (void)tud_audio_clear_ep_out_ff();
}

uint8_t usb_audio_in_write(const uint8_t *src, uint16_t n)
{
    tu_fifo_t *ff = tud_audio_get_ep_in_ff();

    if (!ff || tu_fifo_remaining(ff) < n) {
        return 0;
    }
    return (uint8_t)(tud_audio_write(src, n) == n);
}

/* ---- Interface alt settings ---- */

bool tud_audio_set_itf_cb(uint8_t rhport, tusb_control_request_t const *req)
{
    (void)rhport;
    uint8_t itf = tu_u16_low(tu_le16toh(req->wIndex));
    uint8_t alt = tu_u16_low(tu_le16toh(req->wValue));

    if (itf == ITF_NUM_AUDIO_STREAMING_SPK) {
        s_spk_alt = alt;
    } else if (itf == ITF_NUM_AUDIO_STREAMING_MIC) {
        s_mic_alt = alt;
    }
    return true;
}

bool tud_audio_set_itf_close_ep_cb(uint8_t rhport, tusb_control_request_t const *req)
{
    (void)rhport;
    uint8_t itf = tu_u16_low(tu_le16toh(req->wIndex));
    if (itf == ITF_NUM_AUDIO_STREAMING_SPK) {
        s_spk_alt = 0;
    } else if (itf == ITF_NUM_AUDIO_STREAMING_MIC) {
        s_mic_alt = 0;
    }
    return true;
}

/* ---- Host → device (speaker / TX IQ) ---- */

/* TinyUSB 0.21: called in the USB interrupt once an OUT packet is in the FIFO. */
bool tud_audio_rx_done_isr(uint8_t rhport, uint16_t n_bytes_received,
                           uint8_t func_id, uint8_t ep_out,
                           uint8_t cur_alt_setting)
{
    (void)rhport;
    (void)func_id;
    (void)ep_out;
    (void)cur_alt_setting;

    /* The packet goes into TinyUSB's OUT FIFO; the I2S interrupt takes it
     * out at the codec's rate (audio_i2s_tx_block). Only note the arrival. */
    if (n_bytes_received) {
        audio_usb_out_packet_seen();
    }
    return true;
}

/* Feedback for the asynchronous OUT endpoint: TinyUSB holds the OUT FIFO at
 * half full and tells the host to send more or fewer samples per frame. */
void tud_audio_feedback_params_cb(uint8_t func_id, uint8_t alt_itf,
                                  audio_feedback_params_t *feedback_param)
{
    (void)func_id;
    (void)alt_itf;
    feedback_param->method = AUDIO_FEEDBACK_METHOD_FIFO_COUNT;
    feedback_param->sample_freq = s_sample_rate;
}

/* ---- Device → host (mic / RX IQ) ---- */

/*
 * IN stream counters (3.11, hunting the 11.5 s spike bursts). Updated here,
 * in the USB interrupt (TinyUSB 0.21), just after TinyUSB queued the next IN
 * packet (it was in tud_task, before queuing, on 0.17).
 */
#define IN_PKT_MIN_BYTES (95u * 4u)   /* below this flow control sends 0 bytes */
static uint32_t s_in_packets;          /* IN packets scheduled */
static uint32_t s_in_empty;            /* scheduled with < 95 frames = empty packet */
static uint32_t s_in_gap_events;       /* USB frame number moved >= 2 since the last */
static uint32_t s_in_gap_max;          /* largest move, in frames */
static uint16_t s_in_lvl_min = 0xFFFFu; /* IN FIFO fill (bytes), min / max */
static uint16_t s_in_lvl_max;
static uint16_t s_in_last_frame;
static uint8_t  s_in_have_frame;

static uint16_t usb_frame_number(void)
{
    USB_OTG_DeviceTypeDef *dev =
        (USB_OTG_DeviceTypeDef *)(USB_OTG_FS_PERIPH_BASE + USB_OTG_DEVICE_BASE);
    return (uint16_t)((dev->DSTS & USB_OTG_DSTS_FNSOF) >> USB_OTG_DSTS_FNSOF_Pos);
}

bool tud_audio_tx_done_isr(uint8_t rhport, uint16_t n_bytes_sent, uint8_t func_id,
                           uint8_t ep_in, uint8_t cur_alt_setting)
{
    (void)rhport;
    (void)n_bytes_sent;
    (void)func_id;
    (void)ep_in;
    (void)cur_alt_setting;

    /* Nothing to load here: the I2S interrupt fills the IN FIFO at the
     * codec's rate (audio_i2s_rx_block) and TinyUSB's flow control picks
     * the packet size (95/96/97 frames). Only count. */
    tu_fifo_t *ff = tud_audio_get_ep_in_ff();
    uint16_t lvl = ff ? tu_fifo_count(ff) : 0u;
    uint16_t fn = usb_frame_number();

    s_in_packets++;
    if (lvl < IN_PKT_MIN_BYTES) {
        s_in_empty++;
    }
    if (lvl < s_in_lvl_min) {
        s_in_lvl_min = lvl;
    }
    if (lvl > s_in_lvl_max) {
        s_in_lvl_max = lvl;
    }
    if (s_in_have_frame) {
        uint16_t gap = (uint16_t)((fn - s_in_last_frame) & 0x7FFu); /* 11-bit FS frame */
        if (gap >= 2u) {
            s_in_gap_events++;
            if (gap > s_in_gap_max) {
                s_in_gap_max = gap;
            }
        }
    }
    s_in_last_frame = fn;
    s_in_have_frame = 1;
    return true;
}

static void put_u32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
    p[2] = (uint8_t)(v >> 16);
    p[3] = (uint8_t)(v >> 24);
}

uint16_t usb_audio_stats(uint8_t *buf, uint16_t cap, uint8_t clear)
{
    uint32_t w[USB_AUDIO_STATS_WORDS];
    uint16_t i;

    if (cap < sizeof(w)) {
        return 0;
    }
    w[0]  = 3u | (USB_AUDIO_STATS_WORDS << 16);  /* layout version | word count */
    w[1]  = board_millis();
    w[2]  = s_in_packets;
    w[3]  = s_in_empty;
    w[4]  = s_in_gap_events;
    w[5]  = s_in_gap_max;
    w[6]  = s_in_lvl_min;
    w[7]  = s_in_lvl_max;
    w[8]  = audio_rx_dropped();      /* IN FIFO full, block dropped */
    w[9]  = audio_tx_underruns();    /* OUT FIFO ran empty */
    w[10] = audio_tx_dropped();      /* OUT FIFO nearly full, block dropped */
    w[11] = audio_out_level_min();
    w[12] = audio_out_level_max();
    w[13] = board_reset_flags()      /* why the pill last started (3.12, board.h) */
          | ((uint32_t)app_i2c_retry_info() << 16);   /* power-up I2C retry (3.5) */
    w[14] = proficio_ep0_in_cut;     /* EP0 IN ended with data unsent (3.13) */
    w[15] = usb_otg_irq_count();     /* USB interrupts since power-up (3.13) */
    for (i = 0; i < USB_AUDIO_STATS_WORDS; i++) {
        put_u32(&buf[i * 4u], w[i]);
    }
    if (clear) {
        s_in_lvl_min = 0xFFFFu;
        s_in_lvl_max = 0u;
        s_in_gap_max = 0u;
        audio_out_level_reset();
    }
    return (uint16_t)sizeof(w);
}

/* ---- Clock / feature unit controls (minimal) ---- */

/* UAC2 class request as laid out in the SETUP packet. TinyUSB 0.17 had this as
 * audio_control_request_t; 0.21 dropped it. */
typedef struct TU_ATTR_PACKED {
    uint8_t  bmRequestType;
    uint8_t  bRequest;          /* AUDIO20_CS_REQ_CUR / RANGE */
    uint8_t  bChannelNumber;    /* wValue low byte */
    uint8_t  bControlSelector;  /* wValue high byte */
    uint8_t  bInterface;        /* wIndex low byte */
    uint8_t  bEntityID;         /* wIndex high byte */
    uint16_t wLength;
} uac2_request_t;
TU_VERIFY_STATIC(sizeof(uac2_request_t) == sizeof(tusb_control_request_t), "uac2_request_t layout");

static bool clock_get(uint8_t rhport, uac2_request_t const *request)
{
    if (request->bEntityID != UAC2_ENTITY_CLOCK) {
        return false;
    }
    if (request->bControlSelector == AUDIO20_CS_CTRL_SAM_FREQ) {
        if (request->bRequest == AUDIO20_CS_REQ_CUR) {
            audio20_control_cur_4_t cur = {.bCur = (int32_t)tu_htole32(s_sample_rate)};
            return tud_audio_buffer_and_schedule_control_xfer(
                rhport, (tusb_control_request_t const *)request, &cur, sizeof(cur));
        }
        if (request->bRequest == AUDIO20_CS_REQ_RANGE) {
            audio20_control_range_4_n_t(1) range = {
                .wNumSubRanges = tu_htole16(1),
                .subrange = {{.bMin = (int32_t)96000, .bMax = (int32_t)96000, .bRes = 0}}
            };
            return tud_audio_buffer_and_schedule_control_xfer(
                rhport, (tusb_control_request_t const *)request, &range, sizeof(range));
        }
    }
    if (request->bControlSelector == AUDIO20_CS_CTRL_CLK_VALID &&
        request->bRequest == AUDIO20_CS_REQ_CUR) {
        audio20_control_cur_1_t v = {.bCur = 1};
        return tud_audio_buffer_and_schedule_control_xfer(
            rhport, (tusb_control_request_t const *)request, &v, sizeof(v));
    }
    return false;
}

static bool clock_set(uint8_t rhport, uac2_request_t const *request,
                      uint8_t const *buf)
{
    (void)rhport;
    if (request->bEntityID != UAC2_ENTITY_CLOCK) {
        return false;
    }
    if (request->bControlSelector == AUDIO20_CS_CTRL_SAM_FREQ &&
        request->bRequest == AUDIO20_CS_REQ_CUR) {
        s_sample_rate = (uint32_t)((audio20_control_cur_4_t const *)buf)->bCur;
        return true;
    }
    return false;
}

static bool feature_get(uint8_t rhport, uac2_request_t const *request)
{
    if (request->bEntityID != UAC2_ENTITY_SPK_FEATURE_UNIT) {
        return false;
    }
    if (request->bControlSelector == AUDIO20_FU_CTRL_MUTE &&
        request->bRequest == AUDIO20_CS_REQ_CUR) {
        audio20_control_cur_1_t m = {.bCur = s_mute[request->bChannelNumber]};
        return tud_audio_buffer_and_schedule_control_xfer(
            rhport, (tusb_control_request_t const *)request, &m, sizeof(m));
    }
    if (request->bControlSelector == AUDIO20_FU_CTRL_VOLUME) {
        if (request->bRequest == AUDIO20_CS_REQ_RANGE) {
            audio20_control_range_2_n_t(1) r = {
                .wNumSubRanges = tu_htole16(1),
                .subrange = {{.bMin = tu_htole16(-5120), .bMax = tu_htole16(0), .bRes = tu_htole16(256)}}
            };
            return tud_audio_buffer_and_schedule_control_xfer(
                rhport, (tusb_control_request_t const *)request, &r, sizeof(r));
        }
        if (request->bRequest == AUDIO20_CS_REQ_CUR) {
            audio20_control_cur_2_t c = {.bCur = tu_htole16(s_volume[request->bChannelNumber])};
            return tud_audio_buffer_and_schedule_control_xfer(
                rhport, (tusb_control_request_t const *)request, &c, sizeof(c));
        }
    }
    return false;
}

static bool feature_set(uint8_t rhport, uac2_request_t const *request,
                        uint8_t const *buf)
{
    (void)rhport;
    if (request->bEntityID != UAC2_ENTITY_SPK_FEATURE_UNIT) {
        return false;
    }
    if (request->bControlSelector == AUDIO20_FU_CTRL_MUTE &&
        request->bRequest == AUDIO20_CS_REQ_CUR) {
        s_mute[request->bChannelNumber] = ((audio20_control_cur_1_t const *)buf)->bCur;
        return true;
    }
    if (request->bControlSelector == AUDIO20_FU_CTRL_VOLUME &&
        request->bRequest == AUDIO20_CS_REQ_CUR) {
        s_volume[request->bChannelNumber] = ((audio20_control_cur_2_t const *)buf)->bCur;
        return true;
    }
    return false;
}

bool tud_audio_get_req_entity_cb(uint8_t rhport, tusb_control_request_t const *p_request)
{
    uac2_request_t const *req = (uac2_request_t const *)p_request;
    if (req->bEntityID == UAC2_ENTITY_CLOCK) {
        return clock_get(rhport, req);
    }
    if (req->bEntityID == UAC2_ENTITY_SPK_FEATURE_UNIT) {
        return feature_get(rhport, req);
    }
    return false;
}

bool tud_audio_set_req_entity_cb(uint8_t rhport, tusb_control_request_t const *p_request,
                                 uint8_t *buf)
{
    uac2_request_t const *req = (uac2_request_t const *)p_request;
    if (req->bEntityID == UAC2_ENTITY_CLOCK) {
        return clock_set(rhport, req, buf);
    }
    if (req->bEntityID == UAC2_ENTITY_SPK_FEATURE_UNIT) {
        return feature_set(rhport, req, buf);
    }
    return false;
}
