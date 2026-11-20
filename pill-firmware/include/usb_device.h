/**
 * USB device: vendor (0xFF) + UAC1 audio (Proficio-compatible layout).
 * VID 0x16C0 PID 0x05DC — matches PSoC USBFS descriptors.
 */
#ifndef USB_DEVICE_H
#define USB_DEVICE_H

#include <stdint.h>

#define PROFICIO_USB_VID  0x16C0u
#define PROFICIO_USB_PID  0x05DCu

void usb_device_init(void);
void usb_device_poll(void);
uint8_t usb_device_configured(void);
uint32_t usb_device_bus_resets(void);
uint32_t usb_device_setups(void);
/** OTG_FS interrupts since power-up (3.13). */
uint32_t usb_otg_irq_count(void);
/** EP0 IN transfers the core ended with data still unsent (3.13, dcd_dwc2.c). */
extern volatile uint32_t proficio_ep0_in_cut;
/** IN packets the host did not ACK (timeout condition, 3.18, dcd_dwc2.c). */
extern volatile uint32_t proficio_ep_in_toc;
/** Unfinished EP0 IN replies stopped at a new SETUP; waits on the USB core
 *  that ran out (3.19, dcd_dwc2.c). */
extern volatile uint32_t proficio_ep0_in_abort;
extern volatile uint32_t proficio_usb_wait_timeout;
/** Control transfer timing (3.17): CMD_AUDIO_STATS wIndex 2. Fills buf with
 *  USB_CTRL_STATS_WORDS little-endian uint32 values, returns the byte count.
 *  clear = 1 restarts the two max values after the read. */
#define USB_CTRL_STATS_WORDS 14u
uint16_t usb_ctrl_stats(uint8_t *buf, uint16_t cap, uint8_t clear);

/** True when host enabled streaming alts (TX interface alt 1). */
uint8_t usb_audio_tx_enabled(void);
uint8_t usb_audio_rx_enabled(void);
/** Forget the streaming alts (host gone); the host sets them again. */
void usb_audio_streams_reset(void);

/*
 * Asynchronous audio FIFOs (TinyUSB), used from the I2S DMA interrupt.
 * Blocks are whole stereo frames (multiples of 4 bytes).
 */
/** Bytes waiting in the OUT (host -> radio, TX IQ) FIFO. */
uint16_t usb_audio_out_available(void);
/** Size of the OUT FIFO in bytes. */
uint16_t usb_audio_out_depth(void);
/** Take n bytes from the OUT FIFO. Returns bytes read. */
uint16_t usb_audio_out_read(uint8_t *dst, uint16_t n);
/** Empty the OUT FIFO. */
void     usb_audio_out_clear(void);
/** Put n bytes into the IN (radio -> host, RX IQ) FIFO. All or nothing:
 *  returns 0 if there is no room for the whole block. */
uint8_t  usb_audio_in_write(const uint8_t *src, uint16_t n);

/** Audio stream counters for CMD_AUDIO_STATS (3.11): fills buf with
 *  USB_AUDIO_STATS_WORDS little-endian uint32 values, returns the byte count.
 *  clear = 1 starts a new min / max window after the read. */
#define USB_AUDIO_STATS_WORDS 16u
uint16_t usb_audio_stats(uint8_t *buf, uint16_t cap, uint8_t clear);

#endif /* USB_DEVICE_H */
