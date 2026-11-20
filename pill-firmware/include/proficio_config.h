/**
 * Proficio-on-STM32F411 configuration.
 */
#ifndef PROFICIO_CONFIG_H
#define PROFICIO_CONFIG_H

#define PROFICIO_FW_NAME        "proficio-stm32f411-25MHz"
/*
 * Firmware version reported to host (CMD_GET_VERSION).
 * {Model}.{Build number}  (the bytes are reversed)
 * Model 0-OSB, 1-Proficio, 2-Geminus, 3-MKII-PTT, 4-MKII-ATU
 * MAJOR must stay 3: the client uses it to tell which transceiver is attached.
 * MINOR is a build number (Ron, 2026-10-05): add 1 for EVERY build, so Ron and
 * Stew can see they run the same firmware. One byte. (The PSoC used
 * month + release digit; 232 was copied from it and never changed.)
 * Ranges (Ron, 2026-10-25): the pill uses 1-199, the PSoC 200 and up (3.240
 * now), so the two never collide. ms-sdr needs MINOR >= 1, so not 0.
 *
 * Baseline (2026-10-25): numbering restarted at 1 with the 3.24 code
 * (TinyUSB 0.21, tested: receive, band changes, TUN, FT8 QSO).
 *   1 = 2026-10-25, baseline, same code as old-series 24
 *   2 = 2026-10-27, PTT footswitch (PA6) polled every 35 ms in the main loop
 *       (app.c, as the PSoC); E_PTT was only refreshed by other requests
 *   3 = 2026-10-27, LED: keyed + silent host audio (I) shows solid ON when
 *       USB is up (normal in an SSB pause); I only without a host (app.c)
 *   4 = 2026-10-27, power-up I2C retry: codec / Si5351 set up again at
 *       enumeration or 3 s, every 500 ms, 10 tries (C at power-up, NRST
 *       cleared it) (app.c, pcm3060.c, i2c_bus.c)
 *   5 = 2026-10-27, power-up I2C retry record in stats word 13 bits 16-27
 *       (audio-stats.py "power-up I2C" line)
 *   6 = 2026-10-29, TX raised-cosine envelope in all modes: ramp up after PA on,
 *       ramp down before PA off (TUN press / release splatter, Stew's SA);
 *       non-CW mode change no longer drops TX; entering CW ramps down first
 *       (app.c, audio.c; as PSoC MKII 3.242 but with the CW ramp)
 *   7 = 2026-10-29, LO back to the RX frequency (with RIT) only after TX_Main
 *       is idle, not when TX_Request drops (PA still on in the ramp down)
 *       (si5351a.c, app.c; Stew's question 4)
 *   8 = 2026-10-30, Si5351 CLK1 / CLK2 powered down and outputs disabled at
 *       start-up, as the PSoC (spur comb around the LO on WWV 15 MHz)
 *       (si5351a.c)
 *   9 = 2026-10-30, frequency calibration PPM (CMD_SET_XTAL_INT / _DEC) used:
 *       applied + retune, kept in the flash calibration store, loaded at boot
 *       (app.c, calibration.c); before, it was received and never used
 *  10 = 2026-10-30, USB replies for transceiver / PA temperature and PPM
 *       sent little endian, as the host reads them (the PSoC swap, copied,
 *       sent them big endian on ARM) (usb_vendor.c)
 *
 * Old series (before the restart):
 *   1 = 2026-10-05, TX left word delayed one sample (audio.c)
 *   2 = 2026-10-05, same code as 1 (build to check the version number)
 *   3 = 2026-10-05, fixed USB serial string (usb_descriptors.c)
 *   4 = 2026-10-05, temperature ADC reads no longer wait (RX audio dropouts)
 *   5 = 2026-10-20, non-QSK amp: CW key delay, PTT amp-then-PA order,
 *       entering CW drops TX (same as PSoC MKII 3.233 / 3.234)
 *   6 = 2026-10-20, CW raised-cosine keying envelope on the TX I/Q (audio.c)
 *   7 = 2026-10-21, per-band calibration store (RX IQ, TX IQ, power) in flash
 *       sector 7, opcode CMD_CALIBRATION 0xC8 (calibration.c)
 *   8 = 2026-10-21, calibration: valid masks (type 5), clear (type 6)
 *   9 = 2026-10-22, calibration: clear (type 6) removed (as PSoC MKII 3.239)
 *  10 = 2026-10-23, calibration: no save command, each set written to flash by
 *       the pill (log in sector 7), as PSoC MKII 3.238
 *  11 = 2026-10-24, audio stream counters, read with CMD_AUDIO_STATS 0xC9
 *       (IN empty packets / frame gaps / FIFO levels, OUT under/overruns)
 *  12 = 2026-10-24, USB hang fix: EP0 TX-empty interrupt storm (dcd_dwc2.c);
 *       watchdog (~2 s); reset cause in CMD_AUDIO_STATS word 13
 *  13 = 2026-10-24, hang recorder (hang_rec.c, TIM11): where the CPU was before
 *       a watchdog reset, read with CMD_AUDIO_STATS wIndex 1; EP0 cut-short
 *       and USB interrupt counters (words 14, 15)
 *  14 = 2026-10-24, ISO IN aimed at the frame after the last completion
 *       (dcd_dwc2.c): fixes IN frames skipped in 1 s bursts every 11.5 s
 *       (RX I/Q blocks dropped = waterfall spikes)
 *  15 = 2026-10-25, LED solid on when USB enumerated and O / T (app.c)
 *  16 = 2026-10-25, solid LED turned back on after a host mode change
 *       cleared it (TUN left it off on 15)
 *  17 = 2026-10-25, control transfer timing counters (evidence only), read
 *       with CMD_AUDIO_STATS wIndex 2 (usb_device.c, dcd_dwc2.c)
 *  18 = 2026-10-25, USB hang fix: IN timeout condition (TOC) cleared and
 *       counted (dcd_dwc2.c); left set it re-entered the interrupt forever
 *  19 = 2026-10-25, dcd_dwc2.c hardening: waits on USB core bits bounded and
 *       counted; an EP0 IN reply still armed at a new SETUP is stopped and
 *       FIFO 0 flushed (counted)
 *  20 = 2026-10-25, main loop timing per part (loop_prof.c, CMD_AUDIO_STATS
 *       wIndex 3) and host data -> status time (ctrl stats word 13);
 *       evidence only
 *  21 = 2026-10-25, keyer writes no longer block: one I2C attempt per pass,
 *       retried on later passes (was 8 tries with 5 ms delays, ~48 ms stall
 *       when the keyer did not answer) (cw.c)
 *  22 = 2026-10-25, Si5351 PLL / Multisynth written as one 8-register I2C
 *       burst (~1 ms) instead of 8 writes (~3 ms main-loop stall at each
 *       band change, USB audio packets missed) (si5351a.c)
 *  23 = 2026-10-25, TinyUSB 0.21.0 (was 0.17.0): audio ISO packets handled in
 *       the USB interrupt, not tud_task (branch tinyusb-0.21). Proficio dwc2
 *       patches carried over: 1-packet RX FIFO, 16-word min TX FIFO, bounded
 *       waits, TXFE condition, counters / ctrl timing hooks
 *  24 = 2026-10-25, 0.21: RX FIFO 1 packet + all spare FIFO RAM (175 words);
 *       3.23's 120 words lost SETUPs behind audio OUT packets (PTT timeouts)
 */
#define FIRMWARE_VERSION_MAJOR  3
#define FIRMWARE_VERSION_MINOR  24   /* 3.24 (2026-11-19): QRO (amplifier) power cal stored per band (calibration type 8, int16; valid mask type 9), same flash log. 3.23 (2026-11-18): USB names as the PSoC radios: product "Multus IQ", sound "Multus IQ Sound" / "Multus IQ Receive" / "Multus IQ Transmit" (Ron). 3.22 (2026-11-17): slip restart also resets SPI2 in the RCC + HAL_I2S_Init again, as NRST (3.21 test: stop / start alone left frame errors)
                                        3.21 (2026-11-17): test wValue 3/4 = 3 false pulses on PB13 bit clock / PB12 word select (3.20's mid-frame re-enable never misframed)
                                        3.20 (2026-11-17): receive slip also = I2S2ext frame error on 15 polls in a row (~0.1 s), same restart (many-bit slip during FT8 that 3.19 missed); stats word 7 adds restarts by FRE; test wValue 3/4 = re-enable I2S2ext mid-frame
                                        3.19 (2026-11-17): slip check armed 5 s after receive is steady at power-up; needs values near 0 AND near +32767 (3.18 fired once at power-up)
                                        3.18 (2026-11-17): receive slip recovery (rx_resync.c): no negative RX sample for ~100 ms -> restart I2S + codec while receiving; stats CMD_AUDIO_STATS wIndex 4
                                        3.17 (2026-11-16): WinUSB via Microsoft OS 2.0 descriptors (no Zadig), bcdUSB 2.10, bcdDevice 1.01
                                        3.16 (2026-11-12): CMD_KEYER_BOOT 0x9F, I2C pass-through for keyer-flash.py
                                        3.15 (2026-11-10): heat sink below -40 C = OPEN, above 150 C = SHORT */
#define FIRMWARE_VERSION ((((FIRMWARE_VERSION_MINOR) << 8) & 0xff00) | \
                          ((FIRMWARE_VERSION_MAJOR) & 0x00ff))

/* Set each PIO build by extra_tinyusb.py → include/build_info.h */
#include "build_info.h"

#define PROFICIO_MCU_STM32F411  1

/* Feature gates */
#define PROFICIO_FEAT_USB_VENDOR  1
#define PROFICIO_FEAT_SI5351      1
#define PROFICIO_FEAT_KEYER_I2C   1
#define PROFICIO_FEAT_CW          1
#define PROFICIO_FEAT_BAND        1
#define PROFICIO_FEAT_USB_AUDIO   1
#define PROFICIO_FEAT_PCM3060     1

/* USB audio via TinyUSB UAC2 (see tusb_config.h / usb_audio_uac2.c) */

/* CW keying style: 1 = MKII PIN-diode (key RX per element), 0 = legacy DIN */
#ifndef PROFICIO_CW_MKII
#define PROFICIO_CW_MKII          1
#endif

/* SI5351 I2C 7-bit address */
#define SI5351_I2C_ADDR           0x60
/* PIC keyer I2C 7-bit address (PSoC passed 0x40 to an API that shifts it, same as the codec; was 0x20) */
#define KEYER_I2C_ADDR            0x40

#define KEYER_MEM_Q_SIZE          80
#define CW_DEFAULT_HOLD_TIME      50  /* units × 10 ms ≈ hold (matches PSoC) */

/* Non-QSK amp: amp relays get this long before RF (PSoC MKII 3.234) */
#define KEY_DELAY_DIVIDER         16u /* key delay = TX hold (ms) / 16. TX hold 500 ms -> 31 ms */
#define KEY_DELAY_MIN_MS          20u /* shortest amp relay settle time allowed */
#define KEY_DELAY_MAX_MS          40u
#define AMP_RELEASE_MS            10u /* PTT off: PA off this long before AMP off */

/* CW raised-cosine keying: rise and fall time of the TX I/Q envelope.
 * Must stay well under KEY_DELAY_MIN_MS (the envelope starts half a ramp early). */
#define CW_RAMP_MS                5u
/* Entering CW while transmitting: longest wait for the ramp down and release (3.6) */
#define MODE_DRAIN_MAX_MS         60u

#define TRUE  1
#define FALSE 0

#endif /* PROFICIO_CONFIG_H */
