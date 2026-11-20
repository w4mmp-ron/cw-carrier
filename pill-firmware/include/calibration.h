/**
 * Per-band calibration stored in the radio (RX IQ, TX IQ, power).
 * The pill only stores the values; the server applies them.
 *
 * USB: CMD_CALIBRATION (usbvend.h). wValue high byte = type, low byte = band.
 *   Write (host to pill) sets a value; the pill writes it to flash itself
 *   (build 10, as the PSoC MKII 3.238), read returns it.
 *   CAL_TYPE_SAVE: read only (a write stalls since build 10): 2 bytes =
 *   [write status (CAL_SAVE_*), 1 = values loaded from flash at boot].
 *   CAL_TYPE_VALID: read only, 6 bytes = uint16 masks [RX IQ, TX IQ, power],
 *   bit N = band N has been set (calibrated). Band byte ignored.
 *   Type 6 (clear) was removed in build 9 (stalls), as in the PSoC MKII 3.239.
 * Payloads are little endian:
 *   RX IQ / TX IQ: int16 gain, int16 phase (4 bytes)
 *   Power:         uint16 (2 bytes)
 *   QRO power:     int16 (2 bytes, 3.24); its valid mask = type 9 (uint16, read only)
 * Band index = MSCC band order (0 = 10 m ... 9 = 160 m, 10 = 630 m, 11 = 2200 m).
 * Values are 0 until set; 0 can be a real value, so the server checks
 * CAL_TYPE_VALID and uses its own default for a band that is not set.
 */
#ifndef CALIBRATION_H
#define CALIBRATION_H

#include <stdint.h>

#define CAL_BANDS       12u

#define CAL_TYPE_RX_IQ  1u
#define CAL_TYPE_TX_IQ  2u
#define CAL_TYPE_POWER  3u
#define CAL_TYPE_SAVE   4u
#define CAL_TYPE_VALID  5u
/* Flash only, not reachable with CMD_CALIBRATION (3.9): Si5351 PPM correction,
 * band 0, 2 bytes = int8 ppm_int, int8 ppm_dec (as the PSoC kept in EEPROM). */
#define CAL_TYPE_PPM    7u
/* 3.24 (2026-11-19): QRO (amplifier) power cal per band, int16 (2 bytes), as
 * amplifier_cal.ini (-99 = not calibrated there; here a value like any other).
 * Its valid mask is a read of its own (type 9, read only, uint16), so the type 5
 * read stays 6 bytes for older ms-sdr builds. */
#define CAL_TYPE_QRO       8u
#define CAL_TYPE_QRO_VALID 9u

#define CAL_SAVE_IDLE    0u  /* nothing written since boot */
#define CAL_SAVE_BUSY    1u  /* set, not in flash yet */
#define CAL_SAVE_OK      2u
#define CAL_SAVE_FAILED  3u  /* program / verify error, or flash log full */
#define CAL_SAVE_REFUSED 4u  /* not used since build 10 (writes wait for RX) */

void    cal_init(void);    /* boot: load the values from flash, else 0 / not valid */
void    cal_poll(void);    /* main loop: writes set values to flash */

/* USB: bytes for a read (0 = bad type / band), set from a write (0 = bad) */
uint16_t cal_get(uint8_t type, uint8_t band, uint8_t *dst, uint16_t cap);
uint8_t  cal_set(uint8_t type, uint8_t band, const uint8_t *src, uint16_t len);
uint16_t cal_len(uint8_t type);   /* payload length of a type, 0 = unknown */

/* PPM correction (3.9): stored value, 1 = one was stored; set = keep + write to flash */
uint8_t cal_ppm_get(int8_t *ppm_int, int8_t *ppm_dec);
void    cal_ppm_set(int8_t ppm_int, int8_t ppm_dec);

#endif /* CALIBRATION_H */
