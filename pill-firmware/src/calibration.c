/**
 * Per-band calibration: values live in RAM, each set is written to flash sector 7.
 *
 * Build 10 (2026-10-23): no save command, as the PSoC MKII 3.238 ("the PPM way"): a value
 * set from the host is written by the pill itself. ms-sdr 3.179 no longer sends a save, so
 * build 9 kept the values in RAM only.
 *
 * Flash: sector 7 (0x08060000, 128 KB) is a log of 16-byte entries, appended in order.
 * The linker script stops the app before it (ld/STM32F411CEUX_APP_0x8000.ld). An entry is
 * a few word writes (well under 1 ms), no erase, so no audio gap. cal_poll() writes one
 * dirty value per pass, CAL_QUIET_MS after the last set, never while transmitting.
 * Boot: entries are read in order, the last good one for each type / band wins; a bad
 * entry (power lost while writing) is skipped. When the log is 3/4 full, or holds the
 * old build 7-9 record, the sector is erased (about 1-2 s, before USB starts) and the
 * current values are written back. Log full while running: writes stop until the next
 * boot (status FAILED), values stay in RAM.
 */
#include "calibration.h"
#include "radio_state.h"
#include "board.h"
#include "proficio_config.h"
#include "stm32f4xx_hal.h"
#include <stddef.h>
#include <string.h>

#define CAL_FLASH_SECTOR  FLASH_SECTOR_7
#define CAL_FLASH_ADDR    0x08060000u
#define CAL_FLASH_SIZE    0x20000u
#define CAL_OLD_MAGIC     0x4C414350u   /* "PCAL": build 7-9 single record */
#define CAL_ENTRY_TAG     0xC5u
#define CAL_ENTRY_WORDS   4u
#define CAL_SLOTS         (CAL_FLASH_SIZE / (CAL_ENTRY_WORDS * 4u))
#define CAL_COMPACT_AT    ((CAL_SLOTS * 3u) / 4u)
#define CAL_QUIET_MS      50u
#define CAL_TYPES         4u            /* RX IQ, TX IQ, power, QRO (3.24) */
#define CAL_QRO_INDEX     3u            /* s_valid / s_dirty index of CAL_TYPE_QRO */

typedef struct {
    int16_t gain;
    int16_t phase;
} cal_iq_t;

/* flash entry: [tag, type, band, 0] [value, 4 bytes] [CRC-32 of the first 8 bytes] [0] */
typedef struct {
    uint8_t  tag;
    uint8_t  type;
    uint8_t  band;
    uint8_t  zero;
    uint8_t  value[4];
    uint32_t crc;
    uint32_t pad;
} cal_entry_t;

_Static_assert(sizeof(cal_entry_t) == CAL_ENTRY_WORDS * 4u, "cal_entry_t size");

static cal_iq_t s_rx_iq[CAL_BANDS];
static cal_iq_t s_tx_iq[CAL_BANDS];
static uint16_t s_power[CAL_BANDS];
static int16_t  s_qro[CAL_BANDS];               /* 3.24: QRO (amplifier) power */
static uint16_t s_valid[CAL_TYPES];             /* bit N = band N set: [RX IQ, TX IQ, power, QRO] */
static volatile uint16_t s_dirty[CAL_TYPES];    /* set, not yet in flash */
static volatile uint32_t s_last_set_ms = 0;
static int8_t s_ppm[2];                         /* CAL_TYPE_PPM: ppm_int, ppm_dec (3.9) */
static uint8_t s_ppm_valid = 0;
static volatile uint8_t s_ppm_dirty = 0;
static uint32_t s_next_slot = 0;                /* first free entry slot */
static uint8_t s_save_status = CAL_SAVE_IDLE;
static uint8_t s_loaded = 0;

static uint32_t cal_crc32(const uint8_t *p, uint32_t n)
{
    uint32_t crc = 0xFFFFFFFFu;
    uint8_t b;

    while (n--) {
        crc ^= *p++;
        for (b = 0; b < 8u; b++) {
            crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
        }
    }
    return ~crc;
}

/* s_valid / s_dirty index of a per-band type, CAL_TYPES = not one */
static uint8_t cal_index(uint8_t type)
{
    if (type >= CAL_TYPE_RX_IQ && type <= CAL_TYPE_POWER) {
        return (uint8_t)(type - CAL_TYPE_RX_IQ);
    }
    if (type == CAL_TYPE_QRO) {
        return CAL_QRO_INDEX;
    }
    return CAL_TYPES;
}

/* per-band type of an s_valid / s_dirty index */
static uint8_t cal_type_of(uint8_t index)
{
    return (index == CAL_QRO_INDEX) ? CAL_TYPE_QRO : (uint8_t)(index + CAL_TYPE_RX_IQ);
}

static uint16_t cal_any_dirty(void)
{
    return (uint16_t)(s_dirty[0] | s_dirty[1] | s_dirty[2] | s_dirty[3]);
}

static const cal_entry_t *cal_slot(uint32_t i)
{
    return (const cal_entry_t *)(CAL_FLASH_ADDR + i * sizeof(cal_entry_t));
}

static uint8_t cal_slot_erased(uint32_t i)
{
    const uint32_t *w = (const uint32_t *)cal_slot(i);
    uint32_t k;

    for (k = 0; k < CAL_ENTRY_WORDS; k++) {
        if (w[k] != 0xFFFFFFFFu) {
            return 0u;
        }
    }
    return 1u;
}

/* RAM copy of one value: pointer + length, 0 = bad type */
static uint8_t *cal_value_ptr(uint8_t type, uint8_t band, uint16_t *len)
{
    switch (type) {
    case CAL_TYPE_RX_IQ:
        *len = (uint16_t)sizeof(cal_iq_t);
        return (uint8_t *)&s_rx_iq[band];
    case CAL_TYPE_TX_IQ:
        *len = (uint16_t)sizeof(cal_iq_t);
        return (uint8_t *)&s_tx_iq[band];
    case CAL_TYPE_POWER:
        *len = 2u;
        return (uint8_t *)&s_power[band];
    case CAL_TYPE_QRO:
        *len = 2u;
        return (uint8_t *)&s_qro[band];
    case CAL_TYPE_PPM:
        *len = (uint16_t)sizeof(s_ppm);
        return (band == 0u) ? (uint8_t *)s_ppm : 0;
    default:
        *len = 0u;
        return 0;
    }
}

static void cal_flash_cache_reset(void)
{
    __HAL_FLASH_DATA_CACHE_DISABLE();
    __HAL_FLASH_DATA_CACHE_RESET();
    __HAL_FLASH_DATA_CACHE_ENABLE();
}

static void cal_flash_unlock(void)
{
    HAL_FLASH_Unlock();
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
                           FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
}

/* append one entry at s_next_slot; 1 = written and verified */
static uint8_t cal_append(uint8_t type, uint8_t band)
{
    cal_entry_t e;
    const uint32_t *w = (const uint32_t *)&e;
    uint32_t addr, k;
    uint16_t len;
    const uint8_t *v = cal_value_ptr(type, band, &len);
    uint8_t ok = 1;

    if (v == 0 || s_next_slot >= CAL_SLOTS) {
        return 0u;
    }
    memset(&e, 0, sizeof(e));
    e.tag = CAL_ENTRY_TAG;
    e.type = type;
    e.band = band;
    memcpy(e.value, v, len);
    e.crc = cal_crc32((const uint8_t *)&e, (uint32_t)offsetof(cal_entry_t, crc));

    addr = (uint32_t)cal_slot(s_next_slot);
    s_next_slot++;          /* a failed slot is not reused */
    cal_flash_unlock();
    for (k = 0; ok && k < CAL_ENTRY_WORDS; k++) {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr + k * 4u, w[k]) != HAL_OK) {
            ok = 0;
        }
    }
    HAL_FLASH_Lock();
    cal_flash_cache_reset();
    if (ok && memcmp((const void *)addr, &e, sizeof(e)) != 0) {
        ok = 0;
    }
    return ok;
}

/* boot only: erase the sector and write back every valid value */
static void cal_compact(void)
{
    FLASH_EraseInitTypeDef er;
    uint32_t sector_error = 0;
    uint8_t t, b;

    cal_flash_unlock();
    er.TypeErase = FLASH_TYPEERASE_SECTORS;
    er.Sector = CAL_FLASH_SECTOR;
    er.NbSectors = 1;
    er.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    (void)HAL_FLASHEx_Erase(&er, &sector_error);
    HAL_FLASH_Lock();
    cal_flash_cache_reset();

    s_next_slot = 0;
    for (t = 0; t < CAL_TYPES; t++) {
        for (b = 0; b < CAL_BANDS; b++) {
            if (s_valid[t] & (uint16_t)(1u << b)) {
                (void)cal_append(cal_type_of(t), b);
            }
        }
    }
    if (s_ppm_valid) {
        (void)cal_append(CAL_TYPE_PPM, 0u);
    }
}

void cal_init(void)
{
    const cal_entry_t *e;
    uint32_t i, last_used = 0;
    uint16_t len;
    uint8_t *v;
    uint8_t any = 0, old = (*(const uint32_t *)CAL_FLASH_ADDR == CAL_OLD_MAGIC);

    memset(s_rx_iq, 0, sizeof(s_rx_iq));
    memset(s_tx_iq, 0, sizeof(s_tx_iq));
    memset(s_power, 0, sizeof(s_power));
    memset(s_qro, 0, sizeof(s_qro));
    memset(s_valid, 0, sizeof(s_valid));
    memset(s_ppm, 0, sizeof(s_ppm));
    s_ppm_valid = 0;

    for (i = 0; !old && i < CAL_SLOTS; i++) {
        if (cal_slot_erased(i)) {
            continue;
        }
        last_used = i + 1u;
        e = cal_slot(i);
        if (e->tag != CAL_ENTRY_TAG || e->band >= CAL_BANDS ||
            e->crc != cal_crc32((const uint8_t *)e, (uint32_t)offsetof(cal_entry_t, crc))) {
            continue;
        }
        v = cal_value_ptr(e->type, e->band, &len);
        if (v == 0) {
            continue;
        }
        memcpy(v, e->value, len);
        if (e->type == CAL_TYPE_PPM) {
            s_ppm_valid = 1;
            continue;
        }
        s_valid[cal_index(e->type)] |= (uint16_t)(1u << e->band);
        any = 1;
    }
    s_next_slot = last_used;
    s_loaded = any;

    if (old || s_next_slot >= CAL_COMPACT_AT) {
        cal_compact();
    }
}

uint16_t cal_len(uint8_t type)
{
    switch (type) {
    case CAL_TYPE_RX_IQ:
    case CAL_TYPE_TX_IQ:
        return (uint16_t)sizeof(cal_iq_t);
    case CAL_TYPE_POWER:
    case CAL_TYPE_QRO:
        return 2u;
    case CAL_TYPE_SAVE:
        return 2u;      /* read only: status + loaded */
    case CAL_TYPE_VALID:
        return 6u;      /* read only: [RX IQ, TX IQ, power], kept at 6 bytes (3.24) */
    case CAL_TYPE_QRO_VALID:
        return 2u;      /* read only: QRO mask (3.24) */
    default:
        return 0u;
    }
}

uint16_t cal_get(uint8_t type, uint8_t band, uint8_t *dst, uint16_t cap)
{
    uint16_t n = cal_len(type);
    uint16_t len;
    const uint8_t *v;

    if (n == 0u || n > cap) {
        return 0u;
    }
    if (type == CAL_TYPE_SAVE) {
        dst[0] = s_save_status;
        dst[1] = s_loaded;
        return n;
    }
    if (type == CAL_TYPE_VALID) {
        memcpy(dst, s_valid, n);                    /* first three masks */
        return n;
    }
    if (type == CAL_TYPE_QRO_VALID) {
        memcpy(dst, &s_valid[CAL_QRO_INDEX], n);
        return n;
    }
    if (band >= CAL_BANDS) {
        return 0u;
    }
    v = cal_value_ptr(type, band, &len);
    memcpy(dst, v, n);
    return n;
}

uint8_t cal_set(uint8_t type, uint8_t band, const uint8_t *src, uint16_t len)
{
    uint16_t n;
    uint8_t *v;
    uint8_t idx = cal_index(type);

    if (band >= CAL_BANDS || len != cal_len(type) || idx >= CAL_TYPES) {
        return 0u;
    }
    v = cal_value_ptr(type, band, &n);
    if (v == 0) {
        return 0u;
    }
    memcpy(v, src, len);
    s_valid[idx] |= (uint16_t)(1u << band);
    s_dirty[idx] |= (uint16_t)(1u << band);
    s_last_set_ms = board_millis();
    s_save_status = CAL_SAVE_BUSY;
    return 1u;
}

/* main loop: writes one dirty value per pass, after CAL_QUIET_MS, never in TX */
void cal_poll(void)
{
    uint8_t t, b;

    if ((cal_any_dirty() | s_ppm_dirty) == 0u) {
        return;
    }
    if ((uint32_t)(board_millis() - s_last_set_ms) < CAL_QUIET_MS) {
        return;
    }
    if (TX_Request || E_PTT || E_key_down) {
        return;     /* stays dirty, written after TX */
    }
    if (s_ppm_dirty) {
        s_ppm_dirty = 0;
        if (!cal_append(CAL_TYPE_PPM, 0u)) {
            s_save_status = CAL_SAVE_FAILED;
        } else if (cal_any_dirty() == 0u) {
            s_save_status = CAL_SAVE_OK;
        }
        return;
    }
    for (t = 0; t < CAL_TYPES; t++) {
        for (b = 0; b < CAL_BANDS; b++) {
            if (s_dirty[t] & (uint16_t)(1u << b)) {
                s_dirty[t] &= (uint16_t)~(1u << b);
                if (cal_append(cal_type_of(t), b)) {
                    if (cal_any_dirty() == 0u) {
                        s_save_status = CAL_SAVE_OK;
                    }
                } else {
                    s_save_status = CAL_SAVE_FAILED;
                    if (s_next_slot >= CAL_SLOTS) {
                        memset((void *)s_dirty, 0, sizeof(s_dirty));   /* full: until next boot */
                    }
                }
                return;
            }
        }
    }
}

uint8_t cal_ppm_get(int8_t *ppm_int, int8_t *ppm_dec)
{
    *ppm_int = s_ppm[0];
    *ppm_dec = s_ppm[1];
    return s_ppm_valid;
}

/* main loop: keep the PPM sent by the host and write it to flash (as the PSoC's
 * Set_PPM to EEPROM); cal_poll writes it, never while transmitting */
void cal_ppm_set(int8_t ppm_int, int8_t ppm_dec)
{
    if (s_ppm_valid && s_ppm[0] == ppm_int && s_ppm[1] == ppm_dec) {
        return;     /* unchanged: no flash write */
    }
    s_ppm[0] = ppm_int;
    s_ppm[1] = ppm_dec;
    s_ppm_valid = 1;
    s_ppm_dirty = 1;
    s_last_set_ms = board_millis();
    s_save_status = CAL_SAVE_BUSY;
}
