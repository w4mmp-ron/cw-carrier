/**
 * 3.16: CMD_KEYER_BOOT 0x9F, I2C pass-through to the PIC keyer (see keyer_boot.h).
 * USB callbacks only store the request; keyer_boot_poll() in the main loop does the I2C.
 */
#include "keyer_boot.h"
#include "proficio_config.h"
#include "i2c_bus.h"
#include <string.h>

static uint8_t  s_wbuf[KEYER_BOOT_MAX_WR];
static uint8_t  s_rbuf[KEYER_BOOT_MAX_RD];
static uint8_t  s_addr;
static uint8_t  s_wlen;
static uint8_t  s_rlen;
static uint8_t  s_rgot;
static volatile uint8_t s_state = KB_IDLE;

uint8_t *keyer_boot_setup(uint16_t wValue, uint16_t wIndex, uint16_t wLength)
{
    if (s_state == KB_PENDING) {
        return 0;
    }
    if ((wValue != KEYER_I2C_ADDR && wValue != KEYER_BOOT_ADDR)
        || wIndex > KEYER_BOOT_MAX_RD || wLength > KEYER_BOOT_MAX_WR) {
        s_state = KB_REFUSED;
        return 0;
    }
    s_addr = (uint8_t)wValue;
    s_rlen = (uint8_t)wIndex;
    s_wlen = 0;
    s_rgot = 0;
    if (wLength == 0u) {
        s_state = KB_PENDING;
    } else {
        s_state = KB_IDLE;      /* queued when the data stage is in */
    }
    return s_wbuf;
}

void keyer_boot_out_done(uint16_t len)
{
    if (len > KEYER_BOOT_MAX_WR) {
        len = KEYER_BOOT_MAX_WR;
    }
    s_wlen = (uint8_t)len;
    s_state = KB_PENDING;
}

uint16_t keyer_boot_result(uint8_t *buf, uint16_t cap)
{
    uint16_t n = 0;
    uint8_t st = s_state;

    if (cap < 2u) {
        return 0;
    }
    buf[0] = st;
    buf[1] = (st == KB_OK) ? s_rgot : 0u;
    n = 2u;
    if (st == KB_OK && s_rgot) {
        if ((uint16_t)(n + s_rgot) > cap) {
            return n;
        }
        memcpy(&buf[2], s_rbuf, s_rgot);
        n = (uint16_t)(n + s_rgot);
    }
    return n;
}

void keyer_boot_poll(void)
{
    uint8_t ok = 1;

    if (s_state != KB_PENDING) {
        return;
    }
    if (s_wlen == 0u && s_rlen == 0u) {
        ok = i2c_ready(s_addr);
    } else {
        if (s_wlen) {
            ok = i2c_write_buf(s_addr, s_wbuf, s_wlen);
        }
        if (ok && s_rlen) {
            ok = i2c_read_buf(s_addr, s_rbuf, s_rlen);
            if (ok) {
                s_rgot = s_rlen;
            }
        }
    }
    s_state = ok ? KB_OK : KB_FAIL;
}
