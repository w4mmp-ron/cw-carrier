/**
 * Main loop timing (3.20, evidence only): how long each part of app_poll()
 * takes, so a slow pass (e.g. 155 ms at a band change) names its cause.
 * Read with CMD_AUDIO_STATS wIndex 3.
 */
#ifndef LOOP_PROF_H
#define LOOP_PROF_H

#include <stdint.h>

/* Parts of app_poll(), in order */
enum {
    PROF_CAL = 0,     /* cw_hold_poll + cal_poll */
    PROF_USB,         /* usb_device_poll: tud_task, control requests */
    PROF_AUDIO,       /* audio start / stop + Audio_Main */
    PROF_LED,         /* diag_led_poll */
    PROF_BEAT0,       /* beater case 0..6 = PROF_BEAT0 + case */
    PROF_TAIL = PROF_BEAT0 + 7, /* CW mode block after the beater */
    PROF_PARTS
};

#define LOOP_PROF_STATS_WORDS (4u + PROF_PARTS)

void     loop_prof_init(void);
/** Start of an app_poll() pass. */
void     loop_prof_begin(void);
/** End of one part: time since the last mark goes to that part. */
void     loop_prof_mark(uint8_t part);
/** End of the pass. */
void     loop_prof_end(void);
/** Fills buf with LOOP_PROF_STATS_WORDS little-endian uint32, returns bytes.
 *  clear = 1 restarts the max values after the read. */
uint16_t loop_prof_stats(uint8_t *buf, uint16_t cap, uint8_t clear);

#endif /* LOOP_PROF_H */
