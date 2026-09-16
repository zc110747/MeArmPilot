#include "joystick.h"

#include <avr/pgmspace.h>
#include "bsp/adc.h"
#include "core/arm_control.h"

/* ADC channel per axis (Arduino A0..A3 -> PC0..PC3) and the servo it drives.
   Matches the reference sketch: A0=base(9) A1=left(8) A2=grip(6) A3=right(7).
   Held in flash (PROGMEM) to keep the 2 KB RAM free on the 328P. */
static const uint8_t J_CH[4] PROGMEM = {0, 1, 2, 3};
static const uint8_t J_ID[4] PROGMEM = {9, 8, 6, 7};

static bool g_enabled = true;
static bool g_moved   = false;   /* set when an axis actually moves (edge) */

void joystick_init(void) {
    adc_init();
    g_enabled = true;
}

void joystick_set_enabled(bool on) {
    g_enabled = on;
    if (!on) g_moved = false;   /* drop any pending edge so it can't leak into a seq */
}
bool joystick_is_enabled(void)     { return g_enabled; }

/* Per-axis step. The DIRECTION sense matches the original Arduino sketch
   exactly at the 200/800 thresholds:
     normal axes (9/6/7): raw<200 -> +1, raw>800 -> -1
     left (8)           : raw<200 -> -1, raw>800 -> +1   (inverted)
   BUT a fixed +/-1 per 20 ms scan is far too slow here (the Arduino loop ran
   thousands of times/sec; ours is gated to 20 ms, so +/-1 -> only ~50 deg/s and
   it felt "stuck"). So the step size is now PROPORTIONAL to how far past the
   threshold the stick is pushed: min 2, max 10 deg per 20 ms scan. A light tap
   creeps, a hard push sprints, while the direction edges stay identical. */
int8_t joystick_delta(uint8_t id, int raw) {
    if (raw < 0) raw = 0;
    if (raw > 1023) raw = 1023;

    bool past_hi = raw > 800;   /* beyond upper threshold */
    bool past_lo = raw < 200;   /* beyond lower threshold */
    if (!past_hi && !past_lo) return 0;

    int beyond = past_hi ? (raw - 800) : (200 - raw);  /* 0..~223 / 0..200 */
    int step = 2 + beyond / 30;                          /* ~2..9 */
    if (step > 3) step = 3;

    /* direction: normal axes -> +1 when raw<200 ; left(8) inverted -> +1 when raw>800 */
    bool positive = (id == 8) ? past_hi : past_lo;
    return positive ? (int8_t)step : (int8_t)(-step);
}

/* Called from the main loop (like the original loop() calling
   handleJoystickControl() every iteration). A centred stick (200..800) yields
   delta 0 -> no movement; the forced servo range in arm_nudge() enforces the
   original 30..150 / 20..100 / 40..130 / 80..160 limits. */
void joystick_scan(void) {
    if (!g_enabled) return;
    for (uint8_t i = 0; i < 4; i++) {
        int raw = (int)adc_read(pgm_read_byte(&J_CH[i]));
        uint8_t id = pgm_read_byte(&J_ID[i]);
        int8_t d = joystick_delta(id, raw);
        if (d) { arm_nudge(id, d); g_moved = true; }
    }
}

bool joystick_consume_input(void) {
    bool m = g_moved;
    g_moved = false;
    return m;
}
