extern "C" {
#include "bsp/uart.h"
#include "bsp/servo.h"
#include "bsp/led.h"
#include "bsp/systick.h"
#include "core/arm_control.h"
#include "core/cmd.h"
#include "core/joystick.h"
#include "core/ir_ctrl.h"
#include "core/ir_seq.h"
}

#include <avr/interrupt.h>

/* The loop no longer busy-waits: it spins freely and derives all timing from
   the 1 ms systick (bsp/systick). Sub-systems that need a cadence check the
   tick themselves:
     - arm_tick()  : every 20 ms (servo ramp slew, same speed as before)
     - joystick    : every 20 ms (ADC scan)
     - ir_seq_tick : every loop iteration (reacts to stop/switch immediately)
     - LED heart   : toggles every 500 ms                                   */

#define SLOW_MS    30   /* arm_tick + joystick cadence */
#define LED_MS   500    /* heartbeat period */
/* 设备侧自主变化（摇杆 / 红外）的上报周期。
   ★ 刻意**低于** arm_tick 的 33 Hz：遥测只需要让人眼看得出"界面在跟着走"，
   没必要把每一小步都送出去。真正的优先级由 arm_report_tick() 内部的
   "TX 环为空才发"保证（见 core/arm_control.c）。 */
#define REPORT_MS 60

int main(void) {
    uart_init(115200);        /* COM4 @ 115200 8N1, bidirectional */
    servo_init();             /* Timer1 4-servo scheduler, all -> 90 */
    led_init();               /* onboard LED heartbeat, start ON */
    systick_init();           /* Timer2 1 ms tick */
    joystick_init();          /* ADC + hardware joystick scan (enabled) */
    ir_ctrl_init();           /* NEC IR receiver on PD2 (enabled) + seq engine */
    arm_init();               /* app model reset to 90, MODE_HOLD */
    sei();                    /* enable global interrupts (servo/IR/timer) */

    uart_puts(PSTR("\r\n[meArm] bare-metal AVR ready, servos reset to 90\r\n"));
    uart_puts(PSTR("        type HELP for commands\r\n"));
    arm_status();

    uint32_t last_slow   = 0;
    uint32_t last_led    = 0;
    uint32_t last_report = 0;
    for (;;) {
        /* fast path: never blocks, so IR/serial/sequence events are handled
           with minimal latency */
        cmd_poll();          /* process incoming serial commands */
        ir_ctrl_poll();      /* hardware IR remote -> nudge / seq / stop */
        ir_seq_tick();       /* drive running action set (tick-based) */

        uint32_t now = systick_ms();
        if (now - last_slow >= SLOW_MS) {
            last_slow = now;
            arm_tick();      /* ramp / auto-sweep */
            joystick_scan(); /* hardware joystick -> arm_nudge (if enabled) */
        }
        /* ★ 设备侧自主变化的上报（摇杆 / 红外）：必须在 cmd_poll() 之后 ——
           顺序本身就是优先级。命令先受理并应答，遥测只能捡 TX 空闲的时机
           （arm_report_tick 会自行检查 TX 环是否为空）。 */
        if (now - last_report >= REPORT_MS) {
            last_report = now;
            arm_report_tick();
        }
        if (now - last_led >= LED_MS) {
            last_led = now;
            led_toggle();
        }
    }
    return 0;
}
