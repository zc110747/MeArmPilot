#ifndef CORE_ARM_CONTROL_H
#define CORE_ARM_CONTROL_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Per-servo runtime model:
     MODE_HOLD : drive toward `target` (smooth ramp), then hold
     MODE_AUTO : joystick-like self-sweep (triangle wave) between min..max
   "STOP <id>" simply switches a servo back to HOLD (freezes current angle). */

typedef enum { MODE_HOLD = 0, MODE_AUTO = 1 } servo_mode_t;

void arm_init(void);                 /* reset all to 90 deg, MODE_HOLD */
void arm_tick(void);                 /* call periodically (~20 ms)    */

/* ---- 设备侧自主变化的上报（低优先级）-----------------------------------
   ★ 解决的问题：摇杆 / 红外把舵机拨动之后，**上位机界面必须跟着走**。
   固件不认识关节（标定真值只在上位机那一份），所以只能报**舵机角**，
   由后端唯一那一处换算成关节角。

   两条设计约束（都不可省）：
     ① 只在**外部驱动**的运动上上报（nudge / auto）；
        命令驱动的（SET / RESET）不上报 —— 那一路自有 ACK 与 STATUS 负责，
        多报一份会让上位机把"命令侧"跟着实际位置拖走。
     ② **优先级低于命令应答**：TX 环非空就让位（见 arm_report_tick）。

   调用位置：主循环里 **cmd_poll() 之后**（顺序即优先级）。
   ⚠️ 不要在中断里调用：uart_printf 会占用较长的执行时间。                    */
void arm_report_tick(void);

uint8_t arm_set_angle(uint8_t id, uint8_t angle); /* returns applied (clamped) angle, 255 if bad id */
void arm_nudge(uint8_t id, int8_t delta); /* step current angle by delta (clamped, no ramp) */
void arm_stop(uint8_t id);           /* freeze auto-sweep, hold angle          */
bool arm_auto(uint8_t id);           /* start self-sweep; false if bad id      */
void arm_reset(void);                /* all servos -> 90, MODE_HOLD            */

uint8_t arm_get_angle(uint8_t id);   /* 0 if bad id */
servo_mode_t arm_get_mode(uint8_t id);
bool arm_id_valid(uint8_t id);

/* true when every servo has reached its target (no ramp currently in flight).
   Used by the IR sequence engine to confirm a move settled before the next
   step's hold gap elapses. */
bool arm_all_reached(void);

void arm_status(void);               /* print "S6=.. S7=.. S8=.. S9=.." + modes */

#ifdef __cplusplus
}
#endif

#endif /* CORE_ARM_CONTROL_H */
