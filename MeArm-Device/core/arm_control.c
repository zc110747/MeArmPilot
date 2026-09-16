#include "arm_control.h"
#include "bsp/servo.h"
#include "bsp/uart.h"
#include <avr/pgmspace.h>

/* degrees advanced per arm_tick() (loop ~20 ms) -> limits slew speed */
#define RAMP_STEP 3

typedef struct {
    uint8_t      id;     /* user id 6/7/8/9 */
    servo_ch_t   ch;     /* hardware channel */
    uint8_t      target; /* desired angle (HOLD) */
    uint8_t      current;/* angle actually commanded this tick */
    servo_mode_t mode;
    int8_t       dir;    /* sweep direction for MODE_AUTO */
    /* ★ 本次运动是否由**外部手段**引起（摇杆 / 红外）。
       决定"位置变化要不要主动上报给上位机" —— 见文件末尾的上报一节与
       arm_control.h 的说明。命令驱动的运动置 false。 */
    bool         external;
} arm_servo_t;

static arm_servo_t G[SERVO_COUNT];

/* ---- 上报脏标记 ---------------------------------------------------------
   true = 有"外部驱动的位置变化"还没报出去。
   ⚠️ 它只记"有没有"，**不缓存数值** —— 发送那一刻才取 current，
   因此被推迟/合并掉的永远是过期帧，不会积压（latest-wins）。 */
static bool g_report_dirty = false;

static void sync_to_servo(arm_servo_t *s) {
    servo_set_angle(s->ch, s->current);
}

void arm_init(void) {
    /* build table in fixed id order: 9,8,7,6 */
    static const uint8_t ids[SERVO_COUNT] PROGMEM = {9, 8, 7, 6};
    for (uint8_t i = 0; i < SERVO_COUNT; i++) {
        bool ok;
        uint8_t id = pgm_read_byte(&ids[i]);
        servo_ch_t c = servo_id_to_ch(id, &ok);
        G[i].id = id;
        G[i].ch = c;
        G[i].target = 90;
        G[i].current = 90;
        G[i].mode = MODE_HOLD;
        G[i].dir = 1;
        G[i].external = false;   /* 开机位是"命令位"，不主动上报 */
        sync_to_servo(&G[i]);
    }
    g_report_dirty = false;
}

static arm_servo_t *find(uint8_t id) {
    for (uint8_t i = 0; i < SERVO_COUNT; i++)
        if (G[i].id == id) return &G[i];
    return NULL;
}

bool arm_id_valid(uint8_t id) {
    return find(id) != NULL;
}

void arm_tick(void) {
    bool external_change = false;
    for (uint8_t i = 0; i < SERVO_COUNT; i++) {
        arm_servo_t *s = &G[i];
        uint8_t before = s->current;
        if (s->mode == MODE_AUTO) {
            int next = (int)s->current + s->dir * RAMP_STEP;
            if (next >= servo_max_for(s->ch) ) { next = servo_max_for(s->ch); s->dir = -1; }
            else if (next <= servo_min_for(s->ch)) { next = servo_min_for(s->ch); s->dir = 1; }
            s->current = (uint8_t)next;
            s->target = s->current;
        } else { /* MODE_HOLD: ramp toward target */
            if (s->current < s->target) {
                s->current = (uint8_t)(s->current + RAMP_STEP);
                if (s->current > s->target) s->current = s->target;
            } else if (s->current > s->target) {
                s->current = (uint8_t)(s->current - RAMP_STEP);
                if (s->current < s->target) s->current = s->target;
            }
        }
        sync_to_servo(s);
        /* 只对**外部驱动**的变化置脏：命令驱动的运动由 ACK / STATUS 那条路负责，
           再报一份会让上位机把"命令侧"跟着实际位置拖走。 */
        if (s->current != before && s->external) external_change = true;
    }
    if (external_change) g_report_dirty = true;
}

uint8_t arm_set_angle(uint8_t id, uint8_t angle) {
    arm_servo_t *s = find(id);
    if (!s) return 255;
    /* clamp to the servo's forced range */
    uint8_t lo = servo_min_for(s->ch);
    uint8_t hi = servo_max_for(s->ch);
    if (angle < lo) angle = lo;
    if (angle > hi) angle = hi;
    s->target = angle;
    s->mode = MODE_HOLD;
    /* 命令路径：位置变化由 ACK / STATUS 负责，不主动上报 */
    s->external = false;
    return angle;
}

void arm_stop(uint8_t id) {
    arm_servo_t *s = find(id);
    if (!s) return;
    /* ⚠️ 冻结前先把"外部运动到此为止"这件事**报出去**：
       冻结点往往就是摇杆运动的终点，若此刻丢掉脏标记，
       上位机界面会停在最后一个中间位置，看板上留下一个假的姿态。 */
    if (s->external) g_report_dirty = true;
    s->mode = MODE_HOLD;   /* freeze: hold current angle, cancel auto */
    s->target = s->current;
    s->external = false;
}

/* Joystick / IR style single-step nudge: shift the desired angle by `delta`
   degrees, clamped to the servo's forced range.
 *
 * ⚠️ 修任务③-A：原来 `s->target = s->current` 会把正在斜坡中的目标角**就地取消**
 * （gripper 全行程约 800ms，这段时间内只要落进一帧 nudge——IR 按键 4/6 或摇杆
 * A2 偶发漂移——命令就停在半路，表现正是「gripper 有概率不执行」）。
 * 现改为基于 target 增量，且不改写 current：arm_tick() 的斜坡会把 current 平滑
 * 拉向新目标，与 SET 命令的语义一致，斜坡中途的 nudge 只微调终点、不中断运动。 */
void arm_nudge(uint8_t id, int8_t delta) {
    arm_servo_t *s = find(id);
    if (!s || delta == 0) return;
    int v = (int)s->target + delta;
    uint8_t lo = servo_min_for(s->ch);
    uint8_t hi = servo_max_for(s->ch);
    if (v < lo) v = lo;
    if (v > hi) v = hi;
    s->target = (uint8_t)v;
    s->mode   = MODE_HOLD;
    /* ★ 摇杆 / 红外驱动的运动 = **设备侧自主变化** ⇒ 位置变化要主动上报，
       否则上位机界面停在旧值上（"摇杆动了但界面不动"）。 */
    s->external = true;
}

bool arm_auto(uint8_t id) {
    arm_servo_t *s = find(id);
    if (!s) return false;
    s->mode = MODE_AUTO;
    s->dir = (s->current <= (servo_min_for(s->ch) + servo_max_for(s->ch)) / 2) ? 1 : -1;
    /* 自走同样是"设备自己动"⇒ 全程上报（用户需要看到它走到哪了） */
    s->external = true;
    return true;
}

void arm_reset(void) {
    for (uint8_t i = 0; i < SERVO_COUNT; i++) {
        G[i].target = 90;
        G[i].current = 90;
        G[i].mode = MODE_HOLD;
        G[i].dir = 1;
        G[i].external = false;   /* 命令路径 */
        sync_to_servo(&G[i]);
    }
    /* 复位会打断外部运动 ⇒ 之前攒下的脏数据已无意义（它报的是更早的位置，
       而现在的真实位置是 90，由 RESET 自己的 STATUS 应答负责）。 */
    g_report_dirty = false;
}

uint8_t arm_get_angle(uint8_t id) {
    arm_servo_t *s = find(id);
    return s ? s->current : 0;
}

servo_mode_t arm_get_mode(uint8_t id) {
    arm_servo_t *s = find(id);
    return s ? s->mode : MODE_HOLD;
}

bool arm_all_reached(void) {
    for (uint8_t i = 0; i < SERVO_COUNT; i++)
        if (G[i].current != G[i].target) return false;
    return true;
}

void arm_status(void) {
    char m6 = arm_get_mode(6) == MODE_AUTO ? 'A' : 'H';
    char m7 = arm_get_mode(7) == MODE_AUTO ? 'A' : 'H';
    char m8 = arm_get_mode(8) == MODE_AUTO ? 'A' : 'H';
    char m9 = arm_get_mode(9) == MODE_AUTO ? 'A' : 'H';
    uart_printf(PSTR("STATUS S6=%u(%c) S7=%u(%c) S8=%u(%c) S9=%u(%c)\r\n"),
        arm_get_angle(6), m6,
        arm_get_angle(7), m7,
        arm_get_angle(8), m8,
        arm_get_angle(9), m9);
}

/* ---------------------------------------------------------------------------
   设备侧自主变化的上报（低优先级）—— 声明见 arm_control.h
   ---------------------------------------------------------------------------
   ★ 为什么需要：摇杆 / 红外把舵机拨了，**上位机界面必须跟着走**。固件不认识
   关节（标定真值只有上位机那一份），所以只能报**舵机角**，由后端唯一那一处
   换算成关节角。`#` 前缀表示"异步事件"，因此它天然不参与命令-应答门控
   （docs/serial-v1.md §3 的既有约定）。

   ★ 为什么必须"低优先级"：这一行绝不能挤占**命令应答**的 TX 空间。应答被挤掉，
   上位机就认为"机械臂没收到命令"，而那正是历史上最难查的一类故障
   （见 bsp/uart.c 文件头的完整链条）。所以闸门是：

       TX 环**完全空** 才发。

   ⚠️ 被闸门挡下时**保留脏标记**，下一轮再试 —— 值在发送那一刻才重新取，
      所以被合并掉的永远只是过期帧（latest-wins），既不会积压也不会补发旧姿态。

   ⚠️ 调用点必须在 `cmd_poll()` **之后**：顺序本身就是优先级 —— 同一轮里
      命令先受理并应答，遥测只能捡剩下的时机。

   ⚠️ 只在**外部驱动**的运动上置脏（见 arm_tick / arm_nudge / arm_auto）。
      命令驱动的运动不报 —— 那一路有 ACK 与 STATUS，多报一份会让上位机把
      "命令侧"跟着实际位置拖走（拖滑杆时能看到滑杆被回拉）。                      */
void arm_report_tick(void) {
    if (!g_report_dirty) return;
    if (uart_tx_used() != 0) return;   /* 让位于命令应答 / 其它输出 */

    /* 顺序与 arm_status() 一致（S6..S9），便于串口日志直接对照 */
    uart_printf(PSTR("# SERVO S6=%u S7=%u S8=%u S9=%u\r\n"),
        arm_get_angle(6), arm_get_angle(7), arm_get_angle(8), arm_get_angle(9));
    g_report_dirty = false;
}
