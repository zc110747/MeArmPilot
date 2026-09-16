#ifndef BSP_UART_H
#define BSP_UART_H

#include <stdint.h>
#include <stddef.h>
#include <avr/pgmspace.h>

#ifdef __cplusplus
extern "C" {
#endif

/* USART0 @ 115200 8N1 (U2X double speed; ATmega328P / Arduino Uno,
   wired to USB-serial -> PC COM4). Keep in sync with uart_init() in main.cpp.
   RX uses interrupt ring buffer; TX uses interrupt ring buffer so printing
   never blocks the servo ISR / main loop.

   IMPORTANT (AVR RAM budget): every string literal MUST live in flash, e.g.
       uart_puts(PSTR("hello"));
       uart_printf(PSTR("v=%u\r\n"), v);
   avr-gcc copies plain "..." literals into .data (RAM) at boot; on the 328P's
   2 KB that overflowed the stack and reset the MCU under command load. The
   _P variants below read the format/literal straight from flash (LPM).        */

void uart_init(uint32_t baud);

/* blocking-until-buffered send (returns after byte is queued, not after wire) */
void uart_putc(char c);
void uart_puts(PGM_P s);                 /* s MUST point into flash (PSTR) */

/* printf-like; format string MUST be in flash (PSTR). Uses avr-libc vsnprintf_P:
   %s reads a RAM string, %S (uppercase) reads a flash string. */
int uart_printf(PGM_P fmt, ...);

/* non-blocking RX: returns byte (0..255) or -1 when the ring is empty */
int uart_getc_nowait(void);

/* ---- 链路丢弃计数（修任务④-②）-----------------------------------------
   ★ 为什么必须有这个：历史上「gripper 偶发不执行」之所以难查，是因为
   RX/TX 环满时的丢弃**完全静默** —— 症状只表现为"偶发"，没有任何读数。
   现在两处丢弃都计数，`STATS` 命令可读出：
     rx_drop > 0  ⇒ 收到的字节被丢过 ⇒ 某条指令可能残缺 ⇒ 对应 ACK 不会来
     tx_drop > 0  ⇒ 发出的字符被丢过 ⇒ 回执可能缺字（极少见，仅爆发输出时）
   ⚠️ 计数是**饱和的 uint8**（255 封顶），只用于"有没有发生过"，
      不适合当精确计量。压测时读一次、清一次即可。
   ⚠️ 读的时候字节可能只被读走一部分（ISR 还在跑），这是**可接受的近似**：
      我们只关心"是否为 0"。                                                    */
uint8_t uart_rx_drop_count(void);
uint8_t uart_tx_drop_count(void);
void uart_clear_drop_counters(void);

/* ---- TX 环水位（"低优先级"上报的闸门）-----------------------------------
   ★ 为什么需要：固件要在一台舵机被**外部手段**（硬件摇杆 / 红外遥控）改动时
   主动上报遥测。它绝不能挤占**命令应答**的 TX 空间 —— 应答被挤掉，上位机
   就认为"机械臂没收到命令"，而这正是历史上最难查的一类故障（见 uart.c 文件
   头的完整链条与 STATS 的由来）。

   所以上报前先问一句"TX 环空不空"：空才发，不空就留到下一轮。被推迟的帧
   在下一轮**重新取值**，所以合并掉的都只是过期帧（latest-wins）。

   ⚠️ 读的时候 ISR 可能正在推进 tx_tail ⇒ 结果是**保守**的（可能读到偏大的值），
      只会让上报更晚发出，绝不会让它在忙时抢跑。方向是安全的。
   ⚠️ 返回值是"环里还有多少字节没发出去"，不是剩余空间。                      */
uint8_t uart_tx_used(void);

#ifdef __cplusplus
}
#endif

#endif /* BSP_UART_H */
