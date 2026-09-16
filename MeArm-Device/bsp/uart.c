#include "uart.h"

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <stdarg.h>
#include <string.h>
#include <stdio.h> /* vsnprintf_P */

/* ---- ring buffers --------------------------------------------------------
   ⚠️ 修任务③：RX 缓冲曾为 64B。后端每条 JR 拆成 2 条 SET（高频拖动时 ~33ms 一帧），
   舵机大电流负载下主循环偶发繁忙，64B 会被写满而**静默丢字节** —— 这会让第一条
   SET 解析错乱、OK 不来，进而 gripper 的第二条 SET 被牺牲（「有概率不执行」）。
   放大到 256B（ATmega328P 有 2KB RAM，足够），单帧命令(~25B)可缓冲十余条。

   ★★ 修任务④（2026-09-15，真机实测复现）——**上一轮只扩缓冲，没修掉根因**：
   实测后端报 `ERR 固件未应答 SET（>600ms）（SET 6 76）`（SET 6 = gripper）。
   真实链条是一条**并发路径**：
     ① 主循环发 ACK 走 uart_putc()，内里 `while (tx_full())` **忙等**；
     ② 忙等期间 cmd_poll() 不执行 ⇒ 停止读 RX 环；
     ③ 但 RXCIE0 中断照常触发、照常往环里写 ⇒ 环满即**静默丢字节**；
     ④ 丢的若是 `SET 6` 的字节 ⇒ 该行残缺 ⇒ process_line 解析失败 ⇒ **ACK 永不到达**；
     ⑤ 后端 600ms 超时 ⇒ 表现为「gripper 偶发不执行」。
   扩到 256B 只是把「多久会满」推后，**没有消除这条路径**；而且丢弃点**零信号**，
   所以症状是「偶发且难复现」。
   放大 TX 环（128→256）让多数输出瞬间入队、忙等窗口≈0；并且**环满时不再静默**，
   计入 tx_drop（可用 STATS 读出）。RX 丢字节同样计数（rx_drop）。
   ⚠️ 注意 uart_putc 的忙等**不可能完全去掉**（TX 环有限），但把它从"塞满 128B
   才能继续"变成"只有极端连续输出才会碰上"，并且**丢字符变成可观测**。            */
#define RX_BUF_SZ 256
#define TX_BUF_SZ 256

static volatile uint8_t rx_buf[RX_BUF_SZ];
static volatile uint8_t rx_head = 0; /* next write position (ISR) */
static volatile uint8_t rx_tail = 0; /* next read position (main)  */

static volatile uint8_t tx_buf[TX_BUF_SZ];
static volatile uint8_t tx_head = 0; /* next write (main)   */
static volatile uint8_t tx_tail = 0; /* next read (ISR)     */

/* ---- drop counters (修任务④-②：把"偶发"变成可观测) ----------------------
   ★ 这两条计数器是本次修复的**可判定性来源**：链路丢字节原本没有任何外部信号，
   症状只表现为"偶发不执行"。计数后可以：
     - 用 `STATS` 命令读出（见 core/cmd.c）
     - 压测时对比「发送条数」与「ACK 条数」的差值是否等于 drop 计数
   volatile：在 ISR 与主循环两个上下文里读写。                                            */
static volatile uint8_t rx_drop = 0; /* RX 环满而丢弃的字节数（饱和计数） */
static volatile uint8_t tx_drop = 0; /* TX 环满而拒绝排队的字节数（饱和计数） */

/* emit a RAM string (used only by uart_printf for its local stack buffer) */
static void uart_puts_ram(const char *s) {
    if (!s) return;
    while (*s) uart_putc(*s++);
}

static uint8_t rx_full(void) {
    return ((rx_head + 1) % RX_BUF_SZ) == rx_tail;
}
static uint8_t tx_full(void) {
    return ((tx_head + 1) % TX_BUF_SZ) == tx_tail;
}

uint8_t uart_rx_drop_count(void) { return rx_drop; }
uint8_t uart_tx_drop_count(void) { return tx_drop; }
void uart_clear_drop_counters(void) { rx_drop = 0; tx_drop = 0; }

/* 见 uart.h：给"低优先级上报"用的水位查询。
   ★ 实现要点：tx_head / tx_tail 都是 uint8_t，环长 256 ⇒ 相减天然回绕，
   不需要 16 位取模运算（在 8 位机上那是非原子的读改写窗口）。
   若哪天把 TX_BUF_SZ 改成非 256，这里必须跟着改 —— 因此保留 % 表达式，
   让编译器按常量约简，改大小也不会静默算错。 */
uint8_t uart_tx_used(void) {
    return (uint8_t)((uint8_t)(tx_head - tx_tail) % TX_BUF_SZ);
}

void uart_init(uint32_t baud) {
    /* reset heads/tails */
    rx_head = rx_tail = 0;
    tx_head = tx_tail = 0;

    uint16_t ubrr = (uint16_t)((F_CPU / (8UL * baud)) - 1UL); /* U2X0 = 1 */

    UCSR0A = (1 << U2X0);                 /* double speed for lower error   */
    UBRR0H = (uint8_t)(ubrr >> 8);
    UBRR0L = (uint8_t)(ubrr & 0xFF);

    UCSR0B = (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0) | (1 << UDRIE0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00); /* 8N1 */

    /* flush any pending RX data */
    (void)UDR0;
}

ISR(USART_RX_vect) {
    uint8_t d = UDR0;
    if (!rx_full()) {
        rx_buf[rx_head] = d;
        rx_head = (rx_head + 1) % RX_BUF_SZ;
    } else {
        /* ⚠️ 修任务④-②：这里原本是完全静默的丢弃（只在代码里写了句注释）。
         * 静默 = 症状"偶发且不可复现"。计数后可用 STATS 读出，
         * 让"链路丢字节"从一个猜测变成一个**可核对的读数**。 */
        if (rx_drop < 255) rx_drop++;
    }
}

ISR(USART_UDRE_vect) {
    if (tx_tail != tx_head) {
        UDR0 = tx_buf[tx_tail];
        tx_tail = (tx_tail + 1) % TX_BUF_SZ;
    } else {
        /* nothing left to send: disable UDRE interrupt */
        UCSR0B &= ~(1 << UDRIE0);
    }
}

void uart_putc(char c) {
    /* ⚠️ 修任务④-①：这里的忙等是「gripper 偶发不执行」链条的第①环。
     * 原实现 `while (tx_full()) {}` 在 TX 环满时**完全阻塞主循环** ——
     * 115200 baud 下排空 128B 需 ~11ms，期间 cmd_poll() 不跑，
     * 而 RX 中断照常写入并在环满时丢字节（见文件头的完整链条说明）。
     *
     * 现在的做法：TX 环已放大到 256B（多数输出一次入队，忙等窗口≈0）；
     * 万一仍然满，**放弃这一个字节并计数**，而不是无限期阻塞主循环。
     * 宁可丢一个字符并且"记下来"，也不要让主循环停止读 RX ——
     * 后者会让**整条 SET 指令**丢失且无任何痕迹（这正是本次要修的缺陷）。
     *
     * ⚠️ 丢字符只可能发生在"输出爆发"时（HELP / STATUS 等长输出被高频指令夹击），
     * 正常 ACK 只有十几个字节，不会触发。 */
    if (tx_full()) {
        if (tx_drop < 255) tx_drop++;
        return;
    }
    tx_buf[tx_head] = (uint8_t)c;
    tx_head = (tx_head + 1) % TX_BUF_SZ;
    /* make sure the ISR will drain it */
    UCSR0B |= (1 << UDRIE0);
}

void uart_puts(PGM_P s) {
    if (!s) return;
    char c;
    while ((c = (char)pgm_read_byte(s++)) != '\0') uart_putc(c);
}

int uart_printf(PGM_P fmt, ...) {
    char buf[96];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf_P(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n > 0) uart_puts_ram(buf);
    return n;
}

int uart_getc_nowait(void) {
    if (rx_tail == rx_head) return -1;
    int c = rx_buf[rx_tail];
    rx_tail = (rx_tail + 1) % RX_BUF_SZ;
    return c;
}
