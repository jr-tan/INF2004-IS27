/**
 * @file    gpio_irq.h
 *  gpio_irq.h
 *  Shared GPIO interrupt dispatcher (whole team -- Tribunal 1 decision:
 *  "one shared gpio_irq_dispatch_isr module that reads the pin status and
 *  calls each handler, timestamp captured once on entry").
 *
 *  All GPIO pins share ONE interrupt (IO_IRQ_BANK0, number 13), and
 *  tk_def_int() keeps only one handler per number.  So nobody calls
 *  tk_def_int(13) directly: every subsystem attaches its pins here.
 *
 *  Handlers run in interrupt context: timestamp, count, tk_sig_sem /
 *  tk_set_flg only.  Keep each one well under 10 us.
 *
 *  Users: S2 encoders GP0/GP1 (right) and GP7/GP28 (left), S3 barcode GP2.
 *  (S4 polls the IMU, so INT1 on GP3 is claimed but not attached.)
 */

#ifndef GPIO_IRQ_H
#define GPIO_IRQ_H

#include <stdint.h> /* Fixed-width integer types (BARR-C 5.2). */
#include <tk/tkernel.h>

#define GPIO_EV_FALL (1U << 2) /* edge low  */
#define GPIO_EV_RISE (1U << 3) /* edge high */
#define GPIO_EV_BOTH (GPIO_EV_FALL | GPIO_EV_RISE)

#define GPIO_IRQ_NO  13 /* IO_IRQ_BANK0 */
#define GPIO_IRQ_PRI 2

/* events: which edges happened (GPIO_EV_*); t_now_us: hardware timer
   (1 MHz) read once when the interrupt was entered. */
typedef void (*gpio_irq_cb_t)(uint32_t gpio, uint32_t events,
                              uint32_t t_now_us);

/* Attach a handler to one pin and enable the given edges.
   Call from a task or usermain running on CORE 0 (the interrupt is
   routed to core 0).  The pin must already be configured as an input. */
IMPORT ER gpio_irq_attach (uint32_t gpio, uint32_t events,
                           gpio_irq_cb_t cb_val);

/* Statistics: worst dispatcher execution time (us) */
IMPORT volatile uint32_t g_gpio_irq_max_us;

#endif /* GPIO_IRQ_H */

/*** end of file ***/
