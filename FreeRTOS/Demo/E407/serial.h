/*
 * serial.h - Serial console
 *
 * Developed by Werner Almesberger for Actility S.A., and
 * licensed under LGPLv2 by Actility S.A.
 */

#ifndef SERIAL_H
#define	SERIAL_H


/*
 * Receive handler, to be set by subsystem/application processing serial input.
 * The handker may be invoked from an interrupt handler and/or with interrupts
 * disabled.
 */

extern void (*serial_recv)(const char *buf, unsigned n);


/*
 * Initialize the serial console (UART). Must be run before calling any of the
 * other serial_* functions.
 */

void serial_init(void);

/*
 * Output characters on the serial console. This function is safe to run from
 * an * interrupt handler and/or with interrupts disabled, but may loop for a
 * while.
 */

void serial_send_isr(const char *buf, unsigned len);

/*
 * Like serial_send_isr, but not safe to call from an interrupt handler or
 * with interrupts disabled. May defer the actual output and thus return faster
 * than an equivalent call to serial_send_isr.
 */

void serial_send(const char *buf, unsigned len);

#endif /* !SERIAL_H */
