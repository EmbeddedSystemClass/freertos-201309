/*
 * contiki.h - Interface to the Contiki subsystem
 *
 * Developed by Werner Almesberger for Actility S.A., and
 * licensed under LGPLv2 by Actility S.A.
 */

#ifndef CONTIKI_H
#define	CONTIKI_H


extern void contiki_main(void);
extern int serial_line_input_byte(unsigned char c);

#endif /* !CONTIKI_H */
