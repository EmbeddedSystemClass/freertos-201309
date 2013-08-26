/*
 * rf230bb.h - Items common to all rf230bb HALs (internal use)
 *
 * Developed by Werner Almesberger for Actility S.A., and
 * licensed under LGPLv2 by Actility S.A.
 */

#ifndef RF230BB_H
#define	RF230BB_H


/* ----- Transceiver commands and bits ------------------------------------- */


#define	AT86RF230_REG_READ	0x80
#define	AT86RF230_REG_WRITE	0xc0
#define	AT86RF230_BUF_READ	0x20
#define	AT86RF230_BUF_WRITE	0x60

#define	IRQ_TRX_END		0x08


/* ----- Items shared with rf230bb ----------------------------------------- */


extern hal_rx_frame_t rxframe[RF230_CONF_RX_BUFFERS];
extern uint8_t rxframe_head, rxframe_tail;


#endif /* !RF230BB_H */
