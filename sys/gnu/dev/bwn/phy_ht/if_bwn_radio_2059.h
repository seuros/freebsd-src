/*-
 * SPDX-License-Identifier: GPL-2.0
 *
 * Copyright (c) 2011 Rafal Milecki <zajec5@gmail.com>
 * Copyright (c) 2025 Seuros (FreeBSD port)
 *
 * Radio 2059 support for HT PHY (BCM4331)
 * Ported from Linux b43 driver.
 */

#ifndef _IF_BWN_RADIO_2059_H_
#define _IF_BWN_RADIO_2059_H_

/* Radio 2059 core routing */
#define	R2059_C1			0x000
#define	R2059_C2			0x400
#define	R2059_C3			0x800
#define	R2059_ALL			0xC00

/* Radio 2059 register addresses */
#define	R2059_RCAL_CONFIG		0x004
#define	R2059_RFPLL_MASTER		0x011
#define	R2059_RFPLL_MISC_EN		0x02b
#define	R2059_RFPLL_MISC_CAL_RESETN	0x02e
#define	R2059_XTAL_CONFIG2		0x0c0
#define	R2059_RCCAL_START_R1_Q1_P1	0x13c
#define	R2059_RCCAL_X1			0x13d
#define	R2059_RCCAL_TRC0		0x13e
#define	R2059_RCCAL_DONE_OSCCAP		0x140
#define	R2059_RCAL_STATUS		0x145
#define	R2059_RCCAL_MASTER		0x17f

/* PHY register values for channel switch */
struct bwn_phy_ht_chanspec {
	uint16_t	bw1;
	uint16_t	bw2;
	uint16_t	bw3;
	uint16_t	bw4;
	uint16_t	bw5;
	uint16_t	bw6;
};

/* Channel table entry for Radio 2059 */
struct bwn_phy_ht_channeltab_e_r2059 {
	uint16_t	freq;		/* Channel frequency in MHz */
	/* Radio register values */
	uint8_t		radio_syn16;
	uint8_t		radio_syn17;
	uint8_t		radio_syn22;
	uint8_t		radio_syn25;
	uint8_t		radio_syn27;
	uint8_t		radio_syn28;
	uint8_t		radio_syn29;
	uint8_t		radio_syn2c;
	uint8_t		radio_syn2d;
	uint8_t		radio_syn37;
	uint8_t		radio_syn41;
	uint8_t		radio_syn43;
	uint8_t		radio_syn47;
	uint8_t		radio_rxtx4a;
	uint8_t		radio_rxtx58;
	uint8_t		radio_rxtx5a;
	uint8_t		radio_rxtx6a;
	uint8_t		radio_rxtx6d;
	uint8_t		radio_rxtx6e;
	uint8_t		radio_rxtx92;
	uint8_t		radio_rxtx98;
	/* PHY register values */
	struct bwn_phy_ht_chanspec phy_regs;
};

/* Function prototypes */
void	bwn_radio_2059_upload_inittabs(struct bwn_mac *mac);
const struct bwn_phy_ht_channeltab_e_r2059 *
	bwn_radio_2059_get_channeltab(struct bwn_mac *mac, uint16_t freq);
void	bwn_radio_2059_channel_setup(struct bwn_mac *mac,
	    const struct bwn_phy_ht_channeltab_e_r2059 *e);

#endif /* _IF_BWN_RADIO_2059_H_ */
