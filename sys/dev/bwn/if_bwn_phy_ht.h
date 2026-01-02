/*-
 * SPDX-License-Identifier: BSD-2-Clause
 * Copyright (c) 2025 Seuros
 * HT-PHY support header for BCM4331 WiFi chip.
 */

#ifndef _IF_BWN_PHY_HT_H_
#define _IF_BWN_PHY_HT_H_

/*
 * HT-PHY state structure.
 * From Linux b43 phy_ht.h struct b43_phy_ht.
 */
struct bwn_phy_ht {
	uint16_t	rf_ctl_int_save[3];	/* Saved RF control (PA override) */
	bool		tx_pwr_ctl;		/* TX power control enabled */
	uint8_t		tx_pwr_idx[3];		/* Per-core TX power index */
	int32_t		bb_mult_save[3];	/* Saved BB multiplier */
	uint8_t		idle_tssi[3];		/* Idle TSSI per core */
};

int     bwn_phy_ht_attach(struct bwn_mac *);
void    bwn_phy_ht_detach(struct bwn_mac *);
int     bwn_phy_ht_prepare_hw(struct bwn_mac *);
void    bwn_phy_ht_init_pre(struct bwn_mac *);
int     bwn_phy_ht_init(struct bwn_mac *);
void    bwn_phy_ht_exit(struct bwn_mac *);
uint16_t bwn_phy_ht_read(struct bwn_mac *, uint16_t);
void    bwn_phy_ht_write(struct bwn_mac *, uint16_t, uint16_t);
uint16_t bwn_phy_ht_rf_read(struct bwn_mac *, uint16_t);
void    bwn_phy_ht_rf_write(struct bwn_mac *, uint16_t, uint16_t);
int     bwn_phy_ht_hwpctl(struct bwn_mac *);
void    bwn_phy_ht_rf_onoff(struct bwn_mac *, int);
void    bwn_phy_ht_switch_analog(struct bwn_mac *, int);
int     bwn_phy_ht_switch_channel(struct bwn_mac *, uint32_t);
uint32_t bwn_phy_ht_get_default_chan(struct bwn_mac *);
void    bwn_phy_ht_set_antenna(struct bwn_mac *, int);
int     bwn_phy_ht_im(struct bwn_mac *, int);
bwn_txpwr_result_t bwn_phy_ht_recalc_txpwr(struct bwn_mac *, int);
void    bwn_phy_ht_set_txpwr(struct bwn_mac *);
void    bwn_phy_ht_task_15s(struct bwn_mac *);
void    bwn_phy_ht_task_60s(struct bwn_mac *);

#endif /* _IF_BWN_PHY_HT_H_ */
