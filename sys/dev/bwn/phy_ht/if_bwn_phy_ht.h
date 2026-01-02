/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2025 FreeBSD Foundation
 *
 * HT PHY support for BCM4331 (D11 rev 29)
 */

#ifndef _IF_BWN_PHY_HT_H_
#define _IF_BWN_PHY_HT_H_

int	bwn_phy_ht_init(struct bwn_mac *mac);
void	bwn_phy_ht_rf_onoff(struct bwn_mac *mac, int on);
void	bwn_phy_ht_switch_analog(struct bwn_mac *mac, int on);
int	bwn_phy_ht_switch_channel(struct bwn_mac *mac, uint32_t newchan);
uint16_t bwn_phy_ht_get_default_chan(struct bwn_mac *mac);

enum bwn_phy_ht_clk_ctl_mode {
	BWN_PHY_HT_CLK_CTL_MODE_FAST,
	BWN_PHY_HT_CLK_CTL_MODE_DYNAMIC,
};

void bwn_phy_ht_clk_ctl(struct bwn_mac *mac,
			  enum bwn_phy_ht_clk_ctl_mode mode);

#endif /* _IF_BWN_PHY_HT_H_ */
