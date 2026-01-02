/*
 * Copyright (c) 2024 The BCM4331 Project
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include "opt_bwn.h"

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/module.h>

#include <machine/bus.h>
#include <machine/resource.h>

#include <dev/pci/pcivar.h>

#include <dev/net/if.h>
#include <dev/net/if_media.h>

#include <dev/bwn/if_bwnvar.h>
#include <dev/bwn/if_bwn_chip.h>
#include <dev/bwn/if_bwn_debug.h>
#include <dev/bwn/if_bwn_phy_common.h>
#include <dev/bwn/phy_ht/if_bwn_phy_ht.h>

/*
 * This is a minimal implementation of HT-PHY init, based on the Linux b43 driver.
 * The goal is to do the absolute minimum required to prevent a system lockup
 * when bringing the interface up.
 *
 * The key is to enable the PHY clock before any PHY/RF register access.
 */
int
bwn_phy_ht_init(struct bwn_mac *mac)
{
	struct bwn_phy *phy = &mac->mac_phy;
	struct bwn_phy_ht *phy_ht = phy->phy_n->p_ht;

	device_printf(mac->mac_sc->sc_dev, "bwn_phy_ht_init: Minimal HT-PHY init\n");

	/*
	 * Based on Linux b43's b43_phy_ht_op_prepare_hardware.
	 * The most critical step to prevent a bus lockup is to ensure
	 * the PHY clock is enabled.
	 */
	bwn_phy_ht_clk_ctl(mac, BWN_PHY_HT_CLK_CTL_MODE_FAST);

	/* Initialize HT-PHY specific data structures. */
	phy_ht->pabias = 0;

	/* TODO: The rest of the full HT-PHY init sequence will go here.
	 * This includes:
	 * - Disabling interference mitigation
	 * - Setting up band
	 * - Writing PHY/Radio tables
	 * - Calibrating the radio
	 */

	device_printf(mac->mac_sc->sc_dev, "bwn_phy_ht_init: Clock enabled, returning.\n");

	return (0);
}

void
bwn_phy_ht_clk_ctl(struct bwn_mac *mac, enum bwn_phy_ht_clk_ctl_mode mode)
{
	uint16_t ctl;

	ctl = BWN_READ_2(mac, BWN_PHY_HT_CLASS_CTL);
	ctl &= ~BWN_PHY_HT_CLASS_CTL_CCK_EN;
	BWN_WRITE_2(mac, BWN_PHY_HT_CLASS_CTL, ctl);

	/*
	 * This is the crucial part. If the PHY clock is not enabled, any
	 * subsequent access to PHY or RF registers will result in a hard
	* system lockup.
	 * The Linux driver (b43) does this early in its init sequence.
	 */
	if (mode == BWN_PHY_HT_CLK_CTL_MODE_FAST) {
		/* Force PHY clock on */
		BWN_PHY_SET(mac, BWN_PHY_HT_AFE_CTL1, 0x000c);
		BWN_PHY_SET(mac, BWN_PHY_HT_AFE_CTL2, 0x000c);
		BWN_PHY_SET(mac, BWN_PHY_HT_PLL_CTL, 0x0008);
	} else {
		/* Let MAC control the PHY clock */
		BWN_PHY_UNSET(mac, BWN_PHY_HT_AFE_CTL1, 0x000c);
		BWN_PHY_UNSET(mac, BWN_PHY_HT_AFE_CTL2, 0x000c);
		BWN_PHY_UNSET(mac, BWN_PHY_HT_PLL_CTL, 0x0008);
	}
}

int
bwn_phy_ht_rf_onoff(struct bwn_mac *mac, int on)
{
	/* Minimal implementation to avoid crash */
	device_printf(mac->mac_sc->sc_dev, "bwn_phy_ht_rf_onoff: called with %d\n", on);
	return (0);
}

void
bwn_phy_ht_switch_analog(struct bwn_mac *mac, int on)
{
	/* Minimal implementation to avoid crash */
	device_printf(mac->mac_sc->sc_dev, "bwn_phy_ht_switch_analog: called with %d\n", on);
}

uint16_t
bwn_phy_ht_get_default_chan(struct bwn_mac *mac)
{
	/* Return a safe default channel */
	return (1);
}

int
bwn_phy_ht_switch_channel(struct bwn_mac *mac, uint16_t channel)
{
	/* Minimal implementation to avoid crash */
	device_printf(mac->mac_sc->sc_dev, "bwn_phy_ht_switch_channel: called with %u\n", channel);
	return (0);
}
