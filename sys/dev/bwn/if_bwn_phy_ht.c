/*-
 * SPDX-License-Identifier: BSD-2-Clause
 * Copyright (c) 2025 Seuros
 *
 * HT-PHY support for BCM4331 WiFi chip.
 * Based on Linux b43 driver phy_ht.c
 */

#include <sys/cdefs.h>
#include "opt_bwn.h"
#include "opt_wlan.h"

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/endian.h>
#include <sys/errno.h>
#include <sys/lock.h>
#include <sys/mutex.h>
#include <machine/bus.h>
#include <machine/resource.h>
#include <sys/bus.h>
#include <sys/rman.h>
#include <sys/socket.h>
#include <sys/sockio.h>

#include <net/ethernet.h>
#include <net/if.h>
#include <net/if_var.h>
#include <net/if_arp.h>
#include <net/if_dl.h>
#include <net/if_llc.h>
#include <net/if_media.h>
#include <net/if_types.h>

#include <dev/pci/pcivar.h>
#include <dev/pci/pcireg.h>

#include <net80211/ieee80211_var.h>
#include <net80211/ieee80211_radiotap.h>
#include <net80211/ieee80211_regdomain.h>
#include <net80211/ieee80211_phy.h>
#include <net80211/ieee80211_ratectl.h>

#include <dev/bwn/if_bwnreg.h>
#include <dev/bwn/if_bwnvar.h>
#include <dev/bwn/if_bwn_debug.h>
#include <dev/bwn/if_bwn_misc.h>
#include <dev/bwn/if_bwn_phy_ht.h>

/* From if_bwn_phy_common.h - avoid full include due to BHND dependency */
extern int bwn_mac_phy_clock_set(struct bwn_mac *mac, int enabled);
extern int bwn_phy_force_clock(struct bwn_mac *mac, int force);

#include <gnu/dev/bwn/phy_ht/if_bwn_radio_2059.h>
#include <gnu/dev/bwn/phy_ht/if_bwn_phy_ht_tables.h>

/*
 * HT PHY register definitions (from Linux b43 phy_ht.h)
 * IMPORTANT: Must use correct PHY routing macros!
 */

/* Direct registers (no routing prefix) */
#define	BWN_PHY_HT_BBCFG		0x001	/* BB config */
#define	BWN_PHY_HT_BBCFG_RSTCCA		0x4000	/* Reset CCA */
#define	BWN_PHY_HT_BBCFG_RSTRX		0x8000	/* Reset RX */
#define	BWN_PHY_HT_BANDCTL		0x009	/* Band control */
#define	BWN_PHY_HT_BANDCTL_5GHZ		0x0001	/* Use 5GHz band */
#define	BWN_PHY_HT_CLASS_CTL		0x0b0	/* Classifier control */
#define	BWN_PHY_HT_CLASS_CTL_CCK_EN	0x0001	/* CCK enable */
#define	BWN_PHY_HT_CLASS_CTL_OFDM_EN	0x0002	/* OFDM enable */
#define	BWN_PHY_HT_CLASS_CTL_WAITED_EN	0x0004	/* Waited enable */

/* RF control command register (direct address) */
#define	BWN_PHY_HT_RF_CTL_CMD		0x810
#define	BWN_PHY_HT_RF_CTL_CMD_FORCE	0x0001
#define	BWN_PHY_HT_RF_CTL_CMD_CHIP0_PU	0x0002

/* RF sequence registers - use EXTG routing */
#define	BWN_PHY_HT_RF_SEQ_MODE		BWN_PHY_EXTG(0x000)
#define	BWN_PHY_HT_RF_SEQ_TRIG		BWN_PHY_EXTG(0x003)
#define	BWN_PHY_HT_RF_SEQ_TRIG_RX2TX	0x0001
#define	BWN_PHY_HT_RF_SEQ_TRIG_TX2RX	0x0002
#define	BWN_PHY_HT_RF_SEQ_TRIG_UPGH	0x0004
#define	BWN_PHY_HT_RF_SEQ_TRIG_UPGL	0x0008
#define	BWN_PHY_HT_RF_SEQ_TRIG_UPGU	0x0010
#define	BWN_PHY_HT_RF_SEQ_TRIG_RST2RX	0x0020
#define	BWN_PHY_HT_RF_SEQ_STATUS	BWN_PHY_EXTG(0x004)

/* RF control internal registers - use EXTG routing */
#define	BWN_PHY_HT_RF_CTL_INT_C1	BWN_PHY_EXTG(0x04c)
#define	BWN_PHY_HT_RF_CTL_INT_C2	BWN_PHY_EXTG(0x06c)
#define	BWN_PHY_HT_RF_CTL_INT_C3	BWN_PHY_EXTG(0x08c)

/* AFE (Analog Front End) registers - use EXTG routing */
#define	BWN_PHY_HT_AFE_C1_OVER		BWN_PHY_EXTG(0x110)
#define	BWN_PHY_HT_AFE_C1		BWN_PHY_EXTG(0x111)
#define	BWN_PHY_HT_AFE_C2_OVER		BWN_PHY_EXTG(0x114)
#define	BWN_PHY_HT_AFE_C2		BWN_PHY_EXTG(0x115)
#define	BWN_PHY_HT_AFE_C3_OVER		BWN_PHY_EXTG(0x118)
#define	BWN_PHY_HT_AFE_C3		BWN_PHY_EXTG(0x119)

/* B-PHY registers - use N_BMODE routing */
#define	BWN_PHY_B_BBCFG			BWN_PHY_N_BMODE(0x001)
#define	BWN_PHY_B_BBCFG_RSTCCA		0x4000
#define	BWN_PHY_B_BBCFG_RSTRX		0x8000
#define	BWN_PHY_HT_TEST			BWN_PHY_N_BMODE(0x00a)

/* Clip threshold registers - use OFDM routing */
#define	BWN_PHY_HT_C1_CLIP1THRES	BWN_PHY_OFDM(0x00e)
#define	BWN_PHY_HT_C2_CLIP1THRES	BWN_PHY_OFDM(0x04e)
#define	BWN_PHY_HT_C3_CLIP1THRES	BWN_PHY_OFDM(0x08e)

/*
 * PHY register access - same as N PHY
 */
uint16_t
bwn_phy_ht_read(struct bwn_mac *mac, uint16_t reg)
{
	BWN_WRITE_2(mac, BWN_PHYCTL, reg);
	return BWN_READ_2(mac, BWN_PHYDATA);
}

void
bwn_phy_ht_write(struct bwn_mac *mac, uint16_t reg, uint16_t value)
{
	BWN_WRITE_2(mac, BWN_PHYCTL, reg);
	BWN_WRITE_2(mac, BWN_PHYDATA, value);
}

static void
bwn_phy_ht_set(struct bwn_mac *mac, uint16_t reg, uint16_t set)
{
	bwn_phy_ht_write(mac, reg, bwn_phy_ht_read(mac, reg) | set);
}

static void
bwn_phy_ht_mask(struct bwn_mac *mac, uint16_t reg, uint16_t mask)
{
	bwn_phy_ht_write(mac, reg, bwn_phy_ht_read(mac, reg) & mask);
}

static void
bwn_phy_ht_maskset(struct bwn_mac *mac, uint16_t reg, uint16_t mask, uint16_t set)
{
	bwn_phy_ht_write(mac, reg, (bwn_phy_ht_read(mac, reg) & mask) | set);
}

/*
 * RF register access for Radio 2059
 * Based on Linux b43 phy_ht.c
 */
uint16_t
bwn_phy_ht_rf_read(struct bwn_mac *mac, uint16_t reg)
{
	/* Radio 2059 uses different access method */
	BWN_WRITE_2(mac, BWN_RFCTL, reg);
	return BWN_READ_2(mac, BWN_RFDATALO);
}

void
bwn_phy_ht_rf_write(struct bwn_mac *mac, uint16_t reg, uint16_t value)
{
	BWN_WRITE_2(mac, BWN_RFCTL, reg);
	BWN_WRITE_2(mac, BWN_RFDATALO, value);
}

static void
bwn_radio_set(struct bwn_mac *mac, uint16_t reg, uint16_t set)
{
	bwn_phy_ht_rf_write(mac, reg, bwn_phy_ht_rf_read(mac, reg) | set);
}

static void
bwn_radio_mask(struct bwn_mac *mac, uint16_t reg, uint16_t mask)
{
	bwn_phy_ht_rf_write(mac, reg, bwn_phy_ht_rf_read(mac, reg) & mask);
}

static void
bwn_radio_maskset(struct bwn_mac *mac, uint16_t reg, uint16_t mask, uint16_t set)
{
	bwn_phy_ht_rf_write(mac, reg, (bwn_phy_ht_rf_read(mac, reg) & mask) | set);
}

/*
 * Wait for a radio register to reach a specific value.
 * Returns 1 on success, 0 on timeout.
 */
static int
bwn_radio_wait_value(struct bwn_mac *mac, uint16_t reg, uint16_t mask,
    uint16_t value, int delay_us, int timeout_us)
{
	int elapsed = 0;
	uint16_t val;

	while (elapsed < timeout_us) {
		val = bwn_phy_ht_rf_read(mac, reg);
		if ((val & mask) == value)
			return (1);
		DELAY(delay_us);
		elapsed += delay_us;
	}
	return (0);
}

/*
 * Radio 2059 resistor calibration (rcal)
 * From Linux b43 phy_ht.c b43_radio_2059_rcal()
 */
static void
bwn_radio_2059_rcal(struct bwn_mac *mac)
{
	/* Enable */
	bwn_radio_set(mac, R2059_C3 | R2059_RCAL_CONFIG, 0x1);
	DELAY(20);

	bwn_radio_set(mac, R2059_C3 | 0x0BF, 0x1);
	bwn_radio_maskset(mac, R2059_C3 | 0x19B, 0x3, 0x2);

	/* Start */
	bwn_radio_set(mac, R2059_C3 | R2059_RCAL_CONFIG, 0x2);
	DELAY(200);

	/* Stop */
	bwn_radio_mask(mac, R2059_C3 | R2059_RCAL_CONFIG, ~0x2);

	/* Wait for completion */
	if (!bwn_radio_wait_value(mac, R2059_C3 | R2059_RCAL_STATUS, 1, 1,
	    100, 1000000))
		device_printf(mac->mac_sc->sc_dev,
		    "HT-PHY: Radio 2059 rcal timeout\n");

	/* Disable */
	bwn_radio_mask(mac, R2059_C3 | R2059_RCAL_CONFIG, ~0x1);

	bwn_radio_set(mac, 0xa, 0x60);
}

/*
 * Radio 2059 RC oscillator calibration (rccal)
 * From Linux b43 phy_ht.c b43_radio_2057_rccal()
 * Note: Uses 2057 name but R2059 registers
 */
static void
bwn_radio_2057_rccal(struct bwn_mac *mac)
{
	static const uint16_t radio_values[3][2] = {
		{ 0x61, 0xE9 }, { 0x69, 0xD5 }, { 0x73, 0x99 },
	};
	int i;

	for (i = 0; i < 3; i++) {
		bwn_phy_ht_rf_write(mac, R2059_RCCAL_MASTER, radio_values[i][0]);
		bwn_phy_ht_rf_write(mac, R2059_RCCAL_X1, 0x6E);
		bwn_phy_ht_rf_write(mac, R2059_RCCAL_TRC0, radio_values[i][1]);

		/* Start */
		bwn_phy_ht_rf_write(mac, R2059_RCCAL_START_R1_Q1_P1, 0x55);

		/* Wait for completion */
		if (!bwn_radio_wait_value(mac, R2059_RCCAL_DONE_OSCCAP, 2, 2,
		    500, 5000000))
			device_printf(mac->mac_sc->sc_dev,
			    "HT-PHY: Radio 2059 rccal timeout (step %d)\n", i);

		/* Stop */
		bwn_phy_ht_rf_write(mac, R2059_RCCAL_START_R1_Q1_P1, 0x15);
	}

	bwn_radio_mask(mac, R2059_RCCAL_MASTER, ~0x1);
}

/*
 * Radio 2059 pre-init: power up radio chip
 * From Linux b43 phy_ht.c b43_radio_2059_init_pre()
 */
static void
bwn_radio_2059_init_pre(struct bwn_mac *mac)
{
	/*
	 * Power up radio chip.
	 * Use BWN_PHY_* macros which have KASSERT for mac_suspended.
	 */
	BWN_PHY_MASK(mac, BWN_PHY_HT_RF_CTL_CMD,
	    ~BWN_PHY_HT_RF_CTL_CMD_CHIP0_PU);
	BWN_PHY_SET(mac, BWN_PHY_HT_RF_CTL_CMD,
	    BWN_PHY_HT_RF_CTL_CMD_FORCE);
	BWN_PHY_MASK(mac, BWN_PHY_HT_RF_CTL_CMD,
	    ~BWN_PHY_HT_RF_CTL_CMD_FORCE);
	BWN_PHY_SET(mac, BWN_PHY_HT_RF_CTL_CMD,
	    BWN_PHY_HT_RF_CTL_CMD_CHIP0_PU);
}

/*
 * Step-by-step radio init for debugging.
 * Call with step 0, 1, 2, etc. to isolate the crash.
 */
static void
bwn_radio_2059_init_step(struct bwn_mac *mac, int step)
{
	static const uint16_t routing[] = { R2059_C1, R2059_C2, R2059_C3 };

	device_printf(mac->mac_sc->sc_dev,
	    "HT-PHY: Radio 2059 step %d starting\n", step);

	switch (step) {
	case 0:
		/* Step 0: Just print, no radio access */
		device_printf(mac->mac_sc->sc_dev,
		    "HT-PHY: Step 0 - no-op\n");
		break;

	case 1:
		/* Step 1: Power up radio via PHY register */
		device_printf(mac->mac_sc->sc_dev,
		    "HT-PHY: Step 1 - radio power up via PHY\n");
		bwn_radio_2059_init_pre(mac);
		break;

	case 2:
		/* Step 2: Upload init tables */
		device_printf(mac->mac_sc->sc_dev,
		    "HT-PHY: Step 2 - upload init tables\n");
		bwn_radio_2059_upload_inittabs(mac);
		break;

	case 3:
		/* Step 3: Set routing bits */
		device_printf(mac->mac_sc->sc_dev,
		    "HT-PHY: Step 3 - routing bits\n");
		BWN_RF_SET(mac, routing[0] | 0x146, 0x3);
		BWN_RF_SET(mac, routing[1] | 0x146, 0x3);
		BWN_RF_SET(mac, routing[2] | 0x146, 0x3);
		break;

	case 4:
		/* Step 4: RFPLL reset sequence */
		device_printf(mac->mac_sc->sc_dev,
		    "HT-PHY: Step 4 - RFPLL reset\n");
		BWN_RF_SET(mac, R2059_RFPLL_MISC_CAL_RESETN, 0x0078);
		BWN_RF_SET(mac, R2059_XTAL_CONFIG2, 0x0080);
		DELAY(2000);
		BWN_RF_MASK(mac, R2059_RFPLL_MISC_CAL_RESETN, ~0x0078);
		BWN_RF_MASK(mac, R2059_XTAL_CONFIG2, ~0x0080);
		break;

	case 5:
		/* Step 5: Clear RFPLL master bit */
		device_printf(mac->mac_sc->sc_dev,
		    "HT-PHY: Step 5 - clear RFPLL master\n");
		BWN_RF_MASK(mac, R2059_RFPLL_MASTER, ~0x0008);
		break;

	default:
		device_printf(mac->mac_sc->sc_dev,
		    "HT-PHY: Step %d - unknown\n", step);
		break;
	}

	device_printf(mac->mac_sc->sc_dev,
	    "HT-PHY: Radio 2059 step %d complete\n", step);
}

/*
 * Full Radio 2059 initialization
 * From Linux b43 phy_ht.c b43_radio_2059_init()
 */
static void
bwn_radio_2059_init(struct bwn_mac *mac)
{
	static const uint16_t routing[] = { R2059_C1, R2059_C2, R2059_C3 };
	int i;

	device_printf(mac->mac_sc->sc_dev, "HT-PHY: Radio 2059 init starting\n");

	/* Step 1: Power up radio via PHY register */
	bwn_radio_2059_init_pre(mac);

	/* Step 2: Upload init tables */
	bwn_radio_2059_upload_inittabs(mac);

	/* Step 3: Set routing bits for all 3 cores */
	for (i = 0; i < 3; i++)
		BWN_RF_SET(mac, routing[i] | 0x146, 0x3);

	/* Step 4: RFPLL reset sequence */
	BWN_RF_SET(mac, R2059_RFPLL_MISC_CAL_RESETN, 0x0078);
	BWN_RF_SET(mac, R2059_XTAL_CONFIG2, 0x0080);
	DELAY(2000);  /* 2ms */
	BWN_RF_MASK(mac, R2059_RFPLL_MISC_CAL_RESETN, ~0x0078);
	BWN_RF_MASK(mac, R2059_XTAL_CONFIG2, ~0x0080);

	/* Step 5: Resistor calibration */
	device_printf(mac->mac_sc->sc_dev, "HT-PHY: Starting rcal\n");
	bwn_radio_2059_rcal(mac);

	/* Step 6: RC oscillator calibration */
	device_printf(mac->mac_sc->sc_dev, "HT-PHY: Starting rccal\n");
	bwn_radio_2057_rccal(mac);

	/* Step 7: Clear RFPLL master bit */
	BWN_RF_MASK(mac, R2059_RFPLL_MASTER, ~0x0008);

	device_printf(mac->mac_sc->sc_dev, "HT-PHY: Radio 2059 init complete\n");
}

int
bwn_phy_ht_attach(struct bwn_mac *mac)
{
	struct bwn_phy_ht *phy_ht;

	device_printf(mac->mac_sc->sc_dev, "HT-PHY: attach\n");

	phy_ht = malloc(sizeof(*phy_ht), M_DEVBUF, M_NOWAIT | M_ZERO);
	if (phy_ht == NULL) {
		device_printf(mac->mac_sc->sc_dev,
		    "HT-PHY: failed to allocate phy_ht\n");
		return (ENOMEM);
	}

	/* Initialize default values */
	phy_ht->tx_pwr_ctl = true;
	for (int i = 0; i < 3; i++) {
		phy_ht->tx_pwr_idx[i] = 0x80;  /* B43_PHY_HT_TXPCTL_CMD_C1_INIT + 1 */
		phy_ht->bb_mult_save[i] = -1;
	}

	mac->mac_phy.phy_ht = phy_ht;
	return (0);
}

void
bwn_phy_ht_detach(struct bwn_mac *mac)
{
	device_printf(mac->mac_sc->sc_dev, "HT-PHY: detach\n");

	if (mac->mac_phy.phy_ht != NULL) {
		free(mac->mac_phy.phy_ht, M_DEVBUF);
		mac->mac_phy.phy_ht = NULL;
	}
}

int
bwn_phy_ht_prepare_hw(struct bwn_mac *mac)
{
	device_printf(mac->mac_sc->sc_dev,
	    "HT-PHY: prepare_hw - enabling radio before initvals\n");

	/*
	 * Power up radio before initvals are loaded.
	 * This prevents crash from writing to un-clocked PHY.
	 * Full radio init with calibration happens later in bwn_phy_ht_init.
	 */
	bwn_radio_2059_init_pre(mac);

	return (0);
}

void
bwn_phy_ht_init_pre(struct bwn_mac *mac)
{
	device_printf(mac->mac_sc->sc_dev, "HT-PHY: init_pre\n");
}

/*
 * Force RF sequence and wait for completion
 */
static void
bwn_phy_ht_force_rf_sequence(struct bwn_mac *mac, uint16_t rf_seq)
{
	uint16_t save_seq_mode;
	int i;

	save_seq_mode = bwn_phy_ht_read(mac, BWN_PHY_HT_RF_SEQ_MODE);
	bwn_phy_ht_set(mac, BWN_PHY_HT_RF_SEQ_MODE, 0x3);

	bwn_phy_ht_set(mac, BWN_PHY_HT_RF_SEQ_TRIG, rf_seq);
	for (i = 0; i < 200; i++) {
		if (!(bwn_phy_ht_read(mac, BWN_PHY_HT_RF_SEQ_STATUS) & rf_seq))
			break;
		DELAY(1000);
	}
	if (i >= 200)
		device_printf(mac->mac_sc->sc_dev,
		    "HT-PHY: RF sequence timeout\n");

	bwn_phy_ht_write(mac, BWN_PHY_HT_RF_SEQ_MODE, save_seq_mode);
}

/*
 * PA (Power Amplifier) override control
 * From Linux b43 phy_ht.c b43_phy_ht_pa_override()
 */
static void
bwn_phy_ht_pa_override(struct bwn_mac *mac, bool enable)
{
	struct bwn_phy_ht *phy_ht = mac->mac_phy.phy_ht;
	static const uint16_t regs[] = {
		BWN_PHY_EXTG(0x04c),  /* RF_CTL_INT_C1 */
		BWN_PHY_EXTG(0x06c),  /* RF_CTL_INT_C2 */
		BWN_PHY_EXTG(0x08c),  /* RF_CTL_INT_C3 */
	};
	int i;

	if (phy_ht == NULL)
		return;

	if (enable) {
		/* Restore saved values */
		for (i = 0; i < 3; i++)
			bwn_phy_ht_write(mac, regs[i], phy_ht->rf_ctl_int_save[i]);
	} else {
		/* Save current values and write override */
		for (i = 0; i < 3; i++) {
			phy_ht->rf_ctl_int_save[i] = bwn_phy_ht_read(mac, regs[i]);
			bwn_phy_ht_write(mac, regs[i], 0x0400);
		}
	}
}

/*
 * AFE unknown init operation
 * From Linux b43 phy_ht.c b43_phy_ht_afe_unk1()
 */
static void
bwn_phy_ht_afe_unk1(struct bwn_mac *mac)
{
	static const uint16_t afe_regs[] = {
		BWN_PHY_EXTG(0x111),  /* AFE_C1 */
		BWN_PHY_EXTG(0x115),  /* AFE_C2 */
		BWN_PHY_EXTG(0x119),  /* AFE_C3 */
	};
	static const uint16_t afe_over_regs[] = {
		BWN_PHY_EXTG(0x110),  /* AFE_C1_OVER */
		BWN_PHY_EXTG(0x114),  /* AFE_C2_OVER */
		BWN_PHY_EXTG(0x118),  /* AFE_C3_OVER */
	};
	int i;

	for (i = 0; i < 3; i++) {
		bwn_phy_ht_set(mac, afe_regs[i], 0x4);
		bwn_phy_ht_set(mac, afe_over_regs[i], 0x4);
		bwn_phy_ht_mask(mac, afe_regs[i], ~0x1);
		bwn_phy_ht_set(mac, afe_over_regs[i], 0x1);
		/* Table write: table 8, offset 5 + i*0x10, value 0 */
		bwn_httab_write(mac, B43_HTTAB16(8, 5 + (i * 0x10)), 0);
		bwn_phy_ht_mask(mac, afe_over_regs[i], ~0x4);
	}
}

/*
 * Zero EXTG registers
 */
static void
bwn_phy_ht_zero_extg(struct bwn_mac *mac)
{
	static const uint16_t base[] = { 0x40, 0x60, 0x80 };
	int i, j;

	for (i = 0; i < 3; i++) {
		for (j = 0; j < 4; j++)
			bwn_phy_ht_write(mac, BWN_PHY_EXTG(base[i] + j), 0);
	}

	for (i = 0; i < 3; i++)
		bwn_phy_ht_write(mac, BWN_PHY_EXTG(base[i] + 0xc), 0);
}

/*
 * Classifier control
 */
static uint16_t
bwn_phy_ht_classifier(struct bwn_mac *mac, uint16_t mask, uint16_t val)
{
	uint16_t tmp;
	uint16_t allowed = BWN_PHY_HT_CLASS_CTL_CCK_EN |
			   BWN_PHY_HT_CLASS_CTL_OFDM_EN |
			   BWN_PHY_HT_CLASS_CTL_WAITED_EN;

	tmp = bwn_phy_ht_read(mac, BWN_PHY_HT_CLASS_CTL);
	tmp &= allowed;
	tmp &= ~mask;
	tmp |= (val & mask);
	bwn_phy_ht_write(mac, BWN_PHY_HT_CLASS_CTL,
	    (bwn_phy_ht_read(mac, BWN_PHY_HT_CLASS_CTL) & ~allowed) | tmp);

	return tmp;
}

/*
 * TX power fix
 * From Linux b43 phy_ht.c b43_phy_ht_tx_power_fix()
 * Writes calibration values to gain tables.
 */
static void
bwn_phy_ht_tx_power_fix(struct bwn_mac *mac)
{
	int i;

	for (i = 0; i < 3; i++) {
		uint32_t tmp;

		/* Read calibration value from table 26 */
		tmp = bwn_httab_read(mac, B43_HTTAB32(26, 0xe8));

		/* Write upper 16 bits to table 7 */
		bwn_httab_write(mac, B43_HTTAB16(7, 0x110 + i), tmp >> 16);

		/* Write lower 8 bits to table 13 */
		bwn_httab_write(mac, B43_HTTAB8(13, 0x63 + (i * 4)), tmp & 0xff);
		bwn_httab_write(mac, B43_HTTAB8(13, 0x73 + (i * 4)), tmp & 0xff);
	}
}

/*
 * Reset CCA
 * From Linux b43 phy_ht.c b43_phy_ht_reset_cca()
 * Force PHY clock on via PSM register before CCA reset.
 */
static void
bwn_phy_ht_reset_cca(struct bwn_mac *mac)
{
	uint16_t bbcfg, psm;

	/* Force PHY clock on via PSM register (safer than bhnd ioctl) */
	psm = BWN_READ_2(mac, BWN_PSM_PHY_HDR);
	BWN_WRITE_2(mac, BWN_PSM_PHY_HDR, psm | BWN_PSM_HDR_MAC_PHY_FORCE_CLK);

	bbcfg = bwn_phy_ht_read(mac, BWN_PHY_HT_BBCFG);
	bwn_phy_ht_write(mac, BWN_PHY_HT_BBCFG, bbcfg | BWN_PHY_HT_BBCFG_RSTCCA);
	DELAY(1);
	bwn_phy_ht_write(mac, BWN_PHY_HT_BBCFG, bbcfg & ~BWN_PHY_HT_BBCFG_RSTCCA);

	/* Restore PSM register */
	BWN_WRITE_2(mac, BWN_PSM_PHY_HDR, psm);

	/* Trigger Reset-to-RX RF sequence to enable receiver */
	bwn_phy_ht_force_rf_sequence(mac, BWN_PHY_HT_RF_SEQ_TRIG_RST2RX);
}

/*
 * B-PHY reset
 */
static void
bwn_phy_ht_bphy_reset(struct bwn_mac *mac, int reset)
{
	uint16_t tmp;

	tmp = BWN_READ_2(mac, BWN_PSM_PHY_HDR);
	BWN_WRITE_2(mac, BWN_PSM_PHY_HDR, tmp | BWN_PSM_HDR_MAC_PHY_FORCE_CLK);

	if (reset)
		bwn_phy_ht_set(mac, BWN_PHY_B_BBCFG,
		    BWN_PHY_B_BBCFG_RSTCCA | BWN_PHY_B_BBCFG_RSTRX);
	else
		bwn_phy_ht_mask(mac, BWN_PHY_B_BBCFG,
		    (uint16_t)~(BWN_PHY_B_BBCFG_RSTCCA | BWN_PHY_B_BBCFG_RSTRX));

	BWN_WRITE_2(mac, BWN_PSM_PHY_HDR, tmp);
}

/*
 * B-PHY initialization
 */
static void
bwn_phy_ht_bphy_init(struct bwn_mac *mac)
{
	uint16_t val;
	int i;

	val = 0x1E1F;
	for (i = 0; i < 16; i++) {
		bwn_phy_ht_write(mac, BWN_PHY_N_BMODE(0x88 + i), val);
		val -= 0x202;
	}
	val = 0x3E3F;
	for (i = 0; i < 16; i++) {
		bwn_phy_ht_write(mac, BWN_PHY_N_BMODE(0x98 + i), val);
		val -= 0x202;
	}
	bwn_phy_ht_write(mac, BWN_PHY_N_BMODE(0x38), 0x668);
}

int
bwn_phy_ht_init(struct bwn_mac *mac)
{
	uint16_t tmp;
	static const uint16_t gain_tab7_14e[] = { 0x010f, 0x010f };
	static const uint16_t gain_tab7_130[] = { 0x777, 0x111, 0x111,
	    0x777, 0x111, 0x111, 0x777, 0x111, 0x111 };
	static const uint16_t gain_tab8_08[] = { 0x8e, 0x96, 0x96, 0x96 };
	static const uint16_t gain_tab8_18[] = { 0x8f, 0x9f, 0x9f, 0x9f };
	static const uint16_t gain_tab8_0c[] = { 0x2, 0x2, 0x2, 0x2 };
	static const uint16_t gain_tab_lna[] = { 0x09, 0x0e, 0x13, 0x18 };

	device_printf(mac->mac_sc->sc_dev, "HT-PHY: init starting\n");

	/* Upload PHY tables first */
	bwn_phy_ht_tables_init(mac);

	/* Basic setup from Linux b43 phy_ht.c */
	bwn_phy_ht_mask(mac, 0x0be, ~0x2);
	bwn_phy_ht_set(mac, 0x23f, 0x7ff);
	bwn_phy_ht_set(mac, 0x240, 0x7ff);
	bwn_phy_ht_set(mac, 0x241, 0x7ff);

	bwn_phy_ht_zero_extg(mac);

	bwn_phy_ht_mask(mac, BWN_PHY_EXTG(0), ~0x3);

	/* Clear AFE overrides */
	bwn_phy_ht_write(mac, BWN_PHY_HT_AFE_C1_OVER, 0);
	bwn_phy_ht_write(mac, BWN_PHY_HT_AFE_C2_OVER, 0);
	bwn_phy_ht_write(mac, BWN_PHY_HT_AFE_C3_OVER, 0);

	bwn_phy_ht_write(mac, BWN_PHY_EXTG(0x103), 0x20);
	bwn_phy_ht_write(mac, BWN_PHY_EXTG(0x101), 0x20);
	bwn_phy_ht_write(mac, 0x20d, 0xb8);
	bwn_phy_ht_write(mac, BWN_PHY_EXTG(0x14f), 0xc8);
	bwn_phy_ht_write(mac, 0x70, 0x50);
	bwn_phy_ht_write(mac, 0x1ff, 0x30);

	/* Enable CCK classifier for 2.4GHz */
	bwn_phy_ht_classifier(mac, BWN_PHY_HT_CLASS_CTL_CCK_EN,
	    BWN_PHY_HT_CLASS_CTL_CCK_EN);

	/* AFE unknown init (from Linux) */
	bwn_phy_ht_afe_unk1(mac);

	/*
	 * Critical table writes for RX gain control (from Linux b43)
	 * These must be done after tables_init and before RF sequences
	 */
	/* Table 7: RX gain control for 3 cores */
	bwn_httab_write_bulk(mac, B43_HTTAB16(7, 0x14e), 2, gain_tab7_14e);
	bwn_httab_write_bulk(mac, B43_HTTAB16(7, 0x15e), 2, gain_tab7_14e);
	bwn_httab_write_bulk(mac, B43_HTTAB16(7, 0x16e), 2, gain_tab7_14e);

	/* Table 7: More gain settings */
	bwn_httab_write_bulk(mac, B43_HTTAB16(7, 0x130), 9, gain_tab7_130);
	bwn_httab_write(mac, B43_HTTAB16(7, 0x120), 0x0777);
	bwn_httab_write(mac, B43_HTTAB16(7, 0x124), 0x0777);

	/* Table 8: AGC settings for 3 cores */
	bwn_httab_write(mac, B43_HTTAB16(8, 0x00), 0x02);
	bwn_httab_write(mac, B43_HTTAB16(8, 0x10), 0x02);
	bwn_httab_write(mac, B43_HTTAB16(8, 0x20), 0x02);

	bwn_httab_write_bulk(mac, B43_HTTAB16(8, 0x08), 4, gain_tab8_08);
	bwn_httab_write_bulk(mac, B43_HTTAB16(8, 0x18), 4, gain_tab8_18);
	bwn_httab_write_bulk(mac, B43_HTTAB16(8, 0x28), 4, gain_tab8_18);

	bwn_httab_write_bulk(mac, B43_HTTAB16(8, 0x0c), 4, gain_tab8_0c);
	bwn_httab_write_bulk(mac, B43_HTTAB16(8, 0x1c), 4, gain_tab8_0c);
	bwn_httab_write_bulk(mac, B43_HTTAB16(8, 0x2c), 4, gain_tab8_0c);

	/* PHY register gain settings */
	bwn_phy_ht_maskset(mac, 0x0280, 0xff00, 0x3e);
	bwn_phy_ht_maskset(mac, 0x0283, 0xff00, 0x3e);
	bwn_phy_ht_maskset(mac, BWN_PHY_OFDM(0x0141), 0xff00, 0x46);
	bwn_phy_ht_maskset(mac, 0x0283, 0xff00, 0x40);

	/* LNA gain tables for 3 cores */
	bwn_httab_write_bulk(mac, B43_HTTAB16(0, 0x8), 4, gain_tab_lna);
	bwn_httab_write_bulk(mac, B43_HTTAB16(1, 0x8), 4, gain_tab_lna);
	bwn_httab_write_bulk(mac, B43_HTTAB16(2, 0x8), 4, gain_tab_lna);

	/* OFDM AGC settings */
	/*
	 * OFDM AGC settings - CRITICAL for RX to work.
	 * These values were WRONG - ported incorrectly from Linux.
	 * Linux phy_ht.c line 957-959:
	 *   b43_phy_maskset(dev, B43_PHY_OFDM(0x24), 0x3f, 0xd);
	 */
	bwn_phy_ht_maskset(mac, BWN_PHY_OFDM(0x24), 0x3f, 0xd);
	bwn_phy_ht_maskset(mac, BWN_PHY_OFDM(0x64), 0x3f, 0xd);
	bwn_phy_ht_maskset(mac, BWN_PHY_OFDM(0xa4), 0x3f, 0xd);

	bwn_phy_ht_set(mac, 0xb1, 0x91);
	bwn_phy_ht_write(mac, 0x32f, 0x0003);
	bwn_phy_ht_write(mac, 0x077, 0x0010);
	bwn_phy_ht_write(mac, 0x0b4, 0x0258);
	bwn_phy_ht_mask(mac, 0x17e, ~0x4000);

	bwn_phy_ht_write(mac, 0x0b9, 0x0072);

	bwn_phy_ht_set(mac, BWN_PHY_EXTG(0x060), 0x1);
	bwn_phy_ht_set(mac, BWN_PHY_EXTG(0x064), 0x1);
	bwn_phy_ht_set(mac, BWN_PHY_EXTG(0x080), 0x1);
	bwn_phy_ht_set(mac, BWN_PHY_EXTG(0x084), 0x1);

	/* Copy table entries for calibration (from Linux) */
	tmp = bwn_httab_read(mac, B43_HTTAB16(7, 0x144));
	bwn_httab_write(mac, B43_HTTAB16(7, 0x14a), tmp);
	tmp = bwn_httab_read(mac, B43_HTTAB16(7, 0x154));
	bwn_httab_write(mac, B43_HTTAB16(7, 0x15a), tmp);
	tmp = bwn_httab_read(mac, B43_HTTAB16(7, 0x164));
	bwn_httab_write(mac, B43_HTTAB16(7, 0x16a), tmp);

	/* Reset CCA */
	tmp = bwn_phy_ht_read(mac, BWN_PHY_HT_BBCFG);
	bwn_phy_ht_write(mac, BWN_PHY_HT_BBCFG, tmp | BWN_PHY_HT_BBCFG_RSTCCA);
	bwn_phy_ht_write(mac, BWN_PHY_HT_BBCFG, tmp & ~BWN_PHY_HT_BBCFG_RSTCCA);

	/* Enable MAC PHY clock before RF sequences (from Linux) */
	bwn_mac_phy_clock_set(mac, true);

	/* PA override off before RF sequences (from Linux) */
	bwn_phy_ht_pa_override(mac, false);

	/* Force RF sequences */
	bwn_phy_ht_force_rf_sequence(mac, BWN_PHY_HT_RF_SEQ_TRIG_RX2TX);
	bwn_phy_ht_force_rf_sequence(mac, BWN_PHY_HT_RF_SEQ_TRIG_RST2RX);

	/* PA override on after RF sequences (from Linux) */
	bwn_phy_ht_pa_override(mac, true);

	/* Classifier setup */
	bwn_phy_ht_classifier(mac, 0, 0);

	/* B-PHY init for 2.4GHz */
	bwn_phy_ht_bphy_init(mac);

	/* Upload late-init table */
	bwn_phy_ht_tables_init_late(mac);

	/*
	 * Full radio initialization with calibrations.
	 * This must happen after PHY tables are loaded!
	 * Note: rf_onoff() is called before init() in FreeBSD,
	 * so we do radio init here, not in rf_onoff().
	 */
	bwn_radio_2059_init(mac);

	/*
	 * TX power fix - writes calibration values to gain tables.
	 * Linux calls this at the end of init and after each channel switch.
	 */
	bwn_phy_ht_tx_power_fix(mac);

	device_printf(mac->mac_sc->sc_dev, "HT-PHY: init complete\n");

	return (0);
}

void
bwn_phy_ht_exit(struct bwn_mac *mac)
{
	device_printf(mac->mac_sc->sc_dev, "HT-PHY: exit\n");
}

int
bwn_phy_ht_hwpctl(struct bwn_mac *mac)
{
	return (0);
}

void
bwn_phy_ht_rf_onoff(struct bwn_mac *mac, int on)
{
	device_printf(mac->mac_sc->sc_dev, "HT-PHY: rf_onoff(%d)\n", on);

	/*
	 * RF power control.
	 * Linux phy_ht.c b43_phy_ht_op_software_rfkill():
	 * When unblocking (turning on), call radio_2059_init().
	 */
	if (on) {
		/*
		 * Full radio init when turning on.
		 * This includes PLL setup and calibrations.
		 */
		bwn_radio_2059_init(mac);
	} else {
		BWN_PHY_MASK(mac, BWN_PHY_HT_RF_CTL_CMD,
		    ~BWN_PHY_HT_RF_CTL_CMD_CHIP0_PU);
	}
}

void
bwn_phy_ht_switch_analog(struct bwn_mac *mac, int on)
{
	device_printf(mac->mac_sc->sc_dev, "HT-PHY: switch_analog(%d)\n", on);

	/*
	 * Control power to analog PHY circuitry (AFE).
	 * From Linux b43 phy_ht.c b43_phy_ht_op_switch_analog()
	 */
	if (on) {
		/* Power up: set AFE control, clear override */
		bwn_phy_ht_write(mac, BWN_PHY_HT_AFE_C1, 0x00cd);
		bwn_phy_ht_write(mac, BWN_PHY_HT_AFE_C1_OVER, 0x0000);
		bwn_phy_ht_write(mac, BWN_PHY_HT_AFE_C2, 0x00cd);
		bwn_phy_ht_write(mac, BWN_PHY_HT_AFE_C2_OVER, 0x0000);
		bwn_phy_ht_write(mac, BWN_PHY_HT_AFE_C3, 0x00cd);
		bwn_phy_ht_write(mac, BWN_PHY_HT_AFE_C3_OVER, 0x0000);
	} else {
		/* Power down: set override, modify AFE control */
		bwn_phy_ht_write(mac, BWN_PHY_HT_AFE_C1_OVER, 0x07ff);
		bwn_phy_ht_write(mac, BWN_PHY_HT_AFE_C1, 0x00fd);
		bwn_phy_ht_write(mac, BWN_PHY_HT_AFE_C2_OVER, 0x07ff);
		bwn_phy_ht_write(mac, BWN_PHY_HT_AFE_C2, 0x00fd);
		bwn_phy_ht_write(mac, BWN_PHY_HT_AFE_C3_OVER, 0x07ff);
		bwn_phy_ht_write(mac, BWN_PHY_HT_AFE_C3, 0x00fd);
	}
}

int
bwn_phy_ht_switch_channel(struct bwn_mac *mac, uint32_t newchan)
{
	const struct bwn_phy_ht_channeltab_e_r2059 *e;
	uint16_t freq;

	/*
	 * Get the frequency for the requested channel.
	 * newchan is the IEEE channel number, convert to MHz.
	 */
	freq = bwn_get_centre_freq(mac);

	device_printf(mac->mac_sc->sc_dev,
	    "HT-PHY: switch_channel(%u) freq=%u MHz\n", newchan, freq);

	/* Look up channel in Radio 2059 table */
	e = bwn_radio_2059_get_channeltab(mac, freq);
	if (e == NULL) {
		device_printf(mac->mac_sc->sc_dev,
		    "HT-PHY: no channel table entry for %u MHz\n", freq);
		return (EINVAL);
	}

	/* Program the radio for this channel */
	bwn_radio_2059_channel_setup(mac, e);

	/*
	 * Reset B-PHY (CCK baseband) for 2GHz operation.
	 * This is critical - without it, the receiver won't work!
	 * Linux phy_ht.c line 779: b43_phy_ht_bphy_reset(dev, false);
	 */
	bwn_phy_ht_bphy_reset(mac, 0);

	/* Update PHY bandwidth registers */
	bwn_phy_ht_write(mac, 0x1ce, e->phy_regs.bw1);
	bwn_phy_ht_write(mac, 0x1cf, e->phy_regs.bw2);
	bwn_phy_ht_write(mac, 0x1d0, e->phy_regs.bw3);
	bwn_phy_ht_write(mac, 0x1d1, e->phy_regs.bw4);
	bwn_phy_ht_write(mac, 0x1d2, e->phy_regs.bw5);
	bwn_phy_ht_write(mac, 0x1d3, e->phy_regs.bw6);

	/*
	 * Enable OFDM classifier for RX (except channel 14)
	 * This is critical for detecting WiFi networks!
	 */
	if (newchan != 14) {
		bwn_phy_ht_classifier(mac, BWN_PHY_HT_CLASS_CTL_OFDM_EN,
		    BWN_PHY_HT_CLASS_CTL_OFDM_EN);
		/* Clear HT_TEST bits 0x840 for 2GHz (Linux line 796) */
		bwn_phy_ht_mask(mac, BWN_PHY_HT_TEST, ~0x840);
	}

	/* TX power fix after channel switch (Linux line 800) */
	bwn_phy_ht_tx_power_fix(mac);

	/*
	 * Reset CCA and trigger RST2RX to enable receiver on new channel.
	 * Note: Skip force_clock which crashes. Just toggle RSTCCA directly.
	 */
	{
		uint16_t bbcfg;
		bbcfg = bwn_phy_ht_read(mac, BWN_PHY_HT_BBCFG);
		bwn_phy_ht_write(mac, BWN_PHY_HT_BBCFG, bbcfg | BWN_PHY_HT_BBCFG_RSTCCA);
		DELAY(1);
		bwn_phy_ht_write(mac, BWN_PHY_HT_BBCFG, bbcfg & ~BWN_PHY_HT_BBCFG_RSTCCA);
	}
	bwn_phy_ht_force_rf_sequence(mac, BWN_PHY_HT_RF_SEQ_TRIG_RST2RX);

	/*
	 * Final register write from Linux phy_ht.c line 804:
	 *   b43_phy_write(dev, 0x017e, 0x3830);
	 * This is done at the end of every channel setup.
	 */
	bwn_phy_ht_write(mac, 0x17e, 0x3830);

	/* Store the current channel */
	mac->mac_phy.chan = newchan;

	device_printf(mac->mac_sc->sc_dev,
	    "HT-PHY: channel switch to %u MHz complete\n", freq);

	return (0);
}

uint32_t
bwn_phy_ht_get_default_chan(struct bwn_mac *mac)
{
	return (1);
}

void
bwn_phy_ht_set_antenna(struct bwn_mac *mac, int antenna)
{
}

int
bwn_phy_ht_im(struct bwn_mac *mac, int mode)
{
	return (0);
}

bwn_txpwr_result_t
bwn_phy_ht_recalc_txpwr(struct bwn_mac *mac, int ignore_tssi)
{
	return (BWN_TXPWR_RES_DONE);
}

void
bwn_phy_ht_set_txpwr(struct bwn_mac *mac)
{
}

void
bwn_phy_ht_task_15s(struct bwn_mac *mac)
{
}

void
bwn_phy_ht_task_60s(struct bwn_mac *mac)
{
}
