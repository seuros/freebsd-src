/*-
 * SPDX-License-Identifier: GPL-2.0
 *
 * Copyright (c) 2011 Rafal Milecki <zajec5@gmail.com>
 * Copyright (c) 2025 Seuros (FreeBSD port)
 *
 * HT-PHY tables header for BCM4331.
 * Ported from Linux b43 driver.
 */

#ifndef _IF_BWN_PHY_HT_TABLES_H_
#define _IF_BWN_PHY_HT_TABLES_H_

/* Forward declaration */
struct bwn_mac;

/* Table offset macros - encoding table ID and element size */
#define	B43_HTTAB_TYPEMASK	0xF0000000
#define	B43_HTTAB_8BIT		0x10000000
#define	B43_HTTAB_16BIT		0x20000000
#define	B43_HTTAB_32BIT		0x30000000

#define	B43_HTTAB8(table, offset)	(((table) << 10) | (offset) | B43_HTTAB_8BIT)
#define	B43_HTTAB16(table, offset)	(((table) << 10) | (offset) | B43_HTTAB_16BIT)
#define	B43_HTTAB32(table, offset)	(((table) << 10) | (offset) | B43_HTTAB_32BIT)

/* Function prototypes */
void	bwn_phy_ht_tables_init(struct bwn_mac *mac);
void	bwn_phy_ht_tables_init_late(struct bwn_mac *mac);
void	bwn_httab_write(struct bwn_mac *mac, uint32_t offset, uint32_t value);
void	bwn_httab_write_bulk(struct bwn_mac *mac, uint32_t offset,
	    unsigned int nr_elements, const void *data);
uint32_t bwn_httab_read(struct bwn_mac *mac, uint32_t offset);

/* Late-init table exported for external use */
extern const uint32_t bwn_httab_0x1a_0xc0_late[];
#define	BWN_HTTAB_1A_C0_LATE_SIZE	128

#endif /* _IF_BWN_PHY_HT_TABLES_H_ */
