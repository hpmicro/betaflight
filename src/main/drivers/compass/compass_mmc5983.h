/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include "drivers/io_types.h"

#define MMC5983_7BITI2C_ADDRESS		0x30

#define MMC5983_PRODUCT_ID			0x30

#define MMC5983_REG_DATA			0x00
#define MMC5983_REG_TEMP			0x07
#define MMC5983_REG_STATUS			0x08
#define MMC5983_REG_CTRL0			0x09
#define MMC5983_REG_CTRL1			0x0A
#define MMC5983_REG_CTRL2			0x0B
#define MMC5983_REG_CTRL3			0x0C
#define MMC5983_REG_PRODUCTID1		0x2F

/* Bit definition for status register 0x08 */
#define MMC5983_MM_DONE				0x01
#define MMC5983_MT_DONE				0x02
#define MMC5983_OTP_READ_DONE		0x10

/* Bit definition for control register 0 0x09 */
#define MMC5983_CMD_TMM				0x01
#define MMC5983_CMD_TMT         	0x02
#define MMC5983_CMD_INT_MD_EN		0x04
#define MMC5983_CMD_SET				0x08
#define MMC5983_CMD_RESET			0x10
#define MMC5983_CMD_AUTO_SR_EN		0x20
#define MMC5983_CMD_OTP_READ		0x40

/* Bit definition for control register 1 0x0A */
#define MMC5983_CMD_BW00			0x00
#define MMC5983_CMD_BW01			0x01
#define MMC5983_CMD_BW10			0x02
#define MMC5983_CMD_BW11			0x03
#define MMC5983_CMD_X_INHIBIT		0x04
#define MMC5983_CMD_Y_INHIBIT		0x08
#define MMC5983_CMD_Z_INHIBIT		0x10
#define MMC5983_CMD_SW_RST			0x80

/* Bit definition for control register 2 0x0B */
#define MMC5983_CMD_CM_FREQ_OFF		0x00
#define MMC5983_CMD_CM_FREQ_1HZ		0x01
#define MMC5983_CMD_CM_FREQ_10HZ	0x02
#define MMC5983_CMD_CM_FREQ_20HZ	0x03
#define MMC5983_CMD_CM_FREQ_50HZ	0x04
#define MMC5983_CMD_CM_FREQ_100HZ	0x05
#define MMC5983_CMD_CM_FREQ_200HZ	0x06
#define MMC5983_CMD_CM_FREQ_1000HZ	0x07
#define MMC5983_CMD_CMM_EN			0x08

#define MMC5983_CMD_PART_SET1		0x00
#define MMC5983_CMD_PART_SET25		0x10
#define MMC5983_CMD_PART_SET75		0x20
#define MMC5983_CMD_PART_SET100		0x30
#define MMC5983_CMD_PART_SET250		0x40
#define MMC5983_CMD_PART_SET500		0x50
#define MMC5983_CMD_PART_SET1000	0x60
#define MMC5983_CMD_PART_SET2000	0x70
#define MMC5983_CMD_EN_PART_SET		0x80

//18-bit mode, null field output (32768)
#define	MMC5983_16BIT_OFFSET		32768
#define	MMC5983_16BIT_SENSITIVITY	4096

#define	MMC5983_18BIT_OFFSET		131072
#define	MMC5983_18BIT_SENSITIVITY	16384

#define MMC5983_T_ZERO				(-75)
#define MMC5983_T_SENSITIVITY		(0.8)	
bool mmc5983Detect(magDev_t *magDev);

