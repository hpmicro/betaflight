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

#include <stdbool.h>
#include <stdint.h>

#include <math.h>

#include "platform.h"

#ifdef USE_MAG_MMC5983

#include "common/axis.h"
#include "common/maths.h"
#include "common/utils.h"

#include "drivers/bus.h"
#include "drivers/bus_i2c.h"
#include "drivers/bus_i2c_busdev.h"
#include "drivers/sensor.h"
#include "drivers/time.h"

#include "compass.h"
#include "compass_mmc5983.h"

#define MMC5983_MAG_I2C_ADDRESS 0x30

// Registers
#define MMC5983_REG_INTERNAL_CTRL0 0x09
#define MMC5983_REG_INTERNAL_CTRL1 0x0A
#define MMC5983_REG_INTERNAL_CTRL2 0x0B
#define MMC5983_REG_INTERNAL_CTRL3 0x0C

// data output rates for 5883L
#define MMC5983_ODR_10HZ  (0x00 << 2)
#define MMC5983_ODR_50HZ  (0x01 << 2)
#define MMC5983_ODR_100HZ (0x02 << 2)
#define MMC5983_ODR_200HZ (0x03 << 2)
// Sensor operation modes
#define MMC5983_MODE_STANDBY    0x00
#define MMC5983_MODE_MEASURE_CONTINUOUS (0x01 << 3)
#define MMC5983_MODE_AUTOSET (0x01 << 7)

#define MMC5983_RNG_2G (0x00 << 4)
#define MMC5983_RNG_8G (0x01 << 4)
#define MMC5983_RNG_2G (0x00 << 4)
#define MMC5983_RNG_8G (0x01 << 4)

#define MMC5983_CMF_1000 (0x7)
#define MMC5983_CMF_200 (0x6)
#define MMC5983_CMF_100 (0x5)
#define MMC5983_CMF_50  (0x4)

#define MMC5983_PRD_1000 (0x7 << 4)
#define MMC5983_PRD_200 (0x6 << 4)
#define MMC5983_PRD_100 (0x5 << 4)
#define MMC5983_PRD_50  (0x4 << 4)

#define MMC5983_RST 0x10
#define MMC5983_SET 0x08
#define MMC5983_TM_M 0x01

#define MMC5983_REG_DATA_OUTPUT_X 0x00
#define MMC5983_REG_DATA_UNLOCK 0x05
#define MMC5983_REG_STATUS 0x08
#define MMC5983_REG_STATUS_MDONE      0x01
#define MMC5983_REG_STATUS_TDONE      0x02
#define MMC5983_REG_STATUS_OTP_READ   0x010

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


#define MMC5983_REG_ID 0x2F
#define MMC5983_ID_VAL 0x30

/*********************************************************************************
* decription: Continuous mode configuration with auto set and reset
*********************************************************************************/
bool MMC5983_Continuous_Mode_With_Auto_SR(magDev_t *magDev, uint8_t bandwith, uint8_t sampling_rate)
{
    extDevice_t *dev = &magDev->dev;
    bool ack = true;
	/* Write reg 0x0A */
	/* Set BW<1:0> = bandwith 
		BW1	BW0	Measurement Time	Bandwidth
		0	0		8ms				100Hz
		0	1		4ms				200Hz
		1	0		2ms				400Hz
		1	1		0.5ms			800Hz
	*/
    ack = ack && busWriteRegister(dev, MMC5983_REG_CTRL1, bandwith);
	
	/* Write reg 0x09 */
	/* Set Auto_SR_en bit '1', Enable the function of automatic set/reset */
    ack = ack && busWriteRegister(dev, MMC5983_REG_CTRL0, MMC5983_CMD_AUTO_SR_EN);

	/* Write reg 0x0B */
	/* Set Cmmm_en bit '1', Enable the continuous mode */	
	/* Set CM_Freq<2:0> = sampling_rate 
				001				1 Hz
				010				10 Hz
				011				20 Hz
				100				50 Hz
				101				100 Hz
				110				200 Hz
				111				1000 Hz
	*/
    ack = ack && busWriteRegister(dev, MMC5983_REG_CTRL2, MMC5983_CMD_CMM_EN|sampling_rate);
	
	return ack;
}

static bool mmc5983Init(magDev_t *magDev)
{
    bool ack = true;
    extDevice_t *dev = &magDev->dev;

    busDeviceRegister(dev);

	/*Work mode setting*/
	ack = MMC5983_Continuous_Mode_With_Auto_SR(magDev, MMC5983_CMD_BW00, MMC5983_CMD_CM_FREQ_50HZ);
    if (!ack) {
        return false;
    }

    magDev->magOdrHz = 100; // MMC5983_ODR_100HZ
    return true;
}

static bool mmc5983Read(magDev_t *magDev, int16_t *magData)
{
    static uint8_t buf[7];
	uint32_t data18bit[3] = {0};
    float mag_out[3];
    static uint8_t status = 0; // request status on first read
    static enum {
        STATE_WAIT_DRDY,
        STATE_READ,
    } state = STATE_WAIT_DRDY;

    extDevice_t *dev = &magDev->dev;

    switch (state) {
        default:
        case STATE_WAIT_DRDY:
            if (status & MMC5983_REG_STATUS_MDONE) {
                // New data is available
                if (busReadRegisterBufferStart(dev, MMC5983_REG_DATA_OUTPUT_X, buf, sizeof(buf))) {
                    state = STATE_READ;
                }
            } else {
                // Read status register to check for data ready - status will be untouched if read fails
                busReadRegisterBufferStart(dev, MMC5983_REG_STATUS, &status, sizeof(status));
            }
            return false;

        case STATE_READ:
            data18bit[0] = (uint32_t)(buf[0]<<10 | buf[1]<<2 | (buf[6]&0xC0)>>6);
            data18bit[1] = (uint32_t)(buf[2]<<10 | buf[3]<<2 | (buf[6]&0x30)>>4);
            data18bit[2] = (uint32_t)(buf[4]<<10 | buf[5]<<2 | (buf[6]&0x0C)>>2);
            mag_out[0] = ((float)data18bit[0] - MMC5983_18BIT_OFFSET)/MMC5983_18BIT_SENSITIVITY; 
            mag_out[1] = ((float)data18bit[1] - MMC5983_18BIT_OFFSET)/MMC5983_18BIT_SENSITIVITY;
            mag_out[2] = ((float)data18bit[2] - MMC5983_18BIT_OFFSET)/MMC5983_18BIT_SENSITIVITY;

            magData[X] = mag_out[0] * 2048;
            magData[Y] = mag_out[1] * 2048;
            magData[Z] = mag_out[2] * 2048;

            state = STATE_WAIT_DRDY;

            // Indicate that new data is required
            status = 0;

            return true;
    }

    return false;
}

bool mmc5983Detect(magDev_t *magDev)
{
    uint8_t status = 0;
    extDevice_t *dev = &magDev->dev;

    if (dev->bus->busType == BUS_TYPE_I2C) {
        dev->busType_u.i2c.address = MMC5983_MAG_I2C_ADDRESS;
    }

    // Must write reset first  - don't care about the result
    busWriteRegister(dev, MMC5983_REG_INTERNAL_CTRL1, MMC5983_CMD_SW_RST);
    delay(50);
    busReadRegisterBufferStart(dev, MMC5983_REG_STATUS, &status, sizeof(status));
	if((status & MMC5983_OTP_READ_DONE) != MMC5983_OTP_READ_DONE) {
        return false;
    }

    uint8_t sig = 0;
    bool ack = busReadRegisterBuffer(dev, MMC5983_REG_ID, &sig, 1);
    if (ack && sig == MMC5983_ID_VAL) {
        // Should be in standby mode after soft reset and sensor is really present
        // Reading ChipID of 0xFF alone is not sufficient to be sure the MMC is present

        magDev->init = mmc5983Init;
        magDev->read = mmc5983Read;
        return true;
    }

    return false;
}
#endif
