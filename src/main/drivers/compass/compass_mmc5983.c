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

#define MMC5983_REG_ID 0x2F
#define MMC5983_ID_VAL 0x30

static bool mmc5983Init(magDev_t *magDev)
{
    bool ack = true;
    extDevice_t *dev = &magDev->dev;

    busDeviceRegister(dev);

    ack = ack && busWriteRegister(dev, MMC5983_REG_INTERNAL_CTRL0, MMC5983_SET);
    ack = ack && busWriteRegister(dev, MMC5983_REG_INTERNAL_CTRL2, MMC5983_CMF_100 | MMC5983_MODE_MEASURE_CONTINUOUS | MMC5983_PRD_100 | MMC5983_MODE_AUTOSET);
    ack = ack && busWriteRegister(dev, MMC5983_REG_INTERNAL_CTRL0, MMC5983_TM_M);

    if (!ack) {
        return false;
    }

    magDev->magOdrHz = 100; // MMC5983_ODR_100HZ
    return true;
}

static bool mmc5983Read(magDev_t *magDev, int16_t *magData)
{
    static uint8_t buf[6];
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
            magData[X] = (int16_t)(buf[1] << 8 | buf[0]) / 8;
            magData[Y] = (int16_t)(buf[3] << 8 | buf[2]) / 8;
            magData[Z] = (int16_t)(buf[5] << 8 | buf[4]) / 8;

            state = STATE_WAIT_DRDY;

            // Indicate that new data is required
            status = 0;

            return true;
    }

    return false;
}

bool mmc5983Detect(magDev_t *magDev)
{

    extDevice_t *dev = &magDev->dev;

    if (dev->bus->busType == BUS_TYPE_I2C) {
        dev->busType_u.i2c.address = MMC5983_MAG_I2C_ADDRESS;
    }

    // Must write reset first  - don't care about the result
    busWriteRegister(dev, MMC5983_REG_INTERNAL_CTRL0, MMC5983_RST);
    delay(20);

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
