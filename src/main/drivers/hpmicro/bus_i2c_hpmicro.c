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
#include <stdlib.h>
#include <string.h>

#include "platform.h"

#if defined(USE_I2C) && !defined(SOFT_I2C)

#include "drivers/io.h"
#include "drivers/time.h"
#include "drivers/nvic.h"
#include "drivers/rcc.h"

#include "drivers/bus_i2c.h"
#include "drivers/bus_i2c_impl.h"
#include "drivers/bus_i2c_utils.h"
#include "hpm_i2c_drv.h"
#include "hpm_soc.h"
#include "hpm_clock_drv.h"
#include "hpm_batt_iomux.h"
#include "hpm_pmic_iomux.h"

#define TEST_TRANSFER_DATA_IN_BYTE  (128U)
uint8_t rx_buff[TEST_TRANSFER_DATA_IN_BYTE];
uint8_t tx_buff[TEST_TRANSFER_DATA_IN_BYTE];
uint32_t sent_data_count;
uint32_t received_data_count;
volatile bool i2c_receive_complete;
volatile bool i2c_transmit_complete;
static void i2c_er_handler(I2CDevice device);
static void i2c_ev_handler(I2CDevice device);

#ifdef STM32F4
#define IOCFG_I2C_PU IO_CONFIG(GPIO_Mode_AF, 0, GPIO_OType_OD, GPIO_PuPd_UP)
#define IOCFG_I2C    IO_CONFIG(GPIO_Mode_AF, 0, GPIO_OType_OD, GPIO_PuPd_NOPULL)
#else // STM32F4
#define IOCFG_I2C   IOCFG_AF_OD
#endif

const i2cHardware_t i2cHardware[I2CDEV_COUNT] = {
#ifdef USE_I2C_DEVICE_1
    {
        .device = I2CDEV_1,
        .reg = HPM_I2C0,
        .sclPins = {
            I2CPINDEF(PA6, IOC_PAD_FUNC_CTL_LOOP_BACK_MASK | IOC_PA06_FUNC_CTL_I2C0_SCL, -1),
            I2CPINDEF(PB11, IOC_PAD_FUNC_CTL_LOOP_BACK_MASK | IOC_PB11_FUNC_CTL_I2C0_SCL, -1),
            I2CPINDEF(PD1, IOC_PAD_FUNC_CTL_LOOP_BACK_MASK | IOC_PD01_FUNC_CTL_I2C0_SCL, -1),
            I2CPINDEF(PE15, IOC_PAD_FUNC_CTL_LOOP_BACK_MASK | IOC_PE15_FUNC_CTL_I2C0_SCL, -1),
            I2CPINDEF(PF5, IOC_PAD_FUNC_CTL_LOOP_BACK_MASK | IOC_PF05_FUNC_CTL_I2C0_SCL, -1),
            I2CPINDEF(PZ11, IOC_PAD_FUNC_CTL_LOOP_BACK_MASK | IOC_PZ11_FUNC_CTL_I2C0_SCL, BIOC_PZ11_FUNC_CTL_SOC_PZ_11),
        },
        .sdaPins = {
            I2CPINDEF(PA5, IOC_PAD_FUNC_CTL_LOOP_BACK_MASK | IOC_PA05_FUNC_CTL_I2C0_SDA, -1),
            I2CPINDEF(PB10, IOC_PAD_FUNC_CTL_LOOP_BACK_MASK | IOC_PB10_FUNC_CTL_I2C0_SDA, -1),
            I2CPINDEF(PD0, IOC_PAD_FUNC_CTL_LOOP_BACK_MASK | IOC_PD00_FUNC_CTL_I2C0_SDA, -1),
            I2CPINDEF(PE14, IOC_PAD_FUNC_CTL_LOOP_BACK_MASK | IOC_PE14_FUNC_CTL_I2C0_SDA, -1),
            I2CPINDEF(PF8, IOC_PAD_FUNC_CTL_LOOP_BACK_MASK | IOC_PF08_FUNC_CTL_I2C0_SDA, -1),
            I2CPINDEF(PZ10, IOC_PAD_FUNC_CTL_LOOP_BACK_MASK | IOC_PZ10_FUNC_CTL_I2C0_SDA, BIOC_PZ10_FUNC_CTL_SOC_PZ_10),
        },
        .rcc = clock_i2c0,
        .ev_irq = IRQn_I2C0,
        .er_irq = IRQn_I2C0,
    },
#endif
};

i2cDevice_t i2cDevice[I2CDEV_COUNT];

static volatile uint16_t i2cErrorCount = 0;

void I2C1_ER_IRQHandler(void)
{
    i2c_er_handler(I2CDEV_1);
}

void I2C1_EV_IRQHandler(void)
{
    i2c_ev_handler(I2CDEV_1);
}

void I2C2_ER_IRQHandler(void)
{
    i2c_er_handler(I2CDEV_2);
}

void I2C2_EV_IRQHandler(void)
{
    i2c_ev_handler(I2CDEV_2);
}


static bool i2cHandleHardwareFailure(I2CDevice device)
{
    i2cErrorCount++;
    // reinit peripheral + clock out garbage
    i2cInit(device);
    return false;
}

bool i2cWriteBuffer(I2CDevice device, uint8_t addr_, uint8_t reg_, uint8_t len_, uint8_t *data)
{
    hpm_stat_t stat;
    if (device == I2CINVALID || device >= I2CDEV_COUNT) {
        return false;
    }

    I2C_TypeDef *I2Cx = i2cDevice[device].reg;

    if (!I2Cx) {
        return false;
    }

    i2cState_t *state = &i2cDevice[device].state;
    if (state == I2C_START) {
        return false;
    }

    timeUs_t timeoutStartUs = microsISR();

    if (status_success != i2c_master_address_write(I2Cx, addr_, (uint8_t *)&reg_, 1, data, len_)) {
        printf("Master write failed");
        while (1) {
        }
    }
    return true;
}

bool i2cBusy(I2CDevice device, bool *error)
{
    i2cState_t *state = &i2cDevice[device].state;

    if (error) {
        *error = state;
    }
    return state == I2C_START ? true : false;
}

bool i2cWait(I2CDevice device)
{
    i2cState_t *state = &i2cDevice[device].state;
    timeUs_t timeoutStartUs = microsISR();

    while (state == I2C_START) {
        if (cmpTimeUs(microsISR(), timeoutStartUs) >= I2C_TIMEOUT_US) {
            return i2cHandleHardwareFailure(device) && i2cWait(device);
        }
    }

    return true;
}

bool i2cWrite(I2CDevice device, uint8_t addr_, uint8_t reg_, uint8_t data)
{
    return i2cWriteBuffer(device, addr_, reg_, 1, &data) && i2cWait(device);
}

bool i2cReadBuffer(I2CDevice device, uint8_t addr_, uint8_t reg_, uint8_t len, uint8_t* buf)
{
    hpm_stat_t stat;
    if (device == I2CINVALID || device >= I2CDEV_COUNT) {
        return false;
    }

    I2C_TypeDef *I2Cx = i2cDevice[device].reg;
    if (!I2Cx) {
        return false;
    }

    i2cState_t *state = &i2cDevice[device].state;
    if (state == I2C_START) {
        return false;
    }
    
    if (status_success != i2c_master_address_read(I2Cx, addr_, (uint8_t *)&reg_, 1, buf, len)) {
        printf("Master read failed\n");
        while (1) {
        }
    }
    return true;
}

bool i2cRead(I2CDevice device, uint8_t addr_, uint8_t reg_, uint8_t len, uint8_t* buf)
{
    return i2cReadBuffer(device, addr_, reg_, len, buf) && i2cWait(device);
}

static void i2c_er_handler(I2CDevice device)
{
    I2C_TypeDef *I2Cx = i2cDevice[device].hardware->reg;

    i2cState_t *state = &i2cDevice[device].state;

}

void i2c_ev_handler(I2CDevice device)
{
    I2C_TypeDef *I2Cx = i2cDevice[device].hardware->reg;

    i2cState_t *state = &i2cDevice[device].state;

}

void i2cInit(I2CDevice device)
{
    hpm_stat_t stat;
    i2c_config_t config;
    uint32_t freq;
    if (device == I2CINVALID)
        return;

    i2cDevice_t *pDev = &i2cDevice[device];
    const i2cHardware_t *hw = pDev->hardware;
    const IO_t scl = pDev->scl;
    const IO_t sda = pDev->sda;

    if (!hw || IOGetOwner(scl) || IOGetOwner(sda)) {
        return;
    }

    I2C_TypeDef *I2Cx = hw->reg;

    memset(&pDev->state, 0, sizeof(pDev->state));

    IOInit(scl, OWNER_I2C_SCL, RESOURCE_INDEX(device));
    IOInit(sda, OWNER_I2C_SDA, RESOURCE_INDEX(device));

    // Enable RCC
    if (hw->rcc) {
        clock_add_to_group(hw->rcc, 0);
    }

    //FIXME

    //i2c_enable_irq(device, I2C_EVENT_TRANSACTION_COMPLETE | I2C_EVENT_FIFO_EMPTY);
    //intc_m_enable_irq_with_priority(pDev->nirq, 1);

    //i2cUnstick(scl, sda);
    board_i2c_bus_clear(I2Cx);
    // Init pins
    IOConfigGPIOAF(scl, pDev->pullUp ? IOCFG_AF_OD_UP : IOCFG_AF_OD, pDev->sclAF, pDev->sclAF2);
    IOConfigGPIOAF(sda, pDev->pullUp ? IOCFG_AF_OD_UP : IOCFG_AF_OD, pDev->sdaAF, pDev->sdaAF2);


    config.i2c_mode = i2c_mode_fast;
    config.is_10bit_addressing = false;
    freq = clock_get_frequency(hw->rcc);
    stat = i2c_init_master(I2Cx, freq, &config);
    if (stat != status_success) {
        while(1);
    }
}

uint16_t i2cGetErrorCounter(void)
{
    return i2cErrorCount;
}

void i2c_isr(I2C_TypeDef *I2Cx)
{
    volatile uint32_t status, irq;
    status = i2c_get_status(I2Cx);
    irq = i2c_get_irq_setting(I2Cx);

    /* transmit */
    if ((status & I2C_EVENT_FIFO_EMPTY) && (irq & I2C_EVENT_FIFO_EMPTY)) {
        while (!i2c_fifo_is_full(I2Cx)) {
            i2c_write_byte(I2Cx, tx_buff[sent_data_count++]);
            if (sent_data_count == TEST_TRANSFER_DATA_IN_BYTE) {
                i2c_disable_irq(I2Cx, I2C_EVENT_FIFO_EMPTY);
                break;
            }
        }
    }

    /* receive */
    if (status & I2C_EVENT_FIFO_FULL) {
        while (!i2c_fifo_is_empty(I2Cx)) {
            rx_buff[received_data_count++] = i2c_read_byte(I2Cx);
        }

        if (received_data_count == TEST_TRANSFER_DATA_IN_BYTE) {
            i2c_disable_irq(I2Cx, I2C_EVENT_FIFO_FULL);
        }
    }

    /* complete */
    if (status & I2C_EVENT_TRANSACTION_COMPLETE) {
        if (I2C_DIR_MASTER_READ == i2c_get_direction(I2Cx)) {
            while ((!i2c_fifo_is_empty(I2Cx)) && (received_data_count < TEST_TRANSFER_DATA_IN_BYTE)) {
                rx_buff[received_data_count++] = i2c_read_byte(I2Cx);
                if (received_data_count == TEST_TRANSFER_DATA_IN_BYTE) {
                    i2c_disable_irq(I2Cx, I2C_EVENT_FIFO_FULL);
                    break;
                }
            }
            i2c_receive_complete = true;
        } else {
            i2c_transmit_complete = true;
        }
        i2c_disable_irq(I2Cx, I2C_EVENT_TRANSACTION_COMPLETE);
        i2c_clear_status(I2Cx, I2C_EVENT_TRANSACTION_COMPLETE);
    }
}

void i2c0_isr(void)
{
    i2c_isr(HPM_I2C0);
}

SDK_DECLARE_EXT_ISR_M(IRQn_I2C0, i2c0_isr)
void i2c1_isr(void)
{
    i2c_isr(HPM_I2C1);
}

SDK_DECLARE_EXT_ISR_M(IRQn_I2C1, i2c1_isr)
void i2c2_isr(void)
{
    i2c_isr(HPM_I2C2);
}

SDK_DECLARE_EXT_ISR_M(IRQn_I2C2, i2c2_isr)
#endif
