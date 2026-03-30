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

/*
 * Author: Dominic Clifton
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "platform.h"

#if defined(USE_GYRO_SPI_MIC6200)

#include "common/axis.h"
#include "common/utils.h"
#include "build/debug.h"

#include "drivers/accgyro/accgyro.h"
#include "drivers/accgyro/accgyro_mpu.h"
#include "drivers/accgyro/accgyro_spi_mic6200.h"
#include "drivers/bus_spi.h"
#include "drivers/exti.h"
#include "drivers/io.h"
#include "drivers/sensor.h"
#include "drivers/time.h"

#include "sensors/gyro.h"

// Allows frequency to be set from the compile line EXTRA_FLAGS by adding e.g.
// -D'MIC6200_CLOCK=12000000'. If using the configurator this simply becomes
// MIC6200_CLOCK=12000000 in the custom settings text box.
#ifndef MIC6200_CLOCK
// Default: 10 MHz max SPI frequency
#define MIC6200_MAX_SPI_CLK_HZ 10000000
#else
// Use the supplied value
#define MIC6200_MAX_SPI_CLK_HZ MIC6200_CLOCK
#endif

#define MIC6200_BANK_SELECT0         0
#define MIC6200_BANK_SELECT1         1
#define MIC6200_BANK_SELECT2         2
#define MIC6200_CHIP_VER_CONST       0x02

#define MIC6200_RA_REG_BANK_SEL      0xFF
#define MIC6200_RA_REG_VER           0x01
#define MIC6200_RA_DEVICE_STATUS1    0X02
#define MIC6200_RA_DEVICE_STATUS2    0X03
#define MIC6200_RA_INT_STATUS1        0x20
#define MIC6200_RA_INT_STATUS2        0x21
#define MIC6200_RA_DEVICE_STATUS3    0x24
#define MIC6200_RA_INT_PAD_CONTROL   0x2D
#define MIC6200_RA_GPIO_CONTROL      0x2E
#define MIC6200_RA_GYRO_XOUT_LSB     0x08
#define MIC6200_RA_GYRO_XOUT_MSB     0x09
#define MIC6200_RA_GYRO_YOUT_LSB     0x0A
#define MIC6200_RA_GYRO_YOUT_MSB     0x0B
#define MIC6200_RA_GYRO_ZOUT_LSB     0x0C
#define MIC6200_RA_GYRO_ZOUT_MSB     0x0D
#define MIC6200_RA_ACCEL_XOUT_LSB    0x0E
#define MIC6200_RA_ACCEL_XOUT_MSB    0x0F
#define MIC6200_RA_ACCEL_YOUT_LSB    0x10
#define MIC6200_RA_ACCEL_YOUT_MSB    0x11
#define MIC6200_RA_ACCEL_ZOUT_LSB    0x12
#define MIC6200_RA_ACCEL_ZOUT_MSB    0x13
#define MIC6200_RA_TEMP_OUT          0x1A
#define MIC6200_RA_POWER_CTRL        0x23
#define MIC6200_RA_DEVICE_STATUS3    0x24
#define MIC6200_RA_SYS_OSC_CTRL      0x39
#define MIC6200_RA_GYRO_ODR_DIV      0x3A
#define MIC6200_RA_XL_ODR_DIV        0x3B
#define MIC6200_RA_GYRO_XL_ODR_CTRL  0x3C
#define MIC6200_RA_PWR_MGMT0         0X40
#define MIC6200_RA_CTRL_CH_EN        0X41
#define MIC6200_RA_CTRL_OSR          0X42
#define MIC6200_RA_CTRL_GYRO_OPT     0X43
#define MIC6200_RA_CTRL_XL_OPT       0X44
#define MIC6200_RA_FILETER_CTRL      0X45
#define MIC6200_RA_INT_GYRO_SRC      0x46
#define MIC6200_RA_INT_XL_SRC        0x47
#define MIC6200_RA_INT_TEMP_SRC      0x48
#define MIC6200_RA_INT_THS_1         0x49
#define MIC6200_RA_INT_THS_2         0x4A
#define MIC6200_RA_INT_THS_3         0x4B
#define MIC6200_RA_GYRO_DRIVE_CTRL   0x4B
#define MIC6200_RA_GYRO_DRIVE_CTRL_2 0x4C
#define MIC6200_RA_GYRO_DRIVE_CTRL_3 0x4D
#define MIC6200_RA_GYRO_DRIVE_CTRL_4 0x4E
#define MIC6200_RA_GYRO_DRIVE_CTRL_5 0x4F
#define MIC6200_RA_GYRO_DRIVE_CTRL_6 0x50
#define MIC6200_RA_GYRO_DRIVE_CTRL_7 0x51
#define MIC6200_RA_GYRO_DRIVE_CTRL_8 0x52
#define MIC6200_RA_GYRO_DRIVE_CTRL_9 0x53
#define MIC6200_RA_GYRO_DRIVE_CTRL_10 0x54
#define MIC6200_RA_GYRO_DRIVE_CTRL_11 0x55
#define MIC6200_RA_GYRO_DRIVE_CTRL_12 0x56
#define MIC6200_RA_GYRO_DRIVE_CTRL_13 0x57

#define MIC6200_RA_XL_LPF_COEFF_BASE      0x74
#define MIC6200_RA_XL_LPF_COEFF_0         0x74
#define MIC6200_RA_XL_LPF_COEFF_1         0x75
#define MIC6200_RA_XL_LPF_COEFF_2         0x76
#define MIC6200_RA_XL_LPF_COEFF_3         0x77
#define MIC6200_RA_XL_LPF_COEFF_4         0x78
#define MIC6200_RA_XL_LPF_COEFF_5         0x79
#define MIC6200_RA_XL_LPF_COEFF_6         0x7A
#define MIC6200_RA_XL_LPF_COEFF_7         0x7B
#define MIC6200_RA_XL_LPF_COEFF_8         0x7C
#define MIC6200_RA_XL_LPF_COEFF_9         0x7D
#define MIC6200_RA_MOT_DEL_THR0      0XA0
#define MIC6200_RA_MOT_DEL_THR1      0XA1
#define MIC6200_RA_MOT_DUR_THR       0XA2
#define MIC6200_RA_MOT_CTRL0         0XA3
#define MIC6200_RA_MOT_CTRL1         0XA4
#define MIC6200_RA_MOT_CTRL2         0XA5
#define MIC6200_RA_TILT_THR_LSB      0XA6
#define MIC6200_RA_TILT_THR_MSB      0XA7
#define MIC6200_RA_TILT_DEBOUNCE     0XA8
#define MIC6200_RA_NOMOT_THRE_LSB    0XA9
#define MIC6200_RA_NOMOT_THRE_MSB    0XAA
#define MIC6200_RA_NOMOT_DEBOUNCE    0XAB
#define MIC6200_RA_SHK_THR_LSB       0XAC
#define MIC6200_RA_SHK_THR_MSB       0XAD
#define MIC6200_RA_PK_P2P_THR_LSB    0XAE
#define MIC6200_RA_PK_P2P_THR_MSB    0XAF
#define MIC6200_RA_TAP_EV_THR_LSB    0XB0
#define MIC6200_RA_TAP_EV_THR_MSB    0XB1
#define MIC6200_RA_TAP_SHOCK_DUR     0XB2
#define MIC6200_RA_TAP_QUIET_DUR     0XB3
#define MIC6200_RA_INT_AUTO_CLEAR    0xF1

#define MIC6200_RA_TAP_LATENCY_DURATION     0xB4   // 5.3.15 - TAP Latency Duration Register
#define MIC6200_RA_TAP_MASK                 0xB5   // 5.3.15 - Tap Mask (相邻字节)

#define MIC6200_RA_FREEFALL_THRESHOLD_L     0xB6   // 5.3.16 - Freefall Threshold Register (低字节)
#define MIC6200_RA_FREEFALL_THRESHOLD_H     0xB7   // 5.3.16 - Freefall Threshold Register (高字节)

#define MIC6200_RA_FREEFALL_DURATION        0xB8   // 5.3.17 - Freefall Duration Register

#define MIC6200_RA_SIX_DEGREES_HIGH_THRESH_L 0xB9  // 5.3.18 - Six Degrees High Threshold Register (低字节)
#define MIC6200_RA_SIX_DEGREES_HIGH_THRESH_H 0xBA  // 5.3.18 - Six Degrees High Threshold Register (高字节)

#define MIC6200_RA_SIX_DEGREES_LOW_THRESH_L  0xBB  // 5.3.19 - Six Degrees Low Threshold Register (低字节)
#define MIC6200_RA_SIX_DEGREES_LOW_THRESH_H  0xBC  // 5.3.19 - Six Degrees Low Threshold Register (高字节)

#define MIC6200_RA_SIX_DEGREE_DURATION      0xBD   // 5.3.20 - Six Degree Duration Register

#define MIC6200_RA_SIX_DEGREES_STATUS       0xBE   // 5.3.21 - Six Degrees Status Register

#define MIC6200_RA_FIFO_CONTROL_0         0x80   // FIFO Control Register 0
#define MIC6200_RA_FIFO_CONTROL_1         0x81   // FIFO Control Register 1
#define MIC6200_RA_FIFO_CONTROL_2         0x82   // FIFO Control Register 2

#define MIC6200_RA_FIFO_CONTROL_3         0x83   // FIFO Control Register 3
#define MIC6200_RA_FIFO_THRESHOLD         0x84   // FIFO Threshold (与 0x83 相邻，可能为高字节或独立配置)

#define MIC6200_RA_FIFO_BURST_LENGTH      0x85   // FIFO Burst Length Register

#define MIC6200_RA_FIFO_CONTROL_4         0x86   // FIFO Control Register 4

#define MIC6200_RA_FIFO_STATUS            0x87   // FIFO Status Register

#define MIC6200_RA_FIFO_SAMPLE_COUNT_L    0x88   // FIFO Sample Count (低字节)
#define MIC6200_RA_FIFO_SAMPLE_COUNT_H    0x89   // FIFO Sample Count (高字节)

#define MIC6200_RA_FIFO_READ_POINTER_L    0x8A   // FIFO Read Pointer (低字节)
#define MIC6200_RA_FIFO_READ_POINTER_H    0x8B   // FIFO Read Pointer (高字节)

#define MIC6200_RA_FIFO_WRITE_POINTER_L   0x8C   // FIFO Write Pointer (低字节)
#define MIC6200_RA_FIFO_WRITE_POINTER_H   0x8D   // FIFO Write Pointer (高字节)

#define MIC6200_RA_FIFO_NEXT_DATA_TYPE_L  0x8E   // FIFO Next Data Type (低字节)
#define MIC6200_RA_FIFO_NEXT_DATA_TYPE_H  0x8F   // FIFO Next Data Type (高字节)

// FIFO 数据输出寄存器块 (连续 6 个字节：0x90 ~ 0x95)
#define MIC6200_RA_FIFO_DATA_OUT_BASE     0x90   // 起始地址
#define MIC6200_RA_FIFO_DATA_OUT_0        0x90   // FIFO Data Out Register 0
#define MIC6200_RA_FIFO_DATA_OUT_1        0x91   // FIFO Data Out Register 1
#define MIC6200_RA_FIFO_DATA_OUT_2        0x92   // FIFO Data Out Register 2
#define MIC6200_RA_FIFO_DATA_OUT_3        0x93   // FIFO Data Out Register 3
#define MIC6200_RA_FIFO_DATA_OUT_4        0x94   // FIFO Data Out Register 4
#define MIC6200_RA_FIFO_DATA_OUT_5        0x95   // FIFO Data Out Register 5
// ============================================================================
// MIC6200 中断控制寄存器 (Page 0)
// ============================================================================

#define MIC6200_RA_INTERRUPT_STATUS_1     0x20   // Interrupt Register 1 - 状态标志位
#define MIC6200_RA_INTERRUPT_STATUS_2     0x21   // Interrupt Register 2 - 状态标志位

#define MIC6200_RA_INTERRUPT_CTRL_0       0x2A   // Interrupt Control Register 0 - 使能控制
#define MIC6200_RA_INTERRUPT_CTRL_1       0x2B   // Interrupt Control Register 1 - 使能控制
#define MIC6200_RA_INTERRUPT_CTRL_2       0x2C   // Interrupt Control Register 2 - 使能控制

#define MIC6200_RA_INT_PAD_CONTROL        0x2D   // INT Pad Control Register - 引脚极性/模式
#define MIC6200_RA_GPIO_CONTROL           0x2E   // GPIO Control Register - 通用IO配置


// ============================================================================
// MIC6200 滤波器系数寄存器 (Page 0)
// ============================================================================

// Gyro LPF Coefficients: 连续10个字节 (0x60 ~ 0x69)
#define MIC6200_RA_GYRO_LPF_COEFF_BASE    0x60   // 起始地址
#define MIC6200_RA_GYRO_LPF_COEFF_0       0x60
#define MIC6200_RA_GYRO_LPF_COEFF_1       0x61
#define MIC6200_RA_GYRO_LPF_COEFF_2       0x62
#define MIC6200_RA_GYRO_LPF_COEFF_3       0x63
#define MIC6200_RA_GYRO_LPF_COEFF_4       0x64
#define MIC6200_RA_GYRO_LPF_COEFF_5       0x65
#define MIC6200_RA_GYRO_LPF_COEFF_6       0x66
#define MIC6200_RA_GYRO_LPF_COEFF_7       0x67
#define MIC6200_RA_GYRO_LPF_COEFF_8       0x68
#define MIC6200_RA_GYRO_LPF_COEFF_9       0x69

// XL (Accelerometer) LPF Coefficients: 连续10个字节 (0x74 ~ 0x7D)
#define MIC6200_RA_XL_LPF_COEFF_BASE      0x74   // 起始地址
#define MIC6200_RA_XL_LPF_COEFF_0         0x74
#define MIC6200_RA_XL_LPF_COEFF_1         0x75
#define MIC6200_RA_XL_LPF_COEFF_2         0x76
#define MIC6200_RA_XL_LPF_COEFF_3         0x77
#define MIC6200_RA_XL_LPF_COEFF_4         0x78
#define MIC6200_RA_XL_LPF_COEFF_5         0x79
#define MIC6200_RA_XL_LPF_COEFF_6         0x7A
#define MIC6200_RA_XL_LPF_COEFF_7         0x7B
#define MIC6200_RA_XL_LPF_COEFF_8         0x7C
#define MIC6200_RA_XL_LPF_COEFF_9         0x7D

#define MIC6200_PWR_MGMT0_GYRO_ACCEL_MODE_OFF 0x00
#define MIC6200_PWR_MGMT0_MODE_ACCEL_EN       0x20
#define MIC6200_PWR_MGMT0_MODE_GYRO_EN        0x10
#define MIC6200_PWR_MGMT0_MODE_WAKE           0x01


#define MIC6200_INT_ENABLE_BIT_XL_INT         (1 << 1)
#define MIC6200_INT_ENABLE_BIT_GYRO_INT         (1 << 0)
// void dump_registers(const extDevice_t *dev)
// {     
//     for (uint8_t bank = 0; bank < 3; bank++) {
//         spiWriteRegMic6200(dev, MIC6200_RA_REG_BANK_SEL, bank);
//         printf("\n=== Bank %d ===\n", bank);
        
//         for (uint8_t reg = 0x00; reg < 0x80; reg++) {
//             if (reg % 16 == 0) {
//                 printf("\n0x%02X: ", reg);
//             }
//             uint8_t value = spiReadRegMic6200(dev, reg);
//             printf("%02X ", value);
//         }
//         printf("\n");
//     }
    
//     spiWriteRegMic6200(dev, MIC6200_RA_REG_BANK_SEL, 0);

// }
// void dump_registers2(const extDevice_t *dev)
// {
//     spiWriteRegMic6200(dev, MIC6200_RA_REG_BANK_SEL, 0);
//     printf("\n=== Bank %d ===\n", 0);
    
//     for (uint8_t reg = 0x08; reg < 0x14; reg++) {
//         if (reg % 8 == 0) {
//             printf("\n0x%02X: ", reg);
//         }
//         uint8_t value = spiReadRegMic6200(dev, reg);
//         printf("%02X ", value);
//     }
//     printf("\n");

// }
uint8_t mic6200SpiDetect(const extDevice_t *dev)
{
    spiWriteRegMic6200(dev, MIC6200_RA_PWR_MGMT0, 0x00);

    uint8_t icmDetected = MPU_NONE;
    uint8_t attemptsRemaining = 20;
        delay(150);
    spiWriteRegMic6200(dev, MIC6200_RA_PWR_MGMT0, MIC6200_PWR_MGMT0_MODE_WAKE);
        delay(150);
    // dump_registers(dev);
    do {
        delay(150);
        const uint8_t whoAmI = spiReadRegMic6200(dev, MPU_RA_WHO_AM_I_LEGACY);
        switch (whoAmI) {
        case MIC6200_WHO_AM_I_CONST:
            const uint8_t whoAmIVer = spiReadRegMic6200(dev, MIC6200_RA_REG_VER);
            if (whoAmIVer == MIC6200_CHIP_VER_CONST) {
                icmDetected = MIC6200_SPI;
            } else {
                icmDetected = MPU_NONE;
            }
            break;
        default:
            icmDetected = MPU_NONE;
            break;
        }
        if (icmDetected != MPU_NONE) {
            break;
        }
        if (!attemptsRemaining) {
            return MPU_NONE;
        }
    } while (attemptsRemaining--);


    return icmDetected;
}

void mic6200AccInit(accDev_t *acc)
{
    acc->acc_1G = 2048;
}

bool mic6200SpiAccDetect(accDev_t *acc)
{
    switch (acc->mpuDetectionResult.sensor) {
    case MIC6200_SPI:
        break;
    default:
        return false;
    }

    acc->initFn = mic6200AccInit;
    acc->readFn = mpuAccReadSPI;

    return true;
}

static void turnGyroAccOff(const extDevice_t *dev)
{
    spiWriteRegMic6200(dev, MIC6200_RA_PWR_MGMT0, MIC6200_PWR_MGMT0_GYRO_ACCEL_MODE_OFF);
}

// Turn on gyro and acc on in Low Noise mode
static void turnGyroAccOn(const extDevice_t *dev)
{
    spiWriteRegMic6200(dev, MIC6200_RA_PWR_MGMT0, MIC6200_PWR_MGMT0_MODE_ACCEL_EN | MIC6200_PWR_MGMT0_MODE_GYRO_EN | MIC6200_PWR_MGMT0_MODE_WAKE);
    delay(1);
}

static void setUserBank(const extDevice_t *dev, const uint8_t user_bank)
{
    spiWriteRegMic6200(dev, MIC6200_RA_REG_BANK_SEL, user_bank & 7);
}

void MIC6200_config_read_auto_clear(const extDevice_t *dev, bool enable)
{
    if (enable) {
        spiWriteRegMic6200(dev, MIC6200_RA_INT_AUTO_CLEAR, 3);
    } else {
        spiWriteRegMic6200(dev, MIC6200_RA_INT_AUTO_CLEAR, 0);
    }
}

static void MIC6200_EnableInt(const extDevice_t *dev, const uint8_t int_enable)
{
    setUserBank(dev, MIC6200_BANK_SELECT0);
    spiWriteRegMic6200(dev, MIC6200_RA_INTERRUPT_CTRL_1, int_enable);
}

static void MIC6200_Setup(const extDevice_t *dev)
{
    setUserBank(dev, MIC6200_BANK_SELECT0);

    // power domain reset
    spiWriteRegMic6200(dev, MIC6200_RA_REG_BANK_SEL, 0x00);
    spiWriteRegMic6200(dev, MIC6200_RA_PWR_MGMT0, 0x33); // Switch to Standby mode
    spiWriteRegMic6200(dev, MIC6200_RA_POWER_CTRL, 0x80); // reset
    delay(1);
    spiWriteRegMic6200(dev, MIC6200_RA_PWR_MGMT0, 0x33); // Switch to Standby mode
    spiWriteRegMic6200(dev, MIC6200_RA_POWER_CTRL, 0x77); // XL / GYRO / FIFO Power On & RESET
    delay(1);
    spiWriteRegMic6200(dev, MIC6200_RA_POWER_CTRL, 0x07); // XL / GYRO / FIFO Power On
    spiWriteRegMic6200(dev, MIC6200_RA_POWER_CTRL, 0x0F); // XL / GYRO / FIFO Power On, HOLD Disable

    spiWriteRegMic6200(dev, MIC6200_RA_INT_THS_1, 0xC8); // Analog Control 2. CP_VPM_CLK = PLL Clk 64x use xC8 (1.6MHz)
    spiWriteRegMic6200(dev, MIC6200_RA_INT_THS_3, 0x00); // Gyro Drive Control, disable drive loop and PLL/AGC

    spiWriteRegMic6200(dev, MIC6200_RA_GYRO_DRIVE_CTRL_2, 0x0B); // X Axis: Sense Control 1. SCSA_EN, SMIX_EN, SCCSA_VDCIN_CTRL_EN
    spiWriteRegMic6200(dev, MIC6200_RA_GYRO_DRIVE_CTRL_3, 0x0B); // Y Axis: Sense Control 1. SCSA_EN, SMIX_EN, SCCSA_VDCIN_CTRL_EN
    spiWriteRegMic6200(dev, MIC6200_RA_GYRO_DRIVE_CTRL_4, 0x0B); // Z Axis: Sense Control 1. SCSA_EN, SMIX_EN, SCCSA_VDCIN_CTRL_EN

    spiWriteRegMic6200(dev, MIC6200_RA_GYRO_DRIVE_CTRL_8, 0xA7); // Analog Control. Temp sensor Enable, PMU ALDO GYR bias current to minimum 3uA

    spiWriteRegMic6200(dev, MIC6200_RA_GYRO_DRIVE_CTRL_12, 0x5A); // AGC ON

    spiWriteRegMic6200(dev, MIC6200_RA_GYRO_DRIVE_CTRL_13, 0x84); // KP
    spiWriteRegMic6200(dev, 0x5C, 0x00); // High SGAIN count
    spiWriteRegMic6200(dev, MIC6200_RA_GYRO_DRIVE_CTRL_11, 0x50); // High SGAIN count

    // Enable dithering
    spiWriteRegMic6200(dev, 0xE7, 0xAA); // Dithering ampl to 66% for drive, X, Y, Z
    spiWriteRegMic6200(dev, 0x7E, 0x03); // PRBS Control

    // PMU ibias fixed setting
    spiWriteRegMic6200(dev, 0xF4, 0xA2); // SenZ/Y CSA ITRIM 1.25uA
    spiWriteRegMic6200(dev, 0xF5, 0x1C); // PMU ITRIM 3  Max out the dcsa current
    spiWriteRegMic6200(dev, 0xF7, 0x55); // PMU ITRIM 5  X & Y sense GSDM
    spiWriteRegMic6200(dev, 0xF8, 0x55); // PMU ITRIM 6  Z sense GSDM, DRV GSDM

    // Notch filter and selftest
    spiWriteRegMic6200(dev, 0xD8, 0x10); // disable notch filter now
    spiWriteRegMic6200(dev, 0xDC, 0x18); // notch filter decimation 24(minimum)
    spiWriteRegMic6200(dev, 0xDB, 0x3F); // Disable all pilot tone TCO/TCS comp
    spiWriteRegMic6200(dev, 0xE6, 0x0F); // Powerdown dummy

    // For Qmeas
    spiWriteRegMic6200(dev, 0xE1, 0x47);
    spiWriteRegMic6200(dev, 0xE3, 0x7E);
    spiWriteRegMic6200(dev, 0xE0, 0x50);

    spiWriteRegMic6200(dev, MIC6200_RA_REG_BANK_SEL, 0x02);
    spiWriteRegMic6200(dev, 0x9E, 0xF0); // Vctrl output when setpoint exceed upper limit
    spiWriteRegMic6200(dev, MIC6200_RA_PK_P2P_THR_MSB, 0x10); // Vctrl output when setpoint under lower limit
    spiWriteRegMic6200(dev, MIC6200_RA_MOT_DEL_THR1, 0xA4);

    // Qmeas channel mode
    spiWriteRegMic6200(dev, MIC6200_RA_TILT_THR_LSB, 0x20);

    spiWriteRegMic6200(dev, 0xC8, 0x3F); // Sense CSA Slow Clock Select/SCSA RFB Itrim

    spiWriteRegMic6200(dev, MIC6200_RA_REG_BANK_SEL, 0x00);

    spiWriteRegMic6200(dev, 0x57, 0x08); // Trigger Manual of VCTRL
    spiWriteRegMic6200(dev, 0x57, 0x88); // Trigger Manual of VCTRL
    spiWriteRegMic6200(dev, 0x57, 0x08); // Trigger Manual of VCTRL

    spiWriteRegMic6200(dev, MIC6200_RA_POWER_CTRL, 0x00);
    spiWriteRegMic6200(dev, MIC6200_RA_PWR_MGMT0, 0x00);
}

void MIC6200_Read_Sensitivity_Coef(extDevice_t *dev)
{
    uint8_t reg_data = 0;
    uint8_t tmp_data = 0;
    setUserBank(dev, MIC6200_BANK_SELECT1);
    reg_data = spiReadRegMic6200(dev, 0x4A);
    if (tmp_data > 0x08)
    {
      dev->sxz = (int8_t)(tmp_data - 16);
    }
    else
    {
      dev->sxz = (int8_t)(tmp_data);
    }

    tmp_data = (reg_data & 0xF0) >> 4;
    if (tmp_data > 0x08)
    {
      dev->szx = (int8_t)(tmp_data - 16);
    }
    else
    {
      dev->szx = (int8_t)(tmp_data);
    }

    setUserBank(dev, MIC6200_BANK_SELECT0);
}

void  MIC6200_Set_Gyro_Odr(const extDevice_t *dev, int odr)
{
    if (odr != 0)
    {
        uint8_t odr_ctl = 0;
        int odr_prescale = 0;

        setUserBank(dev, MIC6200_BANK_SELECT0);

        odr_ctl = spiReadRegMic6200(dev, 0x3C);
        odr_ctl = odr_ctl & 0xF9;

        odr_prescale = dev->sys_osc / odr;
        if (odr_prescale <= 255)
        {
            odr_ctl = odr_ctl | 0x01;
            spiWriteRegMic6200(dev, 0x3C, odr_ctl);
            spiWriteRegMic6200(dev, 0x3A, (uint8_t)odr_prescale);
        }
        else if (odr_prescale <= (255 * 32))
        {
            odr_ctl = odr_ctl | 0x03;
            spiWriteRegMic6200(dev, 0x3C, odr_ctl);
            odr_prescale = dev->sys_osc / 32 / odr;
            spiWriteRegMic6200(dev, 0x3A, (uint8_t)odr_prescale);
        }
        else if (odr_prescale <= (255 * 256))
        {
            odr_ctl = odr_ctl | 0x05;
            spiWriteRegMic6200(dev, 0x3C, odr_ctl);
            odr_prescale = dev->sys_osc / 256 / odr;
            spiWriteRegMic6200(dev, 0x3A, (uint8_t)odr_prescale);
        }
        else
        {
            odr_ctl = odr_ctl | 0x07;
            spiWriteRegMic6200(dev, 0x3C, odr_ctl);
            odr_prescale = dev->sys_osc / 16384 / odr;
            spiWriteRegMic6200(dev, 0x3A, (uint8_t)odr_prescale);
        }
    }

}

void  MIC6200_Set_Acc_Odr(const extDevice_t *dev, int odr)
{
  if (odr != 0)
  {
    uint8_t odr_ctl = 0;
    int odr_prescale = 0;

    setUserBank(dev, MIC6200_BANK_SELECT0);

    odr_ctl = spiReadRegMic6200(dev, 0x3C);
    odr_ctl = odr_ctl & 0x9F;

    odr_prescale = dev->sys_osc / odr;
    if (odr_prescale <= 255)
    {
      odr_ctl = odr_ctl | 0x10;
      spiWriteRegMic6200(dev, 0x3C, odr_ctl);
      spiWriteRegMic6200(dev, 0x3B, (uint8_t)odr_prescale);
    }
    else if (odr_prescale <= (255 * 32))
    {
      odr_ctl = odr_ctl | 0x30;
      spiWriteRegMic6200(dev, 0x3C, odr_ctl);
      odr_prescale = dev->sys_osc / 32 / odr;
      spiWriteRegMic6200(dev, 0x3B, (uint8_t)odr_prescale);
    }
    else if (odr_prescale <= (255 * 256))
    {
      odr_ctl = odr_ctl | 0x50;
      spiWriteRegMic6200(dev, 0x3C, odr_ctl);
      odr_prescale = dev->sys_osc / 256 / odr;
      spiWriteRegMic6200(dev, 0x3B, (uint8_t)odr_prescale);
    }
    else
    {
      odr_ctl = odr_ctl | 0x70;
      spiWriteRegMic6200(dev, 0x3C, odr_ctl);
      odr_prescale = dev->sys_osc / 16384 / odr;
      spiWriteRegMic6200(dev, 0x3B, (uint8_t)odr_prescale);
    }
  }

}

void mic6200GyroInit(gyroDev_t *gyro)
{
    extDevice_t *dev = &gyro->dev;
    spiSetClkDivisor(dev, spiCalculateDivider(MIC6200_MAX_SPI_CLK_HZ));

    dev->sys_osc = 819000;
    mpuGyroInit(gyro);
    gyro->accDataReg = MIC6200_RA_ACCEL_XOUT_LSB;
    gyro->gyroDataReg = MIC6200_RA_GYRO_XOUT_LSB;

    // Turn off ACC and GYRO so they can be configured
    // See section 12.9 in ICM-42688-P datasheet v1.7
    setUserBank(dev, MIC6200_BANK_SELECT0);
    turnGyroAccOff(dev);
    delay(100);
    spiWriteRegMic6200(dev, MIC6200_RA_INTERRUPT_CTRL_1, 0x01);

    MIC6200_Read_Sensitivity_Coef(dev);
    //MIC6200_Setup
    MIC6200_Setup(dev);
    // Turn on gyro and acc on again so ODR and FSR can be configured
    turnGyroAccOn(dev);

    spiWriteRegMic6200(dev, MIC6200_RA_REG_BANK_SEL, 0x00);

    spiWriteRegMic6200(dev, MIC6200_RA_GYRO_DRIVE_CTRL_6, 0x81);
    spiWriteRegMic6200(dev, MIC6200_RA_PWR_MGMT0, 0x33);
    spiWriteRegMic6200(dev, MIC6200_RA_POWER_CTRL, 0x77);
    delay(1);
    spiWriteRegMic6200(dev, MIC6200_RA_POWER_CTRL, 0x07);
    spiWriteRegMic6200(dev, MIC6200_RA_POWER_CTRL, 0x0F);
    spiWriteRegMic6200(dev, MIC6200_RA_GYRO_DRIVE_CTRL_6, 0x89);

    spiWriteRegMic6200(dev, 0x1D, 0x04);

    spiWriteRegMic6200(dev, 0xF6, 0x29);
    spiWriteRegMic6200(dev, 0xF9, 0xA2);
    spiWriteRegMic6200(dev, 0x53, 0xD3);

    spiWriteRegMic6200(dev, MIC6200_RA_SYS_OSC_CTRL, 0x88);
    MIC6200_Set_Gyro_Odr(dev, 1000);
    MIC6200_Set_Acc_Odr(dev, 250);
    spiWriteRegMic6200(dev, 0x3D, 0x00);
    spiWriteRegMic6200(dev, 0x3E, 0x00);
    spiWriteRegMic6200(dev, 0x3F, 0x00);

    spiWriteRegMic6200(dev, MIC6200_RA_CTRL_CH_EN, 0x00);
    spiWriteRegMic6200(dev, MIC6200_RA_CTRL_OSR, 0x42);
    spiWriteRegMic6200(dev, MIC6200_RA_INT_GYRO_SRC, 0x80);
    spiWriteRegMic6200(dev, MIC6200_RA_CTRL_GYRO_OPT, 0x90);  //2000
    spiWriteRegMic6200(dev, MIC6200_RA_CTRL_XL_OPT, 0x10);  //2G
    spiWriteRegMic6200(dev, MIC6200_RA_INT_XL_SRC, 0x20);
    spiWriteRegMic6200(dev, MIC6200_RA_INT_TEMP_SRC, 0x00);

    spiWriteRegMic6200(dev, MIC6200_RA_GYRO_LPF_COEFF_0, 0x05);
    spiWriteRegMic6200(dev, MIC6200_RA_GYRO_LPF_COEFF_1, 0x0F);
    spiWriteRegMic6200(dev, MIC6200_RA_GYRO_LPF_COEFF_2, 0x0B);
    spiWriteRegMic6200(dev, MIC6200_RA_GYRO_LPF_COEFF_3, 0x1E);
    spiWriteRegMic6200(dev, MIC6200_RA_GYRO_LPF_COEFF_4, 0x05);
    spiWriteRegMic6200(dev, MIC6200_RA_GYRO_LPF_COEFF_5, 0x0F);
    spiWriteRegMic6200(dev, MIC6200_RA_GYRO_LPF_COEFF_6, 0xA1);
    spiWriteRegMic6200(dev, MIC6200_RA_GYRO_LPF_COEFF_7, 0x69);
    spiWriteRegMic6200(dev, MIC6200_RA_GYRO_LPF_COEFF_8, 0xB7);
    spiWriteRegMic6200(dev, MIC6200_RA_GYRO_LPF_COEFF_9, 0x25);

    spiWriteRegMic6200(dev, MIC6200_RA_FILETER_CTRL, 0x11);
    spiWriteRegMic6200(dev, 0xD8, 0x17);

    spiWriteRegMic6200(dev, MIC6200_RA_XL_LPF_COEFF_0, 0x15);
    spiWriteRegMic6200(dev, MIC6200_RA_XL_LPF_COEFF_1, 0x01);
    spiWriteRegMic6200(dev, MIC6200_RA_XL_LPF_COEFF_2, 0x29);
    spiWriteRegMic6200(dev, MIC6200_RA_XL_LPF_COEFF_3, 0x02);
    spiWriteRegMic6200(dev, MIC6200_RA_XL_LPF_COEFF_4, 0x15);
    spiWriteRegMic6200(dev, MIC6200_RA_XL_LPF_COEFF_5, 0x01);
    spiWriteRegMic6200(dev, MIC6200_RA_XL_LPF_COEFF_6, 0xA5);
    spiWriteRegMic6200(dev, MIC6200_RA_XL_LPF_COEFF_7, 0xDC);
    spiWriteRegMic6200(dev, MIC6200_RA_XL_LPF_COEFF_8, 0xF7);
    spiWriteRegMic6200(dev, MIC6200_RA_XL_LPF_COEFF_9, 0x60);

    spiWriteRegMic6200(dev, MIC6200_RA_REG_BANK_SEL, 0x01);
    spiWriteRegMic6200(dev, 0x31, 0x00);

    spiWriteRegMic6200(dev, MIC6200_RA_REG_BANK_SEL, 0x02);

    spiWriteRegMic6200(dev, 0xC0, 0x00);

    spiWriteRegMic6200(dev, MIC6200_RA_REG_BANK_SEL, 0x00);

    spiWriteRegMic6200(dev, 0x54, 0x23);
    spiWriteRegMic6200(dev, 0x54, 0x2B);
    spiWriteRegMic6200(dev, 0x54, 0x23);

    spiWriteRegMic6200(dev, 0x4A, 0x48);
    spiWriteRegMic6200(dev, 0x57, 0x08);
    spiWriteRegMic6200(dev, 0x4B, 0xDD);
    spiWriteRegMic6200(dev, 0x51, 0x85);
    delay(1);

    spiWriteRegMic6200(dev, 0x51, 0x05);
    delay(4);

    spiWriteRegMic6200(dev, MIC6200_RA_REG_BANK_SEL, 0x01);
    spiWriteRegMic6200(dev, 0x31, 0x25);
    spiWriteRegMic6200(dev, MIC6200_RA_REG_BANK_SEL, 0x00);
    spiWriteRegMic6200(dev, 0x57, 0x02);

    spiWriteRegMic6200(dev, MIC6200_RA_PWR_MGMT0, 0x31);
    delay(7);
    spiWriteRegMic6200(dev, 0x4A, 0x40);

    spiWriteRegMic6200(dev, MIC6200_RA_REG_BANK_SEL, 0x00);

    spiWriteRegMic6200(dev, MIC6200_RA_GYRO_DRIVE_CTRL_6, 0x81); // XL continuous, use system clock, set to frontend, otherwise will rail-rail
    spiWriteRegMic6200(dev, MIC6200_RA_PWR_MGMT0, 0x33); // Modes (Ctrl 1). GY + XL ASYNC mode, STANDBY Mode
    spiWriteRegMic6200(dev, MIC6200_RA_POWER_CTRL, 0x77);
    delay(1);
    spiWriteRegMic6200(dev, MIC6200_RA_POWER_CTRL, 0x07);
    spiWriteRegMic6200(dev, MIC6200_RA_POWER_CTRL, 0x0F);
    spiWriteRegMic6200(dev, MIC6200_RA_GYRO_DRIVE_CTRL_6, 0x89); // XL continuous, use PLL clock

    spiWriteRegMic6200(dev, 0x1D, 0x04); // temp sensor clock

    spiWriteRegMic6200(dev, 0xF6, 0x29); // For XL
    spiWriteRegMic6200(dev, 0xF9, 0xA2);
    spiWriteRegMic6200(dev, 0x53, 0xD3); // Drive Sense agc mixer enabled, drv_buf=off and bypassed, issue #41;PMU_ALDO_XL_IB_N=69uA

    spiWriteRegMic6200(dev, MIC6200_RA_SYS_OSC_CTRL, 0x88); // system clock and ODR
    spiWriteRegMic6200(dev, 0x3D, 0x00);
    spiWriteRegMic6200(dev, 0x3E, 0x00);
    spiWriteRegMic6200(dev, 0x3F, 0x00);

    spiWriteRegMic6200(dev, MIC6200_RA_CTRL_CH_EN, 0x00); // Channel Enable (Ctrl 2). Bit7 controls the signal PMU_TSR_PD_O. Disabled by HW in SLEEP mode, becomes available in STANDBY and WAKE modes
    spiWriteRegMic6200(dev, MIC6200_RA_CTRL_OSR, 0x42); // OSR (Ctrl 3) Gyro OSR128, IDR=12.8k/24, XL OSR is 512, XL WTD filter is disabled.
    spiWriteRegMic6200(dev, MIC6200_RA_INT_GYRO_SRC, 0x80); // XL Decimation (Ctrl 7). 0x0: 1 samples, Bit7 = '0', Decimation count = Sinc Order + 1; If Bit7 ='1' Decimation count is power of 2
    spiWriteRegMic6200(dev, MIC6200_RA_INT_XL_SRC, 0x20); // Gyro/XL/TCO (Ctrl 8). Disable XL UPDATE mode, DRV_CMP-auto zero, XL analog PWR ctrl b[2]=1 =>enables the SDM pwr mngmt btwn samples
    spiWriteRegMic6200(dev, MIC6200_RA_INT_TEMP_SRC, 0x00); // Analog Control 1.

    spiWriteRegMic6200(dev, 0x60, 0x05); // Gyro LPF for notch Enable
    spiWriteRegMic6200(dev, 0x61, 0x0F);
    spiWriteRegMic6200(dev, 0x62, 0x0B);
    spiWriteRegMic6200(dev, 0x63, 0x1E);
    spiWriteRegMic6200(dev, 0x64, 0x05);
    spiWriteRegMic6200(dev, 0x65, 0x0F);
    spiWriteRegMic6200(dev, 0x66, 0xA1);
    spiWriteRegMic6200(dev, 0x67, 0x69);
    spiWriteRegMic6200(dev, 0x68, 0xB7);
    spiWriteRegMic6200(dev, 0x69, 0x25);

    spiWriteRegMic6200(dev, MIC6200_RA_FILETER_CTRL, 0x11); // Gyro/XL Filter (Ctrl 6). Enable Gyro/XL LPF
    spiWriteRegMic6200(dev, 0xD8, 0x17); // Enable notch filter

    spiWriteRegMic6200(dev, 0x74, 0x15); // XL LPF
    spiWriteRegMic6200(dev, 0x75, 0x01);
    spiWriteRegMic6200(dev, 0x76, 0x29);
    spiWriteRegMic6200(dev, 0x77, 0x02);
    spiWriteRegMic6200(dev, 0x78, 0x15);
    spiWriteRegMic6200(dev, 0x79, 0x01);
    spiWriteRegMic6200(dev, 0x7A, 0xA5);
    spiWriteRegMic6200(dev, 0x7B, 0xDC);
    spiWriteRegMic6200(dev, 0x7C, 0xF7);
    spiWriteRegMic6200(dev, 0x7D, 0x60);

    spiWriteRegMic6200(dev, MIC6200_RA_REG_BANK_SEL, 0x01);
    spiWriteRegMic6200(dev, 0x31, 0x00); // Drive gain

    spiWriteRegMic6200(dev, MIC6200_RA_REG_BANK_SEL, 0x02);

    spiWriteRegMic6200(dev, 0xC0, 0x00); // XL ibias

    spiWriteRegMic6200(dev, MIC6200_RA_REG_BANK_SEL, 0x00); // Page0

    spiWriteRegMic6200(dev, 0x54, 0x23); // Toggle gain switch count
    spiWriteRegMic6200(dev, 0x54, 0x2B); // Toggle gain switch count, enable AGC LPF
    spiWriteRegMic6200(dev, 0x54, 0x23); // Toggle gain switch count

    spiWriteRegMic6200(dev, 0x4A, 0x48); // VPM = 20V, charge pump clock enable
    spiWriteRegMic6200(dev, 0x57, 0x08); // Manual VCTRL DAC
    spiWriteRegMic6200(dev, 0x4B, 0xDD); // Gyro Drive Control, enable drive loop and PLL/AGC
    spiWriteRegMic6200(dev, 0x51, 0x85); // PLL_OSC_VCO idle, may no need?
    delay(1);

    spiWriteRegMic6200(dev, 0x51, 0x05); // Exit idle mode
    delay(4); // Delay. Generally it need 1ms to 2ms to lock PLL

    spiWriteRegMic6200(dev, MIC6200_RA_REG_BANK_SEL, 0x01); // Page1
    spiWriteRegMic6200(dev, 0x31, 0x25); // DCSA gain: higher gain for 20V device to reach ~3.2um displacement
    spiWriteRegMic6200(dev, MIC6200_RA_REG_BANK_SEL, 0x00); // Page0
    spiWriteRegMic6200(dev, 0x57, 0x02); // AGC_CTRL_REG_2 AGC OSR mode (set to 0x40 for OSR32, 0x00 for OSR64)

    spiWriteRegMic6200(dev, MIC6200_RA_PWR_MGMT0, 0x31); // Modes (Ctrl 1). 6axis Gyro + Accel Wake Mode
    delay(7); // Delay. To DCSA target amplitude need maximum ~12ms(from 0mV), and high gain stage ~2.8ms, here set 10ms as it is not from 0mV start
    spiWriteRegMic6200(dev, 0x4A, 0x40); // Disable the DRV charge pump, it only need at startup stage to speed time
    //NEED FIX, there will be a problem that interrupt will not work if the last data is not read, the interrupt flag will not be cleared when write 1 to the interrupt register
    //While the gpio interrupt may arrive before the spi transfer done interrupt, which will cause the spi transfer not started in the gpio interrupt
    MIC6200_EnableInt(dev, MIC6200_INT_ENABLE_BIT_XL_INT | MIC6200_INT_ENABLE_BIT_GYRO_INT);

    delay(1000);
}

bool mic6200SpiGyroDetect(gyroDev_t *gyro)
{
    switch (gyro->mpuDetectionResult.sensor) {
    case MIC6200_SPI:
        break;
    default:
        return false;
    }

    gyro->initFn = mic6200GyroInit;
    gyro->readFn = mpuGyroReadSPI;

    gyro->scale = GYRO_SCALE_2000DPS;
    return true;
}

#endif // USE_GYRO_SPI_MIC6200
