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
#include <string.h>

#include "platform.h"

#ifdef USE_SPI

#include "build/debug.h"

#include "drivers/bus_spi.h"
#include "drivers/bus_spi_impl.h"
#include "drivers/dma.h"
#include "drivers/exti.h"
#include "drivers/io.h"
#include "drivers/rcc.h"

#include "pg/bus_spi.h"
#ifdef HPMicro
#include "hpm_clock_drv.h"
#endif
const spiHardware_t spiHardware[] = {
#ifdef STM32F4
    {
        .device = SPIDEV_1,
        .reg = SPI1,
        .sckPins = {
            { DEFIO_TAG_E(PA5) },
            { DEFIO_TAG_E(PB3) },
        },
        .misoPins = {
            { DEFIO_TAG_E(PA6) },
            { DEFIO_TAG_E(PB4) },
        },
        .mosiPins = {
            { DEFIO_TAG_E(PA7) },
            { DEFIO_TAG_E(PB5) },
        },
        .af = GPIO_AF_SPI1,
        .rcc = RCC_APB2(SPI1),
    },
    {
        .device = SPIDEV_2,
        .reg = SPI2,
        .sckPins = {
            { DEFIO_TAG_E(PB10) },
            { DEFIO_TAG_E(PB13) },
        },
        .misoPins = {
            { DEFIO_TAG_E(PB14) },
            { DEFIO_TAG_E(PC2) },
        },
        .mosiPins = {
            { DEFIO_TAG_E(PB15) },
            { DEFIO_TAG_E(PC3) },
        },
        .af = GPIO_AF_SPI2,
        .rcc = RCC_APB1(SPI2),
    },
    {
        .device = SPIDEV_3,
        .reg = SPI3,
        .sckPins = {
            { DEFIO_TAG_E(PB3) },
            { DEFIO_TAG_E(PC10) },
        },
        .misoPins = {
            { DEFIO_TAG_E(PB4) },
            { DEFIO_TAG_E(PC11) },
        },
        .mosiPins = {
            { DEFIO_TAG_E(PB5) },
            { DEFIO_TAG_E(PC12) },
        },
        .af = GPIO_AF_SPI3,
        .rcc = RCC_APB1(SPI3),
    },
#endif
#ifdef STM32F7
    {
        .device = SPIDEV_1,
        .reg = SPI1,
        .sckPins = {
            { DEFIO_TAG_E(PA5), GPIO_AF5_SPI1 },
            { DEFIO_TAG_E(PB3), GPIO_AF5_SPI1 },
        },
        .misoPins = {
            { DEFIO_TAG_E(PA6), GPIO_AF5_SPI1 },
            { DEFIO_TAG_E(PB4), GPIO_AF5_SPI1 },
        },
        .mosiPins = {
            { DEFIO_TAG_E(PA7), GPIO_AF5_SPI1 },
            { DEFIO_TAG_E(PB5), GPIO_AF5_SPI1 },
        },
        .rcc = RCC_APB2(SPI1),
        .dmaIrqHandler = DMA2_ST3_HANDLER,
    },
    {
        .device = SPIDEV_2,
        .reg = SPI2,
        .sckPins = {
            { DEFIO_TAG_E(PA9), GPIO_AF5_SPI2 },
            { DEFIO_TAG_E(PB10), GPIO_AF5_SPI2 },
            { DEFIO_TAG_E(PB13), GPIO_AF5_SPI2 },
            { DEFIO_TAG_E(PD3), GPIO_AF5_SPI2 },
        },
        .misoPins = {
            { DEFIO_TAG_E(PB14), GPIO_AF5_SPI2 },
            { DEFIO_TAG_E(PC2), GPIO_AF5_SPI2 },
        },
        .mosiPins = {
            { DEFIO_TAG_E(PB15), GPIO_AF5_SPI2 },
            { DEFIO_TAG_E(PC1), GPIO_AF5_SPI2 },
            { DEFIO_TAG_E(PC3), GPIO_AF5_SPI2 },
        },
        .rcc = RCC_APB1(SPI2),
        .dmaIrqHandler = DMA1_ST4_HANDLER,
    },
    {
        .device = SPIDEV_3,
        .reg = SPI3,
        .sckPins = {
            { DEFIO_TAG_E(PB3), GPIO_AF6_SPI3 },
            { DEFIO_TAG_E(PC10), GPIO_AF6_SPI3 },
        },
        .misoPins = {
            { DEFIO_TAG_E(PB4), GPIO_AF6_SPI3 },
            { DEFIO_TAG_E(PC11), GPIO_AF6_SPI3 },
        },
        .mosiPins = {
            { DEFIO_TAG_E(PB2), GPIO_AF7_SPI3 },
            { DEFIO_TAG_E(PB5), GPIO_AF6_SPI3 },
            { DEFIO_TAG_E(PC12), GPIO_AF6_SPI3 },
            { DEFIO_TAG_E(PD6), GPIO_AF5_SPI3 },
        },
        .rcc = RCC_APB1(SPI3),
        .dmaIrqHandler = DMA1_ST7_HANDLER,
    },
    {
        .device = SPIDEV_4,
        .reg = SPI4,
        .sckPins = {
            { DEFIO_TAG_E(PE2), GPIO_AF5_SPI4 },
            { DEFIO_TAG_E(PE12), GPIO_AF5_SPI4 },
        },
        .misoPins = {
            { DEFIO_TAG_E(PE5), GPIO_AF5_SPI4 },
            { DEFIO_TAG_E(PE13), GPIO_AF5_SPI4 },
        },
        .mosiPins = {
            { DEFIO_TAG_E(PE6), GPIO_AF5_SPI4 },
            { DEFIO_TAG_E(PE14), GPIO_AF5_SPI4 },
        },
        .rcc = RCC_APB2(SPI4),
        .dmaIrqHandler = DMA2_ST1_HANDLER,
    },
#endif
#ifdef STM32H7
    {
        .device = SPIDEV_1,
        .reg = SPI1,
        .sckPins = {
            { DEFIO_TAG_E(PA5), GPIO_AF5_SPI1 },
            { DEFIO_TAG_E(PB3), GPIO_AF5_SPI1 },
        },
        .misoPins = {
            { DEFIO_TAG_E(PA6), GPIO_AF5_SPI1 },
            { DEFIO_TAG_E(PB4), GPIO_AF5_SPI1 },
        },
        .mosiPins = {
            { DEFIO_TAG_E(PA7), GPIO_AF5_SPI1 },
            { DEFIO_TAG_E(PB5), GPIO_AF5_SPI1 },
            { DEFIO_TAG_E(PD7), GPIO_AF5_SPI1 },
        },
        .rcc = RCC_APB2(SPI1),
        //.dmaIrqHandler = DMA2_ST3_HANDLER,
    },
    {
        .device = SPIDEV_2,
        .reg = SPI2,
        .sckPins = {
            { DEFIO_TAG_E(PA9), GPIO_AF5_SPI2 },
            { DEFIO_TAG_E(PA12), GPIO_AF5_SPI2 },
            { DEFIO_TAG_E(PB10), GPIO_AF5_SPI2 },
            { DEFIO_TAG_E(PB13), GPIO_AF5_SPI2 },
            { DEFIO_TAG_E(PD3), GPIO_AF5_SPI2 },
        },
        .misoPins = {
            { DEFIO_TAG_E(PB14), GPIO_AF5_SPI2 },
            { DEFIO_TAG_E(PC2), GPIO_AF5_SPI2 },
        },
        .mosiPins = {
            { DEFIO_TAG_E(PB15), GPIO_AF5_SPI2 },
            { DEFIO_TAG_E(PC1), GPIO_AF5_SPI2 },
            { DEFIO_TAG_E(PC3), GPIO_AF5_SPI2 },
        },
        .rcc = RCC_APB1L(SPI2),
        //.dmaIrqHandler = DMA1_ST4_HANDLER,
    },
    {
        .device = SPIDEV_3,
        .reg = SPI3,
        .sckPins = {
            { DEFIO_TAG_E(PB3), GPIO_AF6_SPI3 },
            { DEFIO_TAG_E(PC10), GPIO_AF6_SPI3 },
        },
        .misoPins = {
            { DEFIO_TAG_E(PB4), GPIO_AF6_SPI3 },
            { DEFIO_TAG_E(PC11), GPIO_AF6_SPI3 },
        },
        .mosiPins = {
            { DEFIO_TAG_E(PB2), GPIO_AF7_SPI3 },
            { DEFIO_TAG_E(PB5), GPIO_AF7_SPI3 },
            { DEFIO_TAG_E(PC12), GPIO_AF6_SPI3 },
            { DEFIO_TAG_E(PD6), GPIO_AF5_SPI3 },
        },
        .rcc = RCC_APB1L(SPI3),
        //.dmaIrqHandler = DMA1_ST7_HANDLER,
    },
    {
        .device = SPIDEV_4,
        .reg = SPI4,
        .sckPins = {
            { DEFIO_TAG_E(PE2), GPIO_AF5_SPI4 },
            { DEFIO_TAG_E(PE12), GPIO_AF5_SPI4 },
        },
        .misoPins = {
            { DEFIO_TAG_E(PE5), GPIO_AF5_SPI4 },
            { DEFIO_TAG_E(PE13), GPIO_AF5_SPI4 },
        },
        .mosiPins = {
            { DEFIO_TAG_E(PE6), GPIO_AF5_SPI4 },
            { DEFIO_TAG_E(PE14), GPIO_AF5_SPI4 },
        },
        .rcc = RCC_APB2(SPI4),
        //.dmaIrqHandler = DMA2_ST1_HANDLER,
    },
    {
        .device = SPIDEV_5,
        .reg = SPI5,
        .sckPins = {
            { DEFIO_TAG_E(PF7), GPIO_AF5_SPI5 },
        },
        .misoPins = {
            { DEFIO_TAG_E(PF8), GPIO_AF5_SPI5 },
        },
        .mosiPins = {
            { DEFIO_TAG_E(PF11), GPIO_AF5_SPI5 },
        },
        .rcc = RCC_APB2(SPI5),
        //.dmaIrqHandler = DMA2_ST1_HANDLER,
    },
    {
        .device = SPIDEV_6,
        .reg = SPI6,
        .sckPins = {
            { DEFIO_TAG_E(PA5), GPIO_AF8_SPI6 },
            { DEFIO_TAG_E(PB3), GPIO_AF8_SPI6 },
        },
        .misoPins = {
            { DEFIO_TAG_E(PA6), GPIO_AF8_SPI6 },
            { DEFIO_TAG_E(PB4), GPIO_AF8_SPI6 },
        },
        .mosiPins = {
            { DEFIO_TAG_E(PA7), GPIO_AF8_SPI6 },
            { DEFIO_TAG_E(PB5), GPIO_AF8_SPI6 },
        },
        .rcc = RCC_APB4(SPI6),
        //.dmaIrqHandler = DMA2_ST1_HANDLER,
    },
#endif
#ifdef STM32G4
    {
        .device = SPIDEV_1,
        .reg = SPI1,
        .sckPins = {
            { DEFIO_TAG_E(PA5), GPIO_AF5_SPI1 },
            { DEFIO_TAG_E(PB3), GPIO_AF5_SPI1 },
        },
        .misoPins = {
            { DEFIO_TAG_E(PA6), GPIO_AF5_SPI1 },
            { DEFIO_TAG_E(PB4), GPIO_AF5_SPI1 },
        },
        .mosiPins = {
            { DEFIO_TAG_E(PA7), GPIO_AF5_SPI1 },
            { DEFIO_TAG_E(PB5), GPIO_AF5_SPI1 },
        },
        .rcc = RCC_APB2(SPI1),
        //.dmaIrqHandler = DMA2_ST3_HANDLER,
    },
    {
        .device = SPIDEV_2,
        .reg = SPI2,
        .sckPins = {
            { DEFIO_TAG_E(PB13), GPIO_AF5_SPI2 },
        },
        .misoPins = {
            { DEFIO_TAG_E(PA10), GPIO_AF5_SPI2 },
            { DEFIO_TAG_E(PB14), GPIO_AF5_SPI2 },
        },
        .mosiPins = {
            { DEFIO_TAG_E(PA11), GPIO_AF5_SPI2 },
            { DEFIO_TAG_E(PB15), GPIO_AF5_SPI2 },
        },
        .rcc = RCC_APB11(SPI2),
        //.dmaIrqHandler = DMA1_ST4_HANDLER,
    },
    {
        .device = SPIDEV_3,
        .reg = SPI3,
        .sckPins = {
            { DEFIO_TAG_E(PB3), GPIO_AF6_SPI3 },
            { DEFIO_TAG_E(PC10), GPIO_AF6_SPI3 },
        },
        .misoPins = {
            { DEFIO_TAG_E(PB4), GPIO_AF6_SPI3 },
            { DEFIO_TAG_E(PC11), GPIO_AF6_SPI3 },
        },
        .mosiPins = {
            { DEFIO_TAG_E(PB5), GPIO_AF6_SPI3 },
            { DEFIO_TAG_E(PC12), GPIO_AF6_SPI3 },
        },
        .rcc = RCC_APB11(SPI3),
        //.dmaIrqHandler = DMA1_ST7_HANDLER,
    },
#endif
#ifdef AT32F4
    {
        .device = SPIDEV_1,
        .reg = SPI1,
        .sckPins = {
            { DEFIO_TAG_E(PA5) ,GPIO_MUX_5},
            { DEFIO_TAG_E(PB3) ,GPIO_MUX_5},
            { DEFIO_TAG_E(PE13),GPIO_MUX_4},
        },
        .misoPins = {
            { DEFIO_TAG_E(PA6) ,GPIO_MUX_5},
            { DEFIO_TAG_E(PB4) ,GPIO_MUX_5},
            { DEFIO_TAG_E(PE14),GPIO_MUX_4}
        },
        .mosiPins = {
            { DEFIO_TAG_E(PA7) ,GPIO_MUX_5},
            { DEFIO_TAG_E(PB5) ,GPIO_MUX_5},
            { DEFIO_TAG_E(PE15),GPIO_MUX_4},
        },
        .af= 0x00,
        .rcc = RCC_APB2(SPI1),
    },
    {
        .device = SPIDEV_2,
        .reg = SPI2,
        .sckPins = {
             { DEFIO_TAG_E(PB10), GPIO_MUX_5},
             { DEFIO_TAG_E(PB13) ,GPIO_MUX_5},
             { DEFIO_TAG_E(PC7),  GPIO_MUX_5},
             { DEFIO_TAG_E(PD1),  GPIO_MUX_6},

        },
        .misoPins = {
            { DEFIO_TAG_E(PA12),  GPIO_MUX_5},
            { DEFIO_TAG_E(PB14),  GPIO_MUX_5},
            { DEFIO_TAG_E(PC2),   GPIO_MUX_5},
            { DEFIO_TAG_E(PD3),   GPIO_MUX_6},
        },
        .mosiPins = {
            { DEFIO_TAG_E(PB15),  GPIO_MUX_5},
            { DEFIO_TAG_E(PC1),   GPIO_MUX_7},
            { DEFIO_TAG_E(PC3),   GPIO_MUX_5},
            { DEFIO_TAG_E(PD4),   GPIO_MUX_6},
        },
        .af= 0x00,
        .rcc = RCC_APB1(SPI2),
    },
    {
        .device = SPIDEV_3,
        .reg = SPI3,
        .sckPins = {
            { DEFIO_TAG_E(PB3),  GPIO_MUX_6},
            { DEFIO_TAG_E(PB12), GPIO_MUX_7},
            { DEFIO_TAG_E(PC10), GPIO_MUX_6},
        },
        .misoPins = {
            { DEFIO_TAG_E(PB4),  GPIO_MUX_6},
            { DEFIO_TAG_E(PC11), GPIO_MUX_6},
        },
        .mosiPins = {
            { DEFIO_TAG_E(PB2),  GPIO_MUX_7},
            { DEFIO_TAG_E(PB5),  GPIO_MUX_6},
            { DEFIO_TAG_E(PC12),  GPIO_MUX_6},
            { DEFIO_TAG_E(PD0),  GPIO_MUX_6},
        },
        .af= 0x00,
        .rcc = RCC_APB1(SPI3),
    },
    {
        .device = SPIDEV_4,
        .reg = SPI4,
        .sckPins = {
            { DEFIO_TAG_E(PB7),  GPIO_MUX_6},
            { DEFIO_TAG_E(PB13), GPIO_MUX_6},
        },
        .misoPins = {
            { DEFIO_TAG_E(PA11),  GPIO_MUX_6},
            { DEFIO_TAG_E(PB8) ,  GPIO_MUX_6},
            { DEFIO_TAG_E(PD0) ,  GPIO_MUX_5},
        },
        .mosiPins = {
            { DEFIO_TAG_E(PA1),   GPIO_MUX_5},
            { DEFIO_TAG_E(PB9),   GPIO_MUX_6},
        },
        .af= 0x00,
        .rcc = RCC_APB2(SPI4),
    },
#endif
#ifdef HPM6750
    {
        .device = SPIDEV_1,
        .reg = SPI1,
        .sckPins = {
            { DEFIO_TAG_E(PZ3), IOC_PZ03_FUNC_CTL_SPI0_SCLK | IOC_PAD_FUNC_CTL_LOOP_BACK_SET(1), BIOC_PZ03_FUNC_CTL_SOC_PZ_03 | IOC_PAD_FUNC_CTL_LOOP_BACK_MASK, IOC_PAD_PAD_CTL_DS_SET(6) | IOC_PAD_PAD_CTL_PE_SET(1), -1},
        },
        .misoPins = {
            { DEFIO_TAG_E(PZ5),  IOC_PZ05_FUNC_CTL_SPI0_MISO, BIOC_PZ05_FUNC_CTL_SOC_PZ_05, IOC_PAD_PAD_CTL_DS_SET(6) | IOC_PAD_PAD_CTL_PE_SET(1), -1},
        },
        .mosiPins = {
            { DEFIO_TAG_E(PZ4),  IOC_PZ04_FUNC_CTL_SPI0_MOSI, BIOC_PZ04_FUNC_CTL_SOC_PZ_04, IOC_PAD_PAD_CTL_DS_SET(6) | IOC_PAD_PAD_CTL_PE_SET(1), -1},
        },
        .af= 5,
        .rcc = clock_spi0,
    
    },
    {
        .device = SPIDEV_2,
        .reg = SPI2,
        .sckPins = {
            { DEFIO_TAG_E(PD31), IOC_PD31_FUNC_CTL_SPI1_SCLK | IOC_PAD_FUNC_CTL_LOOP_BACK_SET(1), -1, IOC_PAD_PAD_CTL_DS_SET(6) | IOC_PAD_PAD_CTL_PE_SET(1), -1},
        },
        .misoPins = {
            { DEFIO_TAG_E(PD30),  IOC_PD30_FUNC_CTL_SPI1_MISO, -1, IOC_PAD_PAD_CTL_DS_SET(6) | IOC_PAD_PAD_CTL_PE_SET(1), -1},
        },
        .mosiPins = {
            { DEFIO_TAG_E(PE4),  IOC_PE04_FUNC_CTL_SPI1_MOSI, -1, IOC_PAD_PAD_CTL_DS_SET(6) | IOC_PAD_PAD_CTL_PE_SET(1), -1},
        },
        .af= 5,
        .rcc = clock_spi1,
    },
    {
        .device = SPIDEV_3,
        .reg = SPI3,
        .sckPins = {
            { DEFIO_TAG_E(PB0), IOC_PB00_FUNC_CTL_SPI2_SCLK | IOC_PAD_FUNC_CTL_LOOP_BACK_SET(1), -1, IOC_PAD_PAD_CTL_DS_SET(6) | IOC_PAD_PAD_CTL_PE_SET(1), -1},
            { DEFIO_TAG_E(PB21) ,IOC_PB21_FUNC_CTL_SPI2_SCLK | IOC_PAD_FUNC_CTL_LOOP_BACK_SET(1), -1, IOC_PAD_PAD_CTL_DS_SET(6) | IOC_PAD_PAD_CTL_PE_SET(1), -1},
            { DEFIO_TAG_E(PE27),  IOC_PE27_FUNC_CTL_SPI2_SCLK | IOC_PAD_FUNC_CTL_LOOP_BACK_SET(1), -1, IOC_PAD_PAD_CTL_DS_SET(6) | IOC_PAD_PAD_CTL_PE_SET(1), -1},
        },
        .misoPins = {
            { DEFIO_TAG_E(PA31),  IOC_PA31_FUNC_CTL_SPI2_MISO, -1, IOC_PAD_PAD_CTL_DS_SET(6) | IOC_PAD_PAD_CTL_PE_SET(1), -1},
            { DEFIO_TAG_E(PB25),  IOC_PB25_FUNC_CTL_SPI2_MISO, -1, IOC_PAD_PAD_CTL_DS_SET(6) | IOC_PAD_PAD_CTL_PE_SET(1), -1},
            { DEFIO_TAG_E(PE28),  IOC_PE28_FUNC_CTL_SPI2_MISO, -1, IOC_PAD_PAD_CTL_DS_SET(6) | IOC_PAD_PAD_CTL_PE_SET(1), -1},
        },
        .mosiPins = {
            { DEFIO_TAG_E(PA27),  IOC_PA27_FUNC_CTL_SPI2_MOSI, -1, IOC_PAD_PAD_CTL_DS_SET(6) | IOC_PAD_PAD_CTL_PE_SET(1), -1},
            { DEFIO_TAG_E(PB22),  IOC_PB22_FUNC_CTL_SPI2_MOSI, -1, IOC_PAD_PAD_CTL_DS_SET(6) | IOC_PAD_PAD_CTL_PE_SET(1), -1},
            { DEFIO_TAG_E(PE30),  IOC_PE30_FUNC_CTL_SPI2_MOSI, -1, IOC_PAD_PAD_CTL_DS_SET(6) | IOC_PAD_PAD_CTL_PE_SET(1), -1},
        },
        .af= 5,
        .rcc = clock_spi2,
    },
#endif
#ifdef HPM6360
    {
        .device = SPIDEV_1,
        .reg = SPI1,
        .sckPins = {
            { DEFIO_TAG_E(PA8), IOC_PA08_FUNC_CTL_SPI0_SCLK | IOC_PAD_FUNC_CTL_LOOP_BACK_SET(1), -1, IOC_PAD_PAD_CTL_DS_SET(6) | IOC_PAD_PAD_CTL_PE_SET(1), -1},
            { DEFIO_TAG_E(PA12), IOC_PA12_FUNC_CTL_SPI0_SCLK | IOC_PAD_FUNC_CTL_LOOP_BACK_SET(1), -1, IOC_PAD_PAD_CTL_DS_SET(6) | IOC_PAD_PAD_CTL_PE_SET(1), -1},
            { DEFIO_TAG_E(PA30), IOC_PA30_FUNC_CTL_SPI0_SCLK | IOC_PAD_FUNC_CTL_LOOP_BACK_SET(1), -1, IOC_PAD_PAD_CTL_DS_SET(6) | IOC_PAD_PAD_CTL_PE_SET(1), -1},
        },
        .misoPins = {
            { DEFIO_TAG_E(PA7), IOC_PA07_FUNC_CTL_SPI0_MISO, -1, IOC_PAD_PAD_CTL_DS_SET(6) | IOC_PAD_PAD_CTL_PE_SET(1), -1},
            { DEFIO_TAG_E(PA11), IOC_PA11_FUNC_CTL_SPI0_MISO, -1, IOC_PAD_PAD_CTL_DS_SET(6) | IOC_PAD_PAD_CTL_PE_SET(1), -1},
            { DEFIO_TAG_E(PA29), IOC_PA29_FUNC_CTL_SPI0_MISO, -1, IOC_PAD_PAD_CTL_DS_SET(6) | IOC_PAD_PAD_CTL_PE_SET(1), -1},
        },
        .mosiPins = {
            { DEFIO_TAG_E(PA9), IOC_PA09_FUNC_CTL_SPI0_MOSI, -1, IOC_PAD_PAD_CTL_DS_SET(6) | IOC_PAD_PAD_CTL_PE_SET(1), -1},
            { DEFIO_TAG_E(PA13), IOC_PA13_FUNC_CTL_SPI0_MOSI, -1, IOC_PAD_PAD_CTL_DS_SET(6) | IOC_PAD_PAD_CTL_PE_SET(1), -1},
            { DEFIO_TAG_E(PA31), IOC_PA31_FUNC_CTL_SPI0_MOSI, -1, IOC_PAD_PAD_CTL_DS_SET(6) | IOC_PAD_PAD_CTL_PE_SET(1), -1},
        },
        .af= 5,
        .rcc = clock_spi0,
    },
    {
        .device = SPIDEV_2,
        .reg = SPI2,
        .sckPins = {
            { DEFIO_TAG_E(PA18), IOC_PA18_FUNC_CTL_SPI1_SCLK | IOC_PAD_FUNC_CTL_LOOP_BACK_SET(1), -1, IOC_PAD_PAD_CTL_DS_SET(6) | IOC_PAD_PAD_CTL_PE_SET(1), -1},
            { DEFIO_TAG_E(PB4), IOC_PB04_FUNC_CTL_SPI1_SCLK | IOC_PAD_FUNC_CTL_LOOP_BACK_SET(1), -1, IOC_PAD_PAD_CTL_DS_SET(6) | IOC_PAD_PAD_CTL_PE_SET(1), -1},
            { DEFIO_TAG_E(PB9), IOC_PB09_FUNC_CTL_SPI1_SCLK | IOC_PAD_FUNC_CTL_LOOP_BACK_SET(1), -1, IOC_PAD_PAD_CTL_DS_SET(6) | IOC_PAD_PAD_CTL_PE_SET(1), -1},
            { DEFIO_TAG_E(PB29), IOC_PB29_FUNC_CTL_SPI1_SCLK | IOC_PAD_FUNC_CTL_LOOP_BACK_SET(1), -1, IOC_PAD_PAD_CTL_DS_SET(6) | IOC_PAD_PAD_CTL_PE_SET(1), -1},
        },
        .misoPins = {
            { DEFIO_TAG_E(PA17), IOC_PA17_FUNC_CTL_SPI1_MISO, -1, IOC_PAD_PAD_CTL_DS_SET(6) | IOC_PAD_PAD_CTL_PE_SET(1), -1},
            { DEFIO_TAG_E(PB3), IOC_PB03_FUNC_CTL_SPI1_MISO, -1, IOC_PAD_PAD_CTL_DS_SET(6) | IOC_PAD_PAD_CTL_PE_SET(1), -1},
            { DEFIO_TAG_E(PB10), IOC_PB10_FUNC_CTL_SPI1_MISO, -1, IOC_PAD_PAD_CTL_DS_SET(6) | IOC_PAD_PAD_CTL_PE_SET(1), -1},
            { DEFIO_TAG_E(PB28), IOC_PB28_FUNC_CTL_SPI1_MISO, -1, IOC_PAD_PAD_CTL_DS_SET(6) | IOC_PAD_PAD_CTL_PE_SET(1), -1},
        },
        .mosiPins = {
            { DEFIO_TAG_E(PA19), IOC_PA19_FUNC_CTL_SPI1_MOSI, -1, IOC_PAD_PAD_CTL_DS_SET(6) | IOC_PAD_PAD_CTL_PE_SET(1), -1},
            { DEFIO_TAG_E(PB5), IOC_PB05_FUNC_CTL_SPI1_MOSI, -1, IOC_PAD_PAD_CTL_DS_SET(6) | IOC_PAD_PAD_CTL_PE_SET(1), -1},
            { DEFIO_TAG_E(PB11), IOC_PB11_FUNC_CTL_SPI1_MOSI, -1, IOC_PAD_PAD_CTL_DS_SET(6) | IOC_PAD_PAD_CTL_PE_SET(1), -1},
            { DEFIO_TAG_E(PB30), IOC_PB30_FUNC_CTL_SPI1_MOSI, -1, IOC_PAD_PAD_CTL_DS_SET(6) | IOC_PAD_PAD_CTL_PE_SET(1), -1},
        },
        .af= 5,
        .rcc = clock_spi1,
    },
    {
        .device = SPIDEV_3,
        .reg = SPI3,
        .sckPins = {
            { DEFIO_TAG_E(PB15), IOC_PB15_FUNC_CTL_SPI2_SCLK | IOC_PAD_FUNC_CTL_LOOP_BACK_SET(1), -1, IOC_PAD_PAD_CTL_DS_SET(6) | IOC_PAD_PAD_CTL_PE_SET(1), -1},
        },
        .misoPins = {
            { DEFIO_TAG_E(PB14), IOC_PB14_FUNC_CTL_SPI2_MISO, -1, IOC_PAD_PAD_CTL_DS_SET(6) | IOC_PAD_PAD_CTL_PE_SET(1), -1},
            { DEFIO_TAG_E(PC0), IOC_PC00_FUNC_CTL_SPI2_MISO, -1, IOC_PAD_PAD_CTL_DS_SET(6) | IOC_PAD_PAD_CTL_PE_SET(1), -1},
        },
        .mosiPins = {
            { DEFIO_TAG_E(PB16), IOC_PB16_FUNC_CTL_SPI2_MOSI, -1, IOC_PAD_PAD_CTL_DS_SET(6) | IOC_PAD_PAD_CTL_PE_SET(1), -1},
        },
        .af= 5,
        .rcc = clock_spi2,
    },
    {
        .device = SPIDEV_4,
        .reg = SPI4,
        .sckPins = {
            { DEFIO_TAG_E(PA2), IOC_PA02_FUNC_CTL_SPI3_SCLK | IOC_PAD_FUNC_CTL_LOOP_BACK_SET(1), -1, IOC_PAD_PAD_CTL_DS_SET(6) | IOC_PAD_PAD_CTL_PE_SET(1), -1},
            { DEFIO_TAG_E(PC20), IOC_PC20_FUNC_CTL_SPI3_SCLK | IOC_PAD_FUNC_CTL_LOOP_BACK_SET(1), -1, IOC_PAD_PAD_CTL_DS_SET(6) | IOC_PAD_PAD_CTL_PE_SET(1), -1},
        },
        .misoPins = {
            { DEFIO_TAG_E(PA1), IOC_PA01_FUNC_CTL_SPI3_MISO, -1, IOC_PAD_PAD_CTL_DS_SET(6) | IOC_PAD_PAD_CTL_PE_SET(1), -1},
            { DEFIO_TAG_E(PC19), IOC_PC19_FUNC_CTL_SPI3_MISO, -1, IOC_PAD_PAD_CTL_DS_SET(6) | IOC_PAD_PAD_CTL_PE_SET(1), -1},
        },
        .mosiPins = {
            { DEFIO_TAG_E(PA3), IOC_PA03_FUNC_CTL_SPI3_MOSI, -1, IOC_PAD_PAD_CTL_DS_SET(6) | IOC_PAD_PAD_CTL_PE_SET(1), -1},
            { DEFIO_TAG_E(PC21), IOC_PC21_FUNC_CTL_SPI3_MOSI, -1, IOC_PAD_PAD_CTL_DS_SET(6) | IOC_PAD_PAD_CTL_PE_SET(1), -1},
        },
        .af= 5,
        .rcc = clock_spi3,
    },
#endif
};

void spiPinConfigure(const spiPinConfig_t *pConfig)
{
    for (size_t hwindex = 0 ; hwindex < ARRAYLEN(spiHardware) ; hwindex++) {
        const spiHardware_t *hw = &spiHardware[hwindex];

        if (!hw->reg) {
            continue;
        }

        SPIDevice device = hw->device;
        spiDevice_t *pDev = &spiDevice[device];

        for (int pindex = 0 ; pindex < MAX_SPI_PIN_SEL ; pindex++) {
            if (pConfig[device].ioTagSck == hw->sckPins[pindex].pin) {
                pDev->sck = hw->sckPins[pindex].pin;
#if defined(USE_PIN_AF)
                pDev->sckAF = hw->sckPins[pindex].af;
#ifdef HPMicro
                pDev->sckAF2 = hw->sckPins[pindex].af2;
#endif
#endif
            }
            if (pConfig[device].ioTagMiso == hw->misoPins[pindex].pin) {
                pDev->miso = hw->misoPins[pindex].pin;
#if defined(USE_PIN_AF)
                pDev->misoAF = hw->misoPins[pindex].af;
#ifdef HPMicro
                pDev->misoAF2 = hw->misoPins[pindex].af2;
#endif
#endif
            }
            if (pConfig[device].ioTagMosi == hw->mosiPins[pindex].pin) {
                pDev->mosi = hw->mosiPins[pindex].pin;
#if defined(USE_PIN_AF)
                pDev->mosiAF = hw->mosiPins[pindex].af;
#ifdef HPMicro
                pDev->mosiAF2 = hw->mosiPins[pindex].af2;
#endif
#endif
            }
        }

        if (pDev->sck && pDev->miso && pDev->mosi) {
            pDev->dev = hw->reg;
#if !defined(USE_PIN_AF)
            pDev->af = hw->af;
#endif
            pDev->rcc = hw->rcc;
            pDev->leadingEdge = false; // XXX Should be part of transfer context
#if defined(USE_DMA) && defined(USE_HAL_DRIVER)
            pDev->dmaIrqHandler = hw->dmaIrqHandler;
#endif
        }
    }
}
#endif
