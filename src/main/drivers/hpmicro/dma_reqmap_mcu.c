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

#include <stdint.h>

#include "platform.h"

#ifdef USE_DMA_SPEC

#include "timer_def.h"
#include "drivers/bus_spi.h"
#include "drivers/dma_reqmap.h"

typedef struct dmaPeripheralMapping_s {
    dmaPeripheral_e device;
    uint8_t index;
    dmaChannelSpec_t channelSpec[MAX_PERIPHERAL_DMA_OPTIONS];
    uint32_t dmaMuxSrc;
} dmaPeripheralMapping_t;

#define DMA(d, s, c) { d, (dmaResource_t *)s, c}

static const dmaPeripheralMapping_t dmaPeripheralMapping[] = {
#ifdef USE_SPI
#ifdef HPM6750
    { DMA_PERIPH_SPI_SDO,  SPIDEV_1,  { DMA(1, HPM_HDMA_Channel4, 4), DMA(2, HPM_HDMA_Channel5, 5), DMA(3, HPM_HDMA_Channel6, 6), DMA(4, HPM_HDMA_Channel7, 7) }, HPM_DMA_SRC_SPI0_TX, },
    { DMA_PERIPH_SPI_SDI,  SPIDEV_1,  { DMA(1, HPM_HDMA_Channel4, 4), DMA(2, HPM_HDMA_Channel5, 5), DMA(3, HPM_HDMA_Channel6, 6), DMA(4, HPM_HDMA_Channel7, 7) }, HPM_DMA_SRC_SPI0_RX, },
    { DMA_PERIPH_SPI_SDO,  SPIDEV_2,  { DMA(1, HPM_HDMA_Channel4, 4), DMA(2, HPM_HDMA_Channel5, 5), DMA(3, HPM_HDMA_Channel6, 6), DMA(4, HPM_HDMA_Channel7, 7) }, HPM_DMA_SRC_SPI1_TX, },
    { DMA_PERIPH_SPI_SDI,  SPIDEV_2,  { DMA(1, HPM_HDMA_Channel4, 4), DMA(2, HPM_HDMA_Channel5, 5), DMA(3, HPM_HDMA_Channel6, 6), DMA(4, HPM_HDMA_Channel7, 7) }, HPM_DMA_SRC_SPI1_RX, },
    { DMA_PERIPH_SPI_SDO,  SPIDEV_3,  { DMA(1, HPM_HDMA_Channel4, 4), DMA(2, HPM_HDMA_Channel5, 5), DMA(3, HPM_HDMA_Channel6, 6), DMA(4, HPM_HDMA_Channel7, 7) }, HPM_DMA_SRC_SPI2_TX, },
    { DMA_PERIPH_SPI_SDI,  SPIDEV_3,  { DMA(1, HPM_HDMA_Channel4, 4), DMA(2, HPM_HDMA_Channel5, 5), DMA(3, HPM_HDMA_Channel6, 6), DMA(4, HPM_HDMA_Channel7, 7) }, HPM_DMA_SRC_SPI2_RX, },
    { DMA_PERIPH_SPI_SDO,  SPIDEV_4,  { DMA(1, HPM_HDMA_Channel4, 4), DMA(2, HPM_HDMA_Channel5, 5), DMA(3, HPM_HDMA_Channel6, 6), DMA(4, HPM_HDMA_Channel7, 7) }, HPM_DMA_SRC_SPI3_TX, },
    { DMA_PERIPH_SPI_SDI,  SPIDEV_4,  { DMA(1, HPM_HDMA_Channel4, 4), DMA(2, HPM_HDMA_Channel5, 5), DMA(3, HPM_HDMA_Channel6, 6), DMA(4, HPM_HDMA_Channel7, 7) }, HPM_DMA_SRC_SPI3_RX, },
#endif
#ifdef HPM6360
    { DMA_PERIPH_SPI_SDO,  SPIDEV_1,  { DMA(1, HPM_HDMA_Channel5, 5), DMA(2, HPM_HDMA_Channel6, 6), DMA(3, HPM_HDMA_Channel7, 7) }, HPM_DMA_SRC_SPI0_TX, },
    { DMA_PERIPH_SPI_SDI,  SPIDEV_1,  { DMA(1, HPM_HDMA_Channel0, 0), }, HPM_DMA_SRC_SPI0_RX, },
    { DMA_PERIPH_SPI_SDO,  SPIDEV_2,  { DMA(1, HPM_HDMA_Channel5, 5), DMA(2, HPM_HDMA_Channel6, 6), DMA(3, HPM_HDMA_Channel7, 7) }, HPM_DMA_SRC_SPI1_TX, },
    { DMA_PERIPH_SPI_SDO,  SPIDEV_2,  { DMA(1, HPM_HDMA_Channel5, 5), DMA(2, HPM_HDMA_Channel6, 6), DMA(3, HPM_HDMA_Channel7, 7) }, HPM_DMA_SRC_SPI1_TX, },
    { DMA_PERIPH_SPI_SDI,  SPIDEV_2,  { DMA(1, HPM_HDMA_Channel5, 5), DMA(2, HPM_HDMA_Channel6, 6), DMA(3, HPM_HDMA_Channel7, 7) }, HPM_DMA_SRC_SPI1_RX, },
    { DMA_PERIPH_SPI_SDO,  SPIDEV_3,  { DMA(1, HPM_HDMA_Channel5, 5), DMA(2, HPM_HDMA_Channel6, 6), DMA(3, HPM_HDMA_Channel7, 7) }, HPM_DMA_SRC_SPI2_TX, },
    { DMA_PERIPH_SPI_SDI,  SPIDEV_3,  { DMA(1, HPM_HDMA_Channel5, 5), DMA(2, HPM_HDMA_Channel6, 6), DMA(3, HPM_HDMA_Channel7, 7) }, HPM_DMA_SRC_SPI2_RX, },
    { DMA_PERIPH_SPI_SDO,  SPIDEV_4,  { DMA(1, HPM_HDMA_Channel5, 5), DMA(2, HPM_HDMA_Channel6, 6), DMA(3, HPM_HDMA_Channel7, 7) }, HPM_DMA_SRC_SPI3_TX, },
    { DMA_PERIPH_SPI_SDI,  SPIDEV_4,  { DMA(1, HPM_HDMA_Channel5, 5), DMA(2, HPM_HDMA_Channel6, 6), DMA(3, HPM_HDMA_Channel7, 7) }, HPM_DMA_SRC_SPI3_RX, },
#endif
#endif // USE_SPI
};
#undef DMA

const dmaChannelSpec_t *dmaGetChannelSpecByPeripheral(dmaPeripheral_e device, uint8_t index, int8_t opt)
{
    if (opt < 0 || opt >= MAX_PERIPHERAL_DMA_OPTIONS) {
        return NULL;
    }

    for (unsigned i = 0 ; i < ARRAYLEN(dmaPeripheralMapping) ; i++) {
        const dmaPeripheralMapping_t *periph = &dmaPeripheralMapping[i];
        if (periph->device == device && periph->index == index && periph->channelSpec[opt].ref) {
            return &periph->channelSpec[opt];
        }
    }

    return NULL;
}

uint32_t dmaGetMuxSrcSpecByPeripheral(dmaPeripheral_e device, uint8_t index)
{
    for (unsigned i = 0 ; i < ARRAYLEN(dmaPeripheralMapping) ; i++) {
        const dmaPeripheralMapping_t *periph = &dmaPeripheralMapping[i];
        if (periph->device == device && periph->index == index) {
            return periph->dmaMuxSrc;
        }
    }
    return 0;
}

#endif // USE_DMA_SPEC
