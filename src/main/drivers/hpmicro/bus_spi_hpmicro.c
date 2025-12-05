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

#include "hpm_spi.h"
#include "platform.h"
#ifdef USE_SPI

#include "drivers/bus.h"
#include "drivers/bus_spi.h"
#include "drivers/bus_spi_impl.h"
#include "drivers/io.h"
#include "hpm_clock_drv.h"
#include "hpm_spi_drv.h"
#include "hpm_l1c_drv.h"
#include "dma_hpmicro.h"

void spi_rxdma_complete_callback(uint32_t channel)
{
    (void)channel;
}

void spi_txdma_complete_callback(uint32_t channel)
{
    (void)channel;
}

void spiInitDevice(SPIDevice device)
{
    spi_timing_config_t timing_config = {0};
    uint32_t spi_sclk_freq = 4000000;
    spiDevice_t *spi = &(spiDevice[device]);
    uint32_t spi_clcok = clock_get_frequency(spi->rcc);
    spi_format_config_t format_config = {0};

    clock_set_source_divider(spi->rcc, clk_src_pll1_clk1, 5);
    clock_add_to_group(spi->rcc, 0);
    IOInit(IOGetByTag(spi->sck),  OWNER_SPI_SCK,  RESOURCE_INDEX(device));
    IOInit(IOGetByTag(spi->miso), OWNER_SPI_SDI, RESOURCE_INDEX(device));
    IOInit(IOGetByTag(spi->mosi), OWNER_SPI_SDO, RESOURCE_INDEX(device));

    IOConfigGPIOAF(IOGetByTag(spi->sck),  IOCFG_AF_PP, spi->sckAF, spi->sckAF2);
    IOConfigGPIOAF(IOGetByTag(spi->miso), IOCFG_AF_PP, spi->misoAF, spi->misoAF2);
    IOConfigGPIOAF(IOGetByTag(spi->mosi), IOCFG_AF_PP, spi->mosiAF, spi->mosiAF2);

    spi_master_get_default_timing_config(&timing_config);
    timing_config.master_config.clk_src_freq_in_hz = spi_clcok;
    timing_config.master_config.sclk_freq_in_hz = spi_sclk_freq;
    if (status_success != spi_master_timing_init((SPI_Type *)(spi->dev), &timing_config)) {
        printf("SPI master timming init failed\n");
        while (1) {
        }
    }
    /* set SPI format config for master */
    spi_master_get_default_format_config(&format_config);
    format_config.common_config.data_len_in_bits = 8;
    format_config.common_config.mode = spi_master_mode;
    format_config.common_config.cpol = spi_sclk_low_idle;
    format_config.common_config.cpha = spi_sclk_sampling_odd_clk_edges;
    spi_format_init((SPI_Type *)(spi->dev), &format_config);
}

void spiInternalResetDescriptors(busDevice_t *bus)
{
    (void)bus;
}

void spiInternalResetStream(dmaChannelDescriptor_t *descriptor)
{
    (void)descriptor;
}

static bool spiInternalReadWriteBufPolled(spi_type *instance, const uint8_t *txData, uint8_t *rxData, int len)
{
    static uint8_t wbuff[32];
    hpm_stat_t stat;
    spi_control_config_t control_config = {0};
    /* set SPI control config for master */
    spi_master_get_default_control_config(&control_config);
    control_config.master_config.cmd_enable = false;  /* cmd phase control for master */
    control_config.master_config.addr_enable = false; /* address phase control for master */

    if (txData && rxData) {
        control_config.common_config.trans_mode = spi_trans_write_read_together;
        stat = spi_transfer(instance,
                    &control_config,
                    NULL, NULL,
                    (uint8_t *)txData, len, (uint8_t *)rxData, len);
    }
    else if (txData) {
        control_config.common_config.trans_mode = spi_trans_write_only;
        stat = spi_transfer(instance,
                    &control_config,
                    NULL, NULL,
                    (uint8_t *)txData, len, (uint8_t *)0, 0);
    } else if(rxData) {
        control_config.common_config.trans_mode = spi_trans_read_only;
        stat = spi_transfer(instance,
                    &control_config,
                    NULL, NULL,
                    (uint8_t *)0, 0, (uint8_t *)rxData, len);
    } else {
        control_config.common_config.trans_mode = spi_trans_dummy_write;
        stat = spi_transfer(instance,
                    &control_config,
                    NULL, NULL,
                    (uint8_t *)wbuff, len, NULL, len);
    }
    
    if (stat == status_success)
        return true;
    else
        return false;
}

void spiInternalInitStream(const extDevice_t *dev, bool preInit)
{
    (void)preInit;
    hpm_stat_t stat;
    busDevice_t *bus = dev->bus;
    SPI_Type *instance = (SPI_Type *)(bus->busType_u.spi.instance);
    spi_control_config_t control_config = {0};
    if ((spi_is_active(instance) == true))
        return;
    //Disable SPI DMA fisrt
    spi_disable_tx_dma(instance);
    spi_disable_rx_dma(instance);

    spi_master_get_default_control_config(&control_config);
    control_config.master_config.cmd_enable = false;
    control_config.master_config.addr_enable = false;
    control_config.master_config.addr_phase_fmt = spi_address_phase_format_single_io_mode;
    control_config.common_config.tx_dma_enable = true;
    control_config.common_config.rx_dma_enable = true;
    control_config.common_config.trans_mode = spi_trans_write_read_together;
    control_config.common_config.data_phase_fmt = spi_single_io_mode;
    control_config.common_config.dummy_cnt = spi_dummy_count_1;

    stat = spi_setup_dma_transfer(instance,
                        &control_config,
                        0, 0,
                        bus->curSegment->len, bus->curSegment->len);
    if (stat != status_success) {
        while (1) {
        }
    }

}

hpm_stat_t spi_tx_trigger_dma(DMA_Type *dma_ptr, uint8_t ch_num, SPI_Type *spi_ptr, uint32_t src, uint8_t data_width, uint32_t size)
{
    dma_handshake_config_fixed_t config = { 0 };

    config.ch_index = ch_num;
    config.dst = (uint32_t)&spi_ptr->DATA;
    config.dst_fixed = true;
    config.src = src;
    config.src_fixed = false;
    config.data_width = data_width;
    config.size_in_byte = size;
    config.interrupt_mask = DMA_INTERRUPT_MASK_ALL;

    return dma_setup_handshake_fixed(dma_ptr, &config, true);
}

hpm_stat_t spi_rx_trigger_dma(DMA_Type *dma_ptr, uint8_t ch_num, SPI_Type *spi_ptr, uint32_t dst, uint8_t data_width, uint32_t size)
{
    dma_handshake_config_t config;

    dma_default_handshake_config(dma_ptr, &config);
    config.ch_index = ch_num;
    config.dst = dst;
    config.dst_fixed = false;
    config.src = (uint32_t)&spi_ptr->DATA;
    config.src_fixed = true;
    config.data_width = data_width;
    config.size_in_byte = size;

    return dma_setup_handshake(dma_ptr, &config, true);
}

void spiInternalStartDMA(const extDevice_t *dev)
{
    dmaChannelDescriptor_t *dmaTx = dev->bus->dmaTx;
    dmaChannelDescriptor_t *dmaRx = dev->bus->dmaRx;
    dmaRx->userParam = (uint32_t)dev;
    dmaTx->userParam = (uint32_t)dev;
    hpm_stat_t stat;
    busDevice_t *bus = dev->bus;
    SPI_Type *instance = (SPI_Type *)(bus->busType_u.spi.instance);
    if (l1c_dc_is_enabled()) {
        /* cache writeback for sent buff */
        uint32_t aligned_start = HPM_L1C_CACHELINE_ALIGN_DOWN((uint32_t)bus->curSegment->u.buffers.txData);
        uint32_t aligned_end = HPM_L1C_CACHELINE_ALIGN_UP((uint32_t)bus->curSegment->u.buffers.txData + bus->curSegment->len);
        uint32_t aligned_size = aligned_end - aligned_start;
        l1c_dc_writeback(aligned_start, aligned_size);
    }
    stat = spi_tx_trigger_dma(dmaTx->dma,
                            (uint32_t)dmaTx->ref & 0xF,
                            instance,
                            core_local_mem_to_sys_address(HPM_CORE0, (uint32_t)bus->curSegment->u.buffers.txData),
                            DMA_TRANSFER_WIDTH_BYTE,
                            bus->curSegment->len);
    if (stat != status_success) {
        printf("spi tx trigger dma failed\n");
        while (1) {
        }
    }
    stat = spi_rx_trigger_dma(dmaRx->dma,
                            (uint32_t)dmaRx->ref & 0xF,
                            instance,
                            core_local_mem_to_sys_address(HPM_CORE0, (uint32_t)bus->curSegment->u.buffers.rxData),
                            DMA_TRANSFER_WIDTH_BYTE,
                            bus->curSegment->len);
    if (stat != status_success) {
        printf("spi rx trigger dma failed\n");
        while (1) {
        }
    }
}

void spiInternalStopDMA (const extDevice_t *dev)
{
    busDevice_t *bus = dev->bus;
    SPI_Type *instance = (SPI_Type *)(bus->busType_u.spi.instance);
    spi_disable_tx_dma(instance);
    spi_disable_rx_dma(instance);
}

void spiSequenceStart(const extDevice_t *dev)
{
    busDevice_t *bus = dev->bus;
    SPI_Type *instance = (SPI_Type *)(bus->busType_u.spi.instance);
    bool dmaSafe = dev->useDMA;
    uint32_t xferLen = 0;
    uint32_t segmentCount = 0;

    dev->bus->initSegment = true;

    if (dev->busType_u.spi.speed != bus->busType_u.spi.speed) {
        if (hpm_spi_set_sclk_frequency(instance, dev->busType_u.spi.speed) != status_success) {
            printf("hpm_spi_set_sclk_frequency fail\n");
            while (1) {
            }
        }
        bus->busType_u.spi.speed = dev->busType_u.spi.speed;
    }

    if (dev->busType_u.spi.leadingEdge != bus->busType_u.spi.leadingEdge) {

        // Apply setting
        if (dev->busType_u.spi.leadingEdge) {
            spi_set_clock_phase(instance, spi_sclk_sampling_odd_clk_edges);
        } else {
            spi_set_clock_phase(instance, spi_sclk_sampling_even_clk_edges);
        }
        bus->busType_u.spi.leadingEdge = dev->busType_u.spi.leadingEdge;
    }

    // Check that any there are no attempts to DMA to/from CCD SRAM
    for (busSegment_t *checkSegment = (busSegment_t *)bus->curSegment; checkSegment->len; checkSegment++) {
        // Check there is no receive data as only transmit DMA is available
        if (((checkSegment->u.buffers.rxData) && (bus->dmaRx == (dmaChannelDescriptor_t *)NULL)) ||
            (checkSegment->u.buffers.txData)) {
            dmaSafe = false;
            break;
        }
        // Note that these counts are only valid if dmaSafe is true
        segmentCount++;
        xferLen += checkSegment->len;
    }

    if (bus->useDMA && dmaSafe && (!bus->curSegment[segmentCount].negateCS)) {
        // Intialise the init structures for the first transfer
        spiInternalInitStream(dev, false);

        // Assert Chip Select
        IOLo(dev->busType_u.spi.csnPin);

        // Start the transfers
        spiInternalStartDMA(dev);
    } else {
        busSegment_t *lastSegment = NULL;
        bool segmentComplete;

        // Manually work through the segment list performing a transfer for each
        while (bus->curSegment->len) {
            if (!lastSegment || lastSegment->negateCS) {
                // Assert Chip Select if necessary - it's costly so only do so if necessary
                IOLo(dev->busType_u.spi.csnPin);
            }

            spiInternalReadWriteBufPolled(
                    bus->busType_u.spi.instance,
                    bus->curSegment->u.buffers.txData,
                    bus->curSegment->u.buffers.rxData,
                    bus->curSegment->len);


            if (bus->curSegment->negateCS) {
                // Negate Chip Select
                IOHi(dev->busType_u.spi.csnPin);
            }

            segmentComplete = true;
            if (bus->curSegment->callback) {
                switch(bus->curSegment->callback(dev->callbackArg)) {
                case BUS_BUSY:
                    // Repeat the last DMA segment
                    segmentComplete = false;
                    break;

                case BUS_ABORT:
                    bus->curSegment = (busSegment_t *)BUS_SPI_FREE;
                    segmentComplete = false;
                    return;

                case BUS_READY:
                default:
                    // Advance to the next DMA segment
                    break;
                }
            }
            if (segmentComplete) {
                lastSegment = (busSegment_t *)bus->curSegment;
                bus->curSegment++;
            }
        }

        // If a following transaction has been linked, start it
        if (bus->curSegment->u.link.dev) {
            busSegment_t *endSegment = (busSegment_t *)bus->curSegment;
            const extDevice_t *nextDev = endSegment->u.link.dev;
            busSegment_t *nextSegments = (busSegment_t *)endSegment->u.link.segments;
            bus->curSegment = nextSegments;
            endSegment->u.link.dev = NULL;
            endSegment->u.link.segments = NULL;
            spiSequenceStart(nextDev);
        } else {
            // The end of the segment list has been reached, so mark transactions as complete
            bus->curSegment = (busSegment_t *)BUS_SPI_FREE;
        }
    }
}
#endif
