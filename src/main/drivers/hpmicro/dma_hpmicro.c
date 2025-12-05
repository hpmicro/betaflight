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

#include "platform.h"

#ifdef USE_DMA
#include "dma_hpmicro.h"
#include "drivers/dma.h"
#include "hpm_clock_drv.h"

/*
 * DMA descriptors.
 */
dmaChannelDescriptor_t dmaDescriptors[DMA_LAST_HANDLER] = {
    DEFINE_DMA_CHANNEL(HPM_HDMA, 0,  IRQn_HDMA, clock_hdma),
    DEFINE_DMA_CHANNEL(HPM_HDMA, 1,  IRQn_HDMA, clock_hdma),
    DEFINE_DMA_CHANNEL(HPM_HDMA, 2,  IRQn_HDMA, clock_hdma),
    DEFINE_DMA_CHANNEL(HPM_HDMA, 3, IRQn_HDMA, clock_hdma),
    DEFINE_DMA_CHANNEL(HPM_HDMA, 4, IRQn_HDMA, clock_hdma),
    DEFINE_DMA_CHANNEL(HPM_HDMA, 5, IRQn_HDMA, clock_hdma),
    DEFINE_DMA_CHANNEL(HPM_HDMA, 6, IRQn_HDMA, clock_hdma),
    DEFINE_DMA_CHANNEL(HPM_HDMA, 7, IRQn_HDMA, clock_hdma),

    DEFINE_DMA_CHANNEL(HPM_XDMA, 0,  IRQn_XDMA, clock_xdma),
    DEFINE_DMA_CHANNEL(HPM_XDMA, 1,  IRQn_XDMA, clock_xdma),
    DEFINE_DMA_CHANNEL(HPM_XDMA, 2, IRQn_XDMA, clock_xdma),
    DEFINE_DMA_CHANNEL(HPM_XDMA, 3, IRQn_XDMA, clock_xdma),
    DEFINE_DMA_CHANNEL(HPM_XDMA, 4, IRQn_XDMA, clock_xdma),
    DEFINE_DMA_CHANNEL(HPM_XDMA, 5, IRQn_XDMA, clock_xdma),
    DEFINE_DMA_CHANNEL(HPM_XDMA, 6, IRQn_XDMA, clock_xdma),
    DEFINE_DMA_CHANNEL(HPM_XDMA, 7, IRQn_XDMA, clock_xdma),

};

hpm_stat_t dma_setup_handshake_fixed(DMA_Type *ptr,  dma_handshake_config_fixed_t *pconfig, bool start_transfer)
{
    hpm_stat_t stat = status_success;
    dma_channel_config_t config = {0};
    dma_default_channel_config(ptr, &config);

    if (true == pconfig->dst_fixed) {
        config.dst_addr_ctrl = DMA_ADDRESS_CONTROL_FIXED;
        config.dst_mode = DMA_HANDSHAKE_MODE_HANDSHAKE;
    }
    if (true == pconfig->src_fixed) {
        config.src_addr_ctrl = DMA_ADDRESS_CONTROL_FIXED;
        config.src_mode = DMA_HANDSHAKE_MODE_HANDSHAKE;
    }

    if (pconfig->ch_index >= DMA_SOC_CHANNEL_NUM) {
        return status_invalid_argument;
    }

    config.src_width = pconfig->data_width;
    config.dst_width = pconfig->data_width;
    config.src_addr = pconfig->src;
    config.dst_addr = pconfig->dst;
    config.size_in_byte = pconfig->size_in_byte;
    config.interrupt_mask = pconfig->interrupt_mask;
    /*  In DMA handshake case, source burst size must be 1 transfer, that is 0. */
    config.src_burst_size = 0;
    stat = dma_setup_channel(ptr, pconfig->ch_index, &config, start_transfer);
    if (stat != status_success) {
        return stat;
    }
    return stat;
}

void hdma_isr(void)
{
    uint32_t int_stat = HPM_HDMA->INTSTATUS;
    HPM_HDMA->INTSTATUS = int_stat;
    for (int i = DMA1_CH1_HANDLER; i <= DMA1_CH8_HANDLER; i++) {
        const uint8_t index = DMA_IDENTIFIER_TO_INDEX(i);
        dmaCallbackHandlerFuncPtr handler = dmaDescriptors[index].irqHandlerCallback;
        if (handler) {
            dmaDescriptors[index].int_stat = int_stat;
            handler(&dmaDescriptors[index]);
        }
    }
}
SDK_DECLARE_EXT_ISR_M(IRQn_HDMA, hdma_isr)

void xdma_isr(void)
{
    uint32_t int_stat = HPM_XDMA->INTSTATUS;
    HPM_XDMA->INTSTATUS = int_stat;
    for (int i = DMA2_CH1_HANDLER; i <= DMA2_CH8_HANDLER; i++) {
        const uint8_t index = DMA_IDENTIFIER_TO_INDEX(i);
        dmaCallbackHandlerFuncPtr handler = dmaDescriptors[index].irqHandlerCallback;
        if (handler) {
            dmaDescriptors[index].int_stat = int_stat;
            handler(&dmaDescriptors[index]);
        }
    }
}
SDK_DECLARE_EXT_ISR_M(IRQn_XDMA, xdma_isr)

void dmaEnable(dmaIdentifier_e identifier)
{
    (void)identifier;
}

void dmaSetHandler(dmaIdentifier_e identifier, dmaCallbackHandlerFuncPtr callback, uint32_t priority, uint32_t userParam)
{
    const int index = DMA_IDENTIFIER_TO_INDEX(identifier);
    dmaChannelDescriptor_t *desc = &dmaDescriptors[index];
    clock_add_to_group(desc->rcc, 0);
    desc->irqHandlerCallback = callback;
    desc->userParam = userParam;
    dma_enable_channel_interrupt(desc->dma, desc->channel, DMA_INTERRUPT_MASK_TERMINAL_COUNT);
    intc_m_enable_irq_with_priority(desc->irqN, priority == 0 ? 1 : (priority > 7 ? 7 : priority));
}
#endif
