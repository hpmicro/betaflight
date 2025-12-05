/*
 * Copyright (c) 2025 HPMicro
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */
#ifndef _DMA_HPMICRO_H
#define _DMA_HPMICRO_H
#ifdef HPMSOC_HAS_HPMSDK_DMAV2
#include "hpm_dmav2_drv.h"
#else
#include "hpm_dma_drv.h"
#endif

/* @brief Channel config */
typedef struct dma_handshake_config_fixed {
    uint32_t dst;
    uint32_t src;
    uint32_t size_in_byte;
    uint8_t data_width;            /* data width, value defined by DMA_TRANSFER_WIDTH_xxx */
    uint8_t ch_index;
    bool dst_fixed;
    bool src_fixed;
    uint16_t interrupt_mask;        /**< Interrupt mask */
} dma_handshake_config_fixed_t;

hpm_stat_t dma_setup_handshake_fixed(DMA_Type *ptr,  dma_handshake_config_fixed_t *pconfig, bool start_transfer);

#endif