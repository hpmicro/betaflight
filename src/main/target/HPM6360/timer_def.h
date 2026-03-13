/*
 * Copyright (c) 2026 HPMicro
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */

#pragma once
#include "platform.h"
#include "common/utils.h"

typedef enum
{
  GPIO_MUX_0                             = 0x00, /*!< gpio muxing function selection 0 */
  GPIO_MUX_1                             = 0x01, /*!< gpio muxing function selection 1 */
  GPIO_MUX_2                             = 0x02, /*!< gpio muxing function selection 2 */
  GPIO_MUX_3                             = 0x03, /*!< gpio muxing function selection 3 */
  GPIO_MUX_4                             = 0x04, /*!< gpio muxing function selection 4 */
  GPIO_MUX_5                             = 0x05, /*!< gpio muxing function selection 5 */
  GPIO_MUX_6                             = 0x06, /*!< gpio muxing function selection 6 */
  GPIO_MUX_7                             = 0x07, /*!< gpio muxing function selection 7 */
  GPIO_MUX_8                             = 0x08, /*!< gpio muxing function selection 8 */
  GPIO_MUX_9                             = 0x09, /*!< gpio muxing function selection 9 */
  GPIO_MUX_10                            = 0x0A, /*!< gpio muxing function selection 10 */
  GPIO_MUX_11                            = 0x0B, /*!< gpio muxing function selection 11 */
  GPIO_MUX_12                            = 0x0C, /*!< gpio muxing function selection 12 */
  GPIO_MUX_13                            = 0x0D, /*!< gpio muxing function selection 13 */
  GPIO_MUX_14                            = 0x0E, /*!< gpio muxing function selection 14 */
  GPIO_MUX_15                            = 0x0F  /*!< gpio muxing function selection 15 */
} gpio_mux_sel_type;

#define DEF_TIM_CH_GET(ch) CONCAT2(DEF_TIM_CH__, ch)
#define DEF_TIM_CH__CH_CH0  D(0, 0)
#define DEF_TIM_CH__CH_CH1  D(1, 0)
#define DEF_TIM_CH__CH_CH2  D(2, 0)
#define DEF_TIM_CH__CH_CH3  D(3, 0)
#define DEF_TIM_CH__CH_CH4  D(4, 0)
#define DEF_TIM_CH__CH_CH5  D(5, 0)
#define DEF_TIM_CH__CH_CH6  D(6, 0)
#define DEF_TIM_CH__CH_CH7  D(7, 0)
#define DEF_TIM_CH__CH_CH8  D(8, 0)
#define DEF_TIM_CH__CH_CH9  D(9, 0)
#define DEF_TIM_CH__CH_CH10  D(10, 0)
#define DEF_TIM_CH__CH_CH11  D(11, 0)
#define DEF_TIM_CH__CH_CH0N D(0, 1)
#define DEF_TIM_CH__CH_CH1N D(1, 1)
#define DEF_TIM_CH__CH_CH2N D(2, 1)
#define DEF_TIM_CH__CH_CH3N D(3, 1)
#define DEF_TIM_CH__CH_CH4N D(4, 1)
#define DEF_TIM_CH__CH_CH5N D(5, 1)
#define DEF_TIM_CH__CH_CH6N D(6, 1)
#define DEF_TIM_CH__CH_CH7N D(7, 1)
#define DEF_TIM_CH__CH_CH8N D(8, 1)
#define DEF_TIM_CH__CH_CH9N D(9, 1)
#define DEF_TIM_CH__CH_CH10N  D(10, 1)
#define DEF_TIM_CH__CH_CH11N  D(11, 1)
#define USED_TIMERS  ( BIT(1) | BIT(2) )
#define HARDWARE_TIMER_DEFINITION_COUNT    2

#define TIMER_GET_IO_TAG(pin) DEFIO_TAG(pin)

#define DEF_TIM_CHANNEL(ch)                   CONCAT(DEF_TIM_CHANNEL__, DEF_TIM_CH_GET(ch))
#define DEF_TIM_CHANNEL__D(chan_n, n_channel) chan_n
#define DEF_TIM_OUTPUT(ch)         CONCAT(DEF_TIM_OUTPUT__, DEF_TIM_CH_GET(ch))
#define DEF_TIM_OUTPUT__D(chan_n, n_channel) PP_IIF(n_channel, TIMER_OUTPUT_N_CHANNEL, TIMER_OUTPUT_NONE)

#define DEF_TIM_AF(timch, pin)                CONCAT(DEF_TIM_AF__, DEF_TIM_AF_GET(timch, pin))
#define DEF_TIM_AF__D(af_n, tim_n)            GPIO_MUX_ ## af_n  /*GPIO_MUX_1 gpio_mux_sel_type */

//  Parameters in D(...) are target-specific
#define DEF_TIM_AF_GET(timch, pin) CONCAT4(DEF_TIM_AF__, pin, __, timch)
/*
 DEF_TIM(tim, chan, pin, flags, out, dmaopt, upopt)
        @tim,
        @chan    tmr & channel
        @pin     output pin
        @out     0 for normal 1 for N_Channel
*/

#define DEF_TIM(tim, chan, chan2, pin, out, d, c, af, af2, cmp, trg, trgsrc, trggrp, dmasrc) {              \
    tim,                                                                \
    TIMER_GET_IO_TAG(pin),                                              \
    DEF_TIM_CHANNEL(CH_ ## chan),                                       \
    DEF_TIM_CHANNEL(CH_ ## chan2),                                       \
    (DEF_TIM_OUTPUT(CH_ ## chan) | out),                                \
    .dmaRef = d ## _Channel ## c,                                       \
    .alternateFunction = af,                                            \
    .palternateFunction = af2,                                           \
    .cmp_index = cmp,                                                   \
    .trgm = trg,\
    .trgm_src = trgsrc,\
    .trg_grp = trggrp,\
    .dmamuxsrc = dmasrc,\
    .dmaChannel = 0,\
}

#define GPTMR_CAPTURE_IN_CH_POS     2
#define GPTMR_CAPTURE_IN_CH_NEG     3

typedef struct gptmr_input_cap_source {
    //trgmux map signal to gptmr pos edge capture
    TRGM_Type *trgm_pos;
    uint32_t trgmux_in_pos;
    uint32_t trgmux_out_pos;
    uint32_t dmamux_src_pos;

    //trgmux map signal to gptmr neg edge capture
    TRGM_Type *trgm_neg;
    uint32_t trgmux_in_neg;
    uint32_t trgmux_out_neg;
    uint32_t dmamux_src_neg;

    //trgmux map signal to middle link signal, perharps not needed some time
    TRGM_Type *trgm_ref;
    uint32_t trgmux_in_ref;
    uint32_t trgmux_out_ref;
} gptmr_input_cap_source_t;


typedef struct pwm_dshot_trgm_source {
    TRGM_Type *trgm;
    uint32_t trgm_dma_src;
    uint8_t trg_grp;
    uint32_t dmamuxsrc;
} pwm_dshot_trgm_source_t;

/* define dshot telemetry input capture resources map
 * for example: DHSOT_TELEMETRY_CAPTURE_RESOURCE(P9, GPTMR5, TRGM2, TRGM2)
 */
#define DHSOT_TELEMETRY_CAPTURE_RESOURCE(in, gptmr, trgm1, trgm2)    \
.in_cap_trgm_map = { \
    .trgm_pos = HPM_ ## trgm1, \
    .trgmux_in_pos = HPM_ ## trgm1 ## _INPUT_SRC_ ## trgm1 ## _ ## in, \
    .trgmux_out_pos = HPM_ ## trgm1 ## _OUTPUT_SRC_ ## gptmr ## _IN2, \
    .dmamux_src_pos = HPM_DMA_SRC_ ## gptmr ## _2, \
    .trgm_neg = HPM_ ## trgm2, \
    .trgmux_in_neg = HPM_ ## trgm2 ## _INPUT_SRC_ ## trgm2 ## _ ## in, \
    .trgmux_out_neg = HPM_ ## trgm2 ## _OUTPUT_SRC_ ## gptmr ## _IN3, \
    .dmamux_src_neg = HPM_DMA_SRC_ ## gptmr ## _3, \
    .trgm_ref = (TRGM_Type *)(-1), \
    .trgmux_in_ref = -1, \
    .trgmux_out_ref = -1, \
},
/* For example:DSHOT_DMA_CHANNEL_RESOURCE(XDMA,XDMA,XDMA,3,3,7) */
#define DSHOT_DMA_CHANNEL_RESOURCE(a,b,c,chn,ch_neg,ch_pos)   \
        .irqn = IRQn_ ## a, \
        .gptmr_dma_ch_pos = ch_pos, \
        .gptmr_dma_ch_neg = ch_neg, \
        .dma_base = HPM_ ## a, \
        .dmaRef = HPM_ ## a ## _Channel ## chn, \
        .dmaCapNeg = HPM_ ## b ## _Channel ## ch_neg, \
        .dmaCapPos = HPM_ ## c ## _Channel ## ch_pos, \
        .dma_neg = HPM_ ## b, \
        .dma_pos = HPM_ ## c, \
        .dmaChannel = chn,
/* For example: DSHOT_PWM_TRGM_SOURCE(PWM,1,CMP,10,TRGM,1,DMACFG,0) */
#define DSHOT_PWM_TRGM_SOURCE(pwm, index, cmp,cmpidx, trg, trgmidx, grp, grpidx) \
.pwm_trgm = { \
    .trgm = HPM_TRGM ## trgmidx, \
    .trgm_dma_src = HPM_TRGM ## trgmidx ## _DMA_SRC_ ## PWM ## index ## _CMP ## cmpidx, \
    .trg_grp = TRGM_DMACFG_ ## grpidx, \
    .dmamuxsrc = HPM_DMA_SRC_MOT ## index ## _ ## grpidx, \
},
