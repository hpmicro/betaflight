/*
 * Copyright (c) 2026 HPMicro
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */

#include "platform.h"
#include "drivers/timer.h"
#include "hpm_clock_drv.h"
#include "hpm_soc.h"
#include "hpm_iomux.h"

const timerDef_t timerDefinitions[HARDWARE_TIMER_DEFINITION_COUNT] = {
    { .TIMx = HPM_PWM0,  .rcc = clock_mot0,  .inputIrq = IRQn_PWM0},
    { .TIMx = HPM_PWM1,  .rcc = clock_mot1,  .inputIrq = IRQn_PWM1},

};
const timerHardware_t fullTimerHardware[FULL_TIMER_CHANNEL_COUNT] = {
    /* PWM1 */
    {
        .tim = HPM_PWM0,
#ifdef HPM_USE_PWM_OUTPUT_DSHOT
        .tag = TIMER_GET_IO_TAG(PC0),
        .channel = DEF_TIM_CHANNEL(CH_CH0),
#else
        .channel = DEF_TIM_CHANNEL(CH_CH8),
#endif
        .gptmr_clock = clock_gptmr0,
        .gptmr = HPM_GPTMR0,
        .channel_ref = DEF_TIM_CHANNEL(CH_CH4),
        .cap_pin = TIMER_GET_IO_TAG(PB26),
        .dma_req_cmp_index = 10,
        .output = (DEF_TIM_OUTPUT(CH_CH5) | 0),
        .alternateFunction = IOC_PC00_FUNC_CTL_PWM0_P_0,
        .cmp_index = 0,
#ifndef HPM_USE_PWM_OUTPUT_DSHOT
        .pwm_out_trgm_src = HPM_TRGM0_INPUT_SRC_PWM0_CH8REF,
        .pwm_out_trgm_dst = HPM_TRGM0_OUTPUT_SRC_TRGM0_P6,
        .trgm_port_idx = 6,
#endif
        /*For dual-direct dshot*/
        .gptmr_io_function = IOC_PB26_FUNC_CTL_TRGM0_P_06,
        DSHOT_DMA_CHANNEL_RESOURCE(XDMA, HDMA, XDMA, 0,0,4)
        DSHOT_PWM_TRGM_SOURCE(PWM,0,CMP,10,TRGM,0,DMACFG,0)
        DHSOT_TELEMETRY_CAPTURE_RESOURCE(P6, GPTMR0, TRGM0, TRGM0)
    },
    {
        .tim = HPM_PWM0,
#ifdef HPM_USE_PWM_OUTPUT_DSHOT
        .tag = TIMER_GET_IO_TAG(PC1),
        .channel = DEF_TIM_CHANNEL(CH_CH1),
#else
        .channel = DEF_TIM_CHANNEL(CH_CH9),
#endif
        .gptmr_clock = clock_gptmr1,
        .gptmr = HPM_GPTMR1,
        .channel_ref = DEF_TIM_CHANNEL(CH_CH5),
        .cap_pin = TIMER_GET_IO_TAG(PB30),
        .dma_req_cmp_index = 11,
        .output = (DEF_TIM_OUTPUT(CH_CH5) | 0),
        .alternateFunction = IOC_PC01_FUNC_CTL_PWM0_P_1,
        .cmp_index = 2,
#ifndef HPM_USE_PWM_OUTPUT_DSHOT
        .pwm_out_trgm_src = HPM_TRGM0_INPUT_SRC_PWM0_CH9REF,
        .pwm_out_trgm_dst = HPM_TRGM0_OUTPUT_SRC_TRGM0_P10,
        .trgm_port_idx = 10,
#endif
        /*For dual-direct dshot*/
        .gptmr_io_function = IOC_PB30_FUNC_CTL_TRGM0_P_10,
        DSHOT_DMA_CHANNEL_RESOURCE(XDMA, HDMA, XDMA, 1,1,5)
        DSHOT_PWM_TRGM_SOURCE(PWM,0,CMP,11,TRGM,0,DMACFG,1)
        DHSOT_TELEMETRY_CAPTURE_RESOURCE(P10, GPTMR1, TRGM0, TRGM0)
    },
    {
        .tim = HPM_PWM0,
#ifdef HPM_USE_PWM_OUTPUT_DSHOT
        .tag = TIMER_GET_IO_TAG(PC2),
        .channel = DEF_TIM_CHANNEL(CH_CH2),
#else
        .channel = DEF_TIM_CHANNEL(CH_CH10),
#endif
        .gptmr_clock = clock_gptmr2,
        .gptmr = HPM_GPTMR2,
        .channel_ref = DEF_TIM_CHANNEL(CH_CH6),
        .cap_pin = TIMER_GET_IO_TAG(PB29),
        .dma_req_cmp_index = 12,
        .output = (DEF_TIM_OUTPUT(CH_CH5) | 0),
        .alternateFunction = IOC_PC02_FUNC_CTL_PWM0_P_2,
        .cmp_index = 4,
#ifndef HPM_USE_PWM_OUTPUT_DSHOT
        .pwm_out_trgm_src = HPM_TRGM0_INPUT_SRC_PWM0_CH10REF,
        .pwm_out_trgm_dst = HPM_TRGM0_OUTPUT_SRC_TRGM0_P9,
        .trgm_port_idx = 9,
#endif
        /*For dual-direct dshot*/
        .gptmr_io_function = IOC_PB30_FUNC_CTL_TRGM0_P_10,
        DSHOT_DMA_CHANNEL_RESOURCE(XDMA, HDMA, XDMA, 2,2,6)
        DSHOT_PWM_TRGM_SOURCE(PWM,0,CMP,12,TRGM,0,DMACFG,2)
        .in_cap_trgm_map = {
            .trgm_pos = HPM_TRGM1,
            .trgmux_in_pos = HPM_TRGM1_INPUT_SRC_TRGM0_OUTX0,
            .trgmux_out_pos = HPM_TRGM1_OUTPUT_SRC_GPTMR2_IN2,
            .dmamux_src_pos = HPM_DMA_SRC_GPTMR2_2, \
            .trgm_neg = HPM_TRGM1, \
            .trgmux_in_neg = HPM_TRGM1_INPUT_SRC_TRGM0_OUTX0,
            .trgmux_out_neg = HPM_TRGM1_OUTPUT_SRC_GPTMR2_IN3,
            .dmamux_src_neg = HPM_DMA_SRC_GPTMR2_3,
            .trgm_ref = HPM_TRGM0,
            .trgmux_in_ref = HPM_TRGM0_INPUT_SRC_TRGM0_P9,
            .trgmux_out_ref = HPM_TRGM0_OUTPUT_SRC_TRGM0_OUTX0,
        },
    },
    {
        .tim = HPM_PWM0,
#ifdef HPM_USE_PWM_OUTPUT_DSHOT
        .tag = TIMER_GET_IO_TAG(PC3),
        .channel = DEF_TIM_CHANNEL(CH_CH3),
#else
        .channel = DEF_TIM_CHANNEL(CH_CH11),
#endif
        .gptmr_clock = clock_gptmr3,
        .gptmr = HPM_GPTMR3,
        .channel_ref = DEF_TIM_CHANNEL(CH_CH7),
        .cap_pin = TIMER_GET_IO_TAG(PB28),
        .dma_req_cmp_index = 13,
        .output = (DEF_TIM_OUTPUT(CH_CH4) | 0),
        .alternateFunction = IOC_PC03_FUNC_CTL_PWM0_P_3,
        .cmp_index = 6,
#ifndef HPM_USE_PWM_OUTPUT_DSHOT
        .pwm_out_trgm_src = HPM_TRGM0_INPUT_SRC_PWM0_CH11REF,
        .pwm_out_trgm_dst = HPM_TRGM0_OUTPUT_SRC_TRGM0_P8,
        .trgm_port_idx = 8,
#endif
        /*For dual-direct dshot*/
        .gptmr_io_function = IOC_PB28_FUNC_CTL_TRGM0_P_08,
        DSHOT_DMA_CHANNEL_RESOURCE(XDMA, HDMA, XDMA, 3,3,7)
        DSHOT_PWM_TRGM_SOURCE(PWM,0,CMP,13,TRGM,0,DMACFG,3)
        .in_cap_trgm_map = {
            .trgm_pos = HPM_TRGM1,
            .trgmux_in_pos = HPM_TRGM1_INPUT_SRC_TRGM0_OUTX1,
            .trgmux_out_pos = HPM_TRGM1_OUTPUT_SRC_GPTMR3_IN2,
            .dmamux_src_pos = HPM_DMA_SRC_GPTMR3_2, \
            .trgm_neg = HPM_TRGM1, \
            .trgmux_in_neg = HPM_TRGM1_INPUT_SRC_TRGM0_OUTX1,
            .trgmux_out_neg = HPM_TRGM1_OUTPUT_SRC_GPTMR3_IN3,
            .dmamux_src_neg = HPM_DMA_SRC_GPTMR3_3,
            .trgm_ref = HPM_TRGM0,
            .trgmux_in_ref = HPM_TRGM0_INPUT_SRC_TRGM0_P8,
            .trgmux_out_ref = HPM_TRGM0_OUTPUT_SRC_TRGM0_OUTX1,
        },
    }
};
void timerInit(void)
{
    /* enable the timer peripherals */
    for (unsigned i = 0; i < TIMER_CHANNEL_COUNT; i++) {
        
        clock_add_to_group(timerRCC(TIMER_HARDWARE[i].tim), 0);
    }
}

rccPeriphTag_t timerRCC(TIM_TypeDef *tim)
{
    for (int i = 0; i < HARDWARE_TIMER_DEFINITION_COUNT; i++) {
        if (timerDefinitions[i].TIMx == tim) {
            return timerDefinitions[i].rcc;
        }
    }
    return 0;
}

FAST_CODE volatile timCCR_t* timerChCCR(const timerHardware_t *timHw)
{
#ifdef HPM6360
    return (volatile timCCR_t*)((volatile char*)&timHw->tim->CMP[timHw->cmp_index]);
#else
    return (volatile timCCR_t*)((volatile char*)&timHw->tim->SHADOW_VAL[timHw->cmp_index + 1]);
#endif
}
