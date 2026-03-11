/*
 * Copyright (c) 2025-2026 HPMicro
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
    { .TIMx = HPM_PWM2,  .rcc = clock_mot2,  .inputIrq = IRQn_PWM2},
    { .TIMx = HPM_PWM3,  .rcc = clock_mot3,  .inputIrq = IRQn_PWM3},

};
const timerHardware_t fullTimerHardware[FULL_TIMER_CHANNEL_COUNT] = {
    /* PWM1 */
    {
        .tim = HPM_PWM1,
        .tag = TIMER_GET_IO_TAG(PA25),
        .channel = DEF_TIM_CHANNEL(CH_CH4),
        .gptmr_clock = clock_gptmr2,
        .gptmr = HPM_GPTMR2,
        .channel_ref = DEF_TIM_CHANNEL(CH_CH0),
        .cap_pin = TIMER_GET_IO_TAG(PB7),
        .dma_req_cmp_index = 10,
        .output = (DEF_TIM_OUTPUT(CH_CH5) | 0),
        .alternateFunction = 0x10,
        .palternateFunction = 0x10,
        .cmp_index = 0,
        /*For dual-direct dshot*/
        .gptmr_io_function = IOC_PB07_FUNC_CTL_TRGM1_P_02,
        DSHOT_DMA_CHANNEL_RESOURCE(XDMA, HDMA, XDMA, 0,0,4)
        DSHOT_PWM_TRGM_SOURCE(PWM,1,CMP,10,TRGM,1,DMACFG,0)
        DHSOT_TELEMETRY_CAPTURE_RESOURCE(P2, GPTMR2, TRGM1, TRGM1)
    },
    {
        .tim = HPM_PWM1,
        .tag = TIMER_GET_IO_TAG(PA24),
        .channel = DEF_TIM_CHANNEL(CH_CH5),
        .gptmr_clock = clock_gptmr3,
        .gptmr = HPM_GPTMR3,
        .channel_ref = DEF_TIM_CHANNEL(CH_CH1),
        .cap_pin = TIMER_GET_IO_TAG(PB13),
        .dma_req_cmp_index = 11,
        .output = (DEF_TIM_OUTPUT(CH_CH5) | 0),
        .alternateFunction = 0x10,
        .palternateFunction = 0x10,
        .cmp_index = 2,
        /*For dual-direct dshot*/
        .gptmr_io_function = IOC_PB13_FUNC_CTL_TRGM1_P_03,
        DSHOT_DMA_CHANNEL_RESOURCE(XDMA, HDMA, XDMA, 1,1,5)
        DSHOT_PWM_TRGM_SOURCE(PWM,1,CMP,11,TRGM,1,DMACFG,1)
        DHSOT_TELEMETRY_CAPTURE_RESOURCE(P3, GPTMR3, TRGM1, TRGM1)
    },
    {
        .tim = HPM_PWM1,
        .tag = TIMER_GET_IO_TAG(PA20),
        .channel = DEF_TIM_CHANNEL(CH_CH6),
        .gptmr_clock = clock_gptmr4,
        .gptmr = HPM_GPTMR4,
        .channel_ref = DEF_TIM_CHANNEL(CH_CH2),
        .cap_pin = TIMER_GET_IO_TAG(PC16),
        .dma_req_cmp_index = 12,
        .output = (DEF_TIM_OUTPUT(CH_CH5) | 0),
        .alternateFunction = 0x10,
        .palternateFunction = 0x10,
        .cmp_index = 4,
        /*For dual-direct dshot*/
        .gptmr_io_function = IOC_PC16_FUNC_CTL_TRGM2_P_02,
        DSHOT_DMA_CHANNEL_RESOURCE(XDMA, HDMA, XDMA, 2,2,6)
        DSHOT_PWM_TRGM_SOURCE(PWM,1,CMP,12,TRGM,1,DMACFG,2)
        DHSOT_TELEMETRY_CAPTURE_RESOURCE(P2, GPTMR4, TRGM2, TRGM2)
    },
    {
        .tim = HPM_PWM1,
        .tag = TIMER_GET_IO_TAG(PA19),
        .channel = DEF_TIM_CHANNEL(CH_CH7),
        .gptmr_clock = clock_gptmr5,
        .gptmr = HPM_GPTMR5,
        .channel_ref = DEF_TIM_CHANNEL(CH_CH3),
        .cap_pin = TIMER_GET_IO_TAG(PC21),
        .dma_req_cmp_index = 13,
        .output = (DEF_TIM_OUTPUT(CH_CH4) | 0),
        .alternateFunction = 0x10,
        .palternateFunction = 0x10,
        .cmp_index = 6,
        /*For dual-direct dshot*/
        .gptmr_io_function = IOC_PC21_FUNC_CTL_TRGM2_P_03,
        DSHOT_DMA_CHANNEL_RESOURCE(XDMA, HDMA, XDMA, 3,3,7)
        DSHOT_PWM_TRGM_SOURCE(PWM,1,CMP,13,TRGM,1,DMACFG,3)
        DHSOT_TELEMETRY_CAPTURE_RESOURCE(P3, GPTMR5, TRGM2, TRGM2)
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
#ifdef HPM6750
    return (volatile timCCR_t*)((volatile char*)&timHw->tim->CMP[timHw->cmp_index]);
#else
    return (volatile timCCR_t*)((volatile char*)&timHw->tim->SHADOW_VAL[timHw->cmp_index + 1]);
#endif
}
