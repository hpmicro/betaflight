/*
 * Copyright (c) 2025 HPMicro
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "platform.h"

#ifdef USE_DSHOT

#include "drivers/dma.h"
#include "drivers/io.h"
#include "drivers/time.h"
#include "drivers/timer.h"
#include "drivers/system.h"

#include "hpm_soc.h"
#ifdef HPMSOC_HAS_HPMSDK_DMAV2
#include "hpm_dmav2_drv.h"
#else
#include "hpm_dma_drv.h"
#endif
#ifdef HPMSOC_HAS_HPMSDK_DMAV2
#include "hpm_pwmv2_drv.h"
#else
#include "hpm_pwm_drv.h"
#endif
#include "hpm_clock_drv.h"
#include "drivers/dshot.h"
#include "drivers/dshot_dpwm.h"
#include "drivers/dshot_command.h"
#include "drivers/pwm_output_dshot_shared.h"
#include "hpm_trgm_drv.h"
#include "hpm_dmamux_drv.h"
#include "hpm_gptmr_drv.h"

uint32_t dshot_duty_count = 0;
#ifdef USE_DSHOT_TELEMETRY
uint32_t dshot_telemetry_bit_width = 0;
dma_channel_config_t dshot_dma_config[MAX_SUPPORTED_MOTORS] = { 0 };
dma_channel_config_t dshot_cap_pos_edge_config[MAX_SUPPORTED_MOTORS] = { 0 };
dma_channel_config_t dshot_cap_neg_edge_config[MAX_SUPPORTED_MOTORS] = { 0 };
#endif

#if defined(HPMSOC_HAS_HPMSDK_PWM) && defined(HPMSOC_HAS_HPMSDK_DMA)
FAST_CODE void pwmDshotSetDirectionOutput(motorDmaOutput_t * const motor)
{
    const timerHardware_t *time_hw = motor->timerHardware;
#ifdef HPM_USE_PWM_OUTPUT_DSHOT
    const IO_t motorIO = IOGetByTag(time_hw->tag);
#endif
    //Disable DSHOT input dma channels
    dma_disable_channel(time_hw->dma_neg, time_hw->gptmr_dma_ch_neg);
    dma_disable_channel(time_hw->dma_pos, time_hw->gptmr_dma_ch_pos);

    //Set IO to output and enable PWM channel

#ifdef HPM_USE_PWM_OUTPUT_DSHOT
    IOConfigGPIOAF(motorIO, IOCFG_AF_PP, time_hw->alternateFunction, time_hw->palternateFunction);
#else
    trgm_enable_io_output(time_hw->pwm_trgm.trgm, 1 << (time_hw->trgm_port_idx));
#endif
    pwm_enable_output(time_hw->tim, time_hw->channel);

    //Map dma channel to output dma request
    dmamux_config(HPM_DMAMUX, \
                  DMA_SOC_CHN_TO_DMAMUX_CHN(time_hw->dma_base, time_hw->dmaChannel), \
                  time_hw->pwm_trgm.dmamuxsrc, \
                  true);

    //Clear input data
    memset(motor->dmaBuffer_neg_edge, 0, MAX_GCR_EDGES * sizeof(uint32_t));
    memset(motor->dmaBuffer_pos_edge, 0, MAX_GCR_EDGES * sizeof(uint32_t));
}


#ifdef USE_DSHOT_TELEMETRY
FAST_CODE static void pwmDshotSetDirectionInput(motorDmaOutput_t * const motor)
{
    motor->isInput = true;
    /* First set the output pin into input state, so it will not influence the gptmr input capture */
    const timerHardware_t *timer_hw = motor->timerHardware;
    const gptmr_input_cap_source_t *trg_resource = &timer_hw->in_cap_trgm_map;
#ifdef HPM_USE_PWM_OUTPUT_DSHOT
    const IO_t motorIO = IOGetByTag(timer_hw->tag);
#endif
    GPTMR_Type *gptmr_base = timer_hw->gptmr;

#ifdef HPM_USE_PWM_OUTPUT_DSHOT
    IOConfigGPIOAF(motorIO, IOCFG_IN_FLOATING, 0, 0);
#else
    trgm_disable_io_output(timer_hw->pwm_trgm.trgm, 1 << (timer_hw->trgm_port_idx));
#endif

    /* Now we restart gptmr input capture function */

    dmamux_config(HPM_DMAMUX, \
                  DMA_SOC_CHN_TO_DMAMUX_CHN(timer_hw->dma_neg, ((uint32_t)timer_hw->gptmr_dma_ch_neg)), \
                  trg_resource->dmamux_src_neg, \
                  true);

    if (status_success != dma_setup_channel(timer_hw->dma_neg, \
                                            timer_hw->gptmr_dma_ch_neg, \
                                            &dshot_cap_neg_edge_config[motor->index], \
                                            false)) {
        printf(" dma setup channel failed\n");
        while(1);
    }
    if (status_success != dma_setup_channel(timer_hw->dma_pos, \
                                            timer_hw->gptmr_dma_ch_pos, \
                                            &dshot_cap_pos_edge_config[motor->index], \
                                            false)) {
        printf(" dma setup channel failed\n");
        while(1);
    }

    if (!inputStampUs) {
        inputStampUs = micros();
    }
    gptmr_channel_reset_count(gptmr_base, GPTMR_CAPTURE_IN_CH_NEG);
    gptmr_channel_reset_count(gptmr_base, GPTMR_CAPTURE_IN_CH_POS);
    gptmr_start_counter(gptmr_base, GPTMR_CAPTURE_IN_CH_NEG);
    gptmr_start_counter(gptmr_base, GPTMR_CAPTURE_IN_CH_POS);


    gptmr_trigger_channel_software_sync(gptmr_base, 0xF);
    dma_enable_channel(timer_hw->dma_pos, timer_hw->gptmr_dma_ch_pos);
    dma_enable_channel(timer_hw->dma_neg, timer_hw->gptmr_dma_ch_neg);

}
#endif


void pwmCompleteDshotMotorUpdate(void)
{
    /* If there is a dshot command loaded up, time it correctly with motor update*/
    if (!dshotCommandQueueEmpty()) {
        if (!dshotCommandOutputIsEnabled(dshotPwmDevice.count)) {
            return;
        }
    }
}

FAST_CODE static void motor_dshot_transfer_done_handler(dmaChannelDescriptor_t *descriptor)
{
    motorDmaOutput_t * const motor = &dmaMotors[descriptor->userParam];
    uint32_t ch = (uint32_t)(motor->dmaRef) & 0xF;
    if (descriptor->int_stat & (1 << (DMA_STATUS_TC_SHIFT + ch))) {

#ifdef USE_DSHOT_TELEMETRY
        dshotDMAHandlerCycleCounters.irqAt = getCycleCounter();
#endif
        motorDmaOutput_t * const motor = &dmaMotors[descriptor->userParam];
        (void)motor;
#ifdef USE_DSHOT_TELEMETRY
        if (useDshotTelemetry) {
            pwmDshotSetDirectionInput(motor);

            dshotDMAHandlerCycleCounters.changeDirectionCompletedAt = getCycleCounter();
        }
#endif
    }
}

#ifdef USE_DSHOT_TELEMETRY

/**
 * @brief Setup DSHOT TELEMETRY related dma channel.
 *
 */
FAST_CODE static void setup_dshot_telemetry_dma(motorDmaOutput_t * const motor, motorPwmProtocolTypes_e pwmProtocolType)
{
    const timerHardware_t *timerHardware = motor->timerHardware;

    //Source Info From fullTimerHardware in timer_hpm6750.c
    DMA_Type *dma_neg = timerHardware->dma_neg;
    DMA_Type *dma_pos = timerHardware->dma_pos;
    GPTMR_Type *gptmr = timerHardware->gptmr;
    uint8_t motor_idx = motor->index;
    dma_channel_config_t *dma_config;

    gptmr_channel_config_t config;

    gptmr_channel_get_default_config(gptmr, &config);
    uint32_t gptmr_freq = clock_get_frequency(timerHardware->gptmr_clock);

    //dshot_reload_counter is used to decode dshot telemetry message
    dshot_telemetry_bit_width = gptmr_freq / getDshotHz(pwmProtocolType) * MOTOR_BITLENGTH * 4 / 5;

    config.reload = gptmr_freq / 10 - 1;
    config.enable_software_sync = true;
    config.dma_request_event = gptmr_dma_request_on_input_signal_toggle;

    config.mode = gptmr_work_mode_capture_at_falling_edge;
    gptmr_channel_config(gptmr, GPTMR_CAPTURE_IN_CH_NEG, &config, false);
    config.mode = gptmr_work_mode_capture_at_rising_edge;
    gptmr_channel_config(gptmr, GPTMR_CAPTURE_IN_CH_POS, &config, false);

    // Setup dshot_cap_neg_edge_config and dshot_cap_pos_edge_config to decrease change dshot direction time cost
    dma_config = &dshot_cap_neg_edge_config[motor_idx];
    dma_default_channel_config(dma_neg, dma_config);
    dma_config->src_mode = DMA_HANDSHAKE_MODE_HANDSHAKE;
    dma_config->src_width = DMA_TRANSFER_WIDTH_WORD;
    dma_config->src_addr_ctrl = DMA_ADDRESS_CONTROL_FIXED;
    dma_config->src_burst_size = DMA_NUM_TRANSFER_PER_BURST_1T;
    dma_config->dst_width = DMA_TRANSFER_WIDTH_WORD;
    dma_config->dst_addr_ctrl = DMA_ADDRESS_CONTROL_INCREMENT;
    dma_config->dst_mode = DMA_HANDSHAKE_MODE_NORMAL;
    dma_config->size_in_byte = MAX_GCR_EDGES * sizeof(uint32_t);
    dma_config->linked_ptr = 0;
    dma_config->dst_addr = core_local_mem_to_sys_address(HPM_CORE0, (uint32_t)motor->dmaBuffer_neg_edge);
    dma_config->src_addr = (uint32_t)&gptmr->CHANNEL[GPTMR_CAPTURE_IN_CH_NEG].CAPNEG;

    dma_config = &dshot_cap_pos_edge_config[motor_idx];
    dma_default_channel_config(dma_pos, dma_config);
    dma_config->src_mode = DMA_HANDSHAKE_MODE_HANDSHAKE;
    dma_config->src_width = DMA_TRANSFER_WIDTH_WORD;
    dma_config->src_addr_ctrl = DMA_ADDRESS_CONTROL_FIXED;
    dma_config->src_burst_size = DMA_NUM_TRANSFER_PER_BURST_1T;
    dma_config->dst_width = DMA_TRANSFER_WIDTH_WORD;
    dma_config->dst_addr_ctrl = DMA_ADDRESS_CONTROL_INCREMENT;
    dma_config->dst_mode = DMA_HANDSHAKE_MODE_NORMAL;
    dma_config->size_in_byte = MAX_GCR_EDGES * sizeof(uint32_t);
    dma_config->linked_ptr = 0;
    dma_config->dst_addr = core_local_mem_to_sys_address(HPM_CORE0, (uint32_t)motor->dmaBuffer_pos_edge);
    dma_config->src_addr = (uint32_t)&gptmr->CHANNEL[GPTMR_CAPTURE_IN_CH_POS].CAPPOS;

}
#endif

FAST_CODE void pwmDshotStartTransfer(motorDmaOutput_t *motor, uint32_t size)
{
    dma_channel_config_t *ch_config = &dshot_dma_config[motor->index];
    ch_config->size_in_byte = size;
    dma_disable_channel(motor->timerHardware->dma_base,  motor->timerHardware->dmaChannel);
    if (status_success != dma_setup_channel(motor->timerHardware->dma_base, \
                                            motor->timerHardware->dmaChannel, \
                                            ch_config, \
                                            true)) {
        printf(" dma setup channel failed\n");
        while(1);
    }
}

bool pwmDshotMotorHardwareConfig(const timerHardware_t *timerHardware, uint8_t motorIndex, uint8_t reorderedMotorIndex, motorPwmProtocolTypes_e pwmProtocolType, uint8_t output)
{
    static uint8_t cmp_offset = 0;
    cmp_offset+=3;

    dmaResource_t *dmaRef = NULL;

    dmaRef = timerHardware->dmaRef;

    if (dmaRef == NULL) {
        return false;
    }
    dmaIdentifier_e dmaIdentifier = dmaGetIdentifier(dmaRef);

    if (!dmaAllocate(dmaIdentifier, OWNER_MOTOR, RESOURCE_INDEX(reorderedMotorIndex))) {
        return false;
    }

    dmaResource_t *dmaRefCap = timerHardware->dmaCapNeg;
    dmaIdentifier_e dmaIdentifierCap = dmaGetIdentifier(dmaRefCap);
    if (!dmaAllocate(dmaIdentifierCap, OWNER_MOTOR, RESOURCE_INDEX(reorderedMotorIndex))) {
        return false;
    }
    dmaRefCap = timerHardware->dmaCapPos;
    dmaIdentifierCap = dmaGetIdentifier(dmaRefCap);
    if (!dmaAllocate(dmaIdentifierCap, OWNER_MOTOR, RESOURCE_INDEX(reorderedMotorIndex))) {
        return false;
    }

#ifdef USE_DSHOT_TELEMETRY
    if (useDshotTelemetry) {
        output ^= TIMER_OUTPUT_INVERTED;
    }
#endif
    motorDmaOutput_t * const motor = &dmaMotors[motorIndex];
    TIM_TypeDef *timer = timerHardware->tim;

    dma_channel_config_t *ch_config = &dshot_dma_config[motorIndex];
    uint32_t reload = 0;

    reload = (float) timerClock(timer) / getDshotHz(pwmProtocolType) * MOTOR_BITLENGTH - 1;
    dshot_duty_count = reload;

    pwm_cmp_config_t cmp_config[2] = {0};
    pwm_config_t pwm_config = {0};

    // Boolean configureTimer is always true when different channels of the same timer are processed in sequence,
    // causing the timer and the associated DMA initialized more than once.
    // To fix this, getTimerIndex must be expanded to return if a new timer has been requested.
    // However, since the initialization is idempotent, it is left as is in a favor of flash space (for now).
    uint8_t timerIndex = getTimerIndex(timer);
    timerIndex = timerIndex & 0x7F;

    motor->timer = &dmaMotorTimers[timerIndex];
    motor->index = motorIndex;
    motor->timerHardware = timerHardware;
    DMA_Type* base = motor->timerHardware->dma_base;

    pwm_pair_config_t cmp_pair_config = { 0 };
    if (dmaMotorTimers[timerIndex].inited == false) {
        clock_add_to_group(timerRCC(timer), 0);
        pwm_stop_counter(timer);
        reload = (float) timerClock(timer) / getDshotHz(pwmProtocolType) * MOTOR_BITLENGTH - 1;
        /*
         * reload and start counter
         */
        pwm_set_reload(timer, 0, reload);
        pwm_set_start_count(timer, 0, 0);
    }
    if (output & TIMER_OUTPUT_N_CHANNEL) {
        pwm_get_default_pwm_pair_config(timer, &cmp_pair_config);
        cmp_pair_config.pwm[0].invert_output = (output & TIMER_OUTPUT_INVERTED) ? false : true;
        cmp_pair_config.pwm[0].update_trigger = pwm_shadow_register_update_on_modify;
        cmp_pair_config.pwm[0].enable_output = true;
    } else {
        pwm_get_default_pwm_config(timer, &pwm_config);
        pwm_config.update_trigger = pwm_shadow_register_update_on_hw_event;
        pwm_config.enable_output = true;
        pwm_config.dead_zone_in_half_cycle = 0;
        pwm_config.invert_output = (output & TIMER_OUTPUT_INVERTED) ? false : true;
    }
    cmp_config[0].mode = pwm_cmp_mode_output_compare;
    cmp_config[0].cmp = reload;
    cmp_config[0].update_trigger = pwm_shadow_register_update_on_hw_event;

    if (output & TIMER_OUTPUT_N_CHANNEL) {
        if (status_success != pwm_setup_waveform_in_pair(timer, \
                                                         timerHardware->channel, \
                                                         &cmp_pair_config, \
                                                         timerHardware->cmp_index, \
                                                         &cmp_config[0], \
                                                         1)) {
            printf("failed to setup waveform\n");
            while (1) {
            }
        }
    } else {
        if (status_success != pwm_setup_waveform(timer, \
                                                 timerHardware->channel, \
                                                 &pwm_config, \
                                                 timerHardware->cmp_index, \
                                                 &cmp_config[0], \
                                                 1)) {
            printf("failed to setup waveform\n");
            while(1);
        }
        cmp_config[0].mode = pwm_cmp_mode_output_compare;
        cmp_config[0].cmp = (1) + cmp_offset;
        cmp_config[0].update_trigger = pwm_shadow_register_update_on_modify;
        pwm_config.enable_output = false;
        /*
         * config pwm as output driven by cmp
         */
        if (status_success != pwm_setup_waveform(timer, \
                                                 timerHardware->channel_ref, \
                                                 &pwm_config, \
                                                 timerHardware->dma_req_cmp_index, \
                                                 &cmp_config[0], \
                                                 1)) {
            printf("failed to setup waveform\n");
            while(1);
        }
    }
    if (dmaMotorTimers[timerIndex].inited == false) {
        cmp_config[0].mode = pwm_cmp_mode_output_compare;
        cmp_config[0].cmp = reload - 2;
        cmp_config[0].update_trigger = pwm_shadow_register_update_on_modify;
        pwm_load_cmp_shadow_on_match(timer, 23, &cmp_config[0]);
        pwm_start_counter(timer);
        dmaMotorTimers[timerIndex].inited = true;
    }

    pwm_issue_shadow_register_lock_event(timer);

    const gptmr_input_cap_source_t *trg_resource = &(timerHardware->in_cap_trgm_map);
    /* PWM half reload generate dma request */
    trgm_dma_request_config(timerHardware->pwm_trgm.trgm, \
                            timerHardware->pwm_trgm.trg_grp, \
                            timerHardware->pwm_trgm.trgm_dma_src);
    /* dma request trigger dma channel x to work */
    pwm_enable_dma_request(timer, PWM_DMAEN_CMPENX_SET(1<<(timerHardware->dma_req_cmp_index)));

    trgm_output_t trgm_io_config = {0};
    trgm_io_config.invert = 0;
    trgm_io_config.type = trgm_output_same_as_input;
    trgm_io_config.input = trg_resource->trgmux_in_neg;
    trgm_output_config(trg_resource->trgm_neg, trg_resource->trgmux_out_neg, &trgm_io_config);
    memset(&trgm_io_config, 0, sizeof(trgm_io_config));
    trgm_io_config.invert = 0;
    trgm_io_config.type = trgm_output_same_as_input;
    trgm_io_config.input = trg_resource->trgmux_in_pos;
    trgm_output_config(trg_resource->trgm_pos, trg_resource->trgmux_out_pos, &trgm_io_config);

    if ((trg_resource->trgm_ref != 0xFFFFFFFF) && (trg_resource->trgm_ref != 0) && trg_resource->trgmux_in_ref && trg_resource->trgmux_out_ref) {
        memset(&trgm_io_config, 0, sizeof(trgm_io_config));
        trgm_io_config.invert = 0;
        trgm_io_config.type = trgm_output_same_as_input;
        trgm_io_config.input = trg_resource->trgmux_in_ref;
        trgm_output_config(trg_resource->trgm_ref, trg_resource->trgmux_out_ref, &trgm_io_config);

    }
#ifndef HPM_USE_PWM_OUTPUT_DSHOT
    memset(&trgm_io_config, 0, sizeof(trgm_io_config));
    trgm_io_config.invert = 0;
    trgm_io_config.type = trgm_output_same_as_input;
    trgm_io_config.input = timerHardware->pwm_out_trgm_src;
    trgm_output_config(timerHardware->pwm_trgm.trgm, timerHardware->pwm_out_trgm_dst, &trgm_io_config);
#endif
    dmamux_config(HPM_DMAMUX, \
                  DMA_SOC_CHN_TO_DMAMUX_CHN(timerHardware->dma_pos, ((uint32_t)timerHardware->gptmr_dma_ch_pos)), \
                  trg_resource->dmamux_src_pos, \
                  true);
    motor->timerDmaSource = timerHardware->channel;
    motor->timer->timerDmaSources &= ~motor->timerDmaSource;

    motor->dmaBuffer = &dshotDmaBuffer[motorIndex][0];
    motor->dmaBuffer_pos_edge = &dshot_telemetry_pos_buf[motorIndex][0];
    motor->dmaBuffer_neg_edge = &dshot_telemetry_neg_buf[motorIndex][0];

    motor->dmaRef = dmaRef;
    motor->dmaRefCap = dmaRefCap;

#ifdef USE_DSHOT_TELEMETRY
    motor->dshotTelemetryDeadtimeUs = DSHOT_TELEMETRY_DEADTIME_US + 1000000 *
        (16 * MOTOR_BITLENGTH) / getDshotHz(pwmProtocolType);
    motor->timer->outputPeriod = (pwmProtocolType == PWM_TYPE_PROSHOT1000 ? (MOTOR_NIBBLE_LENGTH_PROSHOT) : MOTOR_BITLENGTH) - 1;
#endif
    pwmDshotSetDirectionOutput(motor);

    {
        dmaSetHandler(dmaIdentifier, motor_dshot_transfer_done_handler, 1, motor->index);
        intc_m_enable_irq_with_priority(timerHardware->irqn, 1);
        dma_enable_channel_interrupt(base, timerHardware->dmaChannel, DMA_INTERRUPT_MASK_TERMINAL_COUNT);
    }

    const IO_t cap_pin = IOGetByTag(timerHardware->cap_pin);
    IOConfigGPIOAF(cap_pin, IOCFG_OUT_PP_UP, timerHardware->gptmr_io_function, timerHardware->gptmr_io_function);
#ifdef USE_DSHOT_TELEMETRY
    if (useDshotTelemetry) {
        // avoid high line during startup to prevent bootloader activation
        
        clock_add_to_group(timerHardware->gptmr_clock, 0);
        setup_dshot_telemetry_dma(motor, pwmProtocolType);
    }
#endif

    dma_default_channel_config(base, ch_config);
    ch_config->src_addr = core_local_mem_to_sys_address(HPM_CORE0, (uint32_t)motor->dmaBuffer);
    ch_config->dst_addr = (uint32_t)timerChCCR(motor->timerHardware);
    ch_config->src_mode = DMA_HANDSHAKE_MODE_HANDSHAKE;
    ch_config->src_width = DMA_TRANSFER_WIDTH_WORD;
    ch_config->src_addr_ctrl = DMA_ADDRESS_CONTROL_INCREMENT;
    ch_config->src_burst_size = DMA_NUM_TRANSFER_PER_BURST_1T;
    ch_config->dst_width = DMA_TRANSFER_WIDTH_WORD;
    ch_config->dst_addr_ctrl = DMA_ADDRESS_CONTROL_FIXED;
    ch_config->dst_mode = DMA_HANDSHAKE_MODE_NORMAL;
    motor->configured = true;
    return true;
}
#endif
#endif
