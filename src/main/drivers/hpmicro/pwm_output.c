/*
 * Copyright (c) 2025 HPMicro
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"
#include "dma.h"
#ifdef HPMSOC_HAS_HPMSDK_DMAV2
#include "hpm_dmav2_drv.h"
#else
#include "hpm_dma_drv.h"
#endif

#ifdef USE_PWM_OUTPUT

#include "drivers/io.h"
#include "drivers/motor.h"
#include "drivers/pwm_output.h"
#include "drivers/timer.h"
#include "hpm_clock_drv.h"
#if defined(HPMSOC_HAS_HPMSDK_PWM)
#include "hpm_pwm_drv.h"
#endif
#if defined(HPMSOC_HAS_HPMSDK_PWMV2)
#include "hpm_pwmv2_drv.h"
#error "Not supported yet."
#endif
#include "pg/motor.h"
#include <stdio.h>
FAST_DATA_ZERO_INIT pwmOutputPort_t motors[MAX_SUPPORTED_MOTORS];

#if defined(HPMSOC_HAS_HPMSDK_PWM)
void pwmOutConfig(timerChannel_t *channel, const timerHardware_t *timerHardware,
                  uint32_t hz, uint16_t period, uint16_t value,
                  uint8_t inversion)
{
    (void)value;
    trgm_output_t trgm_io_config = {0};
#if defined(HPMSOC_HAS_HPMSDK_PWM)
    pwm_stop_counter(timerHardware->tim);
    uint32_t reload = 0;
    uint32_t freq;
    pwm_config_t pwm_config = {0};
    pwm_cmp_config_t cmp_config = {0};
    pwm_get_default_pwm_config(timerHardware->tim, &pwm_config);

    pwm_config.enable_output = true;
    pwm_config.dead_zone_in_half_cycle = 0;
    pwm_config.invert_output = inversion ? true : false;
    freq = clock_get_frequency(timerRCC(timerHardware->tim));
    reload = freq / hz * period - 1;
    /*
     * reload and start counter
     */
    pwm_set_reload(timerHardware->tim, 0, reload);
    pwm_set_start_count(timerHardware->tim, 0, 0);

    /*
     * config cmp = RELOAD + 1
     */
    cmp_config.mode = pwm_cmp_mode_output_compare;
    cmp_config.cmp = reload + 1;
    cmp_config.update_trigger = pwm_shadow_register_update_on_modify;
    /*
     * config pwm as output driven by cmp
     */
    if (status_success !=
        pwm_setup_waveform(timerHardware->tim, timerHardware->channel,
                           &pwm_config, timerHardware->cmp_index, &cmp_config,
                           1))
    {
        printf("failed to setup waveform\n");
        while (1)
            ;
    }
    cmp_config.cmp = reload << 1;
    pwm_load_cmp_shadow_on_match(timerHardware->tim, 23, &cmp_config);
    pwm_start_counter(timerHardware->tim);
    pwm_issue_shadow_register_lock_event(timerHardware->tim);

    channel->ccr = timerChCCR(timerHardware);

    channel->tim = timerHardware->tim;

    *channel->ccr = PWM_CMP_CMP_SET(reload / 2);
    memset(&trgm_io_config, 0, sizeof(trgm_io_config));
    trgm_io_config.invert = 0;
    trgm_io_config.type = trgm_output_same_as_input;
    trgm_io_config.input = timerHardware->pwm_out_trgm_src;
    trgm_output_config(timerHardware->pwm_trgm.trgm, timerHardware->pwm_out_trgm_dst, &trgm_io_config);
    trgm_enable_io_output(timerHardware->pwm_trgm.trgm, 1 << (timerHardware->trgm_port_idx));
#endif
}

static FAST_DATA_ZERO_INIT motorDevice_t motorPwmDevice;

static void pwmWriteUnused(uint8_t index, float value)
{
    UNUSED(index);
    UNUSED(value);
}

static void pwmWriteStandard(uint8_t index, float value)
{
    /* TODO: move value to be a number between 0-1 (i.e. percent throttle
     * from mixer) */
    *motors[index].channel.ccr = PWM_CMP_CMP_SET(lrintf((value * motors[index].pulseScale) + motors[index].pulseOffset));
}

void pwmShutdownPulsesForAllMotors(void)
{
    for (int index = 0; index < motorPwmDevice.count; index++)
    {
        // Set the compare register to 0, which stops the output pulsing
        // if the timer overflows
        if (motors[index].channel.ccr)
        {
            *motors[index].channel.ccr = PWM_CMP_CMP_SET(0xFFFFFFFF);
        }
    }
}

void pwmDisableMotors(void) { pwmShutdownPulsesForAllMotors(); }

static motorVTable_t motorPwmVTable;
bool pwmEnableMotors(void)
{
    /* check motors can be enabled */
    return (motorPwmVTable.write != &pwmWriteUnused);
}

bool pwmIsMotorEnabled(uint8_t index)
{ 
    return motors[index].enabled; 
}

static void pwmCompleteOneshotMotorUpdate(void)
{
    for (int index = 0; index < motorPwmDevice.count; index++)
    {
        // Set the compare register to 0, which stops the output pulsing
        // if the timer overflows before the main loop completes again.
        // This compare register will be set to the output value on the
        // next main loop.
        *motors[index].channel.ccr = PWM_CMP_CMP_SET(0xffffffff);
    }
}

static float pwmConvertFromExternal(uint16_t externalValue)
{
    return (float)externalValue;
}

static uint16_t pwmConvertToExternal(float motorValue)
{
    return (uint16_t)motorValue;
}

static motorVTable_t motorPwmVTable = {
    .postInit = motorPostInitNull,
    .enable = pwmEnableMotors,
    .disable = pwmDisableMotors,
    .isMotorEnabled = pwmIsMotorEnabled,
    .shutdown = pwmShutdownPulsesForAllMotors,
    .convertExternalToMotor = pwmConvertFromExternal,
    .convertMotorToExternal = pwmConvertToExternal,
};

motorDevice_t *motorPwmDevInit(const motorDevConfig_t *motorConfig,
                               uint16_t idlePulse, uint8_t motorCount,
                               bool useUnsyncedPwm)
{
    motorPwmDevice.vTable = motorPwmVTable;

    float sMin = 0;
    float sLen = 0;
    switch (motorConfig->motorPwmProtocol)
    {
    default:
    case PWM_TYPE_ONESHOT125:
        sMin = 125e-6f;
        sLen = 125e-6f;
        break;
    case PWM_TYPE_ONESHOT42:
        sMin = 42e-6f;
        sLen = 42e-6f;
        break;
    case PWM_TYPE_MULTISHOT:
        sMin = 5e-6f;
        sLen = 20e-6f;
        break;
    case PWM_TYPE_BRUSHED:
        sMin = 0;
        useUnsyncedPwm = true;
        idlePulse = 0;
        break;
    case PWM_TYPE_STANDARD:
        sMin = 1e-3f;
        sLen = 1e-3f;
        useUnsyncedPwm = true;
        idlePulse = 0;
        break;
    }
    motorPwmDevice.vTable.write = pwmWriteStandard;
    motorPwmDevice.vTable.decodeTelemetry = motorDecodeTelemetryNull;
    motorPwmDevice.vTable.updateComplete = useUnsyncedPwm
                                               ? motorUpdateCompleteNull
                                               : pwmCompleteOneshotMotorUpdate;
    for (int motorIndex = 0;
         motorIndex < MAX_SUPPORTED_MOTORS && motorIndex < motorCount;
         motorIndex++)
    {
        const unsigned reorderedMotorIndex =
            motorConfig->motorOutputReordering[motorIndex];
        const ioTag_t tag = motorConfig->ioTags[reorderedMotorIndex];
        const timerHardware_t *timerHardware = timerAllocate(
            tag, OWNER_MOTOR, RESOURCE_INDEX(reorderedMotorIndex));
        if (timerHardware == NULL)
        {
            /* not enough motors initialised for the mixer or a
             * break in the motors */
            motorPwmDevice.vTable.write = &pwmWriteUnused;
            motorPwmDevice.vTable.updateComplete = motorUpdateCompleteNull;
            /* TODO: block arming and add reason system cannot arm
             */
            return NULL;
        }
        motors[motorIndex].io = IOGetByTag(tag);
        IOInit(motors[motorIndex].io, OWNER_MOTOR,
               RESOURCE_INDEX(reorderedMotorIndex));

        IOConfigGPIOAF(motors[motorIndex].io, IOCFG_AF_PP,
                       timerHardware->alternateFunction,
                       timerHardware->palternateFunction);
        /* standard PWM outputs */
        // margin of safety is 4 periods when unsynced
        const unsigned pwmRateHz = useUnsyncedPwm
                                       ? motorConfig->motorPwmRate
                                       : ceilf(1 / ((sMin + sLen) * 4));

        const uint32_t clock = timerClock(timerHardware->tim);
        /* used to find the desired timer frequency for max resolution
         */
        const unsigned prescaler =
            ((clock / pwmRateHz) + 0xffff) / 0x10000; /* rounding up */
        const uint32_t hz = clock / prescaler;
        const unsigned period = useUnsyncedPwm ? hz / pwmRateHz : 0xffff;

        /*
            if brushed then it is the entire length of the period.
            TODO: this can be moved back to periodMin and periodLen
            once mixer outputs a 0..1 float value.
        */
        /*
         * HPM PWM output compare mode (non-inverted):
         *   HIGH time = reload - CMP
         * The PWM counter runs at the source clock frequency (clock), so all
         * CMP values must be in source-clock ticks, NOT prescaled hz ticks.
         * We want HIGH time = desired_pulse_seconds, so:
         *   CMP = reload - desired_pulse_ticks
         * desired_pulse_ticks = clock * (sMin + sLen * (value - 1000) / 1000)
         *   = clock*sMin + (clock*sLen/1000)*value - clock*sLen
         * CMP = reload + clock*(sLen - sMin) - (clock*sLen/1000)*value
         */
        const uint32_t reload = (clock / hz) * period - 1;
        if (motorConfig->motorPwmProtocol == PWM_TYPE_BRUSHED) {
            motors[motorIndex].pulseScale = period / 1000.0f;
            motors[motorIndex].pulseOffset = 0 - (motors[motorIndex].pulseScale * 1000);
        } else {
            motors[motorIndex].pulseScale = -(sLen * clock) / 1000.0f;
            motors[motorIndex].pulseOffset = (float)reload + clock * (sLen - sMin);
        }

        pwmOutConfig(&motors[motorIndex].channel, timerHardware, hz, period,
                     idlePulse, motorConfig->motorPwmInversion);

        bool timerAlreadyUsed = false;
        for (int i = 0; i < motorIndex; i++)
        {
            if (motors[i].channel.tim == motors[motorIndex].channel.tim)
            {
                timerAlreadyUsed = true;
                break;
            }
        }
        motors[motorIndex].forceOverflow = !timerAlreadyUsed;
        motors[motorIndex].enabled = true;
    }
    return &motorPwmDevice;
}

pwmOutputPort_t *pwmGetMotors(void) { return motors; }

#ifdef USE_SERVOS
static pwmOutputPort_t servos[MAX_SUPPORTED_SERVOS];

void pwmWriteServo(uint8_t index, float value)
{
    if (index < MAX_SUPPORTED_SERVOS && servos[index].channel.ccr)
    {
        *servos[index].channel.ccr = lrintf(value);
    }
}

void servoDevInit(const servoDevConfig_t *servoConfig)
{
    for (uint8_t servoIndex = 0; servoIndex < MAX_SUPPORTED_SERVOS;
         servoIndex++)
    {
        const ioTag_t tag = servoConfig->ioTags[servoIndex];

        if (!tag)
        {
            break;
        }

        servos[servoIndex].io = IOGetByTag(tag);

        IOInit(servos[servoIndex].io, OWNER_SERVO, RESOURCE_INDEX(servoIndex));

        const timerHardware_t *timer =
            timerAllocate(tag, OWNER_SERVO, RESOURCE_INDEX(servoIndex));

        if (timer == NULL)
        {
            /* flag failure and disable ability to arm */
            break;
        }

        IOConfigGPIOAF(servos[servoIndex].io, IOCFG_AF_PP,
                       timer->alternateFunction, timer->palternateFunction);

        pwmOutConfig(&servos[servoIndex].channel, timer, PWM_TIMER_1MHZ,
                     PWM_TIMER_1MHZ / servoConfig->servoPwmRate,
                     servoConfig->servoCenterPulse, 0);
        servos[servoIndex].enabled = true;
    }
}
#endif
#endif // USE_SERVOS
#endif // USE_PWM_OUTPUT

uint32_t xDMA_GetCurrDataCounter(DMA_ARCH_TYPE *dmaResource)
{
    if ((uint32_t)dmaResource & 0x10000000)
    {
        if ((uint32_t)dmaResource & 0x1000)
        { /*XDMA*/
            return dma_get_remaining_transfer_size(HPM_XDMA,
                                                   (uint32_t)dmaResource & 0xF);
        }
        else
        { /*HDMA*/
            return dma_get_remaining_transfer_size(HPM_HDMA,
                                                   (uint32_t)dmaResource & 0xF);
        }
    }
    return 0;
}
