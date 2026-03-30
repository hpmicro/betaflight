/*
 * Copyright (c) 2025 HPMicro
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#ifdef USE_ADC

#include "drivers/io.h"
#include "drivers/adc.h"
#include "drivers/adc_impl.h"
#ifdef HPM6750
#include "hpm_adc12_drv.h"
#elif defined(HPM6360)
#include "hpm_adc16_drv.h"
#endif
#include "hpm_clock_drv.h"
#include "drivers/adc.h"
#include "pg/adc.h"

const adcDevice_t adcHardware[] = {
    {
        .ADCx = ADC1,
        .rccADC = clock_adc0,
    },
    {
        .ADCx = ADC2,
        .rccADC = clock_adc1,
    },
    {
        .ADCx = ADC3,
        .rccADC = clock_adc2,
    },
};

adcDevice_t adcDevice[ADCDEV_COUNT];
/* note these could be packed up for saving space */
const adcTagMap_t adcTagMap[] = {
#ifdef HPM6750
    { DEFIO_TAG_E__PE14, ADC_DEVICES_1, 0, },
    { DEFIO_TAG_E__PE15, ADC_DEVICES_1, 1, },
    { DEFIO_TAG_E__PE16, ADC_DEVICES_123, 2, },
    { DEFIO_TAG_E__PE17, ADC_DEVICES_123, 3, },
    { DEFIO_TAG_E__PE18, ADC_DEVICES_1, 4, },
    { DEFIO_TAG_E__PE19, ADC_DEVICES_1, 5, },
    { DEFIO_TAG_E__PE20, ADC_DEVICES_1, 6, },
    { DEFIO_TAG_E__PE21, ADC_DEVICES_1, 7, },
    { DEFIO_TAG_E__PE22, ADC_DEVICES_1, 8, },
    { DEFIO_TAG_E__PE23, ADC_DEVICES_1, 9, },
    { DEFIO_TAG_E__PE24, ADC_DEVICES_1, 10,  },
    { DEFIO_TAG_E__PE25, ADC_DEVICES_1, 11,  },
    { DEFIO_TAG_E__PE26, ADC_DEVICES_1,  12  },
    { DEFIO_TAG_E__PE27, ADC_DEVICES_1,  13  },
    { DEFIO_TAG_E__PE28, ADC_DEVICES_1,  14  },
    { DEFIO_TAG_E__PE29, ADC_DEVICES_1,  15  },
    { DEFIO_TAG_E__PE30, ADC_DEVICES_1,  16  },
    { DEFIO_TAG_E__PE31, ADC_DEVICES_2,  13,  },
    { DEFIO_TAG_E__PF0,  ADC_DEVICES_2,  14,  },
    { DEFIO_TAG_E__PF1,  ADC_DEVICES_2,  15,  },
    { DEFIO_TAG_E__PF2,  ADC_DEVICES_3,  12,  },
    { DEFIO_TAG_E__PF3,  ADC_DEVICES_3,  13,  },
    { DEFIO_TAG_E__PF4,  ADC_DEVICES_3,  14,  },
    { DEFIO_TAG_E__PF5,  ADC_DEVICES_3,  15,  },
    { DEFIO_TAG_E__PF6,  ADC_DEVICES_3,  16,  },
    { DEFIO_TAG_E__PF8,  ADC_DEVICES_3,  17,  },
#endif
#ifdef HPM6360
    { DEFIO_TAG_E__PC15, ADC_DEVICES_1, 11, },
    { DEFIO_TAG_E__PC16, ADC_DEVICES_1, 12, },
#endif
};

int adcFindTagMapEntry(ioTag_t tag)
{
    for (int i = 0; i < ADC_TAG_MAP_COUNT; i++) {
        if (adcTagMap[i].tag == tag) {
            return i;
        }
    }
    return -1;
}

void adcInitDevice(ADC_TypeDef *adcdev)
{
#ifdef HPM6750
    adc12_config_t cfg;

    /* initialize an ADC instance */
    adc12_get_default_config(&cfg);

    cfg.res            = adc12_res_12_bits;
    cfg.conv_mode      = adc12_conv_mode_period;
    cfg.adc_clk_div    = adc12_clock_divider_4;
    cfg.sel_sync_ahb   = true;


    /* adc12 initialization */
    if (adc12_init(adcdev, &cfg) == status_success) {
    } else {
        printf("ADC%x initialization failed!\n", adcdev);
    }
#elif defined(HPM6360)
    adc16_config_t cfg;

    /* initialize an ADC instance */
    adc16_get_default_config(&cfg);

    cfg.res            = adc16_res_16_bits;
    cfg.conv_mode      = adc16_conv_mode_period;
    cfg.adc_clk_div    = adc16_clock_divider_4;
    cfg.sel_sync_ahb   = true;


    /* adc16 initialization */
    if (adc16_init(adcdev, &cfg) == status_success) {
    } else {
        printf("ADC%x initialization failed!\n", adcdev);
    }
#endif
}

void adcInit(const adcConfig_t *config)
{
    memset(adcOperatingConfig, 0, sizeof(adcOperatingConfig));
    memcpy(adcDevice, adcHardware, sizeof(adcDevice));

    if (config->vbat.enabled) {
        adcOperatingConfig[ADC_BATTERY].tag = config->vbat.ioTag;
    }

    if (config->rssi.enabled) {
        adcOperatingConfig[ADC_RSSI].tag = config->rssi.ioTag;  //RSSI_ADC_CHANNEL;
    }

    if (config->external1.enabled) {
        adcOperatingConfig[ADC_EXTERNAL1].tag = config->external1.ioTag; //EXTERNAL1_ADC_CHANNEL;
    }

    if (config->current.enabled) {
        adcOperatingConfig[ADC_CURRENT].tag = config->current.ioTag;  //CURRENT_METER_ADC_CHANNEL;
    }
#ifdef USE_ADC_INTERNAL
    adcInitCalibrationValues();
#endif
    for (int i = 0; i < ADC_CHANNEL_COUNT; i++) {
        int map;
        int dev;

        if (!adcOperatingConfig[i].tag) {
            continue;
        }

        map = adcFindTagMapEntry(adcOperatingConfig[i].tag);
        if (map < 0) {
            continue;
        }

        // Found a tag map entry for this input pin
        // Find an ADC device that can handle this input pin

        for (dev = 0; dev < ADCDEV_COUNT; dev++) {
            if (!adcDevice[dev].ADCx) {
                // Instance not activated
                continue;
            }
            if (adcTagMap[map].devices & (1 << dev)) {
                // Found an activated ADC instance for this input pin
                break;
            }
        }

        if (dev == ADCDEV_COUNT) {
            // No valid device found, go next channel.
            continue;
        }

        // At this point, map is an entry for the input pin and dev is a valid ADCx for the pin for input i

        adcOperatingConfig[i].adcDevice = dev;
        adcOperatingConfig[i].adcChannel = adcTagMap[map].channel;
        adcOperatingConfig[i].sampleTime = 20U;
        adcOperatingConfig[i].enabled = true;

        adcDevice[dev].channelBits |= (1 << adcTagMap[map].channel);

        // Configure a pin for ADC
        if (adcOperatingConfig[i].tag) {
            IOInit(IOGetByTag(adcOperatingConfig[i].tag), OWNER_ADC_BATT + i, 0);
            IOConfigGPIOAF(IOGetByTag(adcOperatingConfig[i].tag), IO_CONFIG(GPIO_MODE_ANALOG, 0, GPIO_NOPULL), IOC_PAD_FUNC_CTL_ANALOG_MASK, IOC_PAD_FUNC_CTL_ANALOG_MASK);
        }
    }
    // DeInit ADCx with inputs
    // We have to batch call DeInit() for all devices as DeInit() initializes ADCx_COMMON register.

    for (int dev = 0; dev < ADCDEV_COUNT; dev++) {
        adcDevice_t *adc = &adcDevice[dev];

        if (!(adc->ADCx && adc->channelBits)) {
            continue;
        }
        clock_add_to_group(adc->rccADC, 0);
        clock_set_adc_source(adc->rccADC, clk_adc_src_ahb0);
#ifdef HPM6750
        if(adc12_deinit(adc->ADCx) != status_success) {
            printf("Deinit ADC error %d\n", dev);
        }
#elif defined(HPM6360)
        if(adc16_deinit(adc->ADCx) != status_success) {
            printf("Deinit ADC error %d\n", dev);
        }
#endif
    }
    // Configure ADCx with inputs

    int dmaBufferIndex = 0;

    for (int dev = 0; dev < ADCDEV_COUNT; dev++) {
        adcDevice_t *adc = &adcDevice[dev];

        if (!(adc->ADCx && adc->channelBits)) {
            continue;
        }

        adcInitDevice(adc->ADCx);

        // Configure channels

        for (int adcChan = 0; adcChan < ADC_CHANNEL_COUNT; adcChan++) {
            if (!adcOperatingConfig[adcChan].enabled) {
                continue;
            }

            if (adcOperatingConfig[adcChan].adcDevice != dev) {
                continue;
            }

            adcOperatingConfig[adcChan].dmaIndex = dmaBufferIndex++;
#ifdef HPM6750
            adc12_channel_config_t ch_cfg;
            /* get a default channel config */
            adc12_get_channel_default_config(&ch_cfg);
            ch_cfg.ch = adcOperatingConfig[adcChan].adcChannel;
            ch_cfg.sample_cycle = adcOperatingConfig[adcChan].sampleTime;
            adc12_init_channel(adc->ADCx, &ch_cfg);
            adc12_prd_config_t prd_cfg;

            prd_cfg.ch           = adcOperatingConfig[adcChan].adcChannel;
            prd_cfg.prescale     = 22;    /* Set divider: 2^22 clocks */
            prd_cfg.period_count = 5;     /* 104.86ms when AHB clock at 200MHz is ADC clock source */

            adc12_set_prd_config(adc->ADCx, &prd_cfg);
#elif defined(HPM6360)
            adc16_channel_config_t ch_cfg;
            /* get a default channel config */
            adc16_get_channel_default_config(&ch_cfg);
            ch_cfg.ch = adcOperatingConfig[adcChan].adcChannel;
            ch_cfg.sample_cycle = adcOperatingConfig[adcChan].sampleTime;
            adc16_init_channel(adc->ADCx, &ch_cfg);
            adc16_prd_config_t prd_cfg;

            prd_cfg.ch           = adcOperatingConfig[adcChan].adcChannel;
            prd_cfg.prescale     = 22;    /* Set divider: 2^22 clocks */
            prd_cfg.period_count = 5;     /* 104.86ms when AHB clock at 200MHz is ADC clock source */

            adc16_set_prd_config(adc->ADCx, &prd_cfg);
#endif

        }
    }
}

void adcGetChannelValues(void)
{
    for (int i = 0; i < ADC_CHANNEL_INTERNAL_FIRST_ID; i++) {
        if (adcOperatingConfig[i].enabled) {
            uint16_t result;
            adcDevice_t *adc = &adcDevice[adcOperatingConfig[i].adcDevice];
#ifdef HPM6750
            if(status_success != adc12_get_prd_result(adc->ADCx, adcOperatingConfig[i].adcChannel, &result)) {
                printf("Get ADC %d channel %d failed\n", adcOperatingConfig[i].adcDevice, adcOperatingConfig[i].adcChannel);
            } else {
                adcValues[i] = result;
            }
#elif defined(HPM6360)
            if(status_success != adc16_get_prd_result(adc->ADCx, adcOperatingConfig[i].adcChannel, &result)) {
                printf("Get ADC %d channel %d failed\n", adcOperatingConfig[i].adcDevice, adcOperatingConfig[i].adcChannel);
            } else {
                adcValues[i] = result >> 4;
            }
#endif
        }
    }
}
#endif
