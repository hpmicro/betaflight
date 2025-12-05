/*
 * Copyright (c) 2025 HPMicro
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#ifdef USE_LED_STRIP

#include "build/debug.h"

#include "common/color.h"

#include "drivers/dma.h"
#include "drivers/dma_reqmap.h"
#include "drivers/io.h"
#include "drivers/nvic.h"
#include "drivers/rcc.h"
#include "drivers/timer.h"

#include "drivers/light_ws2811strip.h"

#include "hpm_soc.h"
#ifdef HPMSOC_HAS_HPMSDK_DMAV2
#include "hpm_dmav2_drv.h"
#else
#include "hpm_dma_drv.h"
#endif

#if defined(HPMSOC_HAS_HPMSDK_PWM)
#include "hpm_pwm_drv.h"
#endif
#if defined(HPMSOC_HAS_HPMSDK_PWMV2)
#include "hpm_pwmv2_drv.h"
#endif

#include "hpm_clock_drv.h"
#include "hpm_trgm_drv.h"
#include "hpm_dmamux_drv.h"
#include "hpm_gpio_drv.h"
#include "hpm_gptmr_drv.h"
#include "dma_impl.h"

static IO_t ws2811IO = IO_NONE;

static TIM_TypeDef *timer = NULL;
static dmaResource_t *dmaRef = NULL;
static timerHardware_t *timerHardware;

/*
 * Copyright (c) 2021-2022 HPMicro
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */
#define DBG_TAG "drv ws2812"

#include <string.h>
#include <stdbool.h>
#include <stdint.h>

#include "config.h"

#include "hpm_soc.h"
#include "hpm_gpio_drv.h"
#include "hpm_spi_drv.h"
#include "hpm_dmamux_drv.h"
#include "hpm_dma_drv.h"
#include "hpm_clock_drv.h"

typedef struct PACKED
{
#if (WS2812_BYTE_ORDER == WS2812_BYTE_ORDER_GRB)
    uint8_t g;
    uint8_t r;
    uint8_t b;
#elif (WS2812_BYTE_ORDER == WS2812_BYTE_ORDER_RGB)
    uint8_t r;
    uint8_t g;
    uint8_t b;
#elif (WS2812_BYTE_ORDER == WS2812_BYTE_ORDER_BGR)
    uint8_t b;
    uint8_t g;
    uint8_t r;
#endif
} cRGB;

typedef cRGB RGB;

#define APP_SPI_BASE HPM_SPI2
#define APP_SPI_CLK_NAME clock_spi2
#define APP_SPI_IRQ IRQn_SPI2
#define APP_SPI_SCLK_FREQ (6000000UL) // 6MHZ 1tick=160ns, 0:160*2=320ns, 11000000= 0xC0; 1:160*4= 640ns, 11110000=0xF0
#define APP_SPI_ADDR_LEN_IN_BYTES (1U)
#define APP_SPI_DATA_LEN_IN_BITS (8U)
#define APP_SPI_DATA_LEN_IN_BYTE (1U)
#define APP_SPI_RX_DMA HPM_DMA_SRC_SPI2_RX
#define APP_SPI_TX_DMA HPM_DMA_SRC_SPI2_TX

#define SPI_DMA HPM_HDMA
#define SPI_DMAMUX HPM_DMAMUX
#define SPI_TX_DMA_REQ HPM_DMA_SRC_SPI2_TX
#define SPI_TX_DMA_CH 0
#define SPI_TX_DMAMUX_CH DMA_SOC_CHN_TO_DMAMUX_CHN(HPM_HDMA, SPI_TX_DMA_CH)

#define WS2812_0 0xC0
#define WS2812_1 0xF8
#define WS2812_RST 0x00
#define RGB_BIT 24

#ifndef WS2812_PWREN_IOC_PAD
#define WS2812_PWREN_IOC_PAD IOC_PAD_PA30
#endif

#ifndef WS2812_PWREN_IOC_FUNC
#define WS2812_PWREN_IOC_FUNC IOC_PA30_FUNC_CTL_GPIO_A_30
#endif

#ifndef WS2812_PWREN_PORT
#define WS2812_PWREN_PORT GPIO_DO_GPIOA
#endif

#ifndef WS2812_PWREN_PIN
#define WS2812_PWREN_PIN 30
#endif
#define ASSERT_EXPR "\033[0;34;1m"
#define ASSERT_VALUE "\033[0;1m"
#define ASSERT_RESET "\033[0m"
#define KBD_ASSERT_EQUAL_FUNC_MESSAGE(val, x, ...)                                                                                \
    do                                                                                                                            \
    {                                                                                                                             \
        int _val = (int)(val);                                                                                                    \
        int _x = (int)(x);                                                                                                        \
        if (_val != _x)                                                                                                           \
        {                                                                                                                         \
            printf(__VA_ARGS__);                                                                                                  \
            printf(" at %s:%d\r\n", __FILE__, __LINE__);                                                                          \
            printf(" " ASSERT_EXPR #val " == " #x ASSERT_VALUE " but " #val " = %d , " #x " = %d" ASSERT_RESET "\r\n", _val, _x); \
        }                                                                                                                         \
    } while (0)
#define KBD_ASSERT_EQUAL_FUNC(val, x) KBD_ASSERT_EQUAL_FUNC_MESSAGE(val, x, "Assertion EQUAL Failed")
#define RGB_MATRIX_LED_COUNT 10
ATTR_PLACE_AT_NONCACHEABLE static uint8_t send_buff[(RGB_BIT * RGB_MATRIX_LED_COUNT) + 1];

/**
create 24 byte sent by SPI using RGB values.
*/
void ws2812_driver_convert_data(uint8_t R, uint8_t G, uint8_t B, uint8_t led_idx)
{
    for (uint8_t i = 0; i < 8; i++)
    {
        send_buff[led_idx * RGB_BIT + 7 - i] = (G & 0x01) ? WS2812_1 : WS2812_0;
        G = G >> 1;
    }
    for (uint8_t i = 0; i < 8; i++)
    {
        send_buff[led_idx * RGB_BIT + 15 - i] = (R & 0x01) ? WS2812_1 : WS2812_0;
        R = R >> 1;
    }
    for (uint8_t i = 0; i < 8; i++)
    {
        send_buff[led_idx * RGB_BIT + 23 - i] = (B & 0x01) ? WS2812_1 : WS2812_0;
        B = B >> 1;
    }
}

static hpm_stat_t ws2812_driver_trigger_dma(DMA_Type *dma_ptr, uint8_t ch_num, SPI_Type *spi_ptr, uint32_t src, uint8_t data_width, uint32_t size)
{
    dma_handshake_config_t config;

    dma_default_handshake_config(dma_ptr, &config);
    config.ch_index = ch_num;
    config.dst = (uint32_t)&spi_ptr->DATA;
    config.dst_fixed = true;
    config.src = src;
    config.src_fixed = false;
    config.data_width = data_width;
    config.size_in_byte = size;
    config.interrupt_mask = DMA_INTERRUPT_MASK_ALL;

    return dma_setup_handshake(dma_ptr, &config, true);
}

static void ws2812_driver_pin_init(void)
{
    // RGB EN
    // RGB PIN PA29
    HPM_IOC->PAD[IOC_PAD_PE30].FUNC_CTL = IOC_PE30_FUNC_CTL_SPI2_MOSI;
    HPM_IOC->PAD[IOC_PAD_PE30].PAD_CTL = IOC_PAD_PAD_CTL_DS_SET(6) | IOC_PAD_PAD_CTL_PE_SET(1);
}

void ws2812_driver_init(void)
{
    spi_timing_config_t timing_config = {0};
    spi_format_config_t format_config = {0};
    spi_control_config_t control_config = {0};
    uint32_t spi_clcok;
    uint32_t spi_tx_trans_count;

    ws2812_driver_pin_init();

    // GRB DATA, spi
    clock_add_to_group(APP_SPI_CLK_NAME, 0);
    clock_set_source_divider(APP_SPI_CLK_NAME, clk_src_pll1_clk1, 10); /* 666/111 = 6MHz */
    spi_clcok = clock_get_frequency(APP_SPI_CLK_NAME);

    /* set SPI sclk frequency for master */
    spi_master_get_default_timing_config(&timing_config);
    timing_config.master_config.clk_src_freq_in_hz = spi_clcok;
    timing_config.master_config.sclk_freq_in_hz = (6006006UL);

    KBD_ASSERT_EQUAL_FUNC(status_success,
                          spi_master_timing_init(APP_SPI_BASE, &timing_config));

    /* set SPI format config for master */
    spi_master_get_default_format_config(&format_config);
    format_config.master_config.addr_len_in_bytes = 1;
    format_config.common_config.data_len_in_bits = 8;
    format_config.common_config.data_merge = false;
    format_config.common_config.mosi_bidir = false;
    format_config.common_config.lsb = false;
    format_config.common_config.mode = spi_master_mode;
    format_config.common_config.cpol = spi_sclk_high_idle;
    format_config.common_config.cpha = spi_sclk_sampling_even_clk_edges;
    spi_format_init(APP_SPI_BASE, &format_config);

    printf("initialize\r\n");
}

void ws2812_driver_setleds(uint32_t count)
{
    uint32_t spi_tx_trans_count;
    uint32_t i;
    spi_control_config_t control_config = {0};
    uint8_t cmd = 0x00;
    uint32_t addr = 0x00;

    send_buff[(RGB_BIT * count)] = WS2812_RST;

    spi_master_get_default_control_config(&control_config);
    control_config.master_config.cmd_enable = false;
    control_config.master_config.addr_enable = false;
    control_config.master_config.addr_phase_fmt = spi_address_phase_format_single_io_mode;
    control_config.common_config.tx_dma_enable = true;
    control_config.common_config.rx_dma_enable = false;
    control_config.common_config.trans_mode = spi_trans_write_only;
    control_config.common_config.data_phase_fmt = spi_single_io_mode;
    control_config.common_config.dummy_cnt = spi_dummy_count_1;

    spi_tx_trans_count = (count * 24 +  1) / APP_SPI_DATA_LEN_IN_BYTE;

    KBD_ASSERT_EQUAL_FUNC(status_success,
                          spi_setup_dma_transfer(APP_SPI_BASE,
                                                 &control_config,
                                                 &cmd, &addr,
                                                 spi_tx_trans_count, 1));
    dmamux_config(SPI_DMAMUX, SPI_TX_DMAMUX_CH, SPI_TX_DMA_REQ, true);

    KBD_ASSERT_EQUAL_FUNC(status_success,
                          ws2812_driver_trigger_dma(SPI_DMA,
                                                    SPI_TX_DMA_CH,
                                                    APP_SPI_BASE,
                                                    core_local_mem_to_sys_address(HPM_CORE0, (uint32_t)send_buff),
                                                    DMA_TRANSFER_WIDTH_BYTE,
                                                    spi_tx_trans_count));
}

static void WS2811_DMA_IRQHandler(dmaChannelDescriptor_t *descriptor)
{
    ws2811LedDataTransferInProgress = false;
}

bool ws2811LedStripHardwareInit(ioTag_t ioTag)
{
    ws2812_driver_init();
    return true;
}

void ws2811LedStripDMAEnable(uint32_t bytes)
{
    ws2812_driver_setleds(bytes);
    ws2811LedDataTransferInProgress = false;
}

#endif
