# Copyright 2025 HPMicro
# SPDX-License-Identifier: BSD-3-Clause

sdk_inc(.)
 
sdk_src(
    io_hpmicro.c
    serial_uart_hpmicro.c
    bus_spi_hpmicro.c
    system_hpmicro.c
    exti_hpmicro.c
    bus_i2c_hpmicro.c
    timer_hpmicro.c
    adc_hpmicro.c
    pwm_output.c
    pwm_output_dshot.c
    dma_hpmicro.c
    serial_usb_vcp.c
    dma_reqmap_mcu.c
    persistent.c
    usb_msc_hpm.c
    sdio_hpmicro.c
    light_ws2811strip.c
    cdc_acm.c
    msc_sdcard.c
)
