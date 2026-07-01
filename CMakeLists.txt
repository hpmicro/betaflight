# Copyright (c) 2025 HPMicro
# SPDX-License-Identifier: BSD-3-Clause

cmake_minimum_required(VERSION 3.13)

set(RV_ARCH "rv32imafc")
set(RV_ABI "ilp32f")
set(STACK_SIZE 0x10000)

set(CONFIG_FATFS 1)
set(CONFIG_HPM_SPI 1)
set(CONFIG_CHERRYUSB 1)
set(CONFIG_USB_DEVICE 1)
set(CONFIG_USB_DEVICE_CDC_ACM 1)
set(CONFIG_SDMMC 1)
set(CONFIG_USB_DEVICE_MSC 1)

string(TOLOWER ${HPM_BUILD_TYPE} hpm_build_type)
if(BOARD STREQUAL "hpmpilotbf63" OR BOARD STREQUAL "hpm6300evk" OR BOARD STREQUAL "sk_flycontrol_bf")
    string(FIND ${hpm_build_type} "flash_dfu" found)
    if(${found} GREATER_EQUAL 0)
        set(CUSTOM_GCC_LINKER_FILE ${CMAKE_CURRENT_SOURCE_DIR}/src/port/linker_file/HPM6300/gcc/flash_dfu.ld)
        set(CUSTOM_SES_LINKER_FILE ${CMAKE_CURRENT_SOURCE_DIR}/src/port/linker_file/HPM6300/ses/flash_dfu.icf)
    else()
        set(CUSTOM_GCC_LINKER_FILE ${CMAKE_CURRENT_SOURCE_DIR}/src/port/linker_file/HPM6300/gcc/flash_xip.ld)
        set(CUSTOM_SES_LINKER_FILE ${CMAKE_CURRENT_SOURCE_DIR}/src/port/linker_file/HPM6300/ses/flash_xip.icf)
    endif()
    set(CHIP_DEF "HPM6360")
elseif(BOARD STREQUAL "hpmpilotbf67" OR BOARD STREQUAL "hpm6750evk")
    string(FIND ${hpm_build_type} "flash_dfu" found)
    if(${found} GREATER_EQUAL 0)
        set(CUSTOM_GCC_LINKER_FILE ${CMAKE_CURRENT_SOURCE_DIR}/src/port/linker_file/HPM6700/gcc/flash_dfu.ld)
        set(CUSTOM_SES_LINKER_FILE ${CMAKE_CURRENT_SOURCE_DIR}/src/port/linker_file/HPM6700/ses/flash_dfu.icf)
    else()
        set(CUSTOM_GCC_LINKER_FILE ${CMAKE_CURRENT_SOURCE_DIR}/src/port/linker_file/HPM6700/gcc/flash_xip.ld)
        set(CUSTOM_SES_LINKER_FILE ${CMAKE_CURRENT_SOURCE_DIR}/src/port/linker_file/HPM6700/ses/flash_xip.icf)
    endif()
    set(CHIP_DEF "HPM6750")
endif()

find_package(hpm-sdk REQUIRED HINTS $ENV{HPM_SDK_BASE})

project(betaflight)


add_subdirectory(lib)
add_subdirectory(src/main)
add_subdirectory(src/port)
sdk_inc(src/config/configs/HPMPILOTBF67)
sdk_inc(src/config)
string(TOUPPER ${BOARD} BOARD_UPPER)
sdk_inc(configs/configs/${BOARD_UPPER})

sdk_app_src(src/main/main.c)

sdk_compile_definitions(-DUSE_CONFIG=1)
sdk_compile_definitions(-DHPMicro=1)
sdk_compile_definitions(-D${CHIP_DEF}=1)
sdk_compile_definitions(-D__FPU_PRESENT=0)
sdk_compile_definitions(-D__TARGET__="${CHIP_DEF}")
sdk_compile_definitions(-D__REVISION__="0.1.0")
sdk_compile_definitions(-DTARGET_FLASH_SIZE=16384)
sdk_compile_definitions(-DDISABLE_IRQ_PREEMPTIVE=1)
sdk_compile_definitions(-DHPM_SDMMC_HOST_ENABLE_IRQ=1)
sdk_compile_definitions(-DHPM_SPI_DRV_DEFAULT_RETRY_COUNT=50000)


sdk_ses_opt_debug_connection(J-Link)

sdk_ld_options("-lm")

generate_ses_project()
