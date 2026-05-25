/*
 * Copyright (c) 2025-2026 HPMicro
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */

#pragma once

#include <stddef.h>
#include <stdint.h>

#include "common/utils.h"
#include "hpm_dma_drv.h"
#include "hpm_soc.h"
#include "stdio.h"
#include "string.h"
#undef TASK_GYROPID_DESIRED_PERIOD
#undef SCHEDULER_DELAY_LIMIT
#undef USE_DASHBOARD
#undef USE_TELEMETRY_LTM
#undef USE_VTX_RTC6705_SOFTSPI
#undef USE_RX_REDPINE_SPI
#undef USE_RANGEFINDER
#undef USE_RX_EXPRESSLRS
#undef USE_OSD
#undef USE_RX_PPM
#undef USE_RX_PWM
#undef USE_TELEMETRY_FRSKY_HUB
#undef USE_TELEMETRY_HOTT
#undef USE_TELEMETRY_SMARTPORT
#undef USE_TELEMETRY_MAVLINK
#undef USE_RESOURCE_MGMT
#undef USE_TELEMETRY_CRSF
#undef USE_TELEMETRY_GHST
#undef USE_TELEMETRY_IBUS
#undef USE_TELEMETRY_JETIEXBUS
#undef USE_TELEMETRY_SRXL
#undef USE_SERIALRX_JETIEXBUS
#undef USE_I2C
#undef USE_PINIO
#undef USE_SERVOS
#undef USE_MULTI_GYRO
#undef USE_DSHOT_DMAR
#undef USE_DSHOT_BITBANG
#undef USE_RPM_LIMIT
#undef USE_TRANSPONDER
#undef USE_FLASH_M25P16
#undef USE_RX_SPI
#undef USE_OSD_HD

#define FAST_IRQ_HANDLER FAST_CODE
#define TARGET_BOARD_IDENTIFIER "HPM6754"
#define USE_SDCARD_SDIO 1
#define SDIO_DEVICE SDIODEV_1
#define MAX_SUPPORTED_MOTORS 4
#define FULL_TIMER_CHANNEL_COUNT 4
#define USE_QUAD_MIXER_ONLY 1
#define DEFAULT_CPU_OVERCLOCK 1
#define DMA_RAM ATTR_ALIGN (64)
#define DMA_RW_AXI ATTR_ALIGN (64)
#define DMA_RAM_R ATTR_ALIGN (64)
#define DMA_RAM_W ATTR_ALIGN (64)
#define DMA_RAM_RW ATTR_ALIGN (64)
#define DMA_DATA_ZERO_INIT ATTR_ALIGN (64)
#define DMA_DATA ATTR_ALIGN (64)
#define STATIC_DMA_DATA_AUTO static
#define USE_64BIT_TIME
#define LED_STRIP_PIN PY4
#define EEPROM_FILENAME "eeprom.bin"
#define EEPROM_SIZE 32768
#define USE_USB_MSC 1
#define USE_VCP
#define USE_EXST 1
#define U_ID_0 0
#define U_ID_1 1
#define U_ID_2 2
#define USE_EXTI
#define LED0_PIN PD3
#define LED0_INVERTED 1
#define USE_ADC
#define ADC_VBAT_PIN PE16
#define USE_PERSISTENT_OBJECTS 1
#define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_ADC
#define TASK_GYROPID_DESIRED_PERIOD TASK_PERIOD_HZ (1000)
#define SCHEDULER_DELAY_LIMIT 1
#define USABLE_TIMER_CHANNEL_COUNT 0
#define USE_TIMER_DMA
#define USE_UART1
#define UART1_TX_PIN PY6
#define UART1_RX_PIN PY7
#define USE_UART6
#define UART6_TX_PIN PD7
#define UART6_RX_PIN PD6
#define USE_UART8
#define UART8_TX_PIN PE31
#define UART8_RX_PIN PE30
#define GPS_UART SERIAL_PORT_USART3
#define USE_MOTOR
#define SERIAL_PORT_COUNT 4
#define REQUIRE_CC_ARM_PRINTF_SUPPORT
#define DEFAULT_RX_FEATURE FEATURE_RX_SERIAL
#define DEFAULT_FEATURES (FEATURE_TELEMETRY)
#define NVIC_PRIO_MAX 6
#ifndef USE_PWM_OUTPUT
#define USE_PWM_OUTPUT
#endif
#define USE_GYRO_SPI_ICM20602 1
#define USE_I2C_DEVICE_1
#define USE_SPI
#define USE_SPI_DMA_ENABLE_LATE
#define USE_SPI_DEVICE_1
#define USE_SPI_DEVICE_2
#define USE_SPI_DEVICE_3
#define USE_I2C_PULLUP 1
#define USE_PIN_AF
#define USE_DMA_SPEC
#define MAX7456_SPI_CS_PIN PA26
#define MAX7456_SPI_INSTANCE SPI3
#define MOTOR1_PIN PA19
#define MOTOR2_PIN PA20
#define MOTOR3_PIN PA24
#define MOTOR4_PIN PA25
#define USE_RPM_FILTER 1
#define USE_GYRO_SPI_ICM42605 1
#define USE_ACC_SPI_ICM42605 1
#define USE_OSD
#define SPI1_SCK_PIN PZ3
#define SPI1_SDI_PIN PZ4
#define SPI1_SDO_PIN PZ5
#define SPI2_SCK_PIN PD31
#define SPI2_SDI_PIN PE4
#define SPI2_SDO_PIN PD30
#define SPI3_SCK_PIN PB0
#define SPI3_SDI_PIN PA27
#define SPI3_SDO_PIN PA31
#define SDCARD_DETECT_INVERTED
#define SDCARD_DETECT_PIN PE18
#define SDCARD_SPI_CS_PIN PF24
#define SDCARD_SPI_INSTANCE SPI2
#define USE_SPI_GYRO 1
#define GYRO_1_SPI_INSTANCE SPI2
#define GYRO_1_CS_PIN PE3
#define GYRO_1_EXTI_PIN PE15
#define GYRO_2_CS_PIN PZ2
#define GYRO_2_EXTI_PIN PZ7
#define I2C_DEVICE I2CDEV_1
#define TARGET_IO_PORTA 0xffffffff
#define TARGET_IO_PORTB 0xffffffff
#define TARGET_IO_PORTC 0xffffffff
#define TARGET_IO_PORTD 0xffffffff
#define TARGET_IO_PORTE 0xffffffff
#define TARGET_IO_PORTF 0xffffffff
#define TARGET_IO_PORTX 0xffffffff
#define TARGET_IO_PORTY 0xffffffff
#define TARGET_IO_PORTZ 0xffffffff

/** @defgroup GPIO_mode_define GPIO mode define
 * @brief GPIO Configuration Mode
 *        Elements values convention: 0xX0yz00YZ
 *           - X  : GPIO mode or EXTI Mode
 *           - y  : External IT or Event trigger detection
 *           - z  : IO configuration on External IT or Event
 *           - Y  : Output type (Push Pull or Open Drain)
 *           - Z  : IO Direction mode (Input, Output, Alternate or Analog)
 * @{
 */
#define GPIO_MODE_INPUT                                                       \
  0x00000000U /*!< Input Floating Mode                   */
#define GPIO_MODE_OUTPUT_PP                                                   \
  0x00000001U /*!< Output Push Pull Mode                 */
#define GPIO_MODE_OUTPUT_OD                                                   \
  0x00000011U /*!< Output Open Drain Mode                */
#define GPIO_MODE_AF_PP                                                       \
  0x00000002U /*!< Alternate Function Push Pull Mode     */
#define GPIO_MODE_AF_OD                                                       \
  0x00000012U /*!< Alternate Function Open Drain Mode    */

#define GPIO_MODE_ANALOG 0x00000003U /*!< Analog Mode  */
#define GPIO_MODE_SPI 0x00000005U

#define GPIO_MODE_IT_RISING                                                   \
  0x10110000U /*!< External Interrupt Mode with Rising edge trigger           \
                 detection          */
#define GPIO_MODE_IT_FALLING                                                  \
  0x10210000U /*!< External Interrupt Mode with Falling edge trigger          \
                 detection         */
#define GPIO_MODE_IT_RISING_FALLING                                           \
  0x10310000U /*!< External Interrupt Mode with Rising/Falling edge trigger   \
                 detection  */

#define GPIO_MODE_EVT_RISING                                                  \
  0x10120000U /*!< External Event Mode with Rising edge trigger detection */
#define GPIO_MODE_EVT_FALLING                                                 \
  0x10220000U /*!< External Event Mode with Falling edge trigger detection    \
               */
#define GPIO_MODE_EVT_RISING_FALLING                                          \
  0x10320000U /*!< External Event Mode with Rising/Falling edge trigger       \
                 detection       */
/**
 * @}
 */

/** @defgroup GPIO_speed_define  GPIO speed define
 * @brief GPIO Output Maximum frequency
 * @{
 */
#define GPIO_SPEED_FREQ_LOW                                                   \
  0x00000000U /*!< IO works at 2 MHz, please refer to the product datasheet   \
               */
#define GPIO_SPEED_FREQ_MEDIUM                                                \
  0x00000001U /*!< range 12,5 MHz to 50 MHz, please refer to the product      \
                 datasheet */
#define GPIO_SPEED_FREQ_HIGH                                                  \
  0x00000002U /*!< range 25 MHz to 100 MHz, please refer to the product       \
                 datasheet  */
#define GPIO_SPEED_FREQ_VERY_HIGH                                             \
  0x00000003U /*!< range 50 MHz to 200 MHz, please refer to the product       \
                 datasheet  */
/**
 * @}
 */

/** @defgroup GPIO_pull_define GPIO pull define
 * @brief GPIO Pull-Up or Pull-Down Activation
 * @{
 */
#define GPIO_NOPULL 0x00000000U   /*!< No Pull-up or Pull-down activation  */
#define GPIO_PULLUP 0x00000001U   /*!< Pull-up activation                  */
#define GPIO_PULLDOWN 0x00000002U /*!< Pull-down activation                */
#define DEFAULT_BLACKBOX_DEVICE BLACKBOX_DEVICE_SDCARD
#define LED_STRIP_TIMER 1
#define SOFTSERIAL_1_TIMER 2
#define SOFTSERIAL_2_TIMER 3
#define DEFIO_NO_PORTS // suppress 'no pins defined' warning
#undef USE_FLASH
#define FLASH_PAGE_SIZE (0x400)

// belows are internal stuff

extern uint32_t SystemCoreClock;

typedef uint32_t FunctionalState;
typedef uint32_t IRQn_Type;
typedef enum
{
  EXTI_Trigger_Rising = 0x08,
  EXTI_Trigger_Falling = 0x0C,
  EXTI_Trigger_Rising_Falling = 0x10
} EXTITrigger_TypeDef;

typedef GPIO_Type GPIO_TypeDef;
#define GPIOA_BASE HPM_GPIO0_BASE
#if defined(HPM6750) || defined(HPM6360)
typedef PWM_Type TIM_TypeDef;
#else
typedef PWMV2_Type TIM_TypeDef;
#endif

typedef DMA_Type DMA_TypeDef;
typedef uint32_t DMA_Channel_TypeDef;
typedef dma_channel_config_t DMA_InitTypeDef;
typedef SPI_Type SPI_TypeDef;
typedef SPI_Type spi_type;
typedef UART_Type USART_TypeDef;
typedef uint32_t DMA_Stream_TypeDef;
#define USART1 ((USART_TypeDef *)0x0001)
#define USART2 ((USART_TypeDef *)0x0002)
#define USART3 ((USART_TypeDef *)0x0003)
#define USART4 ((USART_TypeDef *)0x0004)
#define USART5 ((USART_TypeDef *)0x0005)
#define USART6 ((USART_TypeDef *)0x0006)
#define USART7 ((USART_TypeDef *)0x0007)
#define USART8 ((USART_TypeDef *)0x0008)

#define UART4 ((USART_TypeDef *)0x0004)
#define UART5 ((USART_TypeDef *)0x0005)
#define UART7 ((USART_TypeDef *)0x0007)
#define UART8 ((USART_TypeDef *)0x0008)

typedef I2C_Type I2C_TypeDef;
