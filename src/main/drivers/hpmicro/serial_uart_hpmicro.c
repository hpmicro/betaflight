/*
 * Copyright (c) 2025 HPMicro
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */

#include "platform.h"
#include "stdio.h"
#include "stdlib.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef USE_UART

#include "drivers/io.h"
#include "drivers/rcc.h"

#include "drivers/serial.h"
#include "drivers/serial_uart.h"
#include "drivers/serial_uart_impl.h"
#include "hpm_clock_drv.h"
#include "hpm_uart_drv.h"
#include "hpm_gptmr_drv.h"

typedef struct {
    uartDevice_t *uart_list[8];
    uint8_t head;
    uint8_t tail;
    uint8_t count;
} uart_queue_t;

static uart_queue_t uart_queue = {0};
static bool gptmr_configured = false;

static bool uart_queue_push(uartDevice_t *uart) {
    if (uart_queue.count >= sizeof(uart_queue.uart_list) / sizeof(uart_queue.uart_list[0])) {
        return false;
    }
    uart_queue.uart_list[uart_queue.tail] = uart;
    uart_queue.tail = (uart_queue.tail + 1) % (sizeof(uart_queue.uart_list) / sizeof(uart_queue.uart_list[0]));
    uart_queue.count++;
    return true;
}

static uartDevice_t *uart_queue_pop(void) {
    if (uart_queue.count == 0) {
        return NULL;
    }
    uartDevice_t *uart = uart_queue.uart_list[uart_queue.head];
    uart_queue.head = (uart_queue.head + 1) % (sizeof(uart_queue.uart_list) / sizeof(uart_queue.uart_list[0]));
    uart_queue.count--;
    return uart;
}

gptmr_channel_config_t config;
static void timer_config(void)
{
    uint32_t gptmr_freq;

    gptmr_freq = board_init_gptmr_clock(BOARD_GPTMR);
    gptmr_channel_get_default_config(BOARD_GPTMR, &config);

    config.reload = gptmr_freq / 1000;
    gptmr_channel_config(BOARD_GPTMR, BOARD_GPTMR_CHANNEL, &config, false);
    gptmr_start_counter(BOARD_GPTMR, BOARD_GPTMR_CHANNEL);

    gptmr_enable_irq(BOARD_GPTMR, GPTMR_CH_RLD_IRQ_MASK(BOARD_GPTMR_CHANNEL));
}

uartPort_t *serialUART(UARTDevice_e device, uint32_t baudRate, portMode_e mode,
               portOptions_e options)
{
    (void)options;
    uartDevice_t *uart = uartDevmap[device];
    if (!uart)
        return NULL;

    const uartHardware_t *hardware = uart->hardware;

    if (!hardware)
        return NULL; // XXX Can't happen !?

    uartPort_t *s = &(uart->port);
    s->port.vTable = uartVTable;

    s->port.baudRate = baudRate;

    s->port.rxBuffer = hardware->rxBuffer;
    s->port.txBuffer = hardware->txBuffer;
    s->port.rxBufferSize = hardware->rxBufferSize;
    s->port.txBufferSize = hardware->txBufferSize;

    s->USARTx = hardware->reg;

    if (hardware->rcc) {
        clock_add_to_group(hardware->rcc, 0);
    }
    if ((mode & MODE_TX) && uart->tx.pin) {
        HPM_IOC->PAD[IO_IOC_INDEX(IOGetByTag(uart->tx.pin))].FUNC_CTL =
        uart->tx.af;
        if (uart->tx.baf) {
            HPM_BIOC->PAD[IO_IOC_INDEX(IOGetByTag(uart->tx.pin))].FUNC_CTL =
            uart->tx.baf;
        }
        if (uart->tx.paf) {
            HPM_PIOC->PAD[IO_IOC_INDEX(IOGetByTag(uart->tx.pin))].FUNC_CTL =
            uart->tx.paf;
        }
    }

    if ((mode & MODE_RX) && uart->rx.pin) {
        HPM_IOC->PAD[IO_IOC_INDEX(IOGetByTag(uart->rx.pin))].FUNC_CTL =
        uart->rx.af;
        if (uart->rx.baf) {
            HPM_BIOC->PAD[IO_IOC_INDEX(IOGetByTag(uart->rx.pin))].FUNC_CTL =
            uart->rx.baf;
        }
        if (uart->rx.paf) {
            HPM_PIOC->PAD[IO_IOC_INDEX(IOGetByTag(uart->rx.pin))].FUNC_CTL =
            uart->rx.paf;
        }
    }

    if (!gptmr_configured) {
        timer_config();
        gptmr_configured = true;
    }

    return s;
}
#ifdef HPM6750
#define UART1_IRQn IRQn_UART0
#define UART6_IRQn IRQn_UART5
#define UART7_IRQn IRQn_UART6
#define UART8_IRQn IRQn_UART7
const uartHardware_t uartHardware[UARTDEV_COUNT] = {

    {
    .device = UARTDEV_1,
    .reg = (USART_TypeDef *)HPM_UART0_BASE,
    .rxPins =
    {
        {
            IO_TAG(PY7),
        },
    },
    .txPins =
    {
        {
            IO_TAG(PY6),
        },
    },
    .af = IOC_PY07_FUNC_CTL_UART0_RXD,
    .rcc = clock_uart0,
    .irqn = IRQn_UART0,
    .txBuffer = uart1TxBuffer,
    .rxBuffer = uart1RxBuffer,
    .txBufferSize = sizeof(uart1TxBuffer),
    .rxBufferSize = sizeof(uart1RxBuffer),
    },
    {
    .device = UARTDEV_6,
    .reg = (USART_TypeDef *)HPM_UART5_BASE,
    .rxPins =
    {
        {
            IO_TAG(PD6),
        },
    },
    .txPins =
    {
        {
            IO_TAG(PD7),
        },
    },
    .af = IOC_PD06_FUNC_CTL_UART5_RXD,
    .rcc = clock_uart5,
    .irqn = IRQn_UART5,
    .txBuffer = uart6TxBuffer,
    .rxBuffer = uart6RxBuffer,
    .txBufferSize = sizeof(uart6TxBuffer),
    .rxBufferSize = sizeof(uart6RxBuffer),
    },
    {
    .device = UARTDEV_8,
    .reg = (USART_TypeDef *)HPM_UART7_BASE,
    .rxPins =
    {
        {
            IO_TAG(PE30),
        },
    },
    .txPins =
    {
        {
            IO_TAG(PE31),
        },
    },
    .af = IOC_PE30_FUNC_CTL_UART7_RXD,
    .rcc = clock_uart7,
    .irqn = IRQn_UART7,
    .txBuffer = uart8TxBuffer,
    .rxBuffer = uart8RxBuffer,
    .txBufferSize = sizeof(uart8TxBuffer),
    .rxBufferSize = sizeof(uart8RxBuffer),
    },

};
#endif

#ifdef HPM6360
#define UART1_IRQn IRQn_UART0
#define UART2_IRQn IRQn_UART1
#define UART3_IRQn IRQn_UART2
#define UART4_IRQn IRQn_UART3
#define UART5_IRQn IRQn_UART4
#define UART6_IRQn IRQn_UART5
#define UART7_IRQn IRQn_UART6
#define UART8_IRQn IRQn_UART7
#define UART1_CLOCK clock_uart0
#define UART2_CLOCK clock_uart1
#define UART3_CLOCK clock_uart2
#define UART4_CLOCK clock_uart3
#define UART5_CLOCK clock_uart4
#define UART6_CLOCK clock_uart5
#define UART7_CLOCK clock_uart6
#define UART8_CLOCK clock_uart7
const uartHardware_t uartHardware[UARTDEV_COUNT] = {

#ifdef USE_UART1
    {
    .device = UARTDEV_1,
    .reg = (USART_TypeDef *)HPM_UART0_BASE,
    .rxPins =
    {
        {
            IO_TAG(PA31),
            .af = IOC_PA31_FUNC_CTL_UART0_RXD,
        },
        {
            IO_TAG(PB23),
            .af = IOC_PB23_FUNC_CTL_UART0_RXD,
        },
        {
            IO_TAG(PY7),
            .af = IOC_PY07_FUNC_CTL_UART0_RXD,
            .paf = PIOC_PY07_FUNC_CTL_SOC_PY_07,
        },
    },
    .txPins =
    {
        {
            IO_TAG(PA30),
            .af = IOC_PA30_FUNC_CTL_UART0_TXD,
        },
        {
            IO_TAG(PB22),
            .af = IOC_PB22_FUNC_CTL_UART0_TXD,
        },
        {
            IO_TAG(PY6),
            .af = IOC_PY06_FUNC_CTL_UART0_TXD,
            .paf = PIOC_PY06_FUNC_CTL_SOC_PY_06,
        },
    },
    .rcc = UART1_CLOCK,
    .irqn = UART1_IRQn,
    .txBuffer = uart1TxBuffer,
    .rxBuffer = uart1RxBuffer,
    .txBufferSize = sizeof(uart1TxBuffer),
    .rxBufferSize = sizeof(uart1RxBuffer),
    },
#endif
#ifdef USE_UART2
    {
    .device = UARTDEV_2,
    .reg = (USART_TypeDef *)HPM_UART1_BASE,
    .rxPins =
    {
        {
            IO_TAG(PA1),
            .af = IOC_PA01_FUNC_CTL_UART1_RXD,
        },
        {
            IO_TAG(PB1),
            .af = IOC_PB01_FUNC_CTL_UART1_RXD,
        },
        {
            IO_TAG(PB25),
            .af = IOC_PB25_FUNC_CTL_UART1_RXD,
        },
    },
    .txPins =
    {
        {
            IO_TAG(PA0),
            .af = IOC_PA00_FUNC_CTL_UART1_TXD,
        },
        {
            IO_TAG(PB0),
            .af = IOC_PB00_FUNC_CTL_UART1_TXD,
        },
        {
            IO_TAG(PB24),
            .af = IOC_PB24_FUNC_CTL_UART1_TXD,
        },
    },
    .rcc = UART2_CLOCK,
    .irqn = UART2_IRQn,
    .txBuffer = uart2TxBuffer,
    .rxBuffer = uart2RxBuffer,
    .txBufferSize = sizeof(uart2TxBuffer),
    .rxBufferSize = sizeof(uart2RxBuffer),
    },
#endif
#ifdef USE_UART3
    {
    .device = UARTDEV_3,
    .reg = (USART_TypeDef *)HPM_UART2_BASE,
    .rxPins =
    {
        {
            IO_TAG(PA3),
            .af = IOC_PA03_FUNC_CTL_UART2_RXD,
        },
        {
            IO_TAG(PB27),
            .af = IOC_PB27_FUNC_CTL_UART2_RXD,
        },
        {
            IO_TAG(PC27),
            .af = IOC_PC27_FUNC_CTL_UART2_RXD,
        },
    },
    .txPins =
    {
        {
            IO_TAG(PA2),
            .af = IOC_PA02_FUNC_CTL_UART2_TXD,
        },
        {
            IO_TAG(PB26),
            .af = IOC_PB26_FUNC_CTL_UART2_TXD,
        },
        {
            IO_TAG(PC26),
            .af = IOC_PC26_FUNC_CTL_UART2_TXD,
        },
    },
    .rcc = UART3_CLOCK,
    .irqn = UART3_IRQn,
    .txBuffer = uart3TxBuffer,
    .rxBuffer = uart3RxBuffer,
    .txBufferSize = sizeof(uart3TxBuffer),
    .rxBufferSize = sizeof(uart3RxBuffer),
    },
#endif
#ifdef USE_UART4
    {
    .device = UARTDEV_4,
    .reg = (USART_TypeDef *)HPM_UART3_BASE,
    .rxPins =
    {
        {
            IO_TAG(PA5),
            .af = IOC_PA05_FUNC_CTL_UART3_RXD,
        },
        {
            IO_TAG(PB29),
            .af = IOC_PB29_FUNC_CTL_UART3_RXD,
        },
        {
            IO_TAG(PZ1),
            .af = IOC_PZ01_FUNC_CTL_UART3_RXD,
            .baf = BIOC_PZ01_FUNC_CTL_SOC_PZ_01,
        },
    },
    .txPins =
    {
        {
            IO_TAG(PA4),
            .af = IOC_PA04_FUNC_CTL_UART3_TXD,
        },
        {
            IO_TAG(PB28),
            .af = IOC_PB28_FUNC_CTL_UART3_TXD,
        },
        {
            IO_TAG(PZ0),
            .af = IOC_PZ00_FUNC_CTL_UART3_TXD,
            .baf = BIOC_PZ00_FUNC_CTL_SOC_PZ_00,
        },
    },
    .rcc = UART4_CLOCK,
    .irqn = UART4_IRQn,
    .txBuffer = uart4TxBuffer,
    .rxBuffer = uart4RxBuffer,
    .txBufferSize = sizeof(uart4TxBuffer),
    .rxBufferSize = sizeof(uart4RxBuffer),
    },
#endif
#ifdef USE_UART5
    {
    .device = UARTDEV_5,
    .reg = (USART_TypeDef *)HPM_UART4_BASE,
    .rxPins =
    {
        {
            IO_TAG(PC7),
            .af = IOC_PC07_FUNC_CTL_UART4_RXD,
        },
        {
            IO_TAG(PZ3),
            .af = IOC_PZ03_FUNC_CTL_UART4_RXD,
            .baf = BIOC_PZ03_FUNC_CTL_SOC_PZ_03,
        },
    },
    .txPins =
    {
        {
            IO_TAG(PC6),
            .af = IOC_PC06_FUNC_CTL_UART4_TXD,   
        },
        {
            IO_TAG(PZ2),
            .af = IOC_PZ02_FUNC_CTL_UART4_TXD,
            .baf = BIOC_PZ02_FUNC_CTL_SOC_PZ_02,
        },
    },
    .rcc = UART5_CLOCK,
    .irqn = UART5_IRQn,
    .txBuffer = uart5TxBuffer,
    .rxBuffer = uart5RxBuffer,
    .txBufferSize = sizeof(uart5TxBuffer),
    .rxBufferSize = sizeof(uart5RxBuffer),
    },
#endif
#ifdef USE_UART6
    {
    .device = UARTDEV_6,
    .reg = (USART_TypeDef *)HPM_UART5_BASE,
    .rxPins =
    {
        {
            IO_TAG(PC9),
            .af = IOC_PC09_FUNC_CTL_UART5_RXD,
        },  
        {
            IO_TAG(PA17),
            .af = IOC_PA17_FUNC_CTL_UART5_RXD,
        },
    },
    .txPins =
    {
        {
            IO_TAG(PC8),
            .af = IOC_PC08_FUNC_CTL_UART5_TXD,   
        },
        {
            IO_TAG(PA16),
            .af = IOC_PA16_FUNC_CTL_UART5_TXD,  
        },
    },
    .rcc = UART6_CLOCK,
    .irqn = UART6_IRQn,
    .txBuffer = uart6TxBuffer,
    .rxBuffer = uart6RxBuffer,
    .txBufferSize = sizeof(uart6TxBuffer),
    .rxBufferSize = sizeof(uart6RxBuffer),
    },
#endif
#ifdef USE_UART7
    {
    .device = UARTDEV_7,
    .reg = (USART_TypeDef *)HPM_UART6_BASE,
    .rxPins =
    {
        {
            IO_TAG(PA19),
            .af = IOC_PA19_FUNC_CTL_UART6_RXD,
        },  
        {
            IO_TAG(PB11),
            .af = IOC_PB11_FUNC_CTL_UART6_RXD,
        },
        {   
            IO_TAG(PC11),
            .af = IOC_PC11_FUNC_CTL_UART6_RXD,
        },
    },
    .txPins =
    {
        {
            IO_TAG(PA18),
            .af = IOC_PA18_FUNC_CTL_UART6_TXD,
        },
        {   
            IO_TAG(PC10),
            .af = IOC_PC10_FUNC_CTL_UART6_TXD,
        },
    },
    .rcc = UART7_CLOCK,
    .irqn = UART7_IRQn,
    .txBuffer = uart7TxBuffer,
    .rxBuffer = uart7RxBuffer,
    .txBufferSize = sizeof(uart7TxBuffer),
    .rxBufferSize = sizeof(uart7RxBuffer),
    },
#endif
#ifdef USE_UART8
    {
    .device = UARTDEV_8,
    .reg = (USART_TypeDef *)HPM_UART7_BASE,
    .rxPins =
    {
        {
            IO_TAG(PA21),
            .af = IOC_PA21_FUNC_CTL_UART7_RXD,
        },  
        {
            IO_TAG(PB13),
            .af = IOC_PB13_FUNC_CTL_UART7_RXD,
        },
        {   
            IO_TAG(PC13),
            .af = IOC_PC13_FUNC_CTL_UART7_RXD,
        },
        {   
            IO_TAG(PY5),
            .af = IOC_PY05_FUNC_CTL_UART7_RXD,
        },
    },
    .txPins =
    {
        {
            IO_TAG(PA20),
            .af = IOC_PA20_FUNC_CTL_UART7_TXD,
        },  
        {
            IO_TAG(PB12),
            .af = IOC_PB12_FUNC_CTL_UART7_TXD,
        },
        {   
            IO_TAG(PC12),
            .af = IOC_PC12_FUNC_CTL_UART7_TXD,
        },
        {       
            IO_TAG(PY4),
            .af = IOC_PY04_FUNC_CTL_UART7_TXD,
        },
    },
    .rcc = UART8_CLOCK,
    .irqn = UART8_IRQn,
    .txBuffer = uart8TxBuffer,
    .rxBuffer = uart8RxBuffer,
    .txBufferSize = sizeof(uart8TxBuffer),
    .rxBufferSize = sizeof(uart8RxBuffer),
    }
#endif
};
#endif

SDK_DECLARE_EXT_ISR_M(BOARD_GPTMR_IRQ, tick_ms_isr)
void tick_ms_isr(void)
{
    if (gptmr_check_status(BOARD_GPTMR, GPTMR_CH_RLD_STAT_MASK(BOARD_GPTMR_CHANNEL))) {
        gptmr_clear_status(BOARD_GPTMR, GPTMR_CH_RLD_STAT_MASK(BOARD_GPTMR_CHANNEL));

        uartDevice_t *uart = uart_queue_pop();
        while (uart != NULL) {
            if (uart->tx.pin) {
                HPM_IOC->PAD[IO_IOC_INDEX(IOGetByTag(uart->tx.pin))].FUNC_CTL = 0;
            }
            if (uart->rx.pin) {
                HPM_IOC->PAD[IO_IOC_INDEX(IOGetByTag(uart->rx.pin))].FUNC_CTL = uart->rx.af;
            }
            
            uart = uart_queue_pop();
        }
        gptmr_stop_counter(BOARD_GPTMR, BOARD_GPTMR_CHANNEL);
        intc_m_disable_irq(BOARD_GPTMR_IRQ);
    }
}

void uartReconfigure(uartPort_t *uartPort)
{
    hpm_stat_t stat;
    uart_config_t config = {0};
    uartDevice_t *uart = container_of(uartPort, uartDevice_t, port);

    uart_default_config((UART_Type *)uartPort->USARTx, &config);
    config.src_freq_in_hz = clock_get_frequency(uart->hardware->rcc);
    config.baudrate = uartPort->port.baudRate;
    config.num_of_stop_bits = (uartPort->port.options & SERIAL_STOPBITS_2)
                  ? stop_bits_2
                  : stop_bits_1;
    config.parity = (uartPort->port.options & SERIAL_PARITY_EVEN) ? parity_even
                                  : parity_none;

#if defined(HPM_IP_FEATURE_UART_RX_EN) && (HPM_IP_FEATURE_UART_RX_EN == 1)
    if (uartPort->port.mode & MODE_RX)
    config.rx_enable = true;
#endif
    stat = uart_init((UART_Type *)uartPort->USARTx, &config);
    if (status_success != stat) {
    while (1)
        ;
    }

    if (uartPort->port.options & SERIAL_BIDIR) {
        if ((uartPort->port.mode & MODE_TX) && uart->tx.pin) {
            HPM_IOC->PAD[IO_IOC_INDEX(IOGetByTag(uart->tx.pin))].FUNC_CTL = 0;
        }
    }
    if (uartPort->port.mode & MODE_RX) {
        uart_enable_irq((UART_Type *)uartPort->USARTx,
                uart_intr_rx_data_avail_or_timeout);
    }
    intc_m_enable_irq_with_priority(uart->hardware->irqn, 1);
}

void uart_isr(uartDevice_t *uart)
{
    uartPort_t *s = &(uart->port);

    uint8_t irq_id = uart_get_irq_id((UART_Type *)(s->USARTx));
    if (irq_id == uart_intr_id_rx_data_avail) {
        if (s->port.rxCallback) {
            s->port.rxCallback(uart_read_byte((UART_Type *)(s->USARTx)),
                    s->port.rxCallbackData);
        } else {
            s->port.rxBuffer[s->port.rxBufferHead] =
            uart_read_byte((UART_Type *)(s->USARTx));
            s->port.rxBufferHead =
            (s->port.rxBufferHead + 1) % s->port.rxBufferSize;
        }
    }
    if (irq_id == uart_intr_id_tx_slot_avail) {
        if (s->port.txBufferTail != s->port.txBufferHead) {
            uart_write_byte((UART_Type *)(s->USARTx),
                    s->port.txBuffer[s->port.txBufferTail]);
            s->port.txBufferTail =
            (s->port.txBufferTail + 1) % s->port.txBufferSize;
        } else {
            uart_disable_irq((UART_Type *)(s->USARTx), uart_intr_tx_slot_avail);
            if (s->port.options & SERIAL_BIDIR) {
               uart_queue_push(uart);
               gptmr_channel_reset_count(BOARD_GPTMR, BOARD_GPTMR_CHANNEL);
               gptmr_clear_status(BOARD_GPTMR, GPTMR_CH_RLD_STAT_MASK(BOARD_GPTMR_CHANNEL));
               gptmr_start_counter(BOARD_GPTMR, BOARD_GPTMR_CHANNEL);
               intc_m_enable_irq_with_priority(BOARD_GPTMR_IRQ, 1);
            }
        }
    }
}

#ifdef USE_UART1
void uart_isr0(void)
{
    uartDevice_t *uart = (uartDevmap[UARTDEV_1]);
    uart_isr(uart);
}
SDK_DECLARE_EXT_ISR_M(UART1_IRQn, uart_isr0)
#endif

#ifdef USE_UART2
void uart_isr1(void)
{
    uartDevice_t *uartPort = (uartDevmap[UARTDEV_2]);
    uart_isr(uartPort);
}
SDK_DECLARE_EXT_ISR_M(UART2_IRQn, uart_isr1)
#endif

#ifdef USE_UART3
void uart_isr2(void)
{
    uartDevice_t *uartPort = (uartDevmap[UARTDEV_3]);
    uart_isr(uartPort);
}
SDK_DECLARE_EXT_ISR_M(UART3_IRQn, uart_isr2)
#endif

#ifdef USE_UART4
void uart_isr3(void)
{
    uartDevice_t *uartPort = (uartDevmap[UARTDEV_4]);
    uart_isr(uartPort);
}
SDK_DECLARE_EXT_ISR_M(UART4_IRQn, uart_isr3)
#endif

#ifdef USE_UART5
void uart_isr4(void)
{
    uartDevice_t *uartPort = (uartDevmap[UARTDEV_5]);
    uart_isr(uartPort);
}
SDK_DECLARE_EXT_ISR_M(UART5_IRQn, uart_isr4)
#endif

#ifdef USE_UART6
void uart_isr5(void)
{
    uartDevice_t *uartPort = (uartDevmap[UARTDEV_6]);
    uart_isr(uartPort);
}
SDK_DECLARE_EXT_ISR_M(UART6_IRQn, uart_isr5)
#endif

#ifdef USE_UART7
void uart_isr6(void)
{
    uartDevice_t *uartPort = (uartDevmap[UARTDEV_7]);
    uart_isr(uartPort);
}
SDK_DECLARE_EXT_ISR_M(UART7_IRQn, uart_isr6)
#endif

#ifdef USE_UART8
void uart_isr7(void)
{
    uartDevice_t *uartPort = (uartDevmap[UARTDEV_8]);
    uart_isr(uartPort);
}
SDK_DECLARE_EXT_ISR_M(UART8_IRQn, uart_isr7)
#endif

#ifdef USE_UART9
void uart_isr8(void)
{
    uartDevice_t *uartPort = (uartDevmap[UARTDEV_9]);
    uart_isr(uartPort);
}
SDK_DECLARE_EXT_ISR_M(UART9_IRQn, uart_isr8)
#endif
#ifdef USE_DMA
void uartTryStartTxDMA(uartPort_t *s) { (void)s; }

#endif
#endif // USE_UART
