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
        hardware->af;
        if (hardware->bioc_func) {
            HPM_BIOC->PAD[IO_IOC_INDEX(IOGetByTag(uart->tx.pin))].FUNC_CTL =
            hardware->bioc_func;
        }
        if (hardware->pioc_func) {
            HPM_PIOC->PAD[IO_IOC_INDEX(IOGetByTag(uart->tx.pin))].FUNC_CTL =
            hardware->pioc_func;
        }
    }

    if ((mode & MODE_RX) && uart->rx.pin) {
        HPM_IOC->PAD[IO_IOC_INDEX(IOGetByTag(uart->rx.pin))].FUNC_CTL =
        hardware->af;
        if (hardware->bioc_func) {
            HPM_BIOC->PAD[IO_IOC_INDEX(IOGetByTag(uart->rx.pin))].FUNC_CTL =
            hardware->bioc_func;
        }
        if (hardware->pioc_func) {
            HPM_PIOC->PAD[IO_IOC_INDEX(IOGetByTag(uart->rx.pin))].FUNC_CTL =
            hardware->pioc_func;
        }
    }

    return s;
}
#ifdef HPM6750
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
                if (uart->tx.pin) {
                    HPM_IOC->PAD[IO_IOC_INDEX(IOGetByTag(uart->tx.pin))]
                    .FUNC_CTL = 0;
                }
                if (uart->rx.pin) {
                    const uartHardware_t *hardware = uart->hardware;
                    HPM_IOC->PAD[IO_IOC_INDEX(IOGetByTag(uart->rx.pin))]
                    .FUNC_CTL = hardware->af;
                }
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
SDK_DECLARE_EXT_ISR_M(IRQn_UART0, uart_isr0)
#endif

#ifdef USE_UART2
void uart_isr1(void)
{
    uartDevice_t *uartPort = (uartDevmap[UARTDEV_2]);
    uart_isr(uartPort);
}
SDK_DECLARE_EXT_ISR_M(IRQn_UART1, uart_isr1)
#endif

#ifdef USE_UART3
void uart_isr2(void)
{
    uartDevice_t *uartPort = (uartDevmap[UARTDEV_3]);
    uart_isr(uartPort);
}
SDK_DECLARE_EXT_ISR_M(IRQn_UART2, uart_isr2)
#endif

#ifdef USE_UART4
void uart_isr3(void)
{
    uartDevice_t *uartPort = (uartDevmap[UARTDEV_4]);
    uart_isr(uartPort);
}
SDK_DECLARE_EXT_ISR_M(IRQn_UART3, uart_isr3)
#endif

#ifdef USE_UART5
void uart_isr4(void)
{
    uartDevice_t *uartPort = (uartDevmap[UARTDEV_5]);
    uart_isr(uartPort);
}
SDK_DECLARE_EXT_ISR_M(IRQn_UART4, uart_isr4)
#endif

#ifdef USE_UART6
void uart_isr5(void)
{
    uartDevice_t *uartPort = (uartDevmap[UARTDEV_6]);
    uart_isr(uartPort);
}
SDK_DECLARE_EXT_ISR_M(IRQn_UART5, uart_isr5)
#endif

#ifdef USE_UART7
void uart_isr6(void)
{
    uartDevice_t *uartPort = (uartDevmap[UARTDEV_7]);
    uart_isr(uartPort);
}
SDK_DECLARE_EXT_ISR_M(IRQn_UART6, uart_isr6)
#endif

#ifdef USE_UART8
void uart_isr7(void)
{
    uartDevice_t *uartPort = (uartDevmap[UARTDEV_8]);
    uart_isr(uartPort);
}
SDK_DECLARE_EXT_ISR_M(IRQn_UART7, uart_isr7)
#endif

#ifdef USE_UART9
void uart_isr8(void)
{
    uartDevice_t *uartPort = (uartDevmap[UARTDEV_9]);
    uart_isr(uartPort);
}
SDK_DECLARE_EXT_ISR_M(IRQn_UART8, uart_isr8)
#endif
#ifdef USE_DMA
void uartTryStartTxDMA(uartPort_t *s) { (void)s; }

#endif

#endif // USE_UART
