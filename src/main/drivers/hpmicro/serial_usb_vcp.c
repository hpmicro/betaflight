/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdint.h>
#include <stdbool.h>

#include "platform.h"
#include "board.h"
#include "usb_config.h"
#include "cli/cli.h"
/*!< endpoint address */
#define CDC_IN_EP  0x81
#define CDC_OUT_EP 0x01
#define CDC_INT_EP 0x83
#ifdef USE_VCP

#include "build/build_config.h"

#include "common/utils.h"

#include "drivers/io.h"

#include "pg/usb.h"

#include "usbd_core.h"
#include "usbd_cdc_acm.h"

#include "drivers/time.h"

#include "drivers/serial.h"
#include "drivers/serial_usb_vcp.h"
#include "hpm_usb_drv.h"
#define RINGBUFFER_SIZE  2048
typedef struct { 
    char buffer[RINGBUFFER_SIZE];
    volatile uint32_t head;
    volatile uint32_t tail;
} CircularBuffer; 

CircularBuffer cdc_buf;

extern void bf_usbd_ep_write_buffer(const void *data, int count);
extern void bf_usbd_ep_start_write(void);
extern bool bf_get_tx_flag(void);
extern uint8_t usbIsConnected(void);
extern uint8_t usbIsConfigured(void);
#define USB_TIMEOUT  50

static vcpPort_t vcpPort;
static uint32_t g_baudRate;


bool usbCableIsInserted(void)
{
    bool result = usb_get_port_ccs(HPM_USB0);

    return result;
}

static void usbVcpSetBaudRate(serialPort_t *instance, uint32_t baudRate)
{
    UNUSED(instance);
    UNUSED(baudRate);
    g_baudRate = baudRate;
    // TODO implement
}

static void usbVcpSetMode(serialPort_t *instance, portMode_e mode)
{
    UNUSED(instance);
    UNUSED(mode);

    // TODO implement
}

static void usbVcpSetCtrlLineStateCb(serialPort_t *instance, void (*cb)(void *context, uint16_t ctrlLineState), void *context)
{
    UNUSED(instance);
    UNUSED(cb);
    UNUSED(context);

    // Register upper driver control line state callback routine with USB driver
}

static void usbVcpSetBaudRateCb(serialPort_t *instance, void (*cb)(serialPort_t *context, uint32_t baud), serialPort_t *context)
{
    UNUSED(instance);
    UNUSED(cb);
    UNUSED(context);

    // Register upper driver baud rate callback routine with USB driver
}

static bool isUsbVcpTransmitBufferEmpty(const serialPort_t *instance)
{
    UNUSED(instance);
    return true;
}

void usbVcpRecieveData(uint8_t *data, uint32_t len)
{ 
    
    if (cdc_buf.head <= cdc_buf.tail) {
        if (RINGBUFFER_SIZE - cdc_buf.tail > len) {
            memcpy(&cdc_buf.buffer[cdc_buf.tail], data, len);
            cdc_buf.tail = cdc_buf.tail + len;
        } else {
            memcpy(&cdc_buf.buffer[cdc_buf.tail], data, RINGBUFFER_SIZE - cdc_buf.tail);
            data += (RINGBUFFER_SIZE - cdc_buf.tail);
            len -= (RINGBUFFER_SIZE - cdc_buf.tail);
            memcpy(&cdc_buf.buffer[0], data, len);
            cdc_buf.tail = len;
            assert(cdc_buf.tail < RINGBUFFER_SIZE);
        }
    } else {
        if (cdc_buf.head - cdc_buf.tail > len) {
            memcpy(&cdc_buf.buffer[cdc_buf.tail], data, len);
            cdc_buf.tail = cdc_buf.tail + len;
        } else {
            assert(0);
        }
    }
}

static uint32_t usbVcpAvailable(const serialPort_t *instance)
{
    UNUSED(instance);

    return (cdc_buf.tail - cdc_buf.head + RINGBUFFER_SIZE) % RINGBUFFER_SIZE; 
}

static uint8_t usbVcpRead(serialPort_t *instance)
{
    UNUSED(instance);

    uint8_t data = cdc_buf.buffer[cdc_buf.head];
    cdc_buf.head = (cdc_buf.head + 1) % RINGBUFFER_SIZE;
    return data;
}

static void usbVcpWriteBuf(serialPort_t *instance, const void *data, int count)
{
    UNUSED(instance);

    if (!(usbIsConnected() && usbIsConfigured())) {
        return;
    }

    bf_usbd_ep_write_buffer(data, count);
}

static bool usbVcpFlush(vcpPort_t *port)
{
    (void)port;
    bf_usbd_ep_start_write();
    uint32_t start = millis();
    while (bf_get_tx_flag()) {
        if (millis() - start > USB_TIMEOUT) {
            return false;
        }
    }
    return true;
}

static void usbVcpWrite(serialPort_t *instance, uint8_t c)
{
    vcpPort_t *port = container_of(instance, vcpPort_t, port);

    bf_usbd_ep_write_buffer(&c, 1);
    if (!port->buffering) {
        usbVcpFlush(port);
    }
}

static void usbVcpBeginWrite(serialPort_t *instance)
{
    vcpPort_t *port = container_of(instance, vcpPort_t, port);
    port->buffering = true;
}

static uint32_t usbTxBytesFree(const serialPort_t *instance)
{
    UNUSED(instance);
    return bf_get_tx_flag() ? 0 : 2048;
}

static void usbVcpEndWrite(serialPort_t *instance)
{
    vcpPort_t *port = container_of(instance, vcpPort_t, port);
    port->buffering = false;
    usbVcpFlush(port);
}

static const struct serialPortVTable usbVTable[] = {
    {
        .serialWrite = usbVcpWrite,
        .serialTotalRxWaiting = usbVcpAvailable,
        .serialTotalTxFree = usbTxBytesFree,
        .serialRead = usbVcpRead,
        .serialSetBaudRate = usbVcpSetBaudRate,
        .isSerialTransmitBufferEmpty = isUsbVcpTransmitBufferEmpty,
        .setMode = usbVcpSetMode,
        .setCtrlLineStateCb = usbVcpSetCtrlLineStateCb,
        .setBaudRateCb = usbVcpSetBaudRateCb,
        .writeBuf = usbVcpWriteBuf,
        .beginWrite = usbVcpBeginWrite,
        .endWrite = usbVcpEndWrite
    }
};
extern void cdc_acm_init(uint8_t busid, uint32_t reg_base);
serialPort_t *usbVcpOpen(void)
{
    vcpPort_t *s;

    board_init_usb((USB_Type *) CONFIG_HPM_USBD_BASE);   
    intc_set_irq_priority(CONFIG_HPM_USBD_IRQn, 1);
    cdc_acm_init(0, CONFIG_HPM_USBD_BASE);

    s = &vcpPort;
    s->port.vTable = usbVTable;

    return (serialPort_t *)s;
}

uint32_t usbVcpGetBaudRate(serialPort_t *instance)
{
    UNUSED(instance);

    return g_baudRate;
}

uint8_t usbVcpIsConnected(void)
{
    return usbIsConnected();
}
#endif
