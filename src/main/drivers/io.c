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

#include "platform.h"

#include "drivers/io.h"
#include "drivers/io_impl.h"
#include "drivers/rcc.h"

#include "common/utils.h"

// io ports defs are stored in array by index now
struct ioPortDef_s {
    rccPeriphTag_t rcc;
};

#if defined(SITL)
const struct ioPortDef_s ioPortDefs[] = { 0 };
#endif

ioRec_t* IO_Rec(IO_t io)
{
    return io;
}

GPIO_TypeDef* IO_GPIO(IO_t io)
{
    const ioRec_t *ioRec = IO_Rec(io);
    return ioRec->gpio;
}

#ifdef HPMicro
uint32_t IO_IOC_INDEX(IO_t io)
{
    const ioRec_t *ioRec = IO_Rec(io);
    return ioRec->ioc_index;
}

uint32_t IO_PIOC_INDEX(IO_t io)
{
    const ioRec_t *ioRec = IO_Rec(io);
    return ioRec->pioc_index;
}

uint32_t IO_BIOC_INDEX(IO_t io)
{
    const ioRec_t *ioRec = IO_Rec(io);
    return ioRec->bioc_index;
}
#endif

uint16_t IO_Pin(IO_t io)
{
    const ioRec_t *ioRec = IO_Rec(io);
    return ioRec->pin;
}

int IO_GPIOPortIdx(IO_t io)
{
    if (!io) {
        return -1;
    }
#ifdef HPMicro
    uint32_t ioc_idx = IO_IOC_INDEX(io);
    return GPIO_GET_PORT_INDEX(ioc_idx);
#else
    return (((size_t)IO_GPIO(io) - GPIOA_BASE) >> 10);
#endif
}

int IO_EXTI_PortSourceGPIO(IO_t io)
{
    return IO_GPIOPortIdx(io);
}

int IO_GPIO_PortSource(IO_t io)
{
    return IO_GPIOPortIdx(io);
}

// zero based pin index
int IO_GPIOPinIdx(IO_t io)
{
    if (!io) {
        return -1;
    }
#ifdef HPMicro
    uint32_t ioc_idx = IO_IOC_INDEX(io);
    return GPIO_GET_PIN_INDEX(ioc_idx);
#else
    return 31 - __builtin_clz(IO_Pin(io));
#endif
}

int IO_EXTI_PinSource(IO_t io)
{
    return IO_GPIOPinIdx(io);
}

int IO_GPIO_PinSource(IO_t io)
{
    return IO_GPIOPinIdx(io);
}

// claim IO pin, set owner and resources
void IOInit(IO_t io, resourceOwner_e owner, uint8_t index)
{
    if (!io) {
        return;
    }
    ioRec_t *ioRec = IO_Rec(io);
    ioRec->owner = owner;
    ioRec->index = index;
}

void IORelease(IO_t io)
{
    if (!io) {
        return;
    }
    ioRec_t *ioRec = IO_Rec(io);
    ioRec->owner = OWNER_FREE;
}

resourceOwner_e IOGetOwner(IO_t io)
{
    if (!io) {
        return OWNER_FREE;
    }
    const ioRec_t *ioRec = IO_Rec(io);
    return ioRec->owner;
}

bool IOIsFreeOrPreinit(IO_t io)
{
    resourceOwner_e owner = IOGetOwner(io);

    if (owner == OWNER_FREE || owner == OWNER_PREINIT) {
        return true;
    }

    return false;
}

#if DEFIO_PORT_USED_COUNT > 0
#ifdef HPMicro
static const uint32_t ioDefUsedMask[DEFIO_PORT_USED_COUNT] = { DEFIO_PORT_USED_LIST };
static const uint32_t ioDefUsedOffset[DEFIO_PORT_USED_COUNT] = { DEFIO_PORT_OFFSET_LIST };
#else
static const uint16_t ioDefUsedMask[DEFIO_PORT_USED_COUNT] = { DEFIO_PORT_USED_LIST };
static const uint8_t ioDefUsedOffset[DEFIO_PORT_USED_COUNT] = { DEFIO_PORT_OFFSET_LIST };
#endif
#else
// Avoid -Wpedantic warning
static const uint16_t ioDefUsedMask[1] = {0};
static const uint8_t ioDefUsedOffset[1] = {0};
#endif
#if DEFIO_IO_USED_COUNT
ioRec_t ioRecs[DEFIO_IO_USED_COUNT];
#else
// Avoid -Wpedantic warning
ioRec_t ioRecs[1];
#endif

// initialize all ioRec_t structures from ROM
// currently only bitmask is used, this may change in future
void IOInitGlobal(void)
{
    ioRec_t *ioRec = ioRecs;
#ifdef HPMicro
#if defined(DEFIO_PORT_A_USED_COUNT) && (DEFIO_PORT_A_USED_COUNT > 0)
    for (unsigned pin = 0; pin < 32; pin++) {
        if (DEFIO_PORT_A_USED_MASK & (1 << pin)) {
            ioRec->gpio = (GPIO_TypeDef *)(GPIOA_BASE);   // ports are 0x400 apart
            ioRec->pin = 1 << pin;
            ioRec->ioc_index = IOC_PAD_PA00 + pin;
            ioRec->pioc_index = (uint32_t)-1;
            ioRec->bioc_index = (uint32_t)-1;
            ioRec++;
        }
    }
#endif
#if defined(DEFIO_PORT_B_USED_COUNT) && (DEFIO_PORT_B_USED_COUNT > 0)
    for (unsigned pin = 0; pin < 32; pin++) {
        if (DEFIO_PORT_B_USED_MASK & (1 << pin)) {
            ioRec->gpio = (GPIO_TypeDef *)(GPIOA_BASE + 0X10);   // ports are 0x400 apart
            ioRec->pin = 1 << pin;
            ioRec->ioc_index = IOC_PAD_PB00 + pin;
            ioRec->pioc_index = (uint32_t)-1;
            ioRec->bioc_index = (uint32_t)-1;
            ioRec++;
        }
    }
#endif
#if defined(DEFIO_PORT_C_USED_COUNT) && (DEFIO_PORT_C_USED_COUNT > 0)
    for (unsigned pin = 0; pin < 32; pin++) {
        if (DEFIO_PORT_C_USED_MASK & (1 << pin)) {
            ioRec->gpio = (GPIO_TypeDef *)(GPIOA_BASE + 0x20);   // ports are 0x400 apart
            ioRec->pin = 1 << pin;
            ioRec->ioc_index = IOC_PAD_PC00 + pin;
            ioRec->pioc_index = (uint32_t)-1;
            ioRec->bioc_index = (uint32_t)-1;
            ioRec++;
        }
    }
#endif
#if defined(DEFIO_PORT_D_USED_COUNT) && (DEFIO_PORT_D_USED_COUNT > 0)
    for (unsigned pin = 0; pin < 32; pin++) {
        if (DEFIO_PORT_D_USED_MASK & (1 << pin)) {
            ioRec->gpio = (GPIO_TypeDef *)(GPIOA_BASE + 0x30);   // ports are 0x400 apart
            ioRec->pin = 1 << pin;
            ioRec->ioc_index = IOC_PAD_PD00 + pin;
            ioRec->pioc_index = (uint32_t)-1;
            ioRec->bioc_index = (uint32_t)-1;
            ioRec++;
        }
    }
#endif
#if defined(DEFIO_PORT_E_USED_COUNT) && (DEFIO_PORT_E_USED_COUNT > 0)
    for (unsigned pin = 0; pin < 32; pin++) {
        if (DEFIO_PORT_E_USED_MASK & (1 << pin)) {
            ioRec->gpio = (GPIO_TypeDef *)(GPIOA_BASE + 0x40);   // ports are 0x400 apart
            ioRec->pin = 1 << pin;
            ioRec->ioc_index = IOC_PAD_PE00 + pin;
            ioRec->pioc_index = (uint32_t)-1;
            ioRec->bioc_index = (uint32_t)-1;
            ioRec++;
        }
    }
#endif
#if defined(DEFIO_PORT_F_USED_COUNT) && (DEFIO_PORT_F_USED_COUNT > 0)
    for (unsigned pin = 0; pin < 32; pin++) {
        if (DEFIO_PORT_F_USED_MASK & (1 << pin)) {
            ioRec->gpio = (GPIO_TypeDef *)(GPIOA_BASE + 0x50);   // ports are 0x400 apart
            ioRec->pin = 1 << pin;
            ioRec->ioc_index = IOC_PAD_PF00 + pin;
            ioRec->pioc_index = (uint32_t)-1;
            ioRec->bioc_index = (uint32_t)-1;
            ioRec++;
        }
    }
#endif
#if defined(DEFIO_PORT_X_USED_COUNT) && (DEFIO_PORT_X_USED_COUNT > 0)
    for (unsigned pin = 0; pin < 32; pin++) {
        if (DEFIO_PORT_X_USED_MASK & (1 << pin)) {
            ioRec->gpio = (GPIO_TypeDef *)(GPIOA_BASE + 0xD0);   // ports are 0x400 apart
            ioRec->pin = 1 << pin;
            ioRec->ioc_index = IOC_PAD_PX00 + pin;
            ioRec->pioc_index = IOC_PAD_PX00 + pin;
            ioRec++;
        }
    }
#endif
#if defined(DEFIO_PORT_Y_USED_COUNT) && (DEFIO_PORT_Y_USED_COUNT > 0)
    for (unsigned pin = 0; pin < 32; pin++) {
        if (DEFIO_PORT_Y_USED_MASK & (1 << pin)) {
            ioRec->gpio = (GPIO_TypeDef *)(GPIOA_BASE + 0xE0);   // ports are 0x400 apart
            ioRec->pin = 1 << pin;
            ioRec->ioc_index = IOC_PAD_PY00 + pin;
            ioRec->pioc_index = IOC_PAD_PY00 + pin;
            ioRec->bioc_index = (uint32_t)-1;
            ioRec++;
        }
    }
#endif
#if defined(DEFIO_PORT_Z_USED_COUNT) && (DEFIO_PORT_Z_USED_COUNT > 0)
    for (unsigned pin = 0; pin < 32; pin++) {
        if (DEFIO_PORT_Z_USED_MASK & (1 << pin)) {
            ioRec->gpio = (GPIO_TypeDef *)(GPIOA_BASE + 0xF0);   // ports are 0x400 apart
            ioRec->pin = 1 << pin;
            ioRec->ioc_index = IOC_PAD_PZ00 + pin;
            ioRec->bioc_index = IOC_PAD_PZ00 + pin;
            ioRec->pioc_index = (uint32_t)-1;
            ioRec++;
        }
    }
#endif
#else
    for (unsigned port = 0; port < ARRAYLEN(ioDefUsedMask); port++) {
        for (unsigned pin = 0; pin < sizeof(ioDefUsedMask[0]) * 8; pin++) {
            if (ioDefUsedMask[port] & (1 << pin)) {
                ioRec->gpio = (GPIO_TypeDef *)(GPIOA_BASE + (port << 10));   // ports are 0x400 apart
                ioRec->pin = 1 << pin;
                ioRec++;
            }
        }
    }
#endif
}

IO_t IOGetByTag(ioTag_t tag)
{
    const int portIdx = DEFIO_TAG_GPIOID(tag);
    const int pinIdx = DEFIO_TAG_PIN(tag);

    if (portIdx < 0 || portIdx >= DEFIO_PORT_USED_COUNT) {
        return NULL;
    }
    // check if pin exists
    if (!(ioDefUsedMask[portIdx] & (1 << pinIdx))) {
        return NULL;
    }
    // count bits before this pin on single port
    int offset = __builtin_popcount(((1 << pinIdx) - 1) & ioDefUsedMask[portIdx]);
    // and add port offset
    offset += ioDefUsedOffset[portIdx];
    return ioRecs + offset;
}

void IOTraversePins(IOTraverseFuncPtr_t fnPtr)
{
    for (uint32_t i = 0; i < DEFIO_IO_USED_COUNT; i++) {
        fnPtr(&ioRecs[i]);
    }
}
