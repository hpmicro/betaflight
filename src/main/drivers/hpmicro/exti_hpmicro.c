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

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#ifdef USE_EXTI
#include "drivers/io_impl.h"
#include "drivers/exti.h"
#include "hpm_gpio_drv.h"
#include "hpm_interrupt.h"
typedef enum {
    HPM_GPIOA = 0,
    HPM_GPIOB,
    HPM_GPIOC,
    HPM_GPIOD,
    HPM_GPIOE,
    HPM_GPIOF,
    HPM_GPIO_RSV0,
    HPM_GPIO_RSV1,
    HPM_GPIO_RSV2,
    HPM_GPIO_RSV3,
    HPM_GPIO_RSV4,
    HPM_GPIO_RSV5,
    HPM_GPIO_RSV6,
    HPM_GPIOX = 13,
    HPM_GPIOY,
    HPM_GPIOZ,
}gpioPortIdx_t;
typedef struct {
    extiCallbackRec_t* handler[32];
    uint8_t priority;
} extiChannelRec_t;

#define EXTI_IRQ_GROUPS 16
extiChannelRec_t extiChannelRecs[EXTI_IRQ_GROUPS];
static uint8_t extiGroupPriority[EXTI_IRQ_GROUPS];

static const uint8_t extiGroupIRQn[EXTI_IRQ_GROUPS] = {
    IRQn_GPIO0_A,  //0
    IRQn_GPIO0_B,  //1
    IRQn_GPIO0_C,  //2
    IRQn_GPIO0_D,  //3
    IRQn_GPIO0_E,  //4
    IRQn_GPIO0_F,  //5
    255,  //6
    255,  //7
    255,  //8
    255,  //9
    255,  //10
    255,  //11
    255,  //12
    IRQn_GPIO0_X, //13
    IRQn_GPIO0_Y, //14
    IRQn_GPIO0_Z, //15
};

static gpio_interrupt_trigger_t triggerLookupTable[] = {
    [BETAFLIGHT_EXTI_TRIGGER_RISING]    = gpio_interrupt_trigger_edge_rising,
    [BETAFLIGHT_EXTI_TRIGGER_FALLING]   = gpio_interrupt_trigger_edge_falling
};

void EXTIInit(void)
{
    memset(extiChannelRecs, 0, sizeof(extiChannelRecs));
    memset(extiGroupPriority, 1, sizeof(extiGroupPriority));
}

void EXTIHandlerInit(extiCallbackRec_t *self, extiHandlerCallback *fn)
{
    self->fn = fn;
}

void EXTIConfig(IO_t io, extiCallbackRec_t *cb, int irqPriority, ioConfig_t config, extiTrigger_t trigger)
{
    int chIdx = IO_GPIOPinIdx(io);
    int portIdx = IO_GPIOPortIdx(io);
    if (portIdx < 0) {
        return;
    }
    if ((chIdx < 0) || (chIdx > 31)) {
        return;
    }
    if (trigger == BETAFLIGHT_EXTI_TRIGGER_BOTH)
        assert(0);
    extiChannelRec_t *rec = &extiChannelRecs[portIdx];
    rec->handler[chIdx] = cb;
    
    gpio_interrupt_trigger_t trigger1 = triggerLookupTable[trigger];

    EXTIDisable(io);

    IOConfigGPIO(io, config);
    
    gpio_config_pin_interrupt(HPM_GPIO0, portIdx, chIdx, trigger1);

    if (extiGroupPriority[portIdx] < irqPriority) {
        extiGroupPriority[portIdx] = irqPriority;
    }
    EXTIEnable(io);
    intc_m_enable_irq_with_priority(extiGroupIRQn[portIdx], extiGroupPriority[portIdx]);
}

void EXTIRelease(IO_t io)
{
    EXTIDisable(io);

    const int chIdx = IO_GPIOPinIdx(io);
    int portIdx = IO_GPIOPortIdx(io);

    if (chIdx < 0) {
        return;
    }

    extiChannelRec_t *rec = &extiChannelRecs[portIdx];
    rec->handler[chIdx] = NULL;
}

void EXTIEnable(IO_t io)
{
    int chIdx = IO_GPIOPinIdx(io);
    int portIdx = IO_GPIOPortIdx(io);
    
    gpio_enable_pin_interrupt(HPM_GPIO0, portIdx, chIdx);
}

void EXTIDisable(IO_t io)
{
    int chIdx = IO_GPIOPinIdx(io);
    int portIdx = IO_GPIOPortIdx(io);
    
    gpio_disable_pin_interrupt(HPM_GPIO0, portIdx, chIdx);

}

FAST_CODE static void gpio_interrupt_handler(GPIO_Type *base, unsigned char port_index, unsigned char group)
{
    uint32_t flag = gpio_get_port_interrupt_flags(base, port_index);
    for (int i = 0; i < 32; i++) {
        if (flag & (1 << i)) {
            gpio_clear_pin_interrupt_flag(base, port_index, i);
            if (extiChannelRecs[group].handler[i]->fn != NULL) {
                extiChannelRecs[group].handler[i]->fn(extiChannelRecs[group].handler[i]);
            }
        }
    }
}

#ifdef IRQn_GPIO0_A
void gpio_porta_isr(void)
{
    gpio_interrupt_handler(HPM_GPIO0, GPIO_DI_GPIOA, HPM_GPIOA);
}
SDK_DECLARE_EXT_ISR_M(IRQn_GPIO0_A, gpio_porta_isr)
#endif

#ifdef IRQn_GPIO0_B
void gpio_portb_isr(void)
{
    gpio_interrupt_handler(HPM_GPIO0, GPIO_DI_GPIOB, HPM_GPIOB);
}
SDK_DECLARE_EXT_ISR_M(IRQn_GPIO0_B , gpio_portb_isr)
#endif

#ifdef IRQn_GPIO0_C
void gpio_portc_isr(void)
{
    gpio_interrupt_handler(HPM_GPIO0, GPIO_DI_GPIOC, HPM_GPIOC);
}
SDK_DECLARE_EXT_ISR_M(IRQn_GPIO0_C , gpio_portc_isr)
#endif

#ifdef IRQn_GPIO0_D
void gpio_portd_isr(void)
{
    gpio_interrupt_handler(HPM_GPIO0, GPIO_DI_GPIOD, HPM_GPIOD);
}
SDK_DECLARE_EXT_ISR_M(IRQn_GPIO0_D , gpio_portd_isr)
#endif

#ifdef IRQn_GPIO0_E
void gpio_porte_isr(void)
{
    gpio_interrupt_handler(HPM_GPIO0, GPIO_DI_GPIOE, HPM_GPIOE);
}
SDK_DECLARE_EXT_ISR_M(IRQn_GPIO0_E , gpio_porte_isr)
#endif

#ifdef IRQn_GPIO0_F
void gpio_portf_isr(void)
{
    gpio_interrupt_handler(HPM_GPIO0, GPIO_DI_GPIOF, HPM_GPIOF);
}
SDK_DECLARE_EXT_ISR_M(IRQn_GPIO0_F , gpio_portf_isr)
#endif

#ifdef IRQn_GPIO0_X
void gpio_portx_isr(void)
{
    gpio_interrupt_handler(HPM_GPIO0, GPIO_DI_GPIOX, HPM_GPIOX);
}
SDK_DECLARE_EXT_ISR_M(IRQn_GPIO0_X, gpio_portx_isr)
#endif

#ifdef IRQn_GPIO0_Y
void gpio_porty_isr(void)
{
    gpio_interrupt_handler(HPM_GPIO0, GPIO_DI_GPIOY, HPM_GPIOY);
}
SDK_DECLARE_EXT_ISR_M(IRQn_GPIO0_Y, gpio_porty_isr)
#endif

#ifdef IRQn_GPIO0_Z
void gpio_portz_isr(void)
{
    gpio_interrupt_handler(HPM_GPIO0, GPIO_DI_GPIOZ, HPM_GPIOZ);
}
SDK_DECLARE_EXT_ISR_M(IRQn_GPIO0_Z, gpio_portz_isr)
#endif

#endif
