/*
 * Copyright (c) 2025 HPMicro
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */

#include "platform.h"
#include "drivers/io.h"
#include "drivers/io_impl.h"
#include "hpm_ioc_regs.h"
#include "hpm_gpio_drv.h"

bool IORead(IO_t io)
{
    if (!io) {
        return false;
    }
    uint32_t ioc_idx = IO_IOC_INDEX(io);
    if (gpio_read_pin(HPM_GPIO0, GPIO_GET_PORT_INDEX(ioc_idx), GPIO_GET_PIN_INDEX(ioc_idx)))
        return true;
    else
        return false;
}

void IOWrite(IO_t io, bool hi)
{
    uint32_t ioc_idx = IO_IOC_INDEX(io);
    gpio_write_pin(HPM_GPIO0, GPIO_GET_PORT_INDEX(ioc_idx), GPIO_GET_PIN_INDEX(ioc_idx), hi == true ? 1 : 0);
}

void IOHi(IO_t io)
{
    uint32_t ioc_idx = IO_IOC_INDEX(io);
    gpio_write_pin(HPM_GPIO0, GPIO_GET_PORT_INDEX(ioc_idx), GPIO_GET_PIN_INDEX(ioc_idx), 1);
}

void IOLo(IO_t io)
{
    uint32_t ioc_idx = IO_IOC_INDEX(io);
    gpio_write_pin(HPM_GPIO0, GPIO_GET_PORT_INDEX(ioc_idx), GPIO_GET_PIN_INDEX(ioc_idx), 0);
}

void IOToggle(IO_t io)
{
    uint32_t ioc_idx = IO_IOC_INDEX(io);
    gpio_toggle_pin(HPM_GPIO0, GPIO_GET_PORT_INDEX(ioc_idx), GPIO_GET_PIN_INDEX(ioc_idx));
}

void IOConfigGPIO(IO_t io, ioConfig_t cfg)
{
    if (!io) {
        return;
    }
    uint32_t ioc_idx = IO_IOC_INDEX(io);
    if (IO_PIOC_INDEX(io) != (uint32_t)-1) {
        HPM_PIOC->PAD[IO_PIOC_INDEX(io)].FUNC_CTL = 3;
    }
    if (IO_BIOC_INDEX(io) != (uint32_t)-1) {
        HPM_BIOC->PAD[IO_BIOC_INDEX(io)].FUNC_CTL = 3;
    }
    HPM_IOC->PAD[ioc_idx].FUNC_CTL = 0;
    bool input = false;
    uint32_t pad_ctl = 0;
    switch(cfg) {
        case IOCFG_OUT_PP:
            pad_ctl = 0;
        break;
        case IOCFG_OUT_PP_UP:
            pad_ctl = IOC_PAD_PAD_CTL_PE_SET(1) | IOC_PAD_PAD_CTL_PS_SET(1);
        break;
        case IOCFG_OUT_OD:
            pad_ctl = IOC_PAD_PAD_CTL_OD_SET(1);
        break;
        case IOCFG_IPD:
            input = true;
            pad_ctl = IOC_PAD_PAD_CTL_PE_SET(1) | IOC_PAD_PAD_CTL_PS_SET(0);
        break;
        case IOCFG_IPU:
            input = true;
            pad_ctl = IOC_PAD_PAD_CTL_PE_SET(1) | IOC_PAD_PAD_CTL_PS_SET(1);
        break;
        case IOCFG_IN_FLOATING:
            input = true;
        break;
        default:
        break;
      
    }
    if (input) {
        gpio_set_pin_input(HPM_GPIO0, GPIO_GET_PORT_INDEX(ioc_idx), GPIO_GET_PIN_INDEX(ioc_idx));
    } else {
        gpio_set_pin_output(HPM_GPIO0, GPIO_GET_PORT_INDEX(ioc_idx), GPIO_GET_PIN_INDEX(ioc_idx));
    }
    HPM_IOC->PAD[IO_IOC_INDEX(io)].PAD_CTL = pad_ctl;

}

FAST_CODE void IOConfigGPIOAF(IO_t io, ioConfig_t cfg, uint32_t af, uint32_t bpio_func)
{
    if (!io) {
        return;
    }
    uint32_t ioc_idx = IO_IOC_INDEX(io);
    if (IO_PIOC_INDEX(io) != (uint32_t)-1) {
        HPM_PIOC->PAD[(uint32_t)ioc_idx].FUNC_CTL = bpio_func;
    }
    if (IO_BIOC_INDEX(io) != (uint32_t)-1) {
        HPM_BIOC->PAD[(uint32_t)ioc_idx].FUNC_CTL = bpio_func;
    }
    HPM_IOC->PAD[(uint32_t)ioc_idx].FUNC_CTL = af;
    bool input = false;
    uint32_t pad_ctl = 0;
    switch(cfg) {
        case IOCFG_AF_OD_UP:
            pad_ctl = IOC_PAD_PAD_CTL_PE_SET(1) | IOC_PAD_PAD_CTL_PS_SET(1) | IOC_PAD_PAD_CTL_OD_MASK;
            break;
        case IOCFG_AF_OD:
            pad_ctl |= IOC_PAD_PAD_CTL_OD_MASK;
            break;
        case IOCFG_OUT_PP_UP:
            pad_ctl = IOC_PAD_PAD_CTL_PE_SET(1) | IOC_PAD_PAD_CTL_PS_SET(1);
        break;
        case IOCFG_OUT_OD:
            pad_ctl = IOC_PAD_PAD_CTL_OD_SET(1);
        break;
        case IOCFG_IPD:
            input = true;
            pad_ctl = IOC_PAD_PAD_CTL_PE_SET(1) | IOC_PAD_PAD_CTL_PS_SET(0);
        break;
        case IOCFG_IPU:
            input = true;
            pad_ctl = IOC_PAD_PAD_CTL_PE_SET(1) | IOC_PAD_PAD_CTL_PS_SET(1);
        break;
        default:
        break;
      
    }
    if (input) {
        gpio_set_pin_input(HPM_GPIO0, (uint32_t)IO_GPIO(io), (uint8_t)IO_Pin(io));
    }
    HPM_IOC->PAD[(uint32_t)ioc_idx].PAD_CTL = pad_ctl;
}
