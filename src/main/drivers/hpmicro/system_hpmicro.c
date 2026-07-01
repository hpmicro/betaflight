/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include "platform.h"
#include "build/debug.h"
#include "drivers/system.h"
#include "hpm_ppor_drv.h"
#include "hpm_dfu_trigger.h"
extern void cycleCounterInit(void);

void c_startup(void)
{
    uint32_t i, size;
    extern uint8_t __bss_start__[], __bss_end__[];
    extern uint8_t __tdata_start__[], __tdata_end__[];
    extern uint8_t __data_start__[], __data_end__[];
    extern uint8_t __ramfunc_start__[], __ramfunc_end__[];
    extern uint8_t __noncacheable_bss_start__[], __noncacheable_bss_end__[];
    extern uint8_t __noncacheable_init_start__[], __noncacheable_init_end__[];
    extern uint8_t __data_load_addr__[], __tdata_load_addr__[];
    extern uint8_t __fast_load_addr__[], __noncacheable_init_load_addr__[];
    extern uint8_t __fast_ram_bss_start__[], __fast_ram_bss_end__[];
    extern uint8_t __fast_ram_init_start__[], __fast_ram_init_end__[], __fast_ram_init_load_addr__[];
    extern uint8_t _estack[];
    extern uint8_t _Min_Stack_Size[];
   
    uint8_t *stackHighMem = (uint8_t *)&_estack;
    const uint32_t stackSize = (uint32_t)&_Min_Stack_Size;
    uint8_t * const stackLowMem = stackHighMem - stackSize; 
    uint8_t *p;
    const uint8_t * const stackCurrent = (uint8_t *)&stackLowMem;

    for (p = stackLowMem; p < stackCurrent; ++p) {
        *p = 0xa5;
    }

#if defined(FLASH_XIP) || defined(FLASH_UF2)
    extern uint8_t __vector_ram_start__[], __vector_ram_end__[], __vector_load_addr__[];
    size = __vector_ram_end__ - __vector_ram_start__;
    for (i = 0; i < size; i++) {
        *(__vector_ram_start__ + i) = *(__vector_load_addr__ + i);
    }
#endif

    /* bss section */
    size = __bss_end__ - __bss_start__;
    for (i = 0; i < size; i++) {
        *(__bss_start__ + i) = 0;
    }

    /* noncacheable bss section */
    size = __noncacheable_bss_end__ - __noncacheable_bss_start__;
    for (i = 0; i < size; i++) {
        *(__noncacheable_bss_start__ + i) = 0;
    }

    /* fast_ram bss section */
    size = __fast_ram_bss_end__ - __fast_ram_bss_start__;
    for (i = 0; i < size; i++) {
        *(__fast_ram_bss_start__ + i) = 0;
    }

    /* data section LMA: etext */
    size = __data_end__ - __data_start__;
    for (i = 0; i < size; i++) {
        *(__data_start__ + i) = *(__data_load_addr__ + i);
    }

    /* ramfunc section LMA: etext + data length */
    size = __ramfunc_end__ - __ramfunc_start__;
    for (i = 0; i < size; i++) {
        *(__ramfunc_start__ + i) = *(__fast_load_addr__ + i);
    }

    /* tdata section LMA: etext + data length + ramfunc length */
    size = __tdata_end__ - __tdata_start__;
    for (i = 0; i < size; i++) {
        *(__tdata_start__ + i) = *(__tdata_load_addr__ + i);
    }

    /* noncacheable init section LMA: etext + data length + ramfunc legnth + tdata length*/
    size = __noncacheable_init_end__ - __noncacheable_init_start__;
    for (i = 0; i < size; i++) {
        *(__noncacheable_init_start__ + i) = *(__noncacheable_init_load_addr__ + i);
    }

    /* fast_ram init section LMA: etext + data length + ramfunc legnth + tdata length*/
    size = __fast_ram_init_end__ - __fast_ram_init_start__;
    for (i = 0; i < size; i++) {
        *(__fast_ram_init_start__ + i) = *(__fast_ram_init_load_addr__ + i);
    }
}

void systemResetToBootloader(bootloaderRequestType_e requestType)
{
    (void)requestType;
    hpm_dfu_reboot_to_dfu();
}

bool isMPUSoftReset(void)
{
    return false;

}

void systemInit(void)
{
    SystemCoreClock = BOARD_CPU_FREQ;
    cycleCounterInit();
}

void systemReset(void)
{
    ppor_sw_reset(HPM_PPOR, 10);
}

void debugInit(void)
{

}