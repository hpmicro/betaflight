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

/*
 * Author: Chris Hockuba (https://github.com/conkerkh)
 *
 */

#include <stdint.h>
#include <stdbool.h>
#include "board.h"
#include "platform.h"

#if defined(USE_USB_MSC)

#include "blackbox/blackbox.h"
#include "drivers/usb_msc.h"
#include "pg/sdcard.h"
#include "usb_config.h"

extern void msc_device_init(uint8_t busid, uint32_t reg_base);
uint8_t mscStart(void)
{
    // Start USB
    board_init_usb((USB_Type *)CONFIG_HPM_USBD_BASE);

    switch (blackboxConfig()->device)
    {
#ifdef USE_SDCARD
    case BLACKBOX_DEVICE_SDCARD:
        switch (sdcardConfig()->mode)
        {
#ifdef USE_SDCARD_SDIO
        case SDCARD_MODE_SDIO:
            break;
#endif
        default:
            return 1;
        }
        break;
#endif

    default:
        return 1;
    }
    intc_m_enable_irq_with_priority(BOARD_APP_SDCARD_SDXC_IRQ, 1);
    msc_device_init(0, CONFIG_HPM_USBD_BASE);

    return 0;
}

#endif
