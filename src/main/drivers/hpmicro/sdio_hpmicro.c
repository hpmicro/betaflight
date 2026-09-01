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
 * Original author: Alain (https://github.com/aroyer-qc)
 * Modified for F4 and BF source: Chris Hockuba (https://github.com/conkerkh)
 *
 * Note: On F4 due to DMA issues it is recommended that motor timers don't run on DMA2.
 *         Therefore avoid using TIM1/TIM8, use TIM2/TIM3/TIM4/TIM5/TIM6/TIM7
 */

/* Include(s) -------------------------------------------------------------------------------------------------------*/

#include <stdbool.h>
#include <string.h>

#include "platform.h"

#ifdef USE_SDCARD_SDIO

#include "drivers/sdmmc_sdio.h"
#include "hpm_sdmmc_sd.h"
#include "hpm_sdmmc_host.h"
#include "hpm_l1c_drv.h"

ATTR_PLACE_AT_NONCACHEABLE_BSS sdmmc_host_t g_sdmmc_host;
sd_card_t g_sd = { .host = &g_sdmmc_host };
#ifdef INIT_EXT_RAM_FOR_DATA
#define MAX_BUF_SIZE_DEFAULT (512U * 1024U)
#else
#define MAX_BUF_SIZE_DEFAULT (32U * 1024U)
#endif

ATTR_PLACE_AT_NONCACHEABLE uint32_t s_write_buf[MAX_BUF_SIZE_DEFAULT / sizeof(uint32_t)];
ATTR_PLACE_AT_NONCACHEABLE uint32_t s_read_buf[MAX_BUF_SIZE_DEFAULT / sizeof(uint32_t)];
typedef struct SD_Handle_s
{
    uint32_t          CSD[4];           // SD card specific data table
    uint32_t          CID[4];           // SD card identification number table
    volatile uint32_t RXCplt;          // SD RX Complete is equal 0 when no transfer
    volatile uint32_t TXCplt;          // SD TX Complete is equal 0 when no transfer

    uint32_t RXErrors;
    uint32_t TXErrors;
} SD_Handle_t;

typedef struct _SD_nonblock_transfer_t {
    uint32_t *buffer;
    uint32_t block_count;
    uint32_t start_block;

} SD_nonblock_transfer_t;
SD_nonblock_transfer_t nonblock_param;
SD_CardInfo_t                      SD_CardInfo;
SD_CardType_t                      SD_CardType;

SD_Handle_t                        SD_Handle;

#if defined(HPM_SDMMC_HOST_ENABLE_IRQ) && (HPM_SDMMC_HOST_ENABLE_IRQ == 1)
SDK_DECLARE_EXT_ISR_M(BOARD_APP_SDCARD_SDXC_IRQ, sdxc_isr)
void sdxc_isr(void)
{
    sdmmchost_irq_handler(&g_sdmmc_host);
}
#endif
void set_csd(uint32_t *buff)
{
    //memcpy(&SD_Handle.CSD[0], buff, sizeof(SD_Handle.CSD));
    SD_Handle.CSD[0] = buff[3];
    SD_Handle.CSD[1] = buff[2];
    SD_Handle.CSD[2] = buff[1];
    SD_Handle.CSD[3] = buff[0];

}
/* Define(s) --------------------------------------------------------------------------------------------------------*/

typedef struct {
    uint32_t *buffer;
    uint32_t BlockSize;
    uint32_t NumberOfBlocks;
} sdReadParameters_t;

sdReadParameters_t sdReadParameters;
static void HAL_SD_Transfer_Callback(void *param);

SD_Error_t SD_CheckWrite(void)
{
    if (SD_Handle.TXCplt != 0) return SD_BUSY;
    return SD_OK;
}

SD_Error_t SD_CheckRead(void)
{
    if (SD_Handle.RXCplt != 0) return SD_BUSY;
    return SD_OK;
}

SD_Error_t SD_ReadBlocks_DMA(uint64_t ReadAddress, uint32_t *buffer, uint32_t BlockSize, uint32_t NumberOfBlocks)
{
    SD_Error_t ErrorState = SD_OK;

    if (BlockSize != 512) {
        return SD_ERROR; // unsupported.
    }

    if ((uint32_t)buffer & 0x0f) {
        return SD_ADDR_MISALIGNED;
    }

    SD_Handle.RXCplt = 1;

    sdReadParameters.buffer = buffer;
    sdReadParameters.BlockSize = BlockSize;
    sdReadParameters.NumberOfBlocks = NumberOfBlocks;

    nonblock_param.block_count = NumberOfBlocks;
    nonblock_param.start_block = ReadAddress;
    nonblock_param.buffer = buffer;
    hpm_stat_t status;
    if ((status = sd_start_read_blocks(&g_sd, (uint8_t *)buffer, ReadAddress, NumberOfBlocks, HAL_SD_Transfer_Callback, &nonblock_param)) != status_success) {
        return SD_ERROR;
    }

    return ErrorState;
}

SD_Error_t SD_WriteBlocks_DMA(uint64_t WriteAddress, uint32_t *buffer, uint32_t BlockSize, uint32_t NumberOfBlocks)
{
    SD_Error_t ErrorState = SD_OK;
    SD_Handle.TXCplt = 1;

    if (BlockSize != 512) {
        return SD_ERROR; // unsupported.
    }

    if ((uint32_t)buffer & 0x0f) {
        return SD_ADDR_MISALIGNED;
    }

    nonblock_param.block_count = NumberOfBlocks;
    nonblock_param.start_block = WriteAddress;
    nonblock_param.buffer = buffer;

    hpm_stat_t status;
    if ((status = sd_start_write_blocks(&g_sd, (uint8_t *)buffer, WriteAddress, NumberOfBlocks, HAL_SD_Transfer_Callback, &nonblock_param)) != status_success) {
        return SD_ERROR;
    }

    return ErrorState;
}

/**
  * @brief Tx Transfer completed callback
  * @param hsd: SD handle
  * @retval None
  */
void HAL_SD_TxCpltCallback(void *param)
{
    UNUSED(param);

    SD_Handle.TXCplt = 0;
}

/**
  * @brief Rx Transfer completed callback
  * @param hsd: SD handle
  * @retval None
  */
void HAL_SD_RxCpltCallback(void *param)
{
    SD_nonblock_transfer_t *context = (SD_nonblock_transfer_t *)param;
    SD_Handle.RXCplt = 0;
    /*
       the SCB_InvalidateDCache_by_Addr() requires a 32-Byte aligned address,
       adjust the address and the D-Cache size to invalidate accordingly.
     */
    uint32_t aligned_start = HPM_L1C_CACHELINE_ALIGN_DOWN((uint32_t) context->buffer);
    uint32_t end_addr = (uint32_t) context->buffer + g_sd.block_size * context->block_count;
    uint32_t aligned_end = HPM_L1C_CACHELINE_ALIGN_UP(end_addr);
    uint32_t aligned_size = aligned_end - aligned_start;
    l1c_dc_invalidate(aligned_start, aligned_size);
}

void HAL_SD_ErrorCallback(sd_card_t *card)
{
    UNUSED(card);
    if (SD_Handle.RXCplt) {
        SD_Handle.RXErrors++;
        SD_Handle.RXCplt = 0;
    }

    if (SD_Handle.TXCplt) {
        SD_Handle.TXErrors++;
        SD_Handle.TXCplt = 0;
    }
}

bool SD_GetState(void)
{
    hpm_stat_t status;
    status = sd_polling_card_status_busy(&g_sd, 0);
    if (status != status_success)
        return false;
    return true;
}

void HAL_SD_Transfer_Callback(void *param)
{
    if (SD_Handle.RXCplt)
        HAL_SD_RxCpltCallback(param);
    if (SD_Handle.TXCplt)
        HAL_SD_TxCpltCallback(param);
}

void HAL_SD_AbortCallback(sd_card_t *card)
{
    UNUSED(card);

    SD_Handle.TXCplt = 0;
    SD_Handle.RXCplt = 0;
}
/** -----------------------------------------------------------------------------------------------------------------*/
/**
  * @brief  Gets the SD card status.
  * @retval SD Card error state
  */
SD_Error_t SD_GetCardInfo(void)
{
    /* The SDK keeps the CID/CSD in decoded form (sd_cid_t bitfields /
     * sd_csd_t fields), so map those directly instead of parsing raw
     * register words the way the STM32 implementation does. */
    SD_CardInfo.SD_cid.ManufacturerID = g_sd.cid.mid;
    SD_CardInfo.SD_cid.OEM_AppliID = g_sd.cid.oid;
    SD_CardInfo.SD_cid.ProdName1 = (uint32_t) (g_sd.cid.pnm >> 8);
    SD_CardInfo.SD_cid.ProdName2 = (uint8_t) (g_sd.cid.pnm & 0xFF);
    SD_CardInfo.SD_cid.ProdRev = g_sd.cid.prv;
    SD_CardInfo.SD_cid.ProdSN = g_sd.cid.psn;
    SD_CardInfo.SD_cid.Reserved1 = 0;
    SD_CardInfo.SD_cid.ManufactDate = g_sd.cid.mdt;
    SD_CardInfo.SD_cid.CID_CRC = g_sd.cid.crc7;
    SD_CardInfo.SD_cid.Reserved2 = 1;

    SD_CardInfo.SD_csd.CSDStruct = g_sd.csd.csd_structure;
    SD_CardInfo.SD_csd.TAAC = g_sd.csd.data_read_access_time1;
    SD_CardInfo.SD_csd.NSAC = g_sd.csd.data_read_access_time2;
    SD_CardInfo.SD_csd.MaxBusClkFrec = g_sd.csd.transfer_speed;
    SD_CardInfo.SD_csd.CardComdClasses = g_sd.csd.card_command_class;
    SD_CardInfo.SD_csd.RdBlockLen = (uint8_t) g_sd.csd.read_block_len;
    SD_CardInfo.SD_csd.PartBlockRead = g_sd.csd.support_read_block_partial;
    SD_CardInfo.SD_csd.WrBlockMisalign = g_sd.csd.support_write_block_misalignment;
    SD_CardInfo.SD_csd.RdBlockMisalign = g_sd.csd.support_read_block_misalignment;
    SD_CardInfo.SD_csd.DSRImpl = g_sd.csd.is_dsr_implemented;
    SD_CardInfo.SD_csd.DeviceSize = g_sd.csd.device_size;
    SD_CardInfo.SD_csd.MaxRdCurrentVDDMin = g_sd.csd.read_current_vdd_min;
    SD_CardInfo.SD_csd.MaxRdCurrentVDDMax = g_sd.csd.read_current_vdd_max;
    SD_CardInfo.SD_csd.MaxWrCurrentVDDMin = g_sd.csd.write_current_vdd_min;
    SD_CardInfo.SD_csd.MaxWrCurrentVDDMax = g_sd.csd.write_current_vdd_max;
    SD_CardInfo.SD_csd.DeviceSizeMul = g_sd.csd.device_size_multiplier;
    SD_CardInfo.SD_csd.WrSpeedFact = g_sd.csd.write_speed_factor;
    SD_CardInfo.SD_csd.MaxWrBlockLen = (uint8_t) g_sd.csd.max_write_block_len;
    SD_CardInfo.SD_csd.WriteBlockPaPartial = g_sd.csd.support_write_block_partial;
    SD_CardInfo.SD_csd.WrProtectGrEnable = g_sd.csd.is_write_protection_group_enabled;
    SD_CardInfo.SD_csd.FileFormatGrouop = g_sd.csd.support_file_format_group;
    SD_CardInfo.SD_csd.CopyFlag = g_sd.csd.support_copy;
    SD_CardInfo.SD_csd.PermWrProtect = g_sd.csd.support_permanent_write_protect;
    SD_CardInfo.SD_csd.TempWrProtect = g_sd.csd.support_temporary_write_protect;
    SD_CardInfo.SD_csd.FileFormat = g_sd.csd.file_format;
    /* No decoded counterparts: SysSpecVersion, EraseGrSize/EraseGrMul,
     * WrProtectGrSize, ManDeflECC, ECC, CSD_CRC and the Reserved fields stay
     * zero. Common code only reads SD_cid, CardCapacity and CardBlockSize. */

    SD_CardInfo.CardCapacity = g_sd.block_count;
    SD_CardInfo.CardBlockSize = g_sd.block_size;
    return SD_OK;
}

/** -----------------------------------------------------------------------------------------------------------------*/
/**
  * @brief  Checks if the SD card is in programming state.
  * @param  pStatus: pointer to the variable that will contain the SD card state
  * @retval SD Card error state
  */
/*
static SD_Error_t SD_IsCardProgramming(uint8_t *pStatus)
{
    uint32_t Response_R1;

    SD_TransmitCommand((SD_CMD_SEND_STATUS | SDIO_CMD_RESPONSE_SHORT), SD_CardRCA, 0);
    if((SDIO->STA & SDIO_STA_CTIMEOUT) != 0)         return SD_CMD_RSP_TIMEOUT;
    else if((SDIO->STA & SDIO_STA_CCRCFAIL) != 0)    return SD_CMD_CRC_FAIL;
    if((uint32_t)SDIO->RESPCMD != SD_CMD_SEND_STATUS) return SD_ILLEGAL_CMD;  // Check if is of desired command
    Response_R1 = SDIO->RESP1;                                                // We have received response, retrieve it for analysis
    *pStatus = (uint8_t)((Response_R1 >> 9) & 0x0000000F);                      // Find out card status

    return CheckOCR_Response(Response_R1);
}
*/

/** -----------------------------------------------------------------------------------------------------------------*/
/**
  * @brief  Initialize the SDIO module, DMA, and IO
  */
bool SD_Initialize_LL(DMA_Stream_TypeDef *dma)
{
    UNUSED(dma);
    return true;
}

SD_Error_t SD_Init(void)
{
    static bool sdInitAttempted = false;
    static SD_Error_t result = SD_ERROR;
    hpm_stat_t status;

    if (sdInitAttempted) {
        return result;
    }

    sdInitAttempted = true;
    intc_m_enable_irq_with_priority(BOARD_APP_SDCARD_SDXC_IRQ, 1);
    status = board_init_sd_host_params(&g_sdmmc_host, BOARD_APP_SDCARD_SDXC_BASE);
    if (status != status_success) {
        return result;
    }
    status = sd_init(&g_sd);
    if (status == status_success) {
        result = SD_OK;
        // SD Card 2.0
        switch (g_sd.status.card_type) {
            case 0:
                SD_CardType = SD_STD_CAPACITY_V1_1;
                if (g_sd.capacity_v2_0_or_high)
                    SD_CardType = SD_STD_CAPACITY_V2_0;
            break;
            case 1:
                SD_CardType = SD_HIGH_CAPACITY;
            break;
            default:
                return SD_ERROR;
            break;
        }
        memcpy(&SD_Handle.CID[0], &g_sd.cid, sizeof(g_sd.cid));
    }
    // Read CSD/CID MSD registers.
    result = SD_GetCardInfo();
    if (result != SD_OK) {
        return result;
    }

    return result;
}
/* ------------------------------------------------------------------------------------------------------------------*/
#endif
