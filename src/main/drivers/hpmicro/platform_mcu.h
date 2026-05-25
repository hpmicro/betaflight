/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once
#ifdef HPM6750
#include "hpm6750.h"
#elif defined(HPM6360)
#include "hpm6360.h"
#endif
#include "hpm_soc.h"
#include "hpm_interrupt.h"
#include "hpm_romapi.h"
#include "board.h"

#define ADC1                             ((ADC_TypeDef *) HPM_ADC0)
#define ADC2                             ((ADC_TypeDef *) HPM_ADC1)
#define ADC3                             ((ADC_TypeDef *) HPM_ADC2)
#define TIM_ICInitTypeDef int
#ifdef HPM6750
typedef ADC12_Type ADC_TypeDef;
#else
typedef ADC16_Type ADC_TypeDef;
#endif
#define RCC_ClocksTypeDef int
typedef enum
{ 
  Bit_RESET = 0,
  Bit_SET
}BitAction;