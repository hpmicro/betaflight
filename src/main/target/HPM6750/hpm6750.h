/*
 * Copyright (c) 2025 HPMicro
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */
#ifndef _HPM6750_H_
#define _HPM6750_H_

#define SPI1                ((SPI_TypeDef *) HPM_SPI0)
#define SPI2                ((SPI_TypeDef *) HPM_SPI1)
#ifdef HPM6750
#define SPI3                ((SPI_TypeDef *) HPM_SPI2)
#else
#define SPI3                ((SPI_TypeDef *) HPM_SPI7)
#endif
#define SPI4                ((SPI_TypeDef *) HPM_SPI3)
#endif