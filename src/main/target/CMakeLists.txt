# Copyright (c) 2024-2025 HPMicro
# SPDX-License-Identifier: BSD-3-Clause

sdk_src(
    config_helper.c 
)

if("${BOARD}" STREQUAL "hpmpilotbf67")
    message(STATUS "Selected board: hpmpilotbf67, including HPM6750 target files")
    sdk_inc(HPM6750)
    sdk_src(
        HPM6750/target.c
        HPM6750/timer_hpm6750.c
    )
endif()
if("${BOARD}" STREQUAL "hpmpilotbf63")
    message(STATUS "Selected board: hpmpilotbf63, including HPM6360 target files")
    sdk_inc(HPM6360)
    sdk_src(
        HPM6360/target.c
        HPM6360/timer_hpm6360.c
    )
endif()