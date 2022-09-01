/*******************************************************************************
 * Copyright (c) 2015-2020 Barefoot Networks, Inc.
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * $Id: $
 *
 ******************************************************************************/
/*!
 * @file bf_pltfm_eeprom.h
 * @date
 *
 * EEPROM parsing
 *
 */

/* Standard includes */
#include <stdint.h>

/* Module includes */
#include <bf_pltfm_types/bf_pltfm_types.h>

/* Local header includes */

#define URL_SIZE 100
#define EEPROM_RAW_DATA_SIZE 4096

char eeprom_raw_data[EEPROM_RAW_DATA_SIZE];

extern bf_pltfm_eeprom_t eeprom;

bf_pltfm_status_t bf_pltfm_bd_type_init();

bf_pltfm_status_t bf_pltfm_bd_type_get(bf_pltfm_board_id_t *board_id);
