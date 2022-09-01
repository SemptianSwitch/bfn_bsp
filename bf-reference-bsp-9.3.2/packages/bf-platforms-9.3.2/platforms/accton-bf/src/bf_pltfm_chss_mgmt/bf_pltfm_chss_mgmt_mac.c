/*******************************************************************************
 * Copyright (c) 2015-2020 Barefoot Networks, Inc.
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * $Id: $
 *
 ******************************************************************************/
/*!
 * @file bf_pltfm_chss_mgmt_mac.c
 * @date
 *
 * Returns MAC address of a port
 *
 */

/* Standard includes */
#include <stdio.h>

/* Module includes */
#include <bf_pltfm_types/bf_pltfm_types.h>
#include <bf_pltfm_chss_mgmt_intf.h>

/* Local header includes */
#include "bf_pltfm_bd_eeprom.h"

bf_pltfm_status_t bf_pltfm_chss_mgmt_sys_mac_addr_get(
    uint8_t *mac_info, uint8_t *num_sys_addresses) {
  uint8_t i;

  if (mac_info == NULL) {
    LOG_ERROR("Invalid argument\n");
    return BF_PLTFM_INVALID_ARG;
  }

  *num_sys_addresses = BF_PLTFM_SYS_MAC_ADDR_MAX;

  for (i = 0; i < 6; i++) {
    mac_info[i] = eeprom.bf_pltfm_local_mac[i];
  }

  return BF_PLTFM_SUCCESS;
}

bf_pltfm_status_t bf_pltfm_chss_mgmt_port_mac_addr_get(
    bf_pltfm_port_info_t *port_info, uint8_t *mac_info) {
  uint8_t i;
  uint16_t index, t;
  bf_pltfm_board_id_t board_id;
  uint32_t max_chnl_id = 4;  // default

  if (port_info == NULL) {
    LOG_ERROR("Invalid argument\n");
    return BF_PLTFM_INVALID_ARG;
  }

  bf_pltfm_chss_mgmt_bd_type_get(&board_id);

  if ((board_id == BF_PLTFM_BD_ID_NEWPORT_P0A) ||
      (board_id == BF_PLTFM_BD_ID_NEWPORT_P0B)) {
    max_chnl_id = 8;
  }

  if ((board_id == BF_PLTFM_BD_ID_MAVERICKS_P0A) ||
      (board_id == BF_PLTFM_BD_ID_MAVERICKS_P0B) ||
      (board_id == BF_PLTFM_BD_ID_MAVERICKS_P0C)) {
    if ((port_info->conn_id > 65) || (port_info->chnl_id >= max_chnl_id)) {
      LOG_ERROR("Invalid argument conn_id: %d chnl_id: %d\n",
                port_info->conn_id,
                port_info->chnl_id);
      return BF_PLTFM_INVALID_ARG;
    }
  } else if ((board_id == BF_PLTFM_BD_ID_MONTARA_P0A) ||
             (board_id == BF_PLTFM_BD_ID_MONTARA_P0B) ||
             (board_id == BF_PLTFM_BD_ID_MONTARA_P0C) ||
             (board_id == BF_PLTFM_BD_ID_NEWPORT_P0B) ||
             (board_id == BF_PLTFM_BD_ID_NEWPORT_P0A)) {
    if ((port_info->conn_id > 33) || (port_info->chnl_id >= max_chnl_id)) {
      LOG_ERROR("Invalid argument conn_id: %d chnl_id: %d\n",
                port_info->conn_id,
                port_info->chnl_id);
      return BF_PLTFM_INVALID_ARG;
    }
  } else {
    LOG_ERROR("Invalid board id 0x%0x\n", board_id);
    return BF_PLTFM_INVALID_ARG;
  }

  index = port_info->conn_id * max_chnl_id + port_info->chnl_id;

  for (i = 0; i < 6; i++) {
    mac_info[i] = eeprom.bf_pltfm_mac_base[i];
  }

  t = mac_info[5] | (mac_info[4] << 8);

  t = t + index;

  mac_info[5] = t & 0xFF;
  mac_info[4] = (t >> 8) & 0xFF;

  LOG_DEBUG("Extended Mac addr: %x:%x:%x:%x:%x:%x \n",
            mac_info[0],
            mac_info[1],
            mac_info[2],
            mac_info[3],
            mac_info[4],
            mac_info[5]);

  return BF_PLTFM_SUCCESS;
}
