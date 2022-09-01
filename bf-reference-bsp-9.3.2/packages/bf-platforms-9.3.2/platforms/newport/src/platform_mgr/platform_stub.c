/*******************************************************************************
 * Copyright (c) 2015-2020 Barefoot Networks, Inc.
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * $Id: $
 *
 ******************************************************************************/

/* Standard includes */
#include <stdio.h>

/* Module includes */
#include <bf_types/bf_types.h>
#include <bf_pltfm_types/bf_pltfm_types.h>
#include <bf_switchd/bf_switchd.h>
#include <bf_bd_cfg/bf_bd_cfg_intf.h>
#include <bf_led/bf_led.h>
#include <bfsys/bf_sal/bf_sys_intf.h>
#include <bf_types/bf_types.h>
#include <bf_port_mgmt/bf_port_mgmt_intf.h>
#include <bf_qsfp/bf_qsfp.h>
#include <bf_pltfm_ext_phy.h>
#include <bf_pltfm_bd_cfg.h>

bf_pltfm_status_t bf_pltfm_platform_ext_phy_config_set(
    bf_dev_id_t dev_id, uint32_t conn, bf_pltfm_qsfp_type_t qsfp_type) {
  (void)dev_id;
  (void)conn;
  (void)qsfp_type;
  return BF_PLTFM_SUCCESS;
}

bf_pltfm_status_t bf_pltfm_ext_phy_init(void) { return BF_PLTFM_SUCCESS; }

bf_pltfm_status_t bf_pltfm_ext_phy_set_mode(bf_pltfm_port_info_t *port_info,
                                            bf_pltfm_ext_phy_cfg_t *port_cfg) {
  (void)port_info;
  (void)port_cfg;
  return BF_PLTFM_SUCCESS;
}

bf_pltfm_status_t bf_pltfm_ext_phy_del_mode(bf_pltfm_port_info_t *port_info,
                                            bf_pltfm_ext_phy_cfg_t *port_cfg) {
  (void)port_info;
  (void)port_cfg;
  return BF_PLTFM_SUCCESS;
}

bf_pltfm_status_t bf_pltfm_ext_phy_init_speed(uint8_t port_num,
                                              bf_port_speed_t port_speed) {
  (void)port_num;
  (void)port_speed;
  return BF_PLTFM_SUCCESS;
}

bf_pltfm_status_t bf_pltfm_ext_phy_conn_eq_set(
    bf_pltfm_port_info_t *port_info, bf_pltfm_ext_phy_cfg_t *port_cfg) {
  (void)port_info;
  (void)port_cfg;
  return BF_PLTFM_SUCCESS;
}

bf_pltfm_status_t bf_pltfm_platform_ext_phy_prbs_set(
    bf_pltfm_port_info_t *port_info, uint8_t direction, uint16_t prbs_mode) {
  (void)port_info;
  (void)direction;
  (void)prbs_mode;
  return BF_PLTFM_SUCCESS;
}
