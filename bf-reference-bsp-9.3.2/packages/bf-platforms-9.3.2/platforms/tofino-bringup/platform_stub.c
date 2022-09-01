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

bf_pltfm_status_t bf_pltfm_platform_port_init(bf_dev_id_t dev_id,
                                              bool warm_init) {
  (void)dev_id;
  (void)warm_init;
  return BF_PLTFM_SUCCESS;
}

bool platform_is_hw(void) { return true; }

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

int bf_pltfm_led_init(int chip_id) {
  (void)chip_id;
  return 0;
}

int bf_pltfm_port_led_set(int chip_id,
                          bf_pltfm_port_info_t *port_info,
                          bf_led_condition_t led_cond) {
  (void)chip_id;
  (void)port_info;
  (void)led_cond;
  return 0;
}

int bf_pltfm_qsfp_init(void *arg) {
  (void)arg;
  return 0;
}

bool bf_pltfm_detect_qsfp(unsigned int module) {
  (void)module;
  return true;
}

int bf_pltfm_qsfp_read_module(unsigned int module,
                              int offset,
                              int len,
                              uint8_t *buf) {
  (void)module;
  (void)offset;
  (void)len;
  (void)buf;
  return 0;
}

int bf_pltfm_qsfp_write_module(unsigned int module,
                               int offset,
                               int len,
                               uint8_t *buf) {
  (void)module;
  (void)offset;
  (void)len;
  (void)buf;
  return 0;
}

int bf_pltfm_qsfp_read_reg(unsigned int module,
                           uint8_t page,
                           int offset,
                           uint8_t *val) {
  (void)module;
  (void)offset;
  (void)page;
  (void)val;
  return 0;
}

int bf_pltfm_qsfp_write_reg(unsigned int module,
                            uint8_t page,
                            int offset,
                            uint8_t val) {
  (void)module;
  (void)offset;
  (void)page;
  (void)val;
  return 0;
}

int bf_pltfm_qsfp_get_presence_mask(uint32_t *port_1_32_pres,
                                    uint32_t *port_32_64_pres,
                                    uint32_t *port_cpu_pres) {
  *port_1_32_pres = 0;
  *port_32_64_pres = 0xffffffffUL;
  *port_cpu_pres = 0xFFFFFFFeUL;
  return 0;
}

void bf_pltfm_qsfp_get_int_mask(uint32_t *port_1_32_ints,
                                uint32_t *port_32_64_ints,
                                uint32_t *port_cpu_ints) {
  *port_1_32_ints = *port_32_64_ints = *port_cpu_ints = 0;
}

int bf_pltfm_qsfp_get_lpmode_mask(uint32_t *port_1_32_lpmod,
                                  uint32_t *port_32_64_lpmode,
                                  uint32_t *port_cpu_lpmod) {
  *port_1_32_lpmod = *port_32_64_lpmode = *port_cpu_lpmod = 0;
  return 0;
}

int bf_pltfm_qsfp_set_lpmode(int port, bool lpmode) {
  (void)port;
  (void)lpmode;
  return 0;
}

int bf_pltfm_qsfp_module_reset(int module, bool reset) {
  (void)module;
  (void)reset;
  return 0;
}

// Returns mac_addr from platform
bf_pltfm_status_t platform_port_mac_addr_get(bf_pltfm_port_info_t *port_info,
                                             uint8_t *mac_addr) {
  (void)port_info;
  (void)mac_addr;
  return BF_PLTFM_SUCCESS;
}
