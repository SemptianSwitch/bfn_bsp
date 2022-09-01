/*******************************************************************************
 * Copyright (c) 2015-2020 Barefoot Networks, Inc.
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * $Id: $
 *
 ******************************************************************************/
#include <bf_bd_cfg/bf_bd_cfg_intf.h>
#include <bf_qsfp/bf_qsfp.h>
#include <bf_led/bf_led.h>
#include <bf_types/bf_types.h>
#include <bfsys/bf_sal/bf_sys_sem.h>

#define BF_PM_INTF_MAP_KEY(conn, channel) ((conn) | ((channel) << 16))

typedef struct bf_pm_qsfp_info_t {
  bool is_present;
  bf_pltfm_qsfp_type_t qsfp_type;
  bf_pltfm_qsfpdd_type_t qsfpdd_type;

  // for Optical module
  bool is_optic;  // 0 for dac;

} bf_pm_qsfp_info_t;

// Store all user configuration per port triggered via SDK
typedef struct bf_pm_intf_cfg_ {
  bf_dev_id_t dev_id;
  uint32_t conn_id;
  uint32_t channel;
  bf_port_speed_t intf_speed;
  uint32_t intf_nlanes;
  int intf_chmask;
  bool admin_up;
  bf_dev_port_t dev_port;
  bf_pltfm_encoding_type_t encoding;

  // For DAC:
  // If PM_AN_FORCE_ENABLE or PM_AN_DEFAULT, trigger AN/LT.
  // If PM_AN_FORCE_DISABLE, trigger DFE tunning.
  //
  // For Optic:
  //  ignore an_policy; always do AN-off and DFE tuning
  //
  // Note: If there is a retimer/ext phy, AN/DFE should be done
  // between local and remote ports. Between ASIC and retimers,,
  // DFE or AN/LT kR can be done depending upon platform need.
  //
  // Check if CLI triggers a change to platform or not - TBD
  bf_pm_port_autoneg_policy_e an_policy;

} bf_pm_intf_cfg_t;

// port structure to coordinate between, bf-pm, qsfp and its fsm
typedef struct bf_pm_intf_ {
  bf_pm_intf_cfg_t intf_cfg;  // from SDK

  bf_pm_qsfp_info_t *qsfp_info;

  // If needed, add ext-phy here

  // Manage
  bool skip_periodic;  // for debugging

  bool self_fsm_running;
  bool intf_added;
} bf_pm_intf_t;

typedef struct {
  bool mtx_inited;
  bf_sys_mutex_t intf_mtx;
} bf_pm_intf_mtx_t;

uint32_t num_lanes_consumed_get(bf_port_speed_t port_speed);

// bf_pltfm_pm_qsfp_info_t *bf_pltfm_pm_qsfp_info_get(uint32_t chnl);

bf_pltfm_status_t pltfm_pm_port_qsfp_is_present(bf_pltfm_port_info_t *port_info,
                                                bool *is_present);

bf_pltfm_status_t qsfp_scan(bf_dev_id_t dev_id);

bf_pltfm_status_t qsfp_fsm(bf_dev_id_t dev_id);

void qsfp_deassert_all_reset_pins(void);

void qsfp_fsm_inserted(int conn_id);

void qsfp_fsm_removed(int conn_id);

void qsfp_state_ha_config_set(bf_dev_id_t dev_id, int conn_id);

void qsfp_state_ha_config_delete(int conn_id);
void qsfp_fsm_ch_notify_not_ready(bf_dev_id_t dev_id, int conn_id, int ch);

bf_status_t bf_pltfm_pm_media_type_get(bf_pltfm_port_info_t *port_info,
                                       bf_media_type_t *media_type);
bf_pltfm_status_t bf_pltfm_pm_ha_mode_set();
bf_pltfm_status_t bf_pltfm_pm_ha_mode_clear();
bool bf_pltfm_pm_is_ha_mode();

bf_pm_qsfp_info_t *bf_pltfm_get_pm_qsfp_info_ptr(int connector);
void bf_pm_intf_add(bf_pltfm_port_info_t *port_info,
                    bf_dev_id_t dev_id,
                    bf_dev_port_t dev_port,
                    bf_pal_front_port_cb_cfg_t *port_cfg,
                    bf_pltfm_encoding_type_t encoding);
void bf_pm_intf_del(bf_pltfm_port_info_t *port_info,
                    bf_pal_front_port_cb_cfg_t *port_cfg);
void bf_pm_intf_enable(bf_pltfm_port_info_t *port_info,
                       bf_pal_front_port_cb_cfg_t *port_cfg);
void bf_pm_intf_disable(bf_pltfm_port_info_t *port_info,
                        bf_pal_front_port_cb_cfg_t *port_cfg);
void bf_pm_intf_init();
void bf_pm_interface_fsm(void);
void bf_pm_qsfp_quick_removal_detected_set(uint32_t conn_id, bool flag);
bool bf_pm_qsfp_quick_removal_detected_get(uint32_t conn_id);
int qsfp_fsm_update_cfg(int conn_id,
                        int first_ch,
                        bf_port_speed_t intf_speed,
                        int intf_nlanes,
                        bf_pltfm_encoding_type_t encoding);
int qsfp_fsm_deinit_cfg(int conn_id, int first_ch);
int qsfp_fsm_get_enabled_mask(int conn_id, int first_ch);
