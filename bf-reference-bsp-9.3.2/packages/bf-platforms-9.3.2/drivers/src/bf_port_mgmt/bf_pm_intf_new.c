/*******************************************************************************
 * Copyright (c) 2015-2020 Barefoot Networks, Inc.
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * $Id: $
 *
 ******************************************************************************/

#include <stdio.h>
#include <stdint.h>
#include <pthread.h>
#include <ctype.h>
#include <unistd.h>

#include <bf_bd_cfg/bf_bd_cfg_intf.h>
#include <bf_port_mgmt/bf_port_mgmt_intf.h>

#include <bf_types/bf_types.h>
#include <bfsys/bf_sal/bf_sys_sem.h>
#include <bfsys/bf_sal/bf_sys_timer.h>
#include <bf_switchd/bf_switchd.h>
#include <tofino/bf_pal/bf_pal_pltfm_porting.h>
#include <bf_pm/bf_pm_intf.h>
#include <bf_pltfm.h>
#include <bf_pltfm_ext_phy.h>
#include "bf_pm_priv.h"

static bf_pm_intf_t intf_obj[MAX_CONNECTORS][MAX_CHAN_PER_CONNECTOR];
static bf_pm_intf_mtx_t intf_mtx[MAX_CONNECTORS][MAX_CHAN_PER_CONNECTOR];

typedef enum {
  PM_INTF_FSM_DISABLED,
  PM_INTF_FSM_INIT,
  PM_INTF_FSM_MEDIA_DETECTED,
  PM_INTF_FSM_WAIT_SERDES_TX_INIT,
  PM_INTF_FSM_WAIT_MEDIA_INIT,
  PM_INTF_FSM_WAIT_LINK_INIT,
  PM_INTF_FSM_LINK_READY,
  PM_INTF_FSM_INCOMPATIBLE_MEDIA,
} pm_intf_fsm_states_t;

static char *pm_intf_fsm_st_to_str[] = {
    "PM_INTF_FSM_DISABLED           ",
    "PM_INTF_FSM_INIT           ",
    "PM_INTF_FSM_MEDIA_DETECTED     ",
    "PM_INTF_FSM_WAIT_SERDES_TX_INIT",
    "PM_INTF_FSM_WAIT_MEDIA_INIT    ",
    "PM_INTF_FSM_WAIT_LINK_INIT     ",
    "PM_INTF_FSM_LINK_READY         ",
    "PM_INTF_FSM_INCOMPATIBLE_MEDIA ",
};

pm_intf_fsm_states_t pm_intf_fsm_st[MAX_CONNECTORS][MAX_CHAN_PER_CONNECTOR];

// assumes caller will hold the lock
static void bf_pm_intf_update_fsm_st(pm_intf_fsm_states_t curr_st,
                                     pm_intf_fsm_states_t next_st,
                                     uint32_t conn_id,
                                     uint32_t chan) {
  if (curr_st != next_st) {
    pm_intf_fsm_st[conn_id][chan] = next_st;
    LOG_DEBUG("PM_INTF_FSM: %2d/%d : %s --> %s",
              conn_id,
              chan,
              pm_intf_fsm_st_to_str[curr_st],
              pm_intf_fsm_st_to_str[next_st]);
  }
}

static void bf_pm_handle_intf_disable(bf_pm_intf_cfg_t *icfg,
                                      bf_pm_intf_t *intf,
                                      bf_pm_qsfp_info_t *qsfp_info) {
  if (!icfg || !intf || !qsfp_info) return;

  bf_pal_front_port_handle_t port_hdl;

  port_hdl.conn_id = icfg->conn_id;
  port_hdl.chnl_id = icfg->channel;

  // Admin-down. Inform the qsfp-fsm to disable the optics
  if (!icfg->admin_up && intf->self_fsm_running && qsfp_info->is_present &&
      qsfp_info->is_optic) {
    for (uint32_t ch = 0; ch < icfg->intf_nlanes; ch++) {
      qsfp_fsm_ch_disable(
          icfg->dev_id, port_hdl.conn_id, port_hdl.chnl_id + ch);
    }
  }

  // command issued to SDK
  if (intf->self_fsm_running) {
    bf_pm_port_serdes_rx_ready_for_bringup_set(icfg->dev_id, &port_hdl, false);
    bf_pm_pltfm_front_port_ready_for_bringup(icfg->dev_id, &port_hdl, false);
    intf->self_fsm_running = false;
  }
}

static bool bf_pm_intf_is_qsfpdd_short(bf_pltfm_qsfpdd_type_t qsfpdd_type) {
  switch (qsfpdd_type) {
    case BF_PLTFM_QSFPDD_CU_0_5_M:
    case BF_PLTFM_QSFPDD_CU_1_M:
    case BF_PLTFM_QSFPDD_CU_LOOP:
      return true;
    default:
      break;
  }

  return false;
}

// Since there is no hardware SI settings for qsfp, map it to qsfpdd.
bf_pltfm_qsfpdd_type_t bf_pm_intf_get_qsfpdd_type(
    bf_pltfm_qsfp_type_t qsfp_type) {
  switch (qsfp_type) {
    case BF_PLTFM_QSFP_CU_0_5_M:
      return BF_PLTFM_QSFPDD_CU_0_5_M;
    case BF_PLTFM_QSFP_CU_1_M:
      return BF_PLTFM_QSFPDD_CU_1_M;
    case BF_PLTFM_QSFP_CU_2_M:
      return BF_PLTFM_QSFPDD_CU_2_M;
    case BF_PLTFM_QSFP_CU_3_M:
      return BF_PLTFM_QSFPDD_CU_2_M;
    // return BF_PLTFM_QSFPDD_CU_3_M;
    case BF_PLTFM_QSFP_CU_LOOP:
      return BF_PLTFM_QSFPDD_CU_LOOP;
    case BF_PLTFM_QSFP_OPT:
      return BF_PLTFM_QSFPDD_OPT;
    case BF_PLTFM_QSFP_UNKNOWN:
      return BF_PLTFM_QSFPDD_UNKNOWN;
  }

  return BF_PLTFM_QSFPDD_UNKNOWN;
}

static void bf_pm_intf_send_asic_serdes_info(bf_pm_intf_cfg_t *icfg,
                                             bf_pm_intf_t *intf,
                                             bf_pm_qsfp_info_t *qsfp_info) {
  uint32_t conn_id, ch;
  int tx_inv = 0, rx_inv = 0;
  bf_pltfm_serdes_lane_tx_eq_t tx_eq;
  bf_pltfm_port_info_t port_info;
  bf_pal_front_port_handle_t port_hdl;
  bf_dev_id_t dev_id;
  bf_pal_serdes_polarity_t asic_serdes_pol[MAX_CHAN_PER_CONNECTOR] = {{0}};
  bf_pal_serdes_tx_eq_params_t asic_serdes_tx_eq;

  if (!icfg || !intf || !qsfp_info) return;

  dev_id = icfg->dev_id;

  if (!bf_pm_intf_is_device_family_tofino2(dev_id)) return;

  port_info.conn_id = port_hdl.conn_id = conn_id = icfg->conn_id;
  port_info.chnl_id = port_hdl.chnl_id = icfg->channel;

  // Note: SDK caches it and applies it only when the port-fsm is run
  for (ch = 0; ch < icfg->intf_nlanes; ch++) {
    bf_bd_port_serdes_polarity_get(&port_info, &rx_inv, &tx_inv);
    asic_serdes_pol[ch].rx_inv = (bool)rx_inv;
    asic_serdes_pol[ch].tx_inv = (bool)tx_inv;

    if (bf_bd_is_this_port_internal(icfg->conn_id, icfg->channel)) {
      bf_bd_port_serdes_tx_params_get(
          &port_info, BF_PLTFM_QSFPDD_CU_0_5_M, &tx_eq, icfg->encoding);
    } else if (bf_qsfp_is_cmis(conn_id)) {
      bf_bd_port_serdes_tx_params_get(
          &port_info, qsfp_info->qsfpdd_type, &tx_eq, icfg->encoding);
    } else {
      bf_bd_port_serdes_tx_params_get(
          &port_info,
          bf_pm_intf_get_qsfpdd_type(qsfp_info->qsfp_type),
          &tx_eq,
          icfg->encoding);
    }

    asic_serdes_tx_eq.tx_eq.tof2[ch].tx_pre1 = tx_eq.tx_pre1;
    asic_serdes_tx_eq.tx_eq.tof2[ch].tx_pre2 = tx_eq.tx_pre2;
    asic_serdes_tx_eq.tx_eq.tof2[ch].tx_main = tx_eq.tx_main;
    asic_serdes_tx_eq.tx_eq.tof2[ch].tx_post1 = tx_eq.tx_post1;
    asic_serdes_tx_eq.tx_eq.tof2[ch].tx_post2 = tx_eq.tx_post2;
    port_info.chnl_id++;
  }

  bf_pm_port_serdes_polarity_set(
      dev_id, &port_hdl, icfg->intf_nlanes, asic_serdes_pol);

  LOG_DEBUG("Pushing the TX-EQ parameters for dev %d: %d/%d encoding:%s ",
            dev_id,
            icfg->conn_id,
            icfg->channel,
            (icfg->encoding == BF_PLTFM_ENCODING_PAM4) ? "PAM4" : "NRZ");

  bf_pm_port_serdes_tx_eq_params_set(
      dev_id, &port_hdl, icfg->intf_nlanes, &asic_serdes_tx_eq);
}

static void bf_pm_intf_send_asic_precoding_info(bf_pm_intf_cfg_t *icfg,
                                                bf_pm_intf_t *intf,
                                                bf_pm_qsfp_info_t *qsfp_info) {
  uint32_t ch;
  bf_pal_front_port_handle_t port_hdl;
  bf_dev_id_t dev_id;
  bf_pm_port_precoding_policy_e precode = PM_PRECODING_DISABLE;

  if (!icfg || !intf || !qsfp_info) return;

  dev_id = icfg->dev_id;
  if ((!bf_pm_intf_is_device_family_tofino2(dev_id)) ||
      bf_bd_is_this_port_internal(icfg->conn_id, icfg->channel)) {
    return;
  }

  port_hdl.conn_id = icfg->conn_id;
  port_hdl.chnl_id = icfg->channel;

  if ((!qsfp_info->is_optic) && (icfg->encoding == BF_PLTFM_ENCODING_PAM4)) {
    if (!bf_pm_intf_is_qsfpdd_short(qsfp_info->qsfpdd_type)) {
      precode = PM_PRECODING_ENABLE;
    }
  }

  for (ch = 0; ch < icfg->intf_nlanes; ch++) {
    bf_pm_port_precoding_tx_set(dev_id, &port_hdl, ch, precode);
    bf_pm_port_precoding_rx_set(dev_id, &port_hdl, ch, precode);
  }
}

static void bf_pm_interface_fsm_run(bf_pm_intf_cfg_t *icfg,
                                    bf_pm_intf_t *intf,
                                    bf_pm_qsfp_info_t *qsfp_info) {
  pm_intf_fsm_states_t st;
  pm_intf_fsm_states_t next_st;
  bf_pal_front_port_handle_t port_hdl;
  bool asic_serdes_tx_ready;
  uint32_t ch = 0;

  if ((!icfg) || (!intf) || (!qsfp_info)) return;

  port_hdl.conn_id = icfg->conn_id;
  port_hdl.chnl_id = icfg->channel;
  next_st = st = pm_intf_fsm_st[port_hdl.conn_id][port_hdl.chnl_id];

  // handle qsfp-removal in any state
  if ((!bf_bd_is_this_port_internal(icfg->conn_id, icfg->channel)) &&
      (!qsfp_info->is_present)) {
    bf_pm_handle_intf_disable(icfg, intf, qsfp_info);
    // since fsm-runs only on admin-enabled, move state to init
    if (icfg->admin_up) {
      next_st = PM_INTF_FSM_INIT;
    }
  }

  switch (st) {
    case PM_INTF_FSM_DISABLED:
      break;
    case PM_INTF_FSM_INIT:
      // check for media presence, if so advance states (internal ports are
      // assumed to always be present)
      if ((bf_bd_is_this_port_internal(icfg->conn_id, icfg->channel)) ||
          (qsfp_info->is_present)) {
        next_st = PM_INTF_FSM_MEDIA_DETECTED;
      }
      break;
    case PM_INTF_FSM_MEDIA_DETECTED:
      // media is present but the data path is not yet being initalized. check
      // to make sure media is supported, set up serdes tx

      // check compatibility of module supported Applications with switch
      // config.
      //
      // For Copper, just allow any speed config or channelized mode
      // Note: We are not catching mismatch like module is 100G DAC but
      // configuration is 400G. Worst case, link will not comeup.
      //
      // For optics, find-match-app is common for both sff8636 and CMIS
      if (qsfp_info->is_optic) {
        if (bf_cmis_find_matching_Application(icfg->conn_id,
                                              icfg->intf_speed,
                                              icfg->intf_nlanes,
                                              icfg->channel /*,
                                              icfg->encoding */) < 0) {
          LOG_ERROR(
              "pm intf fsm: qsfp %d ch %d Matching Application not found. "
              "Speed mismtach between optical-module and configuration \n",
              icfg->conn_id,
              icfg->channel);
          next_st = PM_INTF_FSM_INCOMPATIBLE_MEDIA;
          break;
        }
      }

      // set up switch tx eq settings
      bf_pm_intf_send_asic_serdes_info(icfg, intf, qsfp_info);

      bf_pm_intf_send_asic_precoding_info(icfg, intf, qsfp_info);

      // enable SERDES TX
      if (qsfp_info->is_optic) {
        bf_pm_pltfm_front_port_eligible_for_autoneg(
            icfg->dev_id, &port_hdl, false);
        bf_pm_pltfm_front_port_ready_for_bringup(icfg->dev_id, &port_hdl, true);
        next_st = PM_INTF_FSM_WAIT_SERDES_TX_INIT;
        break;
      } else {  // copper or internal
        switch (icfg->an_policy) {
          case PM_AN_FORCE_DISABLE:
            bf_pm_pltfm_front_port_eligible_for_autoneg(
                icfg->dev_id, &port_hdl, false);
            // since we are not interested in asic-serdes-tx, indicate RX is
            // ready for DFE
            bf_pm_port_serdes_rx_ready_for_bringup_set(
                icfg->dev_id, &port_hdl, true);
            bf_pm_pltfm_front_port_ready_for_bringup(
                icfg->dev_id, &port_hdl, true);
            break;
          case PM_AN_DEFAULT:
          case PM_AN_FORCE_ENABLE:
          default:
            bf_pm_pltfm_front_port_eligible_for_autoneg(
                icfg->dev_id, &port_hdl, true);
            bf_pm_pltfm_front_port_ready_for_bringup(
                icfg->dev_id, &port_hdl, true);
            break;
        }
        intf->self_fsm_running = true;
        // skip media init states for passive copper
        next_st = PM_INTF_FSM_WAIT_LINK_INIT;
        break;
      }

      // we should never get here
      break;
    case PM_INTF_FSM_WAIT_SERDES_TX_INIT:
      // Wait for the SERDES TX init to complete
      // This state is only used for optical cables
      bf_pm_port_serdes_tx_ready_get(
          icfg->dev_id, &port_hdl, &asic_serdes_tx_ready);

      if (!asic_serdes_tx_ready) {  // keep waiting
        break;
      }

      // Tell the qsfp_ch_fsm that its ok to enable the lanes in this link.
      // Loop through all lanes since we only run the pm_intf_fsm
      // for head channels
      for (ch = 0; ch < icfg->intf_nlanes; ch++) {
        qsfp_fsm_ch_enable(
            icfg->dev_id, port_hdl.conn_id, port_hdl.chnl_id + ch);
      }

      // Now that we've advanced the qsfp_ch_fsm, advance the pm_intf_fsm
      // to wait for qsfp init to complete
      next_st = PM_INTF_FSM_WAIT_MEDIA_INIT;
      break;
    case PM_INTF_FSM_WAIT_MEDIA_INIT:
      // Media is present, in high power mode, and is initializing the data
      // path.
      // This state is only used for optical cables

      // poll the qsfp_ch_fsm to see when all lanes in the data path are
      // finished

      if (qsfp_fsm_get_enabled_mask(port_hdl.conn_id, port_hdl.chnl_id) ==
          icfg->intf_chmask) {
        // Now that all names are initialized, is an incoming signal present?
        // Check qsfp RX CDR LOL to determine this
        // TODO RX LOL check

        // an incoming signal is present, tell the SERDES RX we're ready
        bf_pm_port_serdes_rx_ready_for_bringup_set(
            icfg->dev_id, &port_hdl, true);
        intf->self_fsm_running = true;

        // proceed to the next state to wait for the link to come up
        next_st = PM_INTF_FSM_WAIT_LINK_INIT;
        break;
      }

      // keep waiting
      break;
    case PM_INTF_FSM_WAIT_LINK_INIT:

      // Hook to link-up / down event from sdk if required.
      // But we may end up handling too many link toogles
      next_st = PM_INTF_FSM_LINK_READY;
      break;
    case PM_INTF_FSM_LINK_READY:
      // link is up
      // wait for media to be removed or port deletion or reconfig or los
      break;
    case PM_INTF_FSM_INCOMPATIBLE_MEDIA:
      // Media is incompatible with switch configuration, either because media
      // power requirements are too high or the media supported Applications
      // do not match the current switch configuration. Wait for media
      // to be replaced or configuration to be changed
      break;
  }
  if (st != next_st) {
    bf_pm_intf_update_fsm_st(st, next_st, port_hdl.conn_id, port_hdl.chnl_id);
  }
}

// Manages the port bringup
// Runs every 100msec
void bf_pm_interface_fsm(void) {
  // Do not touch hardware or alter the fsm during HA
  if (bf_pltfm_pm_is_ha_mode()) return;

  bf_pm_intf_cfg_t *icfg = NULL;
  bf_pm_intf_t *intf = NULL;
  unsigned long intf_key = 0;
  bf_pm_qsfp_info_t *qsfp_info;
  bf_pm_intf_mtx_t *mtx = NULL;
  uint32_t conn, chan;
  uint32_t num_ports = (uint32_t)bf_bd_cfg_bd_num_port_get();

  // This fsm is common to both qsfp and internal-ports.
  // Run through all the ports; ignores link-status
  for (conn = 1; conn <= num_ports; conn++) {
    for (chan = 0; chan < MAX_CHAN_PER_CONNECTOR; chan++) {
      intf = &intf_obj[conn][chan];
      icfg = &intf->intf_cfg;
      if ((!icfg->admin_up) || (!intf->intf_added)) continue;

      mtx = &intf_mtx[conn][chan];
      if (!mtx->mtx_inited) continue;

      bf_sys_mutex_lock(&mtx->intf_mtx);
      qsfp_info = intf->qsfp_info;

      if ((!intf->skip_periodic) && (icfg->admin_up) && (intf->intf_added)) {
        bf_pm_interface_fsm_run(icfg, intf, qsfp_info);
      }

      bf_sys_mutex_unlock(&mtx->intf_mtx);
    }
  }

  (void)intf_key;
}

void bf_pm_intf_init(void) {
  int port, ch;
  for (port = 0; port < MAX_CONNECTORS; port++) {
    for (ch = 0; ch < MAX_CHAN_PER_CONNECTOR; ch++) {
      pm_intf_fsm_st[port][ch] = PM_INTF_FSM_DISABLED;
    }
  }
}

void bf_pm_intf_add(bf_pltfm_port_info_t *port_info,
                    bf_dev_id_t dev_id,
                    bf_dev_port_t dev_port,
                    bf_pal_front_port_cb_cfg_t *port_cfg,
                    bf_pltfm_encoding_type_t encoding) {
  bf_pm_intf_cfg_t *icfg = NULL;
  bf_pm_intf_t *intf = NULL;
  bf_pm_intf_mtx_t *mtx = NULL;
  uint32_t conn, chan, curchan;

  if ((!port_info) || (!port_cfg)) {
    return;
  }

  conn = port_info->conn_id;
  chan = port_info->chnl_id;
  intf = &intf_obj[conn][chan];
  mtx = &intf_mtx[conn][chan];
  if (!mtx->mtx_inited) {
    bf_sys_mutex_init(&mtx->intf_mtx);
    mtx->mtx_inited = true;
  }
  bf_sys_mutex_lock(&mtx->intf_mtx);
  if (intf->intf_added) {
    // Catch duplicate
    LOG_ERROR(
        "Add failed. Port-already exist on conn %d chan %d \n", conn, chan);
    bf_sys_mutex_unlock(&mtx->intf_mtx);
    return;
  }
  memset(&intf_obj[conn][chan], 0, sizeof(bf_pm_intf_t));
  intf->qsfp_info = bf_pltfm_get_pm_qsfp_info_ptr(port_info->conn_id);
  icfg = &intf->intf_cfg;
  icfg->dev_id = dev_id;
  icfg->conn_id = port_info->conn_id;
  icfg->channel = port_info->chnl_id;
  icfg->intf_speed = port_cfg->speed_cfg;
  icfg->intf_nlanes = port_cfg->num_lanes;
  icfg->dev_port = dev_port;
  icfg->encoding = encoding;

  icfg->intf_chmask = 0;
  for (curchan = chan; curchan < (uint32_t)(chan + icfg->intf_nlanes);
       curchan++) {
    icfg->intf_chmask |= 1 << curchan;
  }

  qsfp_fsm_update_cfg(icfg->conn_id,
                      icfg->channel,
                      icfg->intf_speed,
                      icfg->intf_nlanes,
                      icfg->encoding);

  intf->intf_added = true;
  bf_sys_mutex_unlock(&mtx->intf_mtx);
}

void bf_pm_intf_del(bf_pltfm_port_info_t *port_info,
                    bf_pal_front_port_cb_cfg_t *port_cfg) {
  bf_pm_intf_t *intf = NULL;
  bf_pm_intf_mtx_t *mtx = NULL;
  bf_pm_intf_cfg_t *icfg = NULL;
  uint32_t conn, chan;

  if (!port_info) {
    return;
  }
  conn = port_info->conn_id;
  chan = port_info->chnl_id;
  intf = &intf_obj[conn][chan];
  mtx = &intf_mtx[conn][chan];

  qsfp_fsm_deinit_cfg(conn, chan);

  if (!intf->intf_added) {
    // Catch duplicate
    LOG_ERROR(
        "Del failed. Port does not exist on conn %d chan %d \n", conn, chan);
    return;
  }

  bf_sys_mutex_lock(&mtx->intf_mtx);
  icfg = &intf->intf_cfg;
  icfg->admin_up = false;
  intf->intf_added = false;
  bf_sys_mutex_unlock(&mtx->intf_mtx);
  (void)port_cfg;
}

void bf_pm_intf_enable(bf_pltfm_port_info_t *port_info,
                       bf_pal_front_port_cb_cfg_t *port_cfg) {
  bf_pm_intf_cfg_t *icfg = NULL;
  bf_pm_intf_t *intf = NULL;
  bf_pal_front_port_handle_t port_hdl;
  bf_pm_intf_mtx_t *mtx = NULL;
  uint32_t conn, chan;

  if ((!port_info) || (!port_cfg)) {
    return;
  }

  conn = port_info->conn_id;
  chan = port_info->chnl_id;
  intf = &intf_obj[conn][chan];
  mtx = &intf_mtx[conn][chan];
  if (!intf->intf_added) return;

  bf_sys_mutex_lock(&mtx->intf_mtx);
  icfg = &intf->intf_cfg;
  // Not part of port_cfg :(
  port_hdl.conn_id = icfg->conn_id;
  port_hdl.chnl_id = icfg->channel;
  bf_pm_port_autoneg_get(icfg->dev_id, &port_hdl, &icfg->an_policy);
  bf_pm_intf_update_fsm_st(
      pm_intf_fsm_st[conn][chan], PM_INTF_FSM_INIT, conn, chan);
  icfg->admin_up = true;
  bf_sys_mutex_unlock(&mtx->intf_mtx);
}

void bf_pm_intf_disable(bf_pltfm_port_info_t *port_info,
                        bf_pal_front_port_cb_cfg_t *port_cfg) {
  bf_pm_intf_cfg_t *icfg = NULL;
  bf_pm_intf_t *intf = NULL;
  bf_pm_intf_mtx_t *mtx = NULL;
  // uint32_t conn, chan, curchan;
  uint32_t conn, chan;
  bf_pm_qsfp_info_t *qsfp_info;

  if ((!port_info) || (!port_cfg)) {
    return;
  }
  conn = port_info->conn_id;
  chan = port_info->chnl_id;
  intf = &intf_obj[conn][chan];
  mtx = &intf_mtx[conn][chan];

  if (!intf->intf_added) return;
  bf_sys_mutex_lock(&mtx->intf_mtx);
  icfg = &intf->intf_cfg;
  icfg->admin_up = false;
  icfg->an_policy = 0;
  qsfp_info = intf->qsfp_info;
  bf_pm_handle_intf_disable(icfg, intf, qsfp_info);
  bf_pm_intf_update_fsm_st(
      pm_intf_fsm_st[conn][chan], PM_INTF_FSM_DISABLED, conn, chan);
  bf_sys_mutex_unlock(&mtx->intf_mtx);
}
