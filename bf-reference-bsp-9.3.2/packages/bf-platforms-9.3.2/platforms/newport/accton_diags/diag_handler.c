/*******************************************************************************
 * Copyright (c) 2015-2020 Barefoot Networks, Inc.
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * $Id: $
 *
 ******************************************************************************/
/*!
 * @file diag_handler.c
 * @date
 *
 * Contains implementation of diag commands servicing
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>  //strlen
#include <signal.h>
#include <inttypes.h>
#include <dvm/bf_dma_types.h>
#include <dvm/bf_drv_intf.h>
#include <lld/lld_err.h>
#include <lld/lld_sku.h>
#include <lld/bf_dma_if.h>
#include <tofino/bf_pal/bf_pal_types.h>
#include <bf_pm/bf_pm_intf.h>
#include <port_mgr/bf_port_if.h>
#include <port_mgr/bf_serdes_if.h>
#include <port_mgr/bf_tof2_serdes_if.h>
#include <pipe_mgr/pipe_mgr_intf.h>
#include <bf_pltfm_types/bf_pltfm_types.h>
#include <bf_led/bf_led.h>
#include <bf_qsfp/bf_qsfp.h>
#include <bf_bd_cfg/bf_bd_cfg_intf.h>
#include <bf_pltfm_chss_mgmt_intf.h>
#include <bf_pltfm_mgr/pltfm_mgr_handlers.h>
#include "diag_handler.h"
#include "diag_server.h"

extern newport_diag_info_t newport_diag_info;

static bf_diag_sess_hdl_t diag_snake_sess_hdl = 0;

#define NEWPORT_DIAG_GET_LINESPEED_PARAMS (newport_diag_info.linespeed.params)
#define NEWPORT_DIAG_LINESPEED_MEMSET    \
  memset(&(newport_diag_info.linespeed), \
         0,                              \
         sizeof(newport_diag_linespeed_info_t))
#define NEWPORT_DIAG_GET_PORT_STATS(port, interval) \
  (newport_diag_info.port_stats[port][interval])
#define NEWPORT_DIAG_PORT_STATS_MEMSET    \
  memset(&(newport_diag_info.port_stats), \
         0,                               \
         sizeof(newport_diag_info.port_stats))
#define NEWPORT_DIAG_DEF_PORT_SPEED BF_SPEED_400G
#define NEWPORT_DIAG_DEF_PORT_SPEED_CPU BF_SPEED_100G
#define NEWPORT_DIAG_DEF_PORT_FEC BF_FEC_TYP_REED_SOLOMON
#define NEWPORT_DIAG_CPU_PORT 2
#define NEWPORT_DIAG_ETH_CPU_PORT 33
#define NEWPORT_NUM_CHANNELS 8

/* Set loopback mode */
bf_status_t newport_diag_get_loopback_mode_from_str(
    char *type, bf_diag_port_lpbk_mode_e *loop_mode) {
  if (!strncmp(type, "INTMI", strlen("INTMI"))) {
    *loop_mode = BF_DIAG_PORT_LPBK_MAC;
  } else if (!strncmp(type, "INTMO", strlen("INTMO"))) {
    *loop_mode = BF_DIAG_PORT_LPBK_PCS;
  } else if (!strncmp(type, "EXT1", strlen("EXT1"))) {
    *loop_mode = BF_DIAG_PORT_LPBK_EXT;
  } else if (!strncmp(type, "EXT2", strlen("EXT2"))) {
    *loop_mode = BF_DIAG_PORT_LPBK_EXT;
  } else {
    *loop_mode = BF_DIAG_PORT_LPBK_NONE;
  }

  return BF_SUCCESS;
}

/* Get the Speed string */
char *newport_diag_helper_get_speed_string(bf_port_speed_t speed,
                                           char *ret_buffer) {
  char *buffer;

  ret_buffer[0] = '\0';

  switch (speed) {
    case BF_SPEED_NONE:
      buffer = "Unkn";
      break;
    case BF_SPEED_1G:
      buffer = "1G";
      break;
    case BF_SPEED_10G:
      buffer = "10G";
      break;
    case BF_SPEED_25G:
      buffer = "25G";
      break;
    case BF_SPEED_40G:
      buffer = "40G";
      break;
    case BF_SPEED_40G_NB:
      buffer = "40G_NB";
      break;
    case BF_SPEED_50G:
      buffer = "50G";
      break;
    case BF_SPEED_100G:
      buffer = "100G";
      break;
    case BF_SPEED_200G:
      buffer = "200G";
      break;
    case BF_SPEED_400G:
      buffer = "400G";
      break;
    default:
      buffer = "Unkn";
      break;
  }
  strncpy(ret_buffer, buffer, strlen(buffer));

  return ret_buffer;
}

/* Get the FEC string */
char *newport_diag_helper_get_fec_string(bf_fec_type_t fec, char *ret_buffer) {
  char *buffer;

  ret_buffer[0] = '\0';

  switch (fec) {
    case BF_FEC_TYP_NONE:
      buffer = "None";
      break;
    case BF_FEC_TYP_FIRECODE:
      buffer = "FC";
      break;
    case BF_FEC_TYP_REED_SOLOMON:
      buffer = "RS";
      break;
    default:
      buffer = "Unkn";
      break;
  }
  strncpy(ret_buffer, buffer, strlen(buffer));

  return ret_buffer;
}

/* Get the Auto-neg string */
char *newport_diag_helper_get_autoneg_string(bf_an_state_e an_state,
                                             char *ret_buffer) {
  char *buffer;

  ret_buffer[0] = '\0';

  switch (an_state) {
    case BF_AN_ST_NONE:
      buffer = "None";
      break;
    case BF_AN_ST_RUNNING:
      buffer = "Runn";
      break;
    case BF_AN_ST_GOOD:
      buffer = "Good";
      break;
    case BF_AN_ST_FAILED:
      buffer = "Fail";
      break;
    case BF_AN_ST_RESTARTED:
      buffer = "Restd";
      break;
    case BF_AN_ST_COMPLETE:
      buffer = "Done";
      break;
    default:
      buffer = "None";
      break;
  }

  strncpy(ret_buffer, buffer, strlen(buffer));

  return ret_buffer;
}

/* Get the qsfp code string */
char *newport_diag_helper_get_qsfp_code_string(Ethernet_compliance qsfp_code,
                                               char *ret_buffer) {
  char *buffer;

  ret_buffer[0] = '\0';

  switch (qsfp_code) {
    case ACTIVE_CABLE:
      buffer = "Cable";
      break;
    case LR4_40GBASE:
      buffer = "LR4_40G";
      break;
    case SR4_40GBASE:
      buffer = "SR4_40G";
      break;
    case CR4_40GBASE:
      buffer = "CR4_40G";
      break;
    case SR_10GBASE:
      buffer = "SR_10G";
      break;
    case LR_10GBASE:
      buffer = "LR_10G";
      break;
    case LRM_10GBASE:
      buffer = "LRM_10G";
      break;
    case COMPLIANCE_RSVD:
      buffer = "Rsvd";
      break;
    default:
      buffer = "None";
      break;
  }

  strncpy(ret_buffer, buffer, strlen(buffer));

  return ret_buffer;
}
/* Get the qsfp code string */
char *newport_diag_helper_get_qsfp_ext_code_string(
    Ethernet_extended_compliance qsfp_code, char *ret_buffer) {
  char *buffer;

  ret_buffer[0] = '\0';

  switch (qsfp_code) {
    case AOC_100G_BER_5:
    case AOC_100G_BER_12:
      buffer = "AOC_100/25G";
      break;
    case ACC_100G_BER_5:
    case ACC_100G_BER_12:
      buffer = "ACC_100/25G";
      break;
    case SR4_100GBASE:
      buffer = "SR_100/25G";
      break;
    case LR4_100GBASE:
      buffer = "LR_100/25G";
      break;
    case CR4_100GBASE:
      buffer = "CR4_100G";
      break;
    case ER4_100GBASE:
      buffer = "ER_100/25G";
      break;
    case SR10_100GBASE:
      buffer = "SR10_100G";
      break;
    case CWDM4_100G:
      buffer = "CWDM4_100G";
      break;
    case CLR4_100G:
      buffer = "CLR4_100G";
      break;
    case ER4_40GBASE:
      buffer = "ER4_40G";
      break;
    case CR_25GBASE_CA_S:
    case CR_25GBASE_CA_N:
      buffer = "CR_25G";
      break;
    case SR_10GBASE_4:
      buffer = "SR_10G_4";
      break;
    default:
      buffer = "Other";
      break;
  }

  strncpy(ret_buffer, buffer, strlen(buffer));

  return ret_buffer;
}

/* Get the Port info string */
char *newport_diag_helper_get_port_string(bf_pal_front_port_handle_t *port_info,
                                          char *ret_buffer,
                                          int max_len) {
  ret_buffer[0] = '\0';

  snprintf(
      ret_buffer, max_len, "%d/%d", port_info->conn_id, port_info->chnl_id);

  return ret_buffer;
}

int newport_diag_get_num_active_pipes(bf_dev_id_t dev_id) {
  uint32_t num_pipes = 0;

  lld_sku_get_num_active_pipes(dev_id, &num_pipes);
  return num_pipes;
}

/* Save the linespeed init command */
bf_status_t newport_diag_save_linespeed_init_cmd(
    bf_dev_id_t dev_id,
    bf_diag_sess_hdl_t sess_hdl,
    int start_port,
    int num_ports,
    bf_diag_port_lpbk_mode_e loop_mode,
    char *loop_mode_str) {
  NEWPORT_DIAG_PORT_STATS_MEMSET;
  NEWPORT_DIAG_LINESPEED_MEMSET;
  NEWPORT_DIAG_GET_LINESPEED_PARAMS.dev_id = dev_id;
  NEWPORT_DIAG_GET_LINESPEED_PARAMS.sess_hdl = sess_hdl;
  NEWPORT_DIAG_GET_LINESPEED_PARAMS.start_port = start_port;
  NEWPORT_DIAG_GET_LINESPEED_PARAMS.num_ports = num_ports;
  NEWPORT_DIAG_GET_LINESPEED_PARAMS.loop_mode = loop_mode;
  strncpy(NEWPORT_DIAG_GET_LINESPEED_PARAMS.loop_mode_str,
          loop_mode_str,
          strlen(loop_mode_str));
  NEWPORT_DIAG_GET_LINESPEED_PARAMS.valid = true;

  return BF_SUCCESS;
}

/* Set linespeed */
bf_status_t newport_diag_service_linespeed_init(bf_dev_id_t dev_id,
                                                int start_port,
                                                int num_ports,
                                                char *type) {
  bf_diag_port_lpbk_mode_e loop_mode = BF_DIAG_PORT_LPBK_NONE;
  diag_front_port_t front_port = 0;
  bf_dev_port_t port_arr[NEWPORT_DIAG_MAX_PORTS];
  int port_index = 0;
  bf_pal_front_port_handle_t port_info;
  bf_diag_sess_hdl_t sess_hdl = 0;
  bf_status_t status = BF_SUCCESS;
  bf_dev_id_t dev_id_of_port = 0;

  /* Cleanup old config */
  if (NEWPORT_DIAG_GET_LINESPEED_PARAMS.valid) {
    newport_diag_service_linespeed_end(dev_id);
    NEWPORT_DIAG_LINESPEED_MEMSET;
    NEWPORT_DIAG_PORT_STATS_MEMSET;
  }

  memset(&port_arr[0], 0, sizeof(port_arr));
  memset(&port_info, 0, sizeof(port_info));

  newport_diag_get_loopback_mode_from_str(type, &loop_mode);
  // NEWPORT_DIAG_PRINT("Start Port %d, Num-ports %d \n", start_port,
  // num_ports);

  for (front_port = start_port; front_port < (start_port + num_ports);
       front_port++) {
    port_info.conn_id = front_port;
    status = bf_pm_port_front_panel_port_to_dev_port_get(
        &port_info, &dev_id_of_port, &port_arr[port_index]);
    // NEWPORT_DIAG_PRINT(" FRONT %d , DEV-port %d \n", front_port,
    // port_arr[port_index]);
    if (status != BF_PLTFM_SUCCESS) {
      NEWPORT_DIAG_PRINT(
          " Failed to get dev-port for front-port %d, "
          "Skipping port %d \n",
          front_port,
          front_port);
      continue;
    }
    port_index++;
  }

  status = bf_diag_loopback_pair_test_setup(
      dev_id, &port_arr[0], port_index, loop_mode, &sess_hdl);
  if (status == BF_SUCCESS) {
    /* Save the command */
    newport_diag_save_linespeed_init_cmd(
        dev_id, sess_hdl, start_port, num_ports, loop_mode, type);
  }
  return status;
}

/* Get the mac counters */
bf_status_t newport_diag_linespeed_counter_get(int interval) {
  bf_status_t status = BF_SUCCESS;
  diag_front_port_t front_port = 0;
  bf_dev_port_t dev_port = 0;
  unsigned int pipe, port;
  uint32_t num_pipes = 0;
  bf_pal_front_port_handle_t port_info;
  bf_rmon_counter_array_t counters;
  bf_dev_id_t dev_id = 0;
  bf_dev_id_t dev_id_of_port = 0;

  if (interval >= NEWPORT_DIAG_STATS_MAX_INTERVALS) {
    return BF_INVALID_ARG;
  }

  memset(&port_info, 0, sizeof(port_info));

  dev_id = NEWPORT_DIAG_GET_LINESPEED_PARAMS.dev_id;
  num_pipes = newport_diag_get_num_active_pipes(dev_id);
  for (front_port = NEWPORT_DIAG_GET_LINESPEED_PARAMS.start_port;
       front_port < (NEWPORT_DIAG_GET_LINESPEED_PARAMS.start_port +
                     NEWPORT_DIAG_GET_LINESPEED_PARAMS.num_ports);
       front_port++) {
    port_info.conn_id = front_port;
    status = bf_pm_port_front_panel_port_to_dev_port_get(
        &port_info, &dev_id_of_port, &dev_port);
    if (status != BF_PLTFM_SUCCESS) {
      continue;
    }
    pipe = DEV_PORT_TO_PIPE(dev_port);
    port = DEV_PORT_TO_LOCAL_PORT(dev_port);

    if ((pipe >= num_pipes) || (port >= BF_PIPE_PORT_COUNT)) {
      continue;
    }

    memset(&counters, 0, sizeof(counters));
    bf_pm_port_all_stats_get(dev_id, &port_info, (uint64_t *)&counters);
    NEWPORT_DIAG_GET_PORT_STATS(front_port, interval).rx_bytes =
        counters.format.ctr_ids.OctetsReceived;
    NEWPORT_DIAG_GET_PORT_STATS(front_port, interval).rx_pkts =
        counters.format.ctr_ids.FramesReceivedAll;
    NEWPORT_DIAG_GET_PORT_STATS(front_port, interval).rx_err_pkts =
        counters.format.ctr_ids.FrameswithanyError;
    NEWPORT_DIAG_GET_PORT_STATS(front_port, interval).rx_fcs_err_pkts =
        counters.format.ctr_ids.FramesReceivedwithFCSError;
    NEWPORT_DIAG_GET_PORT_STATS(front_port, interval).tx_pkts =
        counters.format.ctr_ids.FramesTransmittedAll;
    NEWPORT_DIAG_GET_PORT_STATS(front_port, interval).tx_err_pkts =
        counters.format.ctr_ids.FramesTransmittedwithError;
  }

  return BF_SUCCESS;
}

/* Run the linespeed packet test */
bf_status_t newport_diag_service_linespeed_run(int time,
                                               int pkt_size,
                                               int num_packet) {
  bf_status_t status = BF_SUCCESS;
  bool bidir = false;

  status = bf_diag_loopback_pair_test_start(
      NEWPORT_DIAG_GET_LINESPEED_PARAMS.sess_hdl, num_packet, pkt_size, bidir);

  if (status == BF_SUCCESS) {
    NEWPORT_DIAG_GET_LINESPEED_PARAMS.time = time;
    NEWPORT_DIAG_GET_LINESPEED_PARAMS.pkt_size = pkt_size;
    NEWPORT_DIAG_GET_LINESPEED_PARAMS.num_packet = num_packet;

    NEWPORT_DIAG_PORT_STATS_MEMSET;
    /* Sleep for sometime to make sure pkts have been injected */
    bf_sys_usleep(1500000);

    /* Cache mac stats at start of test */
    newport_diag_linespeed_counter_get(0);

    /* Time (in seconds) here is the runtime for the test,
       Sleep for that time
     */
    bf_sys_sleep(time);
    /* Get the mac stats after the test */
    newport_diag_linespeed_counter_get(1);
  }

  return status;
}

/* Stop the linespeed packet test */
bf_status_t newport_diag_service_linespeed_stop() {
  return bf_diag_loopback_pair_test_stop(
      NEWPORT_DIAG_GET_LINESPEED_PARAMS.sess_hdl);
}

/* End the linespeed packet test */
bf_status_t newport_diag_service_linespeed_end() {
  bf_status_t status = BF_SUCCESS;

  /* Cleanup the loopback test  */
  status = bf_diag_loopback_pair_test_cleanup(
      NEWPORT_DIAG_GET_LINESPEED_PARAMS.sess_hdl);
  if (status == BF_SUCCESS) {
    NEWPORT_DIAG_LINESPEED_MEMSET;
    NEWPORT_DIAG_PORT_STATS_MEMSET;
  }
  return status;
}

/* Show the linespeed packet test results */
bf_status_t newport_diag_service_linespeed_show(char *resp_str,
                                                int max_str_len) {
  int c_len = strlen(resp_str), port_cnt = 0;
  diag_front_port_t front_port = 0;
  bf_dev_port_t dev_port = 0;
  unsigned int pipe, port;
  uint32_t num_pipes = 0, mbps = 0;
  bf_port_speed_t speed = 0;
  bf_fec_type_t fec = 0;
  bool state = false;
  char speed_str[200], port_info_str[200];
  bf_pal_front_port_handle_t port_info;
  bf_status_t status = BF_SUCCESS, ret_status = BF_SUCCESS;
  bf_diag_test_status_e test_status;
  bf_diag_loopback_pair_test_stats_t cpu_stats;
  uint64_t rx_pkts = 0, rx_err_pkts = 0, tx_pkts = 0, tx_err_pkts = 0;
  uint64_t rx_fcs_err_pkts = 0;
  int preamble = 8;
  double per = 0;
  bf_dev_id_t dev_id = 0;
  bf_dev_id_t dev_id_of_port = 0;

  (void)port_cnt;
  memset(speed_str, 0, sizeof(speed_str));
  memset(&port_info, 0, sizeof(port_info));
  memset(&port_info_str, 0, sizeof(port_info_str));
  memset(&cpu_stats, 0, sizeof(cpu_stats));

  c_len += snprintf(
      resp_str + c_len,
      (c_len < max_str_len) ? (max_str_len - c_len - 1) : 0,
      "Port     Rx-Pkt(Err)    Tx-Pkt(Err)  Mbps     Status        PER\n");
  c_len += snprintf(
      resp_str + c_len,
      (c_len < max_str_len) ? (max_str_len - c_len - 1) : 0,
      "---------------------------------------------------------------\n");
  if (!NEWPORT_DIAG_GET_LINESPEED_PARAMS.valid) {
    NEWPORT_DIAG_PRINT("Run linespeed init to specify ports first \n");
    return BF_INVALID_ARG;
  }
  dev_id = NEWPORT_DIAG_GET_LINESPEED_PARAMS.dev_id;
  /* Get the test status */
  test_status = bf_diag_loopback_pair_test_status_get(
      NEWPORT_DIAG_GET_LINESPEED_PARAMS.sess_hdl, &cpu_stats);
  if (test_status == BF_DIAG_TEST_STATUS_PASS) {
    ret_status = 0;
  } else {
    ret_status = 1;
  }
  num_pipes = newport_diag_get_num_active_pipes(dev_id);
  for (front_port = NEWPORT_DIAG_GET_LINESPEED_PARAMS.start_port;
       front_port < (NEWPORT_DIAG_GET_LINESPEED_PARAMS.start_port +
                     NEWPORT_DIAG_GET_LINESPEED_PARAMS.num_ports);
       front_port++, port_cnt++) {
    port_info.conn_id = front_port;
    status = bf_pm_port_front_panel_port_to_dev_port_get(
        &port_info, &dev_id_of_port, &dev_port);
    if (status != BF_PLTFM_SUCCESS) {
      continue;
    }
    pipe = DEV_PORT_TO_PIPE(dev_port);
    port = DEV_PORT_TO_LOCAL_PORT(dev_port);

    if ((pipe >= num_pipes) || (port >= BF_PIPE_PORT_COUNT)) {
      continue;
    }
    /* Get oper state and speed */
    bf_pm_port_oper_status_get(dev_id, &port_info, &state);
    bf_pm_port_speed_get(dev_id, &port_info, &speed);
    bf_pm_port_fec_get(dev_id, &port_info, &fec);
    /* Convert to strings */
    newport_diag_helper_get_speed_string(speed, speed_str);
    newport_diag_helper_get_port_string(
        &port_info, port_info_str, sizeof(port_info_str));

#ifdef DEVICE_IS_SW_MODEL
    rx_pkts = (port_cnt % 2) ? cpu_stats.pairs[port_cnt / 2].rx_good : 0;
    rx_err_pkts = (port_cnt % 2) ? cpu_stats.pairs[port_cnt / 2].rx_bad : 0;
    rx_fcs_err_pkts = (port_cnt % 2) ? cpu_stats.pairs[port_cnt / 2].rx_bad : 0;
    tx_pkts = (port_cnt % 2) ? 0 : cpu_stats.pairs[port_cnt / 2].tx_total;
    tx_err_pkts = 0;
#else
    rx_pkts = NEWPORT_DIAG_GET_PORT_STATS(front_port, 1).rx_pkts -
              NEWPORT_DIAG_GET_PORT_STATS(front_port, 0).rx_pkts;
    rx_err_pkts = NEWPORT_DIAG_GET_PORT_STATS(front_port, 1).rx_err_pkts -
                  NEWPORT_DIAG_GET_PORT_STATS(front_port, 0).rx_err_pkts;
    rx_fcs_err_pkts =
        NEWPORT_DIAG_GET_PORT_STATS(front_port, 1).rx_fcs_err_pkts -
        NEWPORT_DIAG_GET_PORT_STATS(front_port, 0).rx_fcs_err_pkts;
    tx_pkts = NEWPORT_DIAG_GET_PORT_STATS(front_port, 1).tx_pkts -
              NEWPORT_DIAG_GET_PORT_STATS(front_port, 0).tx_pkts;
    tx_err_pkts = NEWPORT_DIAG_GET_PORT_STATS(front_port, 1).tx_err_pkts -
                  NEWPORT_DIAG_GET_PORT_STATS(front_port, 0).tx_err_pkts;
#endif

    /* Calculate packet speed */
    if (NEWPORT_DIAG_GET_LINESPEED_PARAMS.time) {
      /* convert to mega bits/s */
      mbps = ((rx_pkts) *
              (NEWPORT_DIAG_GET_LINESPEED_PARAMS.pkt_size + preamble) * 8) /
             (NEWPORT_DIAG_GET_LINESPEED_PARAMS.time * 1000000);
    } else {
      mbps = 0;
    }
    /* Calculate PER */
    if (rx_pkts) {
      per = (double)rx_fcs_err_pkts / rx_pkts;
    } else {
      per = 0;
    }

    c_len += snprintf(resp_str + c_len,
                      (c_len < max_str_len) ? (max_str_len - c_len - 1) : 0,
                      "%-5s %11" PRIu64 "(%" PRIu64 ") %11" PRIu64 "(%" PRIu64
                      ") %5u %-2s %-4s %-2s %10.8g\n",
                      port_info_str,
                      rx_pkts,
                      rx_err_pkts,
                      tx_pkts,
                      tx_err_pkts,
                      mbps,
                      (state) ? "up" : "dn",
                      speed_str,
                      "FD",
                      per);
  }
  c_len += snprintf(resp_str + c_len,
                    (c_len < max_str_len) ? (max_str_len - c_len - 1) : 0,
                    "Loopback mode = %s \n"
                    "Test Period = %" PRIu64
                    " seconds\n"
                    "Packet length = %d bytes\n"
                    "Num of Packets = %d \n"
                    "Test Result = %s\n",
                    NEWPORT_DIAG_GET_LINESPEED_PARAMS.loop_mode_str,
                    NEWPORT_DIAG_GET_LINESPEED_PARAMS.time,
                    NEWPORT_DIAG_GET_LINESPEED_PARAMS.pkt_size,
                    NEWPORT_DIAG_GET_LINESPEED_PARAMS.num_packet,
                    ret_status ? "FAIL" : "PASS");

  /* Always print CPU port stats, useful for debugging */
  if (1) {
    uint32_t i = 0;
    bf_dev_port_t port1 = 0, port2 = 0;
    bf_pal_front_port_handle_t port_info1, port_info2;
    char port_info1_str[200], port_info2_str[200];

    c_len += snprintf(resp_str + c_len,
                      (c_len < max_str_len) ? (max_str_len - c_len - 1) : 0,
                      "\n---CPU Port Stats--- \n");

    c_len += snprintf(resp_str + c_len,
                      (c_len < max_str_len) ? (max_str_len - c_len - 1) : 0,
                      "Port1  Port2  Tx-Total  Rx-Total  Rx-good  Rx-bad\n");
    c_len += snprintf(resp_str + c_len,
                      (c_len < max_str_len) ? (max_str_len - c_len - 1) : 0,
                      "--------------------------------------------------\n");
    for (i = 0; i < cpu_stats.num_pairs; i++) {
      port1 = cpu_stats.pairs[i].port1;
      port2 = cpu_stats.pairs[i].port2;
      status = bf_pm_port_dev_port_to_front_panel_port_get(
          dev_id, port1, &port_info1);
      if (status != BF_PLTFM_SUCCESS) {
        NEWPORT_DIAG_PRINT(" Failed to get front-port from dev-port %d \n",
                           port1);
        continue;
      }
      status = bf_pm_port_dev_port_to_front_panel_port_get(
          dev_id, port2, &port_info2);
      if (status != BF_PLTFM_SUCCESS) {
        NEWPORT_DIAG_PRINT(" Failed to get front-port from dev-port %d \n",
                           port2);
        continue;
      }
      newport_diag_helper_get_port_string(
          &port_info1, port_info1_str, sizeof(port_info1_str));
      newport_diag_helper_get_port_string(
          &port_info2, port_info2_str, sizeof(port_info2_str));

      c_len += snprintf(resp_str + c_len,
                        (c_len < max_str_len) ? (max_str_len - c_len - 1) : 0,
                        "%-5s  %-5s  %-8d  %-8d  %-7d  %-6d\n",
                        port_info1_str,
                        port_info2_str,
                        cpu_stats.pairs[i].tx_total,
                        cpu_stats.pairs[i].rx_total,
                        cpu_stats.pairs[i].rx_good,
                        cpu_stats.pairs[i].rx_bad);
    }
  }
  /* Return the status */
  return ret_status;
}

/* Port-Status show */
bf_status_t newport_diag_service_port_status_show(
    bf_dev_id_t dev_id,
    diag_front_port_t input_front_port,
    char *resp_str,
    int max_str_len) {
  int lanes = 0, num_ports = 0;
  diag_front_port_t front_port = 0;
  bf_dev_port_t dev_port = 0;
  bf_port_speed_t speed = 0;
  bf_fec_type_t fec = 0;
  bf_an_state_e an_state = 0;
  bool tx_pause = false, rx_pause = false, in_loop = false, state = false;
  uint32_t tx_mtu = 0, rx_mtu = 0;
  char speed_str[200], autoneg_str[200], qsfp_code_str[200];
  char qsfp_ext_code_str[32], fec_str[200], encoding_str[200];
  char port_info_str[200], pause_str[200];
  int tx_pre2 = 0, tx_pre1 = 0, tx_main = 0, tx_post1 = 0, tx_post2 = 0;
  int c_len = strlen(resp_str);
  bf_diag_port_lpbk_mode_e diag_loop_mode;
  Ethernet_compliance qsfp_code;
  Ethernet_extended_compliance qsfp_ext_code;
  bf_pal_front_port_handle_t port_info;
  bf_serdes_encoding_mode_t enc_mode = 0;
  bf_status_t status = BF_SUCCESS;
  bf_dev_id_t dev_id_of_port = 0;

  memset(speed_str, 0, sizeof(speed_str));
  memset(fec_str, 0, sizeof(fec_str));
  memset(autoneg_str, 0, sizeof(autoneg_str));
  memset(qsfp_code_str, 0, sizeof(qsfp_code_str));
  memset(qsfp_ext_code_str, 0, sizeof(qsfp_ext_code_str));
  memset(&pause_str, 0, sizeof(pause_str));
  memset(&port_info, 0, sizeof(port_info));
  memset(&port_info_str, 0, sizeof(port_info_str));
  memset(&encoding_str, 0, sizeof(encoding_str));

  c_len += snprintf(resp_str + c_len,
                    (c_len < max_str_len) ? (max_str_len - c_len - 1) : 0,
                    "Port  link lns speed dlpx a-neg pause fec  encod pre2 "
                    "pre1 main post1 post2 mtu   loop Intf    Ext-intf  \n");
  c_len += snprintf(resp_str + c_len,
                    (c_len < max_str_len) ? (max_str_len - c_len - 1) : 0,
                    "------------------------------------------------------"
                    "----------------------------------------------------\n");

  num_ports = bf_bd_cfg_bd_num_port_get();
  for (front_port = 1; front_port <= num_ports; front_port++) {
    if ((input_front_port != -1) && (input_front_port != front_port)) {
      continue;
    }
    port_info.conn_id = front_port;
    status = bf_pm_port_front_panel_port_to_dev_port_get(
        &port_info, &dev_id_of_port, &dev_port);
    if (status != BF_PLTFM_SUCCESS) {
      continue;
    }
    /* Make sure port exists */
    if (bf_port_is_valid(dev_id, dev_port) != BF_SUCCESS) {
      continue;
    }
    memset(speed_str, 0, sizeof(speed_str));
    memset(fec_str, 0, sizeof(fec_str));
    memset(autoneg_str, 0, sizeof(autoneg_str));
    memset(qsfp_code_str, 0, sizeof(qsfp_code_str));
    memset(qsfp_ext_code_str, 0, sizeof(qsfp_ext_code_str));
    memset(&port_info_str, 0, sizeof(port_info_str));
    memset(&encoding_str, 0, sizeof(encoding_str));

    /* Call the get APIs */
    bf_pm_port_oper_status_get(dev_id, &port_info, &state);
    bf_port_num_lanes_get(dev_id, dev_port, &lanes);
    bf_pm_port_speed_get(dev_id, &port_info, &speed);
    bf_pm_port_fec_get(dev_id, &port_info, &fec);
    bf_port_autoneg_state_get(dev_id, dev_port, &an_state);
    bf_port_autoneg_pause_resolution_get(
        dev_id, front_port, &tx_pause, &rx_pause);
    bf_port_mtu_get(dev_id, dev_port, &tx_mtu, &rx_mtu);

    /* Convert to strings */
    newport_diag_helper_get_port_string(
        &port_info, port_info_str, sizeof(port_info_str));
    newport_diag_helper_get_speed_string(speed, speed_str);
    newport_diag_helper_get_fec_string(fec, fec_str);
    newport_diag_helper_get_autoneg_string(an_state, autoneg_str);
    /* Is port in loop */
    bf_diag_port_loopback_mode_get(dev_id, dev_port, &diag_loop_mode);
    in_loop = false;
    if ((diag_loop_mode != BF_DIAG_PORT_LPBK_NONE) &&
        (diag_loop_mode != BF_DIAG_PORT_LPBK_EXT)) {
      in_loop = true;
    }
    /* Get qsfp code */
    qsfp_code = 0;
    bf_qsfp_get_eth_compliance(front_port, &qsfp_code);
    newport_diag_helper_get_qsfp_code_string(qsfp_code, qsfp_code_str);
    qsfp_ext_code = 0;
    bf_qsfp_get_eth_ext_compliance(front_port, &qsfp_ext_code);
    newport_diag_helper_get_qsfp_ext_code_string(qsfp_ext_code,
                                                 qsfp_ext_code_str);
    if ((tx_pause) || (rx_pause)) {
      snprintf(
          pause_str, 200, "%s%s", tx_pause ? "TX" : "", rx_pause ? "RX" : "");
    } else {
      snprintf(pause_str, 200, "None");
    }
    tx_pre2 = 0;
    tx_pre1 = 0;
    tx_main = 0;
    tx_post1 = 0;
    tx_post2 = 0;
    /* Skip CPU port as API cores */
    if (front_port != NEWPORT_DIAG_ETH_CPU_PORT) {
      bf_tof2_serdes_tx_taps_get(dev_id,
                                 dev_port,
                                 0,
                                 &tx_pre2,
                                 &tx_pre1,
                                 &tx_main,
                                 &tx_post1,
                                 &tx_post2);
    }
    /* Get encoding */
    bf_port_encoding_mode_get(dev_id, dev_port, &enc_mode);
    if (enc_mode == BF_SERDES_ENC_MODE_NRZ) {
      snprintf(encoding_str, 200, "NRZ");
    } else if (enc_mode == BF_SERDES_ENC_MODE_PAM4) {
      snprintf(encoding_str, 200, "PAM4");
    } else {
      snprintf(encoding_str, 200, "None");
    }

    c_len += snprintf(resp_str + c_len,
                      (c_len < max_str_len) ? (max_str_len - c_len - 1) : 0,
                      "%-5s %-4s %-3d %-5s %-4s %-5s %-5s %-4s %-5s "
                      "%-4d %-4d %-4d %-5d %-5d %-5d %-4s %-7s %-10s\n",
                      port_info_str,
                      state ? "up" : "down",
                      lanes,
                      speed_str,
                      "FD",
                      autoneg_str,
                      pause_str,
                      fec_str,
                      encoding_str,
                      tx_pre2,
                      tx_pre1,
                      tx_main,
                      tx_post1,
                      tx_post2,
                      tx_mtu,
                      in_loop ? "Yes" : "No",
                      qsfp_code_str,
                      qsfp_ext_code_str);
  }
  c_len += snprintf(resp_str + c_len,
                    (c_len < max_str_len) ? (max_str_len - c_len - 1) : 0,
                    "--------------------------------------------------------"
                    "----------------------------------\n");

  return BF_SUCCESS;
}

/* Show phy info */
bf_status_t newport_diag_service_phy_info_show(bf_dev_id_t dev_id,
                                               char *resp_str,
                                               int max_str_len) {
  int c_len = strlen(resp_str);
  int num_ports = 0;
  diag_front_port_t front_port = 0;
  bf_pal_front_port_handle_t port_info;
  bf_pltfm_port_info_t pltfm_port_info;
  char port_info_str[200], speed_str[200];
  bf_dev_port_t dev_port = 0;
  int lanes = 0, chnl = 0;
  uint32_t phy_mac_blk = 0;
  bf_port_speed_t speed = 0;
  uint32_t tx_lane = 0, rx_lane = 0;
  bf_status_t status = BF_SUCCESS;
  bf_dev_id_t dev_id_of_port = 0;

  memset(&port_info, 0, sizeof(port_info));
  memset(&pltfm_port_info, 0, sizeof(pltfm_port_info));
  memset(&port_info_str, 0, sizeof(port_info_str));
  memset(&speed_str, 0, sizeof(speed_str));

  c_len +=
      snprintf(resp_str + c_len,
               (c_len < max_str_len) ? (max_str_len - c_len - 1) : 0,
               "Port dev-id dev-port phy-mac-blk speed lane tx-lane rx-lane\n");
  c_len +=
      snprintf(resp_str + c_len,
               (c_len < max_str_len) ? (max_str_len - c_len - 1) : 0,
               "----------------------------------------------------------\n");

  num_ports = bf_bd_cfg_bd_num_port_get();
  for (front_port = 1; front_port <= num_ports; front_port++) {
    port_info.conn_id = front_port;
    pltfm_port_info.conn_id = front_port;
    status = bf_pm_port_front_panel_port_to_dev_port_get(
        &port_info, &dev_id_of_port, &dev_port);
    if (status != BF_PLTFM_SUCCESS) {
      continue;
    }
    bf_pm_port_speed_get(dev_id, &port_info, &speed);
    bf_port_num_lanes_get(dev_id, dev_port, &lanes);
    bf_port_map_dev_port_to_mac(dev_id, dev_port, &phy_mac_blk, &chnl);
    bf_bd_cfg_port_tx_phy_lane_get(&pltfm_port_info, &tx_lane);
    bf_bd_cfg_port_rx_phy_lane_get(&pltfm_port_info, &rx_lane);

    /* Convert to strings */
    newport_diag_helper_get_speed_string(speed, speed_str);
    newport_diag_helper_get_port_string(
        &port_info, port_info_str, sizeof(port_info_str));

    c_len += snprintf(resp_str + c_len,
                      (c_len < max_str_len) ? (max_str_len - c_len - 1) : 0,
                      "%-5s %-6d %-8d %-11d %-4s %-4d %-7d %-7d\n",
                      port_info_str,
                      dev_id,
                      dev_port,
                      phy_mac_blk,
                      speed_str,
                      lanes,
                      tx_lane,
                      rx_lane);
  }
  c_len +=
      snprintf(resp_str + c_len,
               (c_len < max_str_len) ? (max_str_len - c_len - 1) : 0,
               "----------------------------------------------------------\n");
  return BF_SUCCESS;
}

/* Show port map */
bf_status_t newport_diag_service_show_pmap(bf_dev_id_t dev_id,
                                           char *resp_str,
                                           int max_str_len) {
  int c_len = strlen(resp_str);
  int num_ports = 0;
  diag_front_port_t front_port = 0;
  bf_dev_port_t dev_port = 0;
  bf_pal_front_port_handle_t port_info;
  char port_info_str[200];
  uint32_t pipe_id = 0, phy_pipe_id = 0;
  bf_status_t status = BF_SUCCESS;
  bf_dev_id_t dev_id_of_port = 0;

  memset(&port_info, 0, sizeof(port_info));
  memset(&port_info_str, 0, sizeof(port_info_str));

  c_len += snprintf(resp_str + c_len,
                    (c_len < max_str_len) ? (max_str_len - c_len - 1) : 0,
                    "Port  dev-port logical-pipe physical-pipe\n");
  c_len += snprintf(resp_str + c_len,
                    (c_len < max_str_len) ? (max_str_len - c_len - 1) : 0,
                    "------------------------------------------\n");

  num_ports = bf_bd_cfg_bd_num_port_get();
  for (front_port = 1; front_port <= num_ports; front_port++) {
    port_info.conn_id = front_port;
    status = bf_pm_port_front_panel_port_to_dev_port_get(
        &port_info, &dev_id_of_port, &dev_port);
    if (status != BF_PLTFM_SUCCESS) {
      continue;
    }
    newport_diag_helper_get_port_string(
        &port_info, port_info_str, sizeof(port_info_str));
    pipe_id = DEV_PORT_TO_PIPE(dev_port);
    lld_sku_map_pipe_id_to_phy_pipe_id(dev_id, pipe_id, &phy_pipe_id);
    c_len += snprintf(resp_str + c_len,
                      (c_len < max_str_len) ? (max_str_len - c_len - 1) : 0,
                      "%-5s %-8d %-12d %-13d \n",
                      port_info_str,
                      dev_port,
                      pipe_id,
                      phy_pipe_id);
  }
  c_len += snprintf(resp_str + c_len,
                    (c_len < max_str_len) ? (max_str_len - c_len - 1) : 0,
                    "------------------------------------------\n");
  return BF_SUCCESS;
}

/* Show temperature */
bf_status_t newport_diag_service_show_temp(bf_dev_id_t dev_id,
                                           char *resp_str,
                                           int max_str_len) {
  int c_len = strlen(resp_str), max_sensors = 6;
  float total_curr = 0, max_peak = 0;
  bf_pltfm_temperature_info_t info, peak;

  (void)dev_id;
  memset(&info, 0, sizeof(info));
  memset(&peak, 0, sizeof(peak));
  /* Call the API */
  bf_pltfm_temperature_get(&info, &peak);

  /* tmp1 */
  c_len += snprintf(resp_str + c_len,
                    (c_len < max_str_len) ? (max_str_len - c_len - 1) : 0,
                    "temperature monitor %d: current=%f, peak=%f\n",
                    1,
                    info.tmp1,
                    peak.tmp1);
  total_curr += info.tmp1;
  if (peak.tmp1 > max_peak) {
    max_peak = peak.tmp1;
  }

  /* tmp2 */
  c_len += snprintf(resp_str + c_len,
                    (c_len < max_str_len) ? (max_str_len - c_len - 1) : 0,
                    "temperature monitor %d: current=%f, peak=%f\n",
                    2,
                    info.tmp2,
                    peak.tmp2);
  total_curr += info.tmp2;
  if (peak.tmp2 > max_peak) {
    max_peak = peak.tmp2;
  }

  /* tmp3 */
  c_len += snprintf(resp_str + c_len,
                    (c_len < max_str_len) ? (max_str_len - c_len - 1) : 0,
                    "temperature monitor %d: current=%f, peak=%f\n",
                    3,
                    info.tmp3,
                    peak.tmp3);
  total_curr += info.tmp3;
  if (peak.tmp3 > max_peak) {
    max_peak = peak.tmp3;
  }

  /* tmp4 */
  c_len += snprintf(resp_str + c_len,
                    (c_len < max_str_len) ? (max_str_len - c_len - 1) : 0,
                    "temperature monitor %d: current=%f, peak=%f\n",
                    4,
                    info.tmp4,
                    peak.tmp4);
  total_curr += info.tmp4;
  if (peak.tmp4 > max_peak) {
    max_peak = peak.tmp4;
  }

  /* tmp5 */
  c_len += snprintf(resp_str + c_len,
                    (c_len < max_str_len) ? (max_str_len - c_len - 1) : 0,
                    "temperature monitor %d: current=%f, peak=%f\n",
                    5,
                    info.tmp5,
                    peak.tmp5);
  total_curr += info.tmp5;
  if (peak.tmp5 > max_peak) {
    max_peak = peak.tmp5;
  }

  /* tmp6 */
  c_len += snprintf(resp_str + c_len,
                    (c_len < max_str_len) ? (max_str_len - c_len - 1) : 0,
                    "temperature monitor %d: current=%f, peak=%f\n",
                    6,
                    info.tmp6,
                    peak.tmp6);
  total_curr += info.tmp6;
  if (peak.tmp6 > max_peak) {
    max_peak = peak.tmp6;
  }

  /* Average temp */
  c_len += snprintf(resp_str + c_len,
                    (c_len < max_str_len) ? (max_str_len - c_len - 1) : 0,
                    "average current temperature= %f\n",
                    total_curr / max_sensors);
  c_len += snprintf(resp_str + c_len,
                    (c_len < max_str_len) ? (max_str_len - c_len - 1) : 0,
                    "maximum peak temperature= %f\n",
                    max_peak);
  return BF_SUCCESS;
}

/* Create vlan */
bf_status_t newport_diag_service_vlan_create(bf_dev_id_t dev_id,
                                             int vlan_id,
                                             int pbm[],
                                             int num_pbm_ports,
                                             int ubm[],
                                             int num_ubm_ports) {
  bf_status_t status = BF_SUCCESS;
  status = bf_diag_vlan_create(dev_id, vlan_id);
  if (status != BF_SUCCESS) {
    return status;
  }
  return newport_diag_service_vlan_add(
      dev_id, vlan_id, pbm, num_pbm_ports, ubm, num_ubm_ports);
}

/* Add ports to vlan */
bf_status_t newport_diag_service_vlan_add(bf_dev_id_t dev_id,
                                          int vlan_id,
                                          int pbm[],
                                          int num_pbm_ports,
                                          int ubm[],
                                          int num_ubm_ports) {
  int index = 0;
  diag_front_port_t port = 0;
  bf_status_t status = BF_SUCCESS;
  bf_pal_front_port_handle_t port_info;
  bf_dev_id_t dev_id_of_port = 0;

  memset(&port_info, 0, sizeof(port_info));

  for (index = 0; index < num_pbm_ports; index++) {
    port_info.conn_id = pbm[index];
    status =
        bf_pm_port_front_panel_port_to_dev_port_get(&port_info, &dev_id_of_port, &port);
    if (status != BF_PLTFM_SUCCESS) {
      NEWPORT_DIAG_PRINT(" Failed to get dev-port for front-port %d \n",
                         pbm[index]);
      continue;
    }
    status |= bf_diag_port_vlan_add(dev_id, port, vlan_id);
  }
  for (index = 0; index < num_ubm_ports; index++) {
    port_info.conn_id = ubm[index];
    status =
        bf_pm_port_front_panel_port_to_dev_port_get(&port_info, &dev_id_of_port, &port);
    if (status != BF_PLTFM_SUCCESS) {
      NEWPORT_DIAG_PRINT(" Failed to get dev-port for front-port %d \n",
                         ubm[index]);
      continue;
    }
    status |= bf_diag_port_default_vlan_set(dev_id, port, vlan_id);
  }
  return status;
}

/* Destroy vlan */
bf_status_t newport_diag_service_vlan_destroy(bf_dev_id_t dev_id, int vlan_id) {
  return bf_diag_vlan_destroy(dev_id, vlan_id);
}

/* Vlan show helper */
bf_status_t newport_diag_vlan_show_helper(bf_dev_id_t dev_id,
                                          int vlan_id,
                                          char *resp_str,
                                          int max_str_len,
                                          int *curr_len) {
  int c_len = *curr_len;
  bf_dev_port_t dev_port = 0, next_port = 0;
  diag_front_port_t front_port = 0;
  bf_status_t status = BF_SUCCESS;
  bool newline_added = false;

  c_len += snprintf(resp_str + c_len,
                    (c_len < max_str_len) ? (max_str_len - c_len - 1) : 0,
                    "%-7d",
                    vlan_id);

  status = bf_diag_vlan_port_get_first(dev_id, vlan_id, &dev_port);
  while (dev_port != -1) {
    bf_pal_front_port_handle_t port_info;
    status = bf_pm_port_dev_port_to_front_panel_port_get(
        dev_id, dev_port, &port_info);
    if (status != BF_PLTFM_SUCCESS) {
      NEWPORT_DIAG_PRINT(" Failed to get front-port from dev-port %d \n",
                         dev_port);
      continue;
    }
    front_port = port_info.conn_id;
    c_len += snprintf(resp_str + c_len,
                      (c_len < max_str_len) ? (max_str_len - c_len - 1) : 0,
                      "%d(%s) ",
                      front_port,
                      "T");

    status =
        bf_diag_vlan_port_get_next(dev_id, vlan_id, dev_port, 1, &next_port);
    dev_port = next_port;
  }

  status = bf_diag_default_vlan_port_get_first(dev_id, vlan_id, &dev_port);
  while (dev_port != -1) {
    bf_pal_front_port_handle_t port_info;
    status = bf_pm_port_dev_port_to_front_panel_port_get(
        dev_id, dev_port, &port_info);
    if (status != BF_PLTFM_SUCCESS) {
      NEWPORT_DIAG_PRINT(" Failed to get front-port from dev-port %d \n",
                         dev_port);
      continue;
    }
    front_port = port_info.conn_id;

    if (!newline_added) {
      c_len += snprintf(resp_str + c_len,
                        (c_len < max_str_len) ? (max_str_len - c_len - 1) : 0,
                        "\n       ");
      newline_added = true;
    }
    c_len += snprintf(resp_str + c_len,
                      (c_len < max_str_len) ? (max_str_len - c_len - 1) : 0,
                      "%d(%s) ",
                      front_port,
                      "U");

    status = bf_diag_default_vlan_port_get_next(
        dev_id, vlan_id, dev_port, 1, &next_port);
    dev_port = next_port;
  }

  c_len += snprintf(resp_str + c_len,
                    (c_len < max_str_len) ? (max_str_len - c_len - 1) : 0,
                    "\n");

  *curr_len = c_len;
  return BF_SUCCESS;
}

/* Show vlan */
bf_status_t newport_diag_service_vlan_show(bf_dev_id_t dev_id,
                                           int input_vlan_id,
                                           char *resp_str,
                                           int max_str_len) {
  int vlan_id = 0, next_vlan_id = 0;
  int c_len = strlen(resp_str);
  bf_status_t status = BF_SUCCESS;

  (void)status;
  c_len += snprintf(resp_str + c_len,
                    (c_len < max_str_len) ? (max_str_len - c_len - 1) : 0,
                    "Vlan   Ports        \n");
  c_len += snprintf(
      resp_str + c_len,
      (c_len < max_str_len) ? (max_str_len - c_len - 1) : 0,
      "------------------------------------------------------------\n");
  /* Dump all vlans */
  if (input_vlan_id != -1) {
    status = newport_diag_vlan_show_helper(
        dev_id, input_vlan_id, resp_str, max_str_len, &c_len);
    if (status == BF_INVALID_ARG) {
      NEWPORT_DIAG_PRINT("Vlan %d does not exist \n", input_vlan_id);
    }
  } else {
    status = bf_diag_vlan_get_first(dev_id, &vlan_id);
    while (vlan_id != -1) {
      newport_diag_vlan_show_helper(
          dev_id, vlan_id, resp_str, max_str_len, &c_len);
      status = bf_diag_vlan_get_next(dev_id, vlan_id, 1, &next_vlan_id);
      vlan_id = next_vlan_id;
    }
  }

  return BF_SUCCESS;
}

/* Run eyescan command  */
bf_status_t newport_diag_service_eyescan(bf_dev_id_t dev_id,
                                         diag_front_port_t front_port,
                                         char *resp_str,
                                         int max_str_len) {
  int c_len = strlen(resp_str);
  bf_dev_port_t dev_port = 0;
  bf_pal_front_port_handle_t port_info;
  bf_status_t status = BF_SUCCESS;
  bf_port_eye_val_t eye_vals;
  int chnl = 0, ln = 0, num_lanes = 0;
  bf_dev_id_t dev_id_of_port = 0;

  memset(&port_info, 0, sizeof(port_info));
  memset(&eye_vals, 0, sizeof(eye_vals));

  c_len += snprintf(resp_str + c_len,
                    (c_len < max_str_len) ? (max_str_len - c_len - 1) : 0,
                    " ******** EyeScan for port %d ******* \n",
                    front_port);

  for (chnl = 0; chnl < NEWPORT_NUM_CHANNELS; chnl++) {
    port_info.conn_id = front_port;
    port_info.chnl_id = chnl;
    status = bf_pm_port_front_panel_port_to_dev_port_get(
        &port_info, &dev_id_of_port, &dev_port);
    if (status != BF_PLTFM_SUCCESS) {
      continue;
    }
    if (bf_port_is_valid(dev_id, dev_port) != BF_SUCCESS) {
      continue;
    }
    bf_pm_port_eye_val_get(dev_id, &port_info, &eye_vals);

    num_lanes = 0;
    bf_port_num_lanes_get(dev_id, dev_port, &num_lanes);
    c_len += snprintf(resp_str + c_len,
                      (c_len < max_str_len) ? (max_str_len - c_len - 1) : 0,
                      " ---------- EyeScan for port %d/%d ----------- \n",
                      front_port,
                      chnl);

    c_len += snprintf(resp_str + c_len,
                      (c_len < max_str_len) ? (max_str_len - c_len - 1) : 0,
                      "Lane  Eye0        Eye1        Eye2       \n");

    for (ln = 0; ln < num_lanes; ln++) {
      c_len += snprintf(resp_str + c_len,
                        (c_len < max_str_len) ? (max_str_len - c_len - 1) : 0,
                        "%-5d %-11f %-11f %-11f\n",
                        ln,
                        eye_vals.eye_val.tof2_channel[ln].eyes_0,
                        eye_vals.eye_val.tof2_channel[ln].eyes_1,
                        eye_vals.eye_val.tof2_channel[ln].eyes_2);
    }
    c_len += snprintf(resp_str + c_len,
                      (c_len < max_str_len) ? (max_str_len - c_len - 1) : 0,
                      "\n");
  }

  return BF_SUCCESS;
}

/* Set LED color */
bf_status_t newport_diag_service_led_set(bf_dev_id_t dev_id,
                                         diag_front_port_t input_front_port,
                                         char *color) {
  int num_ports = 0, chnl = 0;
  bf_led_condition_t led_color = BF_LED_POST_PORT_DEL;
  bf_pltfm_port_info_t port_info;
  diag_front_port_t front_port = 0;

  memset(&port_info, 0, sizeof(port_info));
  if (!color) {
    return BF_INVALID_ARG;
  }

  if (!strncmp(color, "red", strlen("red"))) {
    led_color = BF_LED_PORT_RED;
  } else if (!strncmp(color, "green", strlen("green"))) {
    led_color = BF_LED_PORT_GREEN;
  } else if (!strncmp(color, "blue", strlen("blue"))) {
    led_color = BF_LED_PORT_BLUE;
  } else if (!strncmp(color, "off", strlen("off"))) {
    led_color = BF_LED_POST_PORT_DEL;
  } else {
    return BF_INVALID_ARG;
  }

  if (input_front_port != -1) {
    for (chnl = 0; chnl < NEWPORT_NUM_CHANNELS; chnl++) {
      port_info.conn_id = input_front_port;
      port_info.chnl_id = chnl;
      bf_port_led_set(dev_id, &port_info, led_color);
    }
    return BF_SUCCESS;
  }

  num_ports = bf_bd_cfg_bd_num_port_get();
  for (front_port = 1; front_port <= num_ports; front_port++) {
    for (chnl = 0; chnl < NEWPORT_NUM_CHANNELS; chnl++) {
      port_info.conn_id = front_port;
      port_info.chnl_id = chnl;
      bf_port_led_set(dev_id, &port_info, led_color);
    }
  }

  return BF_SUCCESS;
}

/* Setup snake */
bf_status_t newport_diag_service_snake_init(bf_dev_id_t dev_id,
                                            int start_port,
                                            int num_ports,
                                            char *type) {
  bf_diag_port_lpbk_mode_e loop_mode = BF_DIAG_PORT_LPBK_NONE;
  diag_front_port_t front_port = 0;
  bf_dev_port_t dev_port = 0, port_arr[NEWPORT_DIAG_MAX_PORTS];
  int port_index = 0;
  bf_pal_front_port_handle_t port_info;
  bf_status_t status = BF_SUCCESS;
  bf_dev_id_t dev_id_of_port = 0;

  memset(&port_arr[0], 0, sizeof(port_arr));
  memset(&port_info, 0, sizeof(port_info));

  newport_diag_get_loopback_mode_from_str(type, &loop_mode);

  for (front_port = start_port; front_port < (start_port + num_ports);
       front_port++) {
    port_info.conn_id = front_port;
    status = bf_pm_port_front_panel_port_to_dev_port_get(
        &port_info, &dev_id_of_port, &dev_port);
    if (status != BF_PLTFM_SUCCESS) {
      NEWPORT_DIAG_PRINT(" Failed to get dev-port for front-port %d \n",
                         front_port);
      return status;
    }
    port_arr[port_index] = dev_port;
    port_index++;
  }

  if (diag_snake_sess_hdl != 0) {
    NEWPORT_DIAG_PRINT(
        " Another snake session with hdl %u in progress, cleaning that \n",
        diag_snake_sess_hdl);
    bf_diag_loopback_snake_test_cleanup(diag_snake_sess_hdl);
    diag_snake_sess_hdl = 0;
  }

  return bf_diag_loopback_snake_test_setup(
      dev_id, &port_arr[0], port_index, loop_mode, &diag_snake_sess_hdl);
}

/* Run the snake packet test */
bf_status_t newport_diag_service_snake_run(int pkt_size, int num_packet) {
  bool bidir = false;
  return bf_diag_loopback_snake_test_start(
      diag_snake_sess_hdl, num_packet, pkt_size, bidir);
}

/* Stop the snake packet test */
bf_status_t newport_diag_service_snake_stop() {
  return bf_diag_loopback_snake_test_stop(diag_snake_sess_hdl);
}

/* End the snake packet test */
bf_status_t newport_diag_service_snake_end() {
  bf_status_t sts = BF_SUCCESS;
  sts = bf_diag_loopback_snake_test_cleanup(diag_snake_sess_hdl);
  diag_snake_sess_hdl = 0;
  return sts;
}

/* Snake status */
bf_status_t newport_diag_service_snake_status() {
  bf_diag_port_stats_t stats;
  bf_diag_test_status_e test_status;

  memset(&stats, 0, sizeof(stats));
  test_status =
      bf_diag_loopback_snake_test_status_get(diag_snake_sess_hdl, &stats);
  if (test_status == BF_DIAG_TEST_STATUS_PASS) {
    return 0;
  }
  return 1;
}

/* Get number of lanes based on speed. Lanes are different for PAM4 and NRZ */
static uint32_t get_num_lanes_for_speed(bf_port_speed_t speed_val, bool pam4) {
  uint32_t n_lanes = 0;

  switch (speed_val) {
    case BF_SPEED_400G:
      n_lanes = 8;
      break;
    case BF_SPEED_200G:
      if (pam4) {
        n_lanes = 4;
      } else {
        n_lanes = 8;
      }
      break;
    case BF_SPEED_100G:
      if (pam4) {
        n_lanes = 2;
      } else {
        n_lanes = 4;
      }
      break;
    case BF_SPEED_50G:
      if (pam4) {
        n_lanes = 1;
      } else {
        n_lanes = 2;
      }
      break;
    case BF_SPEED_40G:
      n_lanes = 4;
      break;
    case BF_SPEED_25G:
    case BF_SPEED_10G:
    case BF_SPEED_1G:
    default:
      n_lanes = 1;
      break;
  }

  return n_lanes;
}

static bf_status_t newport_diag_port_add_enable(bf_dev_id_t dev_id,
                                                bf_dev_port_t dev_port,
                                                bf_port_speed_t speed,
                                                bf_fec_type_t fec,
                                                bool pam4) {
  bf_status_t sts = BF_SUCCESS;
  bf_pal_front_port_handle_t port_info;
  bool is_internal = false;

  sts =
      bf_pm_port_dev_port_to_front_panel_port_get(dev_id, dev_port, &port_info);
  if (sts != BF_SUCCESS) {
    LOG_ERROR("ERROR:%d Failed to get port info for dev_id=%d, dev_port=%d\n",
              sts,
              dev_id,
              dev_port);
    return sts;
  }

  sts = bf_pm_is_port_internal(dev_id, &port_info, &is_internal);
  if (sts != BF_SUCCESS) {
    LOG_ERROR(
        "ERROR:%d Failed to get internal port info for dev_id=%d, "
        "dev_port=%d\n",
        sts,
        dev_id,
        dev_port);
    return sts;
  }
  /* Skip internal ports */
  if (is_internal) {
    return BF_SUCCESS;
  }

  uint32_t n_lanes = get_num_lanes_for_speed(speed, pam4);
  sts = bf_pm_port_add_with_lanes(dev_id, &port_info, speed, n_lanes, fec);
  if (sts != BF_SUCCESS) {
    LOG_ERROR("ERROR: port_add failed(%d) for dev_id=%d, dev_port=%d\n",
              sts,
              dev_id,
              dev_port);
  } else {
    /* Enable the port */
    sts = bf_pm_port_enable(dev_id, &port_info);
    if (sts != BF_SUCCESS) {
      LOG_ERROR(
          "ERROR: port_enable failed(%d) for dev_port=%d "
          "port_id=%d\n",
          sts,
          dev_id,
          dev_port);
    }
  }

  return sts;
}

#ifdef DEVICE_IS_SW_MODEL
static bf_status_t newport_diag_port_delete(bf_dev_id_t dev_id,
                                            bf_dev_port_t dev_port) {
  bf_status_t sts = BF_SUCCESS;
  bf_pal_front_port_handle_t port_info;

  sts =
      bf_pm_port_dev_port_to_front_panel_port_get(dev_id, dev_port, &port_info);
  if (sts != BF_SUCCESS) {
    LOG_ERROR("ERROR:%d Failed to get port info for dev_id=%d, dev_port=%d\n",
              sts,
              dev_id,
              dev_port);
    return sts;
  }

  sts = bf_pm_port_delete(dev_id, &port_info);
  if (sts != BF_SUCCESS) {
    LOG_ERROR("ERROR: port_delete failed(%d) for dev_id=%d, dev_port=%d\n",
              sts,
              dev_id,
              dev_port);
  }

  return sts;
}
#endif

/* Ports add */
bf_status_t newport_diag_ports_add(bf_dev_id_t dev_id) {
  unsigned int pipe, port;
  uint32_t num_pipes;
  bf_port_speed_t speed = NEWPORT_DIAG_DEF_PORT_SPEED;
  bf_fec_type_t fec = NEWPORT_DIAG_DEF_PORT_FEC;

#ifdef DEVICE_IS_SW_MODEL
  /* Delete all existing ports (added by default in sw-model) */
  lld_sku_get_num_active_pipes(dev_id, &num_pipes);
  for (pipe = 0; pipe < num_pipes; pipe++) {
    for (port = 0; port < 72; port += 1) {
      newport_diag_port_delete(dev_id, MAKE_DEV_PORT(pipe, port));
    }
  }
#endif

  /* Add all ports in def speed based on chip (for both sw-model and asic) */
  lld_sku_get_num_active_pipes(dev_id, &num_pipes);
  for (pipe = 0; pipe < num_pipes; pipe++) {
    for (port = 8; port < 72; port += 8) {
      newport_diag_port_add_enable(
          dev_id, MAKE_DEV_PORT(pipe, port), speed, fec, true);
    }

/* Do not add the CPU port, bf-diags will add it to send pkts */
#if 0
#if !defined(DEVICE_IS_SW_MODEL)
    /* Add the CPU port on pipe 0 if it has mac */
    if (pipe == 0) {
      bool has_mac = false;
      bf_status_t sts = BF_SUCCESS;
      port = NEWPORT_DIAG_CPU_PORT;
      sts = bf_port_has_mac(dev_id, MAKE_DEV_PORT(pipe, port), &has_mac);
      if (sts != BF_SUCCESS) {
        has_mac = false;
      }
      if (has_mac) {
        newport_diag_port_add_enable(dev_id,
                                     MAKE_DEV_PORT(pipe, port),
                                     NEWPORT_DIAG_DEF_PORT_SPEED_CPU,
                                     BF_FEC_TYP_NONE,
                                     false);
      }
    }
#endif
#endif
  }

  return BF_SUCCESS;
}

/* Set fec */
bf_status_t newport_diag_service_fec(bf_dev_id_t dev_id,
                                     int front_port,
                                     char *mode) {
  bf_status_t status = BF_SUCCESS;
  bf_fec_type_t fec;
  bf_pal_front_port_handle_t port_info;

  if (!mode) {
    return BF_INVALID_ARG;
  }

  memset(&port_info, 0, sizeof(port_info));
  port_info.conn_id = front_port;

  if (!strncmp(mode, "ON", strlen("ON"))) {
    bf_port_speed_t speed = 0;
    bf_pm_port_speed_get(dev_id, &port_info, &speed);
    if ((speed == BF_SPEED_10G) || (speed == BF_SPEED_40G)) {
      /* 10G and 40G only support FC FEC */
      fec = BF_FEC_TYP_FIRECODE;
    } else {
      fec = NEWPORT_DIAG_DEF_PORT_FEC;
    }
  } else if (!strncmp(mode, "OFF", strlen("OFF"))) {
    fec = BF_FEC_TYP_NONE;
  } else {
    return BF_INVALID_ARG;
  }

  status = bf_pm_port_fec_set(dev_id, &port_info, fec);

  return status;
}

/* Set port speed */
bf_status_t newport_diag_service_port_speed(bf_dev_id_t dev_id,
                                            int front_port,
                                            char *speed) {
  bf_status_t status = BF_SUCCESS;
  bf_port_speed_t speed_val = 0;
  bf_pal_front_port_handle_t port_info;

  if (!speed) {
    return BF_INVALID_ARG;
  }

  if (!strncmp(speed, "400000", strlen("400000"))) {
    speed_val = BF_SPEED_400G;
  } else if (!strncmp(speed, "200000", strlen("200000"))) {
    speed_val = BF_SPEED_200G;
  } else if (!strncmp(speed, "100000", strlen("100000"))) {
    speed_val = BF_SPEED_100G;
  } else if (!strncmp(speed, "50000", strlen("50000"))) {
    speed_val = BF_SPEED_50G;
  } else if (!strncmp(speed, "40000", strlen("40000"))) {
    speed_val = BF_SPEED_40G;
  } else if (!strncmp(speed, "25000", strlen("25000"))) {
    speed_val = BF_SPEED_25G;
  } else if (!strncmp(speed, "10000", strlen("10000"))) {
    speed_val = BF_SPEED_10G;
  } else if (!strncmp(speed, "1000", strlen("1000"))) {
    speed_val = BF_SPEED_1G;
  } else {
    return BF_INVALID_ARG;
  }

  memset(&port_info, 0, sizeof(port_info));
  port_info.conn_id = front_port;

  /* Delete the port */
  status = bf_pm_port_delete(dev_id, &port_info);
  if (status != BF_SUCCESS) {
    LOG_ERROR("ERROR: port_del failed(%d) for dev_id=%d, front_port=%d\n",
              status,
              dev_id,
              front_port);
  }

  /* Add the port back with the new speed */
  uint32_t n_lanes = get_num_lanes_for_speed(speed_val, true);
  bf_fec_type_t fec = NEWPORT_DIAG_DEF_PORT_FEC;
  /* RS FEC Is default for speed 400G and 200G only */
  if ((speed_val != BF_SPEED_400G) && (speed_val != BF_SPEED_200G)) {
    fec = BF_FEC_TYP_NONE;
  }

  status =
      bf_pm_port_add_with_lanes(dev_id, &port_info, speed_val, n_lanes, fec);
  if (status != BF_SUCCESS) {
    LOG_ERROR("ERROR: port_add failed(%d) for dev_id=%d, front_port=%d\n",
              status,
              dev_id,
              front_port);
  } else {
    /* Enable the port */
    status = bf_pm_port_enable(dev_id, &port_info);
    if (status != BF_SUCCESS) {
      LOG_ERROR(
          "ERROR: port_enable failed(%d) for dev_port=%d "
          "front_port=%d\n",
          status,
          dev_id,
          front_port);
    }
  }

  return status;
}

/* Set port autoneg */
bf_status_t newport_diag_service_port_autoneg(bf_dev_id_t dev_id,
                                              int front_port,
                                              char *autoneg) {
  bf_status_t status = BF_SUCCESS;
  bf_pm_port_autoneg_policy_e autoneg_val = PM_AN_DEFAULT;
  bf_pal_front_port_handle_t port_info;

  if (!autoneg) {
    return BF_INVALID_ARG;
  }

  if (!strncmp(autoneg, "DEFAULT", strlen("DEFAULT"))) {
    autoneg_val = PM_AN_DEFAULT;
  } else if (!strncmp(autoneg, "ON", strlen("ON"))) {
    autoneg_val = PM_AN_FORCE_ENABLE;
  } else if (!strncmp(autoneg, "OFF", strlen("OFF"))) {
    autoneg_val = PM_AN_FORCE_DISABLE;
  } else {
    return BF_INVALID_ARG;
  }

  memset(&port_info, 0, sizeof(port_info));
  port_info.conn_id = front_port;

  /* Disable port */
  bf_pm_port_disable(dev_id, &port_info);

  /* Sleep for sometime for changes to take effect */
  bf_sys_usleep(500000);
  /* Apply new autoneg setting */
  status = bf_pm_port_autoneg_set(dev_id, &port_info, autoneg_val);
  if (status != BF_PLTFM_SUCCESS) {
    NEWPORT_DIAG_PRINT(" Failed to set auto-neg for front-port %d \n",
                       front_port);
  }
  bf_sys_usleep(500000);

  /* Enable port */
  bf_pm_port_enable(dev_id, &port_info);

  return status;
}

/* Set port interface */
bf_status_t newport_diag_service_port_intf(bf_dev_id_t dev_id,
                                           int port,
                                           char *intf) {
  if (!intf) {
    return BF_INVALID_ARG;
  }

  (void)dev_id;
  (void)port;

  /* Set port interface not supported */
  return BF_INVALID_ARG;
}

/* Set pre-emphasis */
bf_status_t newport_diag_service_pre_emphasis(bf_dev_id_t dev_id,
                                              int front_port,
                                              int value,
                                              bool is_tx_pre2,
                                              bool is_tx_pre1,
                                              bool is_tx_main,
                                              bool is_tx_post1,
                                              bool is_tx_post2) {
  bf_status_t status = BF_SUCCESS, rc = BF_SUCCESS;
  bf_pal_front_port_handle_t port_info;
  int chnl = 0, ln = 0;
  int num_lanes = 0;
  int32_t tx_pre2 = 0, tx_pre1 = 0, tx_main = 0;
  int32_t tx_post1 = 0, tx_post2 = 0;
  bf_dev_port_t dev_port = 0;
  bf_dev_id_t dev_id_of_port = 0;

  memset(&port_info, 0, sizeof(port_info));

  if ((!is_tx_pre2) && (!is_tx_pre1) && (!is_tx_main) && (!is_tx_post1) &&
      (!is_tx_post2)) {
    return BF_INVALID_ARG;
  }
  /* Apply the setting on all eight channels */
  for (chnl = 0; chnl < NEWPORT_NUM_CHANNELS; chnl++) {
    port_info.conn_id = front_port;
    port_info.chnl_id = chnl;

    status = bf_pm_port_front_panel_port_to_dev_port_get(
        &port_info, &dev_id_of_port, &dev_port);
    if (status != BF_PLTFM_SUCCESS) {
      continue;
    }
    if (bf_port_is_valid(dev_id, dev_port) != BF_SUCCESS) {
      continue;
    }

    num_lanes = 0;
    bf_port_num_lanes_get(dev_id, dev_port, &num_lanes);
    for (ln = 0; ln < num_lanes; ln++) {
      rc = bf_tof2_serdes_tx_taps_get(dev_id,
                                      dev_port,
                                      ln,
                                      &tx_pre2,
                                      &tx_pre1,
                                      &tx_main,
                                      &tx_post1,
                                      &tx_post2);
      if (rc != BF_SUCCESS) {
        NEWPORT_DIAG_PRINT(
            " Failed to get pre-emphasis values for dev-port %d, ln %d \n",
            dev_port,
            ln);
        return rc;
      }

      if (is_tx_pre2) {
        tx_pre2 = value;
      } else if (is_tx_pre1) {
        tx_pre1 = value;
      } else if (is_tx_main) {
        tx_main = value;
      } else if (is_tx_post1) {
        tx_post1 = value;
      } else if (is_tx_post2) {
        tx_post2 = value;
      }

      status |= bf_tof2_serdes_tx_taps_set(dev_id,
                                           dev_port,
                                           ln,
                                           tx_pre2,
                                           tx_pre1,
                                           tx_main,
                                           tx_post1,
                                           tx_post2,
                                           true);
    }
  }

  return status;
}

/* Prbs Set */
bf_status_t newport_diag_service_prbs_set(bf_dev_id_t dev_id,
                                          int front_port,
                                          uint32_t p_value) {
  bf_status_t status = BF_SUCCESS;
  bf_port_prbs_mode_t prbs_mode = 0;
  bf_dev_port_t dev_port = 0;
  bf_pal_front_port_handle_t port_info;
  int chnl = 0;
  bf_pal_front_port_handle_t port_list[NEWPORT_NUM_CHANNELS];
  bf_dev_id_t dev_id_of_port = 0;

  memset(&port_info, 0, sizeof(port_info));
  port_info.conn_id = front_port;
  status = bf_pm_port_front_panel_port_to_dev_port_get(
      &port_info, &dev_id_of_port, &dev_port);
  if (status != BF_PLTFM_SUCCESS) {
    NEWPORT_DIAG_PRINT(" Failed to get dev-port for front-port %d \n",
                       front_port);
    return status;
  }

  /* Get prbs mode from value */
  if (p_value == 0) {
    prbs_mode = BF_PORT_PRBS_MODE_9;
  } else if (p_value == 1) {
    prbs_mode = BF_PORT_PRBS_MODE_13;
  } else if (p_value == 2) {
    prbs_mode = BF_PORT_PRBS_MODE_15;
  } else if (p_value == 3) {
    prbs_mode = BF_PORT_PRBS_MODE_23;
  } else if (p_value == 4) {
    prbs_mode = BF_PORT_PRBS_MODE_31;
  } else {
    NEWPORT_DIAG_PRINT(
        "PRBS set not supported for p=%d (port %d)\n", p_value, front_port);
    return BF_INVALID_ARG;
  }

  /* Disable port */
  for (chnl = 0; chnl < NEWPORT_NUM_CHANNELS; chnl++) {
    port_list[chnl].conn_id = front_port;
    port_list[chnl].chnl_id = chnl;
    bf_pm_port_disable(dev_id, &port_list[chnl]);
  }

  /* Set the prbs mode */
  status = bf_pm_port_prbs_mode_set(dev_id,
                                    &port_list[0],
                                    sizeof(port_list) / sizeof(port_list[0]),
                                    prbs_mode);
  if (status != BF_PLTFM_SUCCESS) {
    NEWPORT_DIAG_PRINT(
        "PRBS set failed for front port %d, status=%d \n", front_port, status);
  }

  /* Enable port */
  for (chnl = 0; chnl < NEWPORT_NUM_CHANNELS; chnl++) {
    port_list[chnl].conn_id = front_port;
    port_list[chnl].chnl_id = chnl;
    bf_pm_port_enable(dev_id, &port_list[chnl]);
  }

  return status;
}

/* Prbs Clean */
bf_status_t newport_diag_service_prbs_clean(bf_dev_id_t dev_id,
                                            int front_port) {
  bf_status_t status = BF_SUCCESS;
  bf_dev_port_t dev_port = 0;
  bf_pal_front_port_handle_t port_info;
  int chnl = 0;
  bf_pal_front_port_handle_t port_list[NEWPORT_NUM_CHANNELS];
  bf_dev_id_t dev_id_of_port = 0;

  memset(&port_info, 0, sizeof(port_info));
  port_info.conn_id = front_port;
  status = bf_pm_port_front_panel_port_to_dev_port_get(
      &port_info, &dev_id_of_port, &dev_port);
  if (status != BF_PLTFM_SUCCESS) {
    NEWPORT_DIAG_PRINT(" Failed to get dev-port for front-port %d \n",
                       front_port);
    return status;
  }

  /* Disable port */
  for (chnl = 0; chnl < NEWPORT_NUM_CHANNELS; chnl++) {
    port_list[chnl].conn_id = front_port;
    port_list[chnl].chnl_id = chnl;
    bf_pm_port_disable(dev_id, &port_list[chnl]);
  }

  /* Cleanup prbs */
  status = bf_pm_port_prbs_mode_clear(
      dev_id, &port_list[0], sizeof(port_list) / sizeof(port_list[0]));
  if (status != BF_SUCCESS) {
    NEWPORT_DIAG_PRINT("PRBS cleanup failed for front port %d, status=%d \n",
                       front_port,
                       status);
  }

  /* Enable port */
  for (chnl = 0; chnl < NEWPORT_NUM_CHANNELS; chnl++) {
    port_list[chnl].conn_id = front_port;
    port_list[chnl].chnl_id = chnl;
    bf_pm_port_enable(dev_id, &port_list[chnl]);
  }

  return status;
}

/* Prbs Show */
bf_status_t newport_diag_service_prbs_show(bf_dev_id_t dev_id,
                                           int front_port,
                                           char *resp_str,
                                           int max_str_len) {
  int c_len = strlen(resp_str);
  bf_status_t status = BF_SUCCESS;
  bf_dev_port_t dev_port = 0;
  int ln = 0, num_lanes = 0, chnl = 0;
  int tx_pre2 = 0, tx_pre1 = 0, tx_main = 0, tx_post1 = 0, tx_post2 = 0;
  bf_port_sds_prbs_stats_t stats;
  bf_pal_front_port_handle_t port_hdl;
  bf_port_eye_val_t eye_vals;
  bf_dev_id_t dev_id_of_port = 0;

  memset(&stats, 0, sizeof(stats));

  c_len += snprintf(resp_str + c_len,
                    (c_len < max_str_len) ? (max_str_len - c_len - 1) : 0,
                    " ***** PRBS for port %d ******* \n",
                    front_port);

  for (chnl = 0; chnl < NEWPORT_NUM_CHANNELS; chnl++) {
    port_hdl.conn_id = front_port;
    port_hdl.chnl_id = chnl;
    status = bf_pm_port_front_panel_port_to_dev_port_get(
        &port_hdl, &dev_id_of_port, &dev_port);
    if (status != BF_PLTFM_SUCCESS) {
      continue;
    }
    if (bf_port_is_valid(dev_id, dev_port) != BF_SUCCESS) {
      continue;
    }

    c_len += snprintf(resp_str + c_len,
                      (c_len < max_str_len) ? (max_str_len - c_len - 1) : 0,
                      " ---------- PRBS for port %d/%d ----------- \n",
                      front_port,
                      chnl);

    c_len += snprintf(resp_str + c_len,
                      (c_len < max_str_len) ? (max_str_len - c_len - 1) : 0,
                      "Lane    Tx-Pre2 Tx-Pre1 Tx-Main Tx-Post1 Tx-Post2 "
                      "Errors      Eye0      Eye1      Eye2     \n");

    /* Get the error count */
    memset(&stats, 0, sizeof(stats));
    bf_pm_port_prbs_mode_stats_get(dev_id, &port_hdl, &stats);

    /* Get the eye value */
    memset(&eye_vals, 0, sizeof(eye_vals));
    bf_pm_port_eye_val_get(dev_id, &port_hdl, &eye_vals);

    num_lanes = 0;
    bf_port_num_lanes_get(dev_id, dev_port, &num_lanes);
    for (ln = 0; ln < num_lanes; ln++) {
      /* Get the pre-emphasis values */
      tx_pre2 = 0;
      tx_pre1 = 0;
      tx_main = 0;
      tx_post1 = 0;
      tx_post2 = 0;

      bf_tof2_serdes_tx_taps_get(dev_id,
                                 dev_port,
                                 ln,
                                 &tx_pre2,
                                 &tx_pre1,
                                 &tx_main,
                                 &tx_post1,
                                 &tx_post2);

      c_len += snprintf(resp_str + c_len,
                        (c_len < max_str_len) ? (max_str_len - c_len - 1) : 0,
                        "%-7d %-7d %-7d %-7d %-8d %-8d %-11d %-9f %-9f %-9f\n",
                        ln,
                        tx_pre2,
                        tx_pre1,
                        tx_main,
                        tx_post1,
                        tx_post2,
                        stats.prbs_stats.tof2_channel[ln].errors,
                        eye_vals.eye_val.tof2_channel[ln].eyes_0,
                        eye_vals.eye_val.tof2_channel[ln].eyes_1,
                        eye_vals.eye_val.tof2_channel[ln].eyes_2);
    }

    c_len += snprintf(resp_str + c_len,
                      (c_len < max_str_len) ? (max_str_len - c_len - 1) : 0,
                      "\n");
  }

  return BF_SUCCESS;
}

/* ber */
bf_status_t newport_diag_service_ber(bf_dev_id_t dev_id,
                                     int front_port,
                                     char *resp_str,
                                     int max_str_len) {
  int c_len = strlen(resp_str);
  bf_pal_front_port_handle_t port_hdl;
  bf_port_ber_t stats;
  int chnl = 0, ln = 0;
  bf_status_t status = BF_PLTFM_SUCCESS;
  bf_dev_port_t dev_port = 0;
  bf_dev_id_t dev_id_of_port = 0;

  c_len += snprintf(resp_str + c_len,
                    (c_len < max_str_len) ? (max_str_len - c_len - 1) : 0,
                    " ****** BER for port %d ******* \n",
                    front_port);

  for (chnl = 0; chnl < NEWPORT_NUM_CHANNELS; chnl++) {
    port_hdl.conn_id = front_port;
    port_hdl.chnl_id = chnl;

    status = bf_pm_port_front_panel_port_to_dev_port_get(
        &port_hdl, &dev_id_of_port, &dev_port);
    if (status != BF_PLTFM_SUCCESS) {
      continue;
    }
    if (bf_port_is_valid(dev_id, dev_port) != BF_SUCCESS) {
      continue;
    }
    memset(&stats, 0, sizeof(stats));
    /* Get the BER value */
    bf_pm_port_ber_get(dev_id, &port_hdl, &stats);

    c_len += snprintf(resp_str + c_len,
                      (c_len < max_str_len) ? (max_str_len - c_len - 1) : 0,
                      " ---------- BER for port %d/%d ----------- \n",
                      front_port,
                      chnl);

    c_len += snprintf(resp_str + c_len,
                      (c_len < max_str_len) ? (max_str_len - c_len - 1) : 0,
                      " Aggregate BER: %f \n",
                      stats.ber.tof2.ber_aggr);

    c_len += snprintf(resp_str + c_len,
                      (c_len < max_str_len) ? (max_str_len - c_len - 1) : 0,
                      " Lane BER         Symbol-Errors\n");
    for (ln = 0; ln < 16; ln++) {
      c_len += snprintf(resp_str + c_len,
                        (c_len < max_str_len) ? (max_str_len - c_len - 1) : 0,
                        " %-4d %-11f %" PRIu64 "\n",
                        ln,
                        stats.ber.tof2.ber_per_lane[ln],
                        stats.ber.tof2.sym_err_ctrs[ln]);
    }
    c_len += snprintf(resp_str + c_len,
                      (c_len < max_str_len) ? (max_str_len - c_len - 1) : 0,
                      "\n");
  }

  return BF_SUCCESS;
}
