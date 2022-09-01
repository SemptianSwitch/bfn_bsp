/*******************************************************************************
 * Copyright (c) 2015-2020 Barefoot Networks, Inc.
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * $Id: $
 *
 ******************************************************************************/
/*!
 * @file diag_server.h
 * @date
 *
 * Contains definitions of diag server
 *
 */
#ifndef _NEWPORT_DIAG_SERVER_H
#define _NEWPORT_DIAG_SERVER_H

#include <stdint.h>
#include "stdbool.h"
#include <bf_types/bf_types.h>
#include <diags/bf_diag_api.h>
#include <tofino/pdfixed/pd_common.h>

/* Module header includes */

#define NEWPORT_DIAG_DEF_DEV_ID 0
#define NEWPORT_DIAG_FILE_RESP_MAX_SIZE 64 * 1024
/* Default TCP port base. Can be overridden by command-line */
#define NEWPORT_DIAG_TCP_PORT_BASE_DEFAULT 17000
#define NEWPORT_DIAG_CLIENT_CMD_MAX_SIZE 256
#define NEWPORT_DIAG_SUBTOKEN_CNT_LIMIT 15
#define NEWPORT_DIAG_SUBTOKEN_STR_SIZE 80
#define NEWPORT_DIAG_CMD_OUT_FILENAME "/tmp/newport_diag_result"
#define NEWPORT_DIAG_PRINT printf
/* include internal ports that start from port 600 */
#define NEWPORT_DIAG_MAX_PORTS 700

typedef int diag_front_port_t;

/* LED colors */
typedef enum newport_diag_led_color_e {
  NEWPORT_DIAG_LED_OFF = 0,
  NEWPORT_DIAG_LED_RED,
  NEWPORT_DIAG_LED_GREEN,
  NEWPORT_DIAG_LED_BLUE,
} newport_diag_led_color_t;

#define NEWPORT_LOOP_MODE_STR_LEN 50
/* Linespeed init command params */
typedef struct newport_diag_linespeed_params_s {
  bf_dev_id_t dev_id;                 /* init arg */
  bf_diag_sess_hdl_t sess_hdl;        /* init arg */
  int start_port;                     /* init arg */
  int num_ports;                      /* init arg */
  bf_diag_port_lpbk_mode_e loop_mode; /* init arg */
  char loop_mode_str[NEWPORT_LOOP_MODE_STR_LEN];
  uint64_t time;     /* run arg */
  uint32_t pkt_size; /* run arg */
  int num_packet;    /* run arg */
  bool valid;
} newport_diag_linespeed_params_t;

/* Linespeed command related info */
typedef struct newport_diag_linespeed_info_s {
  newport_diag_linespeed_params_t params;
} newport_diag_linespeed_info_t;

#define NEWPORT_DIAG_STATS_MAX_INTERVALS 2
typedef struct newport_diag_port_stats_s {
  uint64_t rx_bytes;
  uint64_t rx_pkts;
  uint64_t rx_err_pkts;
  uint64_t rx_fcs_err_pkts;
  uint64_t tx_pkts;
  uint64_t tx_err_pkts;
} newport_diag_port_stats_t;

typedef struct newport_diag_info_s {
  int client_sock;
  int server_sock;
  FILE *resp_filep;
  char file_resp_str[NEWPORT_DIAG_FILE_RESP_MAX_SIZE];
  /* State info */
  newport_diag_linespeed_info_t linespeed; /* linespeed cmd info */
  newport_diag_port_stats_t
      port_stats[NEWPORT_DIAG_MAX_PORTS][NEWPORT_DIAG_STATS_MAX_INTERVALS];
} newport_diag_info_t;

#endif
