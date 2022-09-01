/*******************************************************************************
 * Copyright (c) 2015-2020 Barefoot Networks, Inc.
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * $Id: $
 *
 ******************************************************************************/
/*!
 * @file diag_ucli.c
 * @date
 *
 * Contains implementation of diag ucli
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>  //strlen
#include <pthread.h>
#include <signal.h>
#include <dvm/bf_dma_types.h>
#include <dvm/bf_drv_intf.h>
#include "diag_handler.h"
#include "diag_server.h"
#include <bfutils/uCli/ucli.h>
#include <bfutils/uCli/ucli_argparse.h>
#include <bfutils/uCli/ucli_handler_macros.h>

extern void newport_diag_parse_port_list(char *port_string, bool *port_arr);

static ucli_status_t newport_diags_ucli_ucli__linespeed_init__(
    ucli_context_t *uc) {
  int start_port = 0, num_ports = 0;
  const char *type = NULL;
  bf_dev_id_t dev = 0;
  bf_status_t status = BF_SUCCESS;

  UCLI_COMMAND_INFO(uc,
                    "linespeed-init",
                    4,
                    "linespeed-init <dev> <start_port> <num-ports> <type-str>");

  dev = atoi(uc->pargs->args[0]);
  start_port = atoi(uc->pargs->args[1]);
  num_ports = atoi(uc->pargs->args[2]);
  type = uc->pargs->args[3];

  if (dev >= BF_MAX_DEV_COUNT) {
    aim_printf(&uc->pvs,
               "Only %d chips defined. Correct command \n",
               BF_MAX_DEV_COUNT);
    return 0;
  }

  aim_printf(&uc->pvs,
             "newport_diags:: Linespeed init <dev=%d> <start_port=%d> "
             "<num-ports=%d> <type=%s>\n",
             dev,
             start_port,
             num_ports,
             type);

  status = newport_diag_service_linespeed_init(
      dev, start_port, num_ports, (char *)type);
  if (status == BF_SUCCESS) {
    aim_printf(&uc->pvs, "Linespeed init success \n");
  } else {
    aim_printf(&uc->pvs, "Linespeed init failed \n");
  }

  return 0;
}

static ucli_status_t newport_diags_ucli_ucli__linespeed_run__(
    ucli_context_t *uc) {
  int time = 0, pkt_size = 0, num_pkt = 0;
  bf_status_t status = BF_SUCCESS;

  UCLI_COMMAND_INFO(
      uc, "linespeed-run", 3, "linespeed-run <time> <pkt_size> <num_pkt>");

  time = atoi(uc->pargs->args[0]);
  pkt_size = atoi(uc->pargs->args[1]);
  num_pkt = atoi(uc->pargs->args[2]);

  aim_printf(&uc->pvs,
             "newport_diags:: Linespeed run <time=%d> <pkt-size=%d> "
             "<num-pkt=%d> \n",
             time,
             pkt_size,
             num_pkt);

  status = newport_diag_service_linespeed_run(time, pkt_size, num_pkt);
  if (status == BF_SUCCESS) {
    aim_printf(&uc->pvs, "Linespeed run success \n");
  } else {
    aim_printf(&uc->pvs, "Linespeed run failed \n");
  }

  return 0;
}

static ucli_status_t newport_diags_ucli_ucli__linespeed_stop__(
    ucli_context_t *uc) {
  bf_status_t status = BF_SUCCESS;

  UCLI_COMMAND_INFO(uc, "linespeed-stop", 0, "linespeed-stop");

  aim_printf(&uc->pvs, "newport_diags:: Linespeed stop\n");

  status = newport_diag_service_linespeed_stop();
  if (status == BF_SUCCESS) {
    aim_printf(&uc->pvs, "Linespeed stop success \n");
  } else {
    aim_printf(&uc->pvs, "Linespeed stop failed \n");
  }

  return 0;
}

static ucli_status_t newport_diags_ucli_ucli__linespeed_end__(
    ucli_context_t *uc) {
  bf_status_t status = BF_SUCCESS;

  UCLI_COMMAND_INFO(uc, "linespeed-end", 0, "linespeed-end");

  aim_printf(&uc->pvs, "newport_diags:: Linespeed end\n");

  status = newport_diag_service_linespeed_end();
  if (status == BF_SUCCESS) {
    aim_printf(&uc->pvs, "Linespeed end success \n");
  } else {
    aim_printf(&uc->pvs, "Linespeed end failed \n");
  }

  return 0;
}

static ucli_status_t newport_diags_ucli_ucli__linespeed_show__(
    ucli_context_t *uc) {
  char resp_str[NEWPORT_DIAG_FILE_RESP_MAX_SIZE];

  UCLI_COMMAND_INFO(uc, "linespeed-show", 0, "linespeed-show");

  aim_printf(&uc->pvs, "newport_diags:: Linespeed show \n");

  resp_str[0] = '\0';
  newport_diag_service_linespeed_show(resp_str,
                                      NEWPORT_DIAG_FILE_RESP_MAX_SIZE);
  aim_printf(&uc->pvs, "%s\n", resp_str);

  return 0;
}

static ucli_status_t newport_diags_ucli_ucli__port_status__(
    ucli_context_t *uc) {
  bf_dev_id_t dev = 0;
  bf_dev_port_t dev_port = 0;
  char resp_str[NEWPORT_DIAG_FILE_RESP_MAX_SIZE];

  UCLI_COMMAND_INFO(uc, "port-status", 2, "port-status <dev> <port-num>");

  dev = atoi(uc->pargs->args[0]);
  dev_port = atoi(uc->pargs->args[1]);

  aim_printf(&uc->pvs,
             "newport_diags:: port status <dev=%d> <port-num=%d> \n",
             dev,
             dev_port);

  resp_str[0] = '\0';
  newport_diag_service_port_status_show(
      dev, dev_port, resp_str, NEWPORT_DIAG_FILE_RESP_MAX_SIZE);
  aim_printf(&uc->pvs, "%s\n", resp_str);

  return 0;
}

static ucli_status_t newport_diags_ucli_ucli__port_status_all__(
    ucli_context_t *uc) {
  bf_dev_id_t dev = 0;
  bf_dev_port_t dev_port = -1;
  char resp_str[NEWPORT_DIAG_FILE_RESP_MAX_SIZE];

  UCLI_COMMAND_INFO(uc, "port-status-all", 1, "port-status-all <dev>");

  dev = atoi(uc->pargs->args[0]);

  aim_printf(&uc->pvs, "newport_diags:: port status-all <dev=%d>\n", dev);

  resp_str[0] = '\0';
  newport_diag_service_port_status_show(
      dev, dev_port, resp_str, NEWPORT_DIAG_FILE_RESP_MAX_SIZE);
  aim_printf(&uc->pvs, "%s\n", resp_str);

  return 0;
}

static ucli_status_t newport_diags_ucli_ucli__phy_info__(ucli_context_t *uc) {
  bf_dev_id_t dev = 0;
  char resp_str[NEWPORT_DIAG_FILE_RESP_MAX_SIZE];

  UCLI_COMMAND_INFO(uc, "phy-info", 1, "phy-info <dev> ");

  dev = atoi(uc->pargs->args[0]);

  aim_printf(&uc->pvs, "newport_diags:: phy info <dev=%d> \n", dev);

  resp_str[0] = '\0';
  newport_diag_service_phy_info_show(
      dev, resp_str, NEWPORT_DIAG_FILE_RESP_MAX_SIZE);
  aim_printf(&uc->pvs, "%s\n", resp_str);

  return 0;
}

static ucli_status_t newport_diags_ucli_ucli__port_map__(ucli_context_t *uc) {
  bf_dev_id_t dev = 0;
  char resp_str[NEWPORT_DIAG_FILE_RESP_MAX_SIZE];

  UCLI_COMMAND_INFO(uc, "port-map", 1, "port-map <dev> ");

  dev = atoi(uc->pargs->args[0]);

  aim_printf(&uc->pvs, "newport_diags:: port map <dev=%d> \n", dev);

  resp_str[0] = '\0';
  newport_diag_service_show_pmap(
      dev, resp_str, NEWPORT_DIAG_FILE_RESP_MAX_SIZE);
  aim_printf(&uc->pvs, "%s\n", resp_str);

  return 0;
}

static ucli_status_t newport_diags_ucli_ucli__temp__(ucli_context_t *uc) {
  bf_dev_id_t dev = 0;
  char resp_str[NEWPORT_DIAG_FILE_RESP_MAX_SIZE];

  UCLI_COMMAND_INFO(uc, "temp", 1, "temp <dev> ");

  dev = atoi(uc->pargs->args[0]);

  aim_printf(&uc->pvs, "newport_diags:: temp <dev=%d> \n", dev);

  resp_str[0] = '\0';
  newport_diag_service_show_temp(
      dev, resp_str, NEWPORT_DIAG_FILE_RESP_MAX_SIZE);
  aim_printf(&uc->pvs, "%s\n", resp_str);

  return 0;
}

static ucli_status_t newport_diags_ucli_ucli__eyescan__(ucli_context_t *uc) {
  bf_dev_id_t dev = 0;
  bf_dev_port_t dev_port = 0;
  char resp_str[NEWPORT_DIAG_FILE_RESP_MAX_SIZE];

  UCLI_COMMAND_INFO(uc, "eyescan", 2, "eyescan <dev> <port-num>");

  dev = atoi(uc->pargs->args[0]);
  dev_port = atoi(uc->pargs->args[1]);

  aim_printf(&uc->pvs,
             "newport_diags:: eyescan <dev=%d> <port-num=%d>\n",
             dev,
             dev_port);

  resp_str[0] = '\0';
  newport_diag_service_eyescan(
      dev, dev_port, resp_str, NEWPORT_DIAG_FILE_RESP_MAX_SIZE);
  aim_printf(&uc->pvs, "%s\n", resp_str);

  return 0;
}

static ucli_status_t newport_diags_ucli_ucli__led_set__(ucli_context_t *uc) {
  bf_dev_id_t dev = 0;
  bf_dev_port_t dev_port = -1;
  bf_status_t status = BF_SUCCESS;
  const char *color = NULL;

  UCLI_COMMAND_INFO(uc, "led-set", 3, "led-set <dev> <color-name> <port-num>");

  dev = atoi(uc->pargs->args[0]);
  color = uc->pargs->args[1];
  dev_port = atoi(uc->pargs->args[2]);

  aim_printf(&uc->pvs,
             "newport_diags:: LED set <dev=%d> <color=%s> <port-num=%d> \n",
             dev,
             color,
             dev_port);

  status = newport_diag_service_led_set(dev, dev_port, (char *)color);
  if (status == BF_SUCCESS) {
    aim_printf(&uc->pvs, "Led set success \n");
  } else {
    aim_printf(&uc->pvs, "Led set failed \n");
  }

  return 0;
}

static ucli_status_t newport_diags_ucli_ucli__led_set_all__(
    ucli_context_t *uc) {
  bf_dev_id_t dev = 0;
  bf_dev_port_t dev_port = -1;
  bf_status_t status = BF_SUCCESS;
  const char *color = NULL;

  UCLI_COMMAND_INFO(uc, "led-set-all", 2, "led-set-all <dev> <color-name>");

  dev = atoi(uc->pargs->args[0]);
  color = uc->pargs->args[1];

  aim_printf(&uc->pvs,
             "newport_diags:: LED set all <dev=%d> <color=%s>\n",
             dev,
             color);

  status = newport_diag_service_led_set(dev, dev_port, (char *)color);
  if (status == BF_SUCCESS) {
    aim_printf(&uc->pvs, "Led set success \n");
  } else {
    aim_printf(&uc->pvs, "Led set failed \n");
  }

  return 0;
}

/* <auto.ucli.handlers.start> */
/* <auto.ucli.handlers.end> */
static ucli_command_handler_f newport_diags_ucli_ucli_handlers__[] = {
    newport_diags_ucli_ucli__linespeed_init__,
    newport_diags_ucli_ucli__linespeed_run__,
    newport_diags_ucli_ucli__linespeed_stop__,
    newport_diags_ucli_ucli__linespeed_end__,
    newport_diags_ucli_ucli__linespeed_show__,
    newport_diags_ucli_ucli__port_status__,
    newport_diags_ucli_ucli__port_status_all__,
    newport_diags_ucli_ucli__phy_info__,
    newport_diags_ucli_ucli__port_map__,
    newport_diags_ucli_ucli__temp__,
    newport_diags_ucli_ucli__eyescan__,
    newport_diags_ucli_ucli__led_set__,
    newport_diags_ucli_ucli__led_set_all__,
    NULL};

static ucli_module_t newport_diags_ucli_module__ = {
    "newport_ucli", NULL, newport_diags_ucli_ucli_handlers__, NULL, NULL,
};

ucli_node_t *newport_diags_ucli_node_create(void) {
  ucli_node_t *n;
  ucli_module_init(&newport_diags_ucli_module__);
  n = ucli_node_create("newport", NULL, &newport_diags_ucli_module__);
  ucli_node_subnode_add(n, ucli_module_log_node_create("newport"));
  return n;
}

void newport_diag_register_ucli_node() {
  ucli_node_t *ucli_node = newport_diags_ucli_node_create();
  bf_drv_shell_register_ucli(ucli_node);
}
