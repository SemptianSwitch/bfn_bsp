/*******************************************************************************
 * Copyright (c) 2015-2020 Barefoot Networks, Inc.
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * $Id: $
 *
 ******************************************************************************/
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <unistd.h>
#include <sys/stat.h>
#include <errno.h>
#include <dirent.h>

#include <bf_switchd/bf_switchd.h>
#include <bf_bd_cfg/bf_bd_cfg_intf.h>
#include <bf_pltfm.h>
#include <bf_bd_cfg/bf_bd_cfg_bd_map.h>
#include <bf_pltfm_chss_mgmt_intf.h>
#include <bf_pltfm_bd_cfg.h>
#include "platform_priv.h"

#include <bfutils/cJSON.h>
#include <ctx_json/ctx_json_utils.h>

static pltfm_bd_map_t board_map;

/******************************************************************************
*
******************************************************************************/
pltfm_bd_map_t *platform_pltfm_bd_map_get(int *rows) {
  bf_pltfm_board_id_t bd_id;
  bf_pltfm_status_t sts = bf_pltfm_chss_mgmt_bd_type_get(&bd_id);

  if (sts != BF_PLTFM_SUCCESS) {
    return NULL;
  }
  if (!rows) {
    return NULL;
  }
  *rows = board_map.rows;
  if (board_map.bd_id == bd_id) {
    return &board_map;
  }
  return NULL;
}

int platform_name_get_str(char *name, size_t name_size) {
  bf_pltfm_status_t sts;
  bf_pltfm_board_id_t bd_id;

  sts = bf_pltfm_chss_mgmt_bd_type_get(&bd_id);
  if (sts != BF_PLTFM_SUCCESS) {
    snprintf(name, name_size, "Board type: Error");
    name[name_size - 1] = '\0';
    return sts;
  }
  switch (bd_id) {
    // Better to get display string from JSON - TBD
    case BF_PLTFM_BD_ID_NEWPORT_P0A:
      snprintf(name, name_size, "Board type: NEWPORT_P0A");
      break;
    case BF_PLTFM_BD_ID_NEWPORT_P0B:
      snprintf(name, name_size, "Board type: NEWPORT_P0B");
      break;
    default:
      snprintf(name, name_size, "Board type: Unknown");
      break;
  }
  name[name_size - 1] = '\0';
  return 0;
}

int platform_num_ports_get(void) {
  pltfm_bd_map_t *bd_map;
  int bd_map_rows;

  bd_map = platform_pltfm_bd_map_get(&bd_map_rows);
  if (!bd_map) {
    return 0;
  }
  return board_map.num_of_connectors;
}

// Returns mac_addr from platform
bf_pltfm_status_t platform_port_mac_addr_get(bf_pltfm_port_info_t *port_info,
                                             uint8_t *mac_addr) {
  return bf_pltfm_chss_mgmt_port_mac_addr_get(port_info, mac_addr);
}

#define BD_MAP_LOG(flag, format, ...)                                       \
  if (flag) {                                                               \
    bf_sys_log_and_trace(BF_MOD_PLTFM, BF_LOG_INFO, format, ##__VA_ARGS__); \
  }

/* create the board map structure */
int pltfm_create_bd_map(const char *json_file) {
  FILE *file = NULL;
  int fd, rows, sts;
  struct stat stat_b;
  size_t to_allocate, num_items;
  char *config_file_buffer = NULL;
  cJSON *root = NULL, *bd_map_ent_all = NULL, *bd_map_ent;
  bd_map_ent_t *this_bd_map_ent = NULL;
  int err = 0;
  char *param = "Unknown";

  if (!json_file) {
    printf("bad board json file name\n");
    return -1;
  }
  file = fopen(json_file, "r");
  if (file == NULL) {
    printf("Could not open configuration file: %s\n", json_file);
    return -1;
  }

  fd = fileno(file);
  fstat(fd, &stat_b);
  to_allocate = stat_b.st_size + 1;
  sts = -1;

  config_file_buffer = malloc(to_allocate);
  if (config_file_buffer == NULL) {
    LOG_ERROR("Could not allocate memory for bd_map\n");
    goto error_end;
  }

  num_items = fread(config_file_buffer, stat_b.st_size, 1, file);
  if (num_items != 1) {
    LOG_ERROR("error reading %s file\n", json_file);
    goto error_end;
  }

  root = cJSON_Parse(config_file_buffer);
  if (root == NULL) {
    LOG_ERROR("cJSON parsing error before %s\n", cJSON_GetErrorPtr());
    goto error_end;
  }

  int bd_id = 0xFF;
  err = bf_cjson_get_string(root, "board_id", &param);
  if (!err) {
    bd_id = atoi(param);
    LOG_DEBUG("Board-id 0x%0x\n", bd_id);
  }

  int en_log = 0;  // default disable
  err = bf_cjson_get_string(root, "enable_debug_log", &param);
  if (!err) {
    en_log = atoi(param);
    LOG_DEBUG("enable-log %d \n", en_log);
  }

  bd_map_ent_all = cJSON_GetObjectItem(root, "board_lane_map_entry");
  if (bd_map_ent_all == NULL) {
    LOG_ERROR("cJSON parsing error locating board_lane_map_entry\n");
    goto error_end;
  }
  rows = cJSON_GetArraySize(bd_map_ent_all);
  if (rows == 0) {
    LOG_ERROR("cJSON parsing error empty board_lane_map_entry\n");
    goto error_end;
  }
  /* allocate space to hold all bd_map_entry(s) */
  this_bd_map_ent = calloc(sizeof(bd_map_ent_t) * rows, 1);
  if (this_bd_map_ent == NULL) {
    LOG_ERROR("Could not allocate memory for bd_map_ent\n");
    goto error_end;
  }
  board_map.rows = rows;
  board_map.bd_id = bd_id;
  board_map.bd_map = this_bd_map_ent;

  /* parse the json object and populate bd_map_ent */
  int row_number = 0;
  char *tx_dict[] = {"TX_MAIN", "TX_POST1", "TX_POST2", "TX_Pre1", "TX_pre2"};
  char *module[] = {"_Cu_0.5m",
                    "_Cu_1.5m",
                    "_Cu_1m",
                    "_Cu_2.5m",
                    "_Cu_2m",
                    "_Cu_Loop_0dB",
                    "_Opt"};
  int qsfp_dd_idx[] = {BF_PLTFM_QSFPDD_CU_0_5_M,
                       BF_PLTFM_QSFPDD_CU_1_5_M,
                       BF_PLTFM_QSFPDD_CU_1_M,
                       BF_PLTFM_QSFPDD_CU_2_5_M,
                       BF_PLTFM_QSFPDD_CU_2_M,
                       BF_PLTFM_QSFPDD_CU_LOOP,
                       BF_PLTFM_QSFPDD_OPT};

  char *encoding[] = {"_NRZ", "_PAM4"};
  char tx_string[80];
  int nconnectors = 1;
  int prev_conn = 0;
  int mac_block = 0;
  cJSON_ArrayForEach(bd_map_ent, bd_map_ent_all) {
    int err;
    int conn, channel, tx_lane, rx_lane;
    bool rx_pn_swap, tx_pn_swap;
    char *param = "Unknown";
    int device_id = 0;
    int is_internal_port;

    err = tx_lane = rx_lane = is_internal_port = 0;

    err |= bf_cjson_get_string(bd_map_ent, "QSFPDD_Port", &param);
    conn = atoi(param);
    BD_MAP_LOG(en_log, "connector %d \n", conn);
    if (conn != prev_conn) {
      nconnectors++;
    }
    prev_conn = conn;

    err = bf_cjson_get_string(bd_map_ent, "Internal_port", &param);
    if (!strcmp(param, "Yes")) {
      is_internal_port = 1;
    }
    BD_MAP_LOG(en_log, "Internal_port:%s val:%d \n", param, is_internal_port);

    err = bf_cjson_get_string(bd_map_ent, "JBAY Package Net Name", &param);
    mac_block = atoi(param);
    BD_MAP_LOG(en_log, "mac_block:%d \n", mac_block);

    err |= bf_cjson_get_string(bd_map_ent, "QSFPDD_Lane", &param);
    channel = atoi(param);
    BD_MAP_LOG(en_log, "channel:%d \n", channel);

    rx_pn_swap = 0;
    err = bf_cjson_get_string(bd_map_ent, "RX_PN_Swap", &param);
    if (!strcmp(param, "Y")) {
      rx_pn_swap = 1;
    }
    BD_MAP_LOG(en_log, "RX_PN_Swap:%s swap:%d \n", param, rx_pn_swap);

    tx_pn_swap = 0;
    err = bf_cjson_get_string(bd_map_ent, "TX_PN_Swap", &param);
    if (!strcmp(param, "Y")) {
      tx_pn_swap = 1;
    }
    BD_MAP_LOG(en_log, "TX_PN_Swap:%s swap:%d \n", param, tx_pn_swap);

    err |= bf_cjson_get_string(bd_map_ent, "PCB_RX_Lane", &param);
    rx_lane = atoi(param);
    BD_MAP_LOG(en_log, "SD_RX_Lane:%d d \n", rx_lane);

    err |= bf_cjson_get_string(bd_map_ent, "PCB_TX_Lane", &param);
    tx_lane = atoi(param);
    BD_MAP_LOG(en_log, "SD_TX_Lane:%d d \n", tx_lane);
    // Do some sanity
    if ((tx_lane > 7) || (rx_lane > 7)) {
      LOG_ERROR("Rx:%d or tx:%d lane  not valid. Must be 0 to 7 \n",
                rx_lane,
                tx_lane);
      assert(0);
    }

    if (err) {
      goto error_end;
    }
    // Populate
    this_bd_map_ent->connector = conn;
    this_bd_map_ent->device_id = device_id;
    this_bd_map_ent->channel = channel;
    this_bd_map_ent->mac_ch = channel;
    this_bd_map_ent->mac_block = mac_block;
    this_bd_map_ent->tx_lane = tx_lane;
    this_bd_map_ent->tx_pn_swap = tx_pn_swap;
    this_bd_map_ent->rx_lane = rx_lane;
    this_bd_map_ent->rx_pn_swap = rx_pn_swap;
    this_bd_map_ent->is_internal_port = is_internal_port;

    serdes_lane_tx_eq_t *tx_eq_ptr = NULL;
    tx_eq_ptr = calloc(sizeof(serdes_lane_tx_eq_t), 1);
    if (tx_eq_ptr == NULL) {
      LOG_ERROR("Could not allocate memory for bd_map_ent tx_eq\n");
      goto error_end;
    }
    this_bd_map_ent->tx_eq_for_qsfpdd[0] = tx_eq_ptr;

    tx_eq_ptr = calloc(sizeof(serdes_lane_tx_eq_t), 1);
    if (tx_eq_ptr == NULL) {
      LOG_ERROR("Could not allocate memory for bd_map_ent tx_eq\n");
      goto error_end;
    }
    this_bd_map_ent->tx_eq_for_qsfpdd[1] = tx_eq_ptr;

    int enc = 0;
    int i = 0;
    int tx_main, tx_post1, tx_post2, tx_pre1, tx_pre2;
    for (enc = 0; enc < 2; enc++) {
      serdes_lane_tx_eq_t *tx_ptr = this_bd_map_ent->tx_eq_for_qsfpdd[enc];
      if (!tx_ptr) {
        LOG_ERROR("Null tx-ptr\n");
        assert(0);
      }

      for (i = 0; i < MAX_QSFPDD_TYPES - 1; i++) {
        int k = 0;
        int qidx = 0;
        qidx = qsfp_dd_idx[i];
        BD_MAP_LOG(en_log, "module: %s qidx: %d\n", module[i], qidx);
        memset(tx_string, 0, sizeof(&tx_string));
        strcat(tx_string, tx_dict[k++]);
        strcat(tx_string, module[i]);
        strcat(tx_string, encoding[enc]);
        err |= bf_cjson_get_string(bd_map_ent, tx_string, &param);
        tx_main = atoi(param);
        BD_MAP_LOG(en_log, "%s val: %d\n", tx_string, tx_main);
        tx_ptr->tx_main[qidx] = tx_main;

        memset(tx_string, 0, sizeof(&tx_string));
        strcat(tx_string, tx_dict[k++]);
        strcat(tx_string, module[i]);
        strcat(tx_string, encoding[enc]);
        err |= bf_cjson_get_string(bd_map_ent, tx_string, &param);
        tx_post1 = atoi(param);
        BD_MAP_LOG(en_log, "%s val: %d\n", tx_string, tx_post1);
        tx_ptr->tx_post1[qidx] = tx_post1;

        memset(tx_string, 0, sizeof(&tx_string));
        strcat(tx_string, tx_dict[k++]);
        strcat(tx_string, module[i]);
        strcat(tx_string, encoding[enc]);
        err |= bf_cjson_get_string(bd_map_ent, tx_string, &param);
        tx_post2 = atoi(param);
        BD_MAP_LOG(en_log, "%s val: %d\n", tx_string, tx_post2);
        tx_ptr->tx_post2[qidx] = tx_post2;

        memset(tx_string, 0, sizeof(&tx_string));
        strcat(tx_string, tx_dict[k++]);
        strcat(tx_string, module[i]);
        strcat(tx_string, encoding[enc]);
        err |= bf_cjson_get_string(bd_map_ent, tx_string, &param);
        tx_pre1 = atoi(param);
        BD_MAP_LOG(en_log, "%s val: %d\n", tx_string, tx_pre1);
        tx_ptr->tx_pre1[qidx] = tx_pre1;

        memset(tx_string, 0, sizeof(&tx_string));
        strcat(tx_string, tx_dict[k++]);
        strcat(tx_string, module[i]);
        strcat(tx_string, encoding[enc]);
        err |= bf_cjson_get_string(bd_map_ent, tx_string, &param);
        tx_pre2 = atoi(param);
        BD_MAP_LOG(en_log, "%s val: %d\n", tx_string, tx_pre2);
        tx_ptr->tx_pre2[qidx] = tx_pre2;

        if (err) {
          goto error_end;
        }
      }
    }

    row_number++;
    this_bd_map_ent++;
  }
  nconnectors--;  // Index 1 to last port :33
  LOG_DEBUG("Num of connectors : %d\n", nconnectors);
  board_map.num_of_connectors = nconnectors;
  sts = 0;

error_end:
  if (root) {
    cJSON_Delete(root);
  }
  if (config_file_buffer) {
    free(config_file_buffer);
  }
  if (file) {
    fclose(file);
  }
  if (err) pltfm_clear_bd_map();
  if (sts) printf("parsing error\n");
  return sts;
}

/* create the board map structure */
void pltfm_clear_bd_map(void) {
  board_map.rows = 0;
  board_map.bd_id = 0;
  if (board_map.bd_map) {
    if (board_map.bd_map->tx_eq_for_qsfpdd[0]) {
      bf_sys_free(board_map.bd_map->tx_eq_for_qsfpdd[0]);
    }
    if (board_map.bd_map->tx_eq_for_qsfpdd[1]) {
      bf_sys_free(board_map.bd_map->tx_eq_for_qsfpdd[1]);
    }
    bf_sys_free(board_map.bd_map);
  }
  board_map.bd_map = NULL;
}
