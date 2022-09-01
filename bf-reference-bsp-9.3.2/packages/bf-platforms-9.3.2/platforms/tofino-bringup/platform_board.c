/*******************************************************************************
 * Copyright (c) 2015-2020 Barefoot Networks, Inc.
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * $Id: $
 *
 ******************************************************************************/
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <bf_switchd/bf_switchd.h>
#include <bf_bd_cfg/bf_bd_cfg_intf.h>
#include <bf_pltfm.h>
#include <bf_bd_cfg/bf_bd_cfg_bd_map.h>
#include <bf_pltfm_bd_cfg.h>
#include <bfsys/bf_sal/bf_sys_mem.h>

static pltfm_bd_map_t pltfm_tofino_bringup_map;
static unsigned int tx_eq_pre_default[6] = {6, 6, 6, 6, 6, 3};
static unsigned int tx_eq_post_default[6] = {0, 0, 0, 0, 0, 16};
static unsigned int tx_eq_attn_default[6] = {0, 0, 0, 0, 0, 0};

/******************************************************************************
*
******************************************************************************/
pltfm_bd_map_t *platform_pltfm_bd_map_get(int *rows) {
  if (!rows) {
    return NULL;
  }
  *rows = 1;
  return &pltfm_tofino_bringup_map;
}

int platform_name_get_str(char *name, size_t name_size) {
  if (!name) {
    return -1;
  }
  snprintf(name, name_size, "Board type: Tof-1 Bringup");
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
  return (bd_map->rows / QSFP_NUM_CHN);
}
static int pltfm_parse_tx_eq_val(cJSON *tx_eq,
                                 unsigned int *bd_map_tx,
                                 unsigned int *tx_default) {
  cJSON *cu_short, *cu_1m, *cu_2m, *cu_3m, *cu_loop_0db, *optical;

  cu_short = cJSON_GetObjectItem(tx_eq, "cu_short");
  cu_1m = cJSON_GetObjectItem(tx_eq, "cu_1m");
  cu_2m = cJSON_GetObjectItem(tx_eq, "cu_2m");
  cu_3m = cJSON_GetObjectItem(tx_eq, "cu_3m");
  cu_loop_0db = cJSON_GetObjectItem(tx_eq, "cu_loop_0db");
  optical = cJSON_GetObjectItem(tx_eq, "optical");

  if (cu_short != NULL && (cu_short->type & cJSON_Number)) {
    bd_map_tx[0] = cu_short->valueint;
  } else {
    bd_map_tx[0] = tx_default[0];
  }
  if (cu_1m != NULL && (cu_1m->type & cJSON_Number)) {
    bd_map_tx[1] = cu_1m->valueint;
  } else {
    bd_map_tx[1] = tx_default[1];
  }
  if (cu_2m != NULL && (cu_2m->type & cJSON_Number)) {
    bd_map_tx[2] = cu_2m->valueint;
  } else {
    bd_map_tx[2] = tx_default[2];
  }
  if (cu_3m != NULL && (cu_3m->type & cJSON_Number)) {
    bd_map_tx[3] = cu_3m->valueint;
  } else {
    bd_map_tx[3] = tx_default[3];
  }
  if (cu_loop_0db != NULL && (cu_loop_0db->type & cJSON_Number)) {
    bd_map_tx[4] = cu_loop_0db->valueint;
  } else {
    bd_map_tx[4] = tx_default[4];
  }
  if (optical != NULL && (optical->type & cJSON_Number)) {
    bd_map_tx[5] = optical->valueint;
  } else {
    bd_map_tx[5] = tx_default[5];
  }
  return 0;
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
    printf("Could not allocate memory for bd_map\n");
    goto error_end;
  }

  num_items = fread(config_file_buffer, stat_b.st_size, 1, file);
  if (num_items != 1) {
    printf("error reading %s file\n", json_file);
    goto error_end;
  }
  root = cJSON_Parse(config_file_buffer);
  if (root == NULL) {
    printf("cJSON parsing error before %s\n", cJSON_GetErrorPtr());
    goto error_end;
  }
  bd_map_ent_all = cJSON_GetObjectItem(root, "board_lane_map_entry");
  if (bd_map_ent_all == NULL) {
    printf("cJSON parsing error locating board_lane_map_entry\n");
    goto error_end;
  }
  rows = cJSON_GetArraySize(bd_map_ent_all);
  if (rows == 0) {
    printf("cJSON parsing error empty board_lane_map_entry\n");
    goto error_end;
  }
  /* allocate space to hold all bd_map_entry(s) */
  this_bd_map_ent = calloc(sizeof(bd_map_ent_t) * rows, 1);
  if (this_bd_map_ent == NULL) {
    printf("Could not allocate memory for bd_map_ent\n");
    goto error_end;
  }
  pltfm_tofino_bringup_map.rows = rows;
  pltfm_tofino_bringup_map.bd_map = this_bd_map_ent;
  pltfm_tofino_bringup_map.bd_id = BF_PLTFM_BD_ID_MONTARA_P0B;
  /* parse the json object and populate bd_map_ent */
  cJSON_ArrayForEach(bd_map_ent, bd_map_ent_all) {
    cJSON *conn, *channel, *mac_blk, *mac_ch;
    cJSON *tx_lane, *rx_lane, *tx_pn_swap, *rx_pn_swap;
    cJSON *tx_eq_pre, *tx_eq_post, *tx_eq_attn;

    conn = cJSON_GetObjectItem(bd_map_ent, "connector");
    channel = cJSON_GetObjectItem(bd_map_ent, "lane");
    mac_blk = cJSON_GetObjectItem(bd_map_ent, "mac_block");
    mac_ch = cJSON_GetObjectItem(bd_map_ent, "mac_ch");
    tx_lane = cJSON_GetObjectItem(bd_map_ent, "tx_lane");
    tx_pn_swap = cJSON_GetObjectItem(bd_map_ent, "tx_pn_swap");
    rx_lane = cJSON_GetObjectItem(bd_map_ent, "rx_lane");
    rx_pn_swap = cJSON_GetObjectItem(bd_map_ent, "rx_pn_swap");
    tx_eq_pre = cJSON_GetObjectItem(bd_map_ent, "tx_eq_pre");
    tx_eq_post = cJSON_GetObjectItem(bd_map_ent, "tx_eq_post");
    tx_eq_attn = cJSON_GetObjectItem(bd_map_ent, "tx_eq_attn");
    if (conn != NULL && (conn->type & cJSON_Number)) {
      this_bd_map_ent->connector = conn->valueint;
    } else {
      goto error_end;
    }
    if (channel != NULL && (channel->type & cJSON_Number)) {
      this_bd_map_ent->channel = channel->valueint;
    } else {
      goto error_end;
    }
    if (mac_blk != NULL && (mac_blk->type & cJSON_Number)) {
      this_bd_map_ent->mac_block = mac_blk->valueint;
    } else {
      goto error_end;
    }
    if (mac_ch != NULL && (mac_ch->type & cJSON_Number)) {
      this_bd_map_ent->mac_ch = mac_ch->valueint;
    } else {
      goto error_end;
    }
    if (tx_lane != NULL && (tx_lane->type & cJSON_Number)) {
      this_bd_map_ent->tx_lane = tx_lane->valueint;
    } else {
      goto error_end;
    }
    if (tx_pn_swap != NULL && (tx_pn_swap->type & cJSON_Number)) {
      this_bd_map_ent->tx_pn_swap = tx_pn_swap->valueint;
    } else {
      /* set the default if not provided in json file */
      this_bd_map_ent->tx_pn_swap = 0;
    }
    if (rx_lane != NULL && (rx_lane->type & cJSON_Number)) {
      this_bd_map_ent->rx_lane = rx_lane->valueint;
    } else {
      goto error_end;
    }
    if (rx_pn_swap != NULL && (rx_pn_swap->type & cJSON_Number)) {
      this_bd_map_ent->rx_pn_swap = rx_pn_swap->valueint;
    } else {
      /* set the default if not provided in json file */
      this_bd_map_ent->rx_pn_swap = 0;
    }
    if (tx_eq_pre) {
      pltfm_parse_tx_eq_val(
          tx_eq_pre, &(this_bd_map_ent->tx_eq_pre[0]), tx_eq_pre_default);
    } else {
      /* the values would be zeroes ; *** TBD */
    }
    if (tx_eq_post) {
      pltfm_parse_tx_eq_val(
          tx_eq_post, &(this_bd_map_ent->tx_eq_post[0]), tx_eq_post_default);
    } else {
      /* the values would be zeroes ; *** TBD */
    }
    if (tx_eq_attn) {
      pltfm_parse_tx_eq_val(
          tx_eq_attn, &(this_bd_map_ent->tx_eq_attn[0]), tx_eq_attn_default);
    } else {
      /* the values would be zeroes ; *** TBD */
    }
#if 0 /* for debugging only */
    printf(
        "%2d %1d %2d %1d %1d %1d %1d %1d: %1d %1d %1d %1d %1d %1d: %1d %1d %1d "
        "%1d %1d %1d: %1d %1d %1d %1d %1d %1d\n",
        this_bd_map_ent->connector,
        this_bd_map_ent->channel,
        this_bd_map_ent->mac_block,
        this_bd_map_ent->mac_ch,
        this_bd_map_ent->tx_lane,
        this_bd_map_ent->tx_pn_swap,
        this_bd_map_ent->rx_lane,
        this_bd_map_ent->rx_pn_swap,
        this_bd_map_ent->tx_eq_pre[0],
        this_bd_map_ent->tx_eq_pre[1],
        this_bd_map_ent->tx_eq_pre[2],
        this_bd_map_ent->tx_eq_pre[3],
        this_bd_map_ent->tx_eq_pre[4],
        this_bd_map_ent->tx_eq_pre[5],
        this_bd_map_ent->tx_eq_post[0],
        this_bd_map_ent->tx_eq_post[1],
        this_bd_map_ent->tx_eq_post[2],
        this_bd_map_ent->tx_eq_post[3],
        this_bd_map_ent->tx_eq_post[4],
        this_bd_map_ent->tx_eq_post[5],
        this_bd_map_ent->tx_eq_attn[0],
        this_bd_map_ent->tx_eq_attn[1],
        this_bd_map_ent->tx_eq_attn[2],
        this_bd_map_ent->tx_eq_attn[3],
        this_bd_map_ent->tx_eq_attn[4],
        this_bd_map_ent->tx_eq_attn[5]);
#endif
    this_bd_map_ent++;
  }
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
  if (sts) printf("parsing error\n");
  return sts;
}

/* create the board map structure */
void pltfm_clear_bd_map(void) {
  pltfm_tofino_bringup_map.rows = 0;
  bf_sys_free(pltfm_tofino_bringup_map.bd_map);
  pltfm_tofino_bringup_map.bd_map = NULL;
}
