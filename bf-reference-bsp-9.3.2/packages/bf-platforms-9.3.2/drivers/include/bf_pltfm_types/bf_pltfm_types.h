/*******************************************************************************
 * Copyright (c) 2015-2020 Barefoot Networks, Inc.
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * $Id: $
 *
 ******************************************************************************/
#ifndef _BF_PLTFM_TYPES_H
#define _BF_PLTFM_TYPES_H

#include <stdbool.h>
#include <bfsys/bf_sal/bf_sys_intf.h>

#ifdef __cplusplus
extern "C" {
#endif

#define LOG_ERROR(...) \
  bf_sys_log_and_trace(BF_MOD_PLTFM, BF_LOG_ERR, __VA_ARGS__)
#define LOG_WARNING(...) \
  bf_sys_log_and_trace(BF_MOD_PLTFM, BF_LOG_WARN, __VA_ARGS__)
#define LOG_DEBUG(...) \
  bf_sys_log_and_trace(BF_MOD_PLTFM, BF_LOG_DBG, __VA_ARGS__)

#define MAX_CHAN_PER_CONNECTOR 8

// connectors are numbered starting from 1, Hence total connectors = 1
// unused + 64 + 1 CPU
#define MAX_CONNECTORS 66

// Excluding cpu port.
//
// Note really not necessary to have this macro, since non-qsfp ports
// are derived from internal-port. Keep it for backwardation
#define BF_PLAT_MAX_QSFP 65

/*
 * Identifies the type of the board
 */
typedef enum bf_pltfm_board_id_e {
  BF_PLTFM_BD_ID_MAVERICKS_P0A = 0x0234,
  BF_PLTFM_BD_ID_MAVERICKS_P0B = 0x1234,
  BF_PLTFM_BD_ID_MAVERICKS_P0C = 0x5234,
  BF_PLTFM_BD_ID_MONTARA_P0A = 0x2234,
  BF_PLTFM_BD_ID_MONTARA_P0B = 0x3234,
  BF_PLTFM_BD_ID_MAVERICKS_P0B_EMU = 0x4234,
  BF_PLTFM_BD_ID_MONTARA_P0C = 0x6234,
  BF_PLTFM_BD_ID_NEWPORT_P0A = 0x1134,
  BF_PLTFM_BD_ID_NEWPORT_P0B = 0x2134,
  BF_PLTFM_BD_ID_UNKNOWN = 0XFFFF
} bf_pltfm_board_id_t;
/*
 * Identifies the type of the QSFP connected
 */
typedef enum bf_pltfm_qsfp_type_t {

  BF_PLTFM_QSFP_CU_0_5_M = 0,
  BF_PLTFM_QSFP_CU_1_M = 1,
  BF_PLTFM_QSFP_CU_2_M = 2,
  BF_PLTFM_QSFP_CU_3_M = 3,
  BF_PLTFM_QSFP_CU_LOOP = 4,
  BF_PLTFM_QSFP_OPT = 5,
  BF_PLTFM_QSFP_UNKNOWN = 6
} bf_pltfm_qsfp_type_t;

/*
 * Identifies the type of the QSFP-DD connected
 */
typedef enum bf_pltfm_qsfpdd_type_t {
  BF_PLTFM_QSFPDD_CU_0_5_M = 0,
  BF_PLTFM_QSFPDD_CU_1_M = 1,
  BF_PLTFM_QSFPDD_CU_1_5_M = 2,
  BF_PLTFM_QSFPDD_CU_2_M = 3,
  BF_PLTFM_QSFPDD_CU_2_5_M = 4,
  BF_PLTFM_QSFPDD_CU_LOOP = 5,
  BF_PLTFM_QSFPDD_OPT = 6,

  // Keep this last
  BF_PLTFM_QSFPDD_UNKNOWN,
} bf_pltfm_qsfpdd_type_t;

typedef struct bf_pltfm_serdes_lane_tx_eq_ {
  int32_t tx_main;
  int32_t tx_pre1;
  int32_t tx_pre2;
  int32_t tx_post1;
  int32_t tx_post2;
} bf_pltfm_serdes_lane_tx_eq_t;

typedef enum bf_pltfm_encoding_type_ {
  BF_PLTFM_ENCODING_NRZ = 0,
  BF_PLTFM_ENCODING_PAM4
} bf_pltfm_encoding_type_t;
/*
 * Encapsulates the information of a port on the board
 */
typedef struct bf_pltfm_port_info_t {
  uint32_t conn_id;
  uint32_t chnl_id;
} bf_pltfm_port_info_t;

/*
 * Identifies an error code
 */
typedef int bf_pltfm_status_t;

#define BF_PLTFM_STATUS_VALUES                                         \
  BF_PLTFM_STATUS_(BF_PLTFM_SUCCESS, "Success"),                       \
      BF_PLTFM_STATUS_(BF_PLTFM_INVALID_ARG, "Invalid Arguments"),     \
      BF_PLTFM_STATUS_(BF_PLTFM_OBJECT_NOT_FOUND, "Object Not Found"), \
      BF_PLTFM_STATUS_(BF_PLTFM_COMM_FAILED,                           \
                       "Communication Failed with Hardware"),          \
      BF_PLTFM_STATUS_(BF_PLTFM_OBJECT_ALREADY_EXISTS,                 \
                       "Object Already Exists"),                       \
      BF_PLTFM_STATUS_(BF_PLTFM_OTHER, "Other")

enum bf_pltfm_status_enum {
#define BF_PLTFM_STATUS_(x, y) x
  BF_PLTFM_STATUS_VALUES,
  BF_PLTFM_STS_MAX
#undef BF_PLTFM_STATUS_
};

static const char *bf_pltfm_err_strings[BF_PLTFM_STS_MAX + 1] = {
#define BF_PLTFM_STATUS_(x, y) y
    BF_PLTFM_STATUS_VALUES, "Unknown error"
#undef BF_PLTFM_STATUS_
};

static inline const char *bf_pltfm_err_str(bf_pltfm_status_t sts) {
  if (BF_PLTFM_STS_MAX <= sts || 0 > sts) {
    return bf_pltfm_err_strings[BF_PLTFM_STS_MAX];
  } else {
    return bf_pltfm_err_strings[sts];
  }
}

#ifdef __cplusplus
}
#endif /* C++ */

#endif
