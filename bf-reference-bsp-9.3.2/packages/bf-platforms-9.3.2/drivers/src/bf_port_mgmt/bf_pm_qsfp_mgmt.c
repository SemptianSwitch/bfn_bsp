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
#include <bf_pltfm_ext_phy.h>
#include <bf_pltfm_types/bf_pltfm_types.h>

#include <bf_types/bf_types.h>
#include <bfsys/bf_sal/bf_sys_sem.h>
#include <bfsys/bf_sal/bf_sys_timer.h>

#include <tofino/bf_pal/bf_pal_pltfm_porting.h>
#include <bf_pm/bf_pm_intf.h>
#include <bf_bd_cfg/bf_bd_cfg_porting.h>
#include <bf_qsfp/bf_qsfp.h>
#include <bf_pltfm_qsfp.h>
#include "bf_pm_priv.h"

#define POLL_QSFP_INT_FLAGS 1  // set to 0 to only read flags when IntL is 0
#if POLL_QSFP_INT_FLAGS
#define FLAG_POLL_INTERVAL_MS 1500  // this time can be adjusted to account for
                                    // how long it takes modules to reassert
                                    // continuing condition flags after clear-
                                    // on-read, to prevent log noise
#endif
#define MONITOR_TEMPS 0
#if MONITOR_TEMPS
#define TEMP_POLL_INTERVAL_MS 5000
#endif

/*****************************************************************************
  QSFP Mgmt

  This code enforces timing constraints imposed by SFF-8636 and CMIS.

  It implements the following bring-up sequence for compliant optical QSFPs.
  The goal is to place the QSFP into a known, stable state with their CDRs
  locked to a valid electrical signal from Tofino and the lasers OFF.

               __
  MODPRSL        \________________________________________________


               ______ t_init_reset--|_____________________________
  RESETL             \______________/ t_reset---|

               ___________________________________________________
  LPMODE (hw)

                  _____________________________________
  LPMODE (sw)  XX/                                     \__________

                                                  ________________
  TX_DISABLE   XX\_______________________________/

  Serdes TX    XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX/ PRBS

  At platform power-on, ResetL and LPmode are both asserted. During
  bf_pltfm_pm_init, all ResetL pins are deasserted. The above diagram
  starts immediately after this deassertion. When the module is detected
  by qsfp_scan, the module state machine starts to bring up the module.
  First, the module type (optical/copper) and memory map type is read. Next,
  the module is reset using the external resetL pin. After t_reset, the QSFP
  can be configurd by SW. Software sets TX disable in the module to prevent
  chattering.  Next, the Tofino serdes is configured to transmit a PRBS
  pattern with pre-determined (board-specific) Tx EQ settings. The module now
  has an input signal to train its input equalizers and lock its CDR, so
  LPMODE is disabled in either hardware or software, depending on the module
  requirements.

  For a limited set of modules, high power mode init is needed. For these
  modules, the following sequence applies

               __
  MODPRSL        \________________________________________________


               ______ t_init_reset--|_____________________________
  RESETL             \______________/ t_reset---|

               _________
  LPMODE (hw)           \_________________________________________


  TX_DISABLE   XX\________________________________________________

  Serdes TX    XXXXXXXXXXXX/ PRBS


*****************************************************************************/

/****************************************************************************
 * This .c file performs all interrupt flag servicing for the platforms
 * module. The following list shows which routine services each flag or if
 * the flag is ignored. When a flag is serviced and/or logged, it is also
cleared
 * so that it doesn't get double reported by downstream loggers. Each flag must
 * be serviced/logged in only one place on a given pass through the SM
 *
 * There are two types of flags. Status flags indicate the current status
 * and are expected to be set again after being cleared as long as the condition
 * persists. Event flags are expected to occur once and not be set again until
 * the event occurs again. The implications here are that Event flags must be
 * captured in some way if they are not handled immediately in the current
 * state machine loop, so they don't get lost

 * Common lane-specific status flags, SFF-8636 and CMIS
  FLAG_TX_LOS              Status   Service in CHSM
  FLAG_RX_LOS              Status   Service in CHSM
  FLAG_TX_ADAPT_EQ_FAULT   Event    Service in CHSM
  FLAG_TX_FAULT            Event?   Service in CHSM
  FLAG_TX_LOL              Status   Service in CHSM
  FLAG_RX_LOL              Status   Service in CHSM

 * Common module-level flags
  FLAG_TEMP_HIGH_ALARM     Status   Log only
  FLAG_TEMP_LOW_ALARM      Status   Ignore
  FLAG_TEMP_HIGH_WARN      Status   Ignore
  FLAG_TEMP_LOW_WARN       Status   Ignore
  FLAG_VCC_HIGH_ALARM      Status   Log only
  FLAG_VCC_LOW_ALARM       Status   Log only
  FLAG_VCC_HIGH_WARN       Status   Ignore
  FLAG_VCC_LOW_WARN        Status   Ignore
  FLAG_VENDOR_SPECIFIC              Ignore

 * Common lane-specific monitor flags
  FLAG_RX_PWR_HIGH_ALARM   Status   Log only
  FLAG_RX_PWR_LOW_ALARM    Status   Log only
  FLAG_RX_PWR_HIGH_WARN    Status   Ignore
  FLAG_RX_PWR_LOW_WARN     Status   Ignore
  FLAG_TX_BIAS_HIGH_ALARM  Status   Log only
  FLAG_TX_BIAS_LOW_ALARM   Status   Log only
  FLAG_TX_BIAS_HIGH_WARN   Status   Ignore
  FLAG_TX_BIAS_LOW_WARN    Status   Ignore
  FLAG_TX_PWR_HIGH_ALARM   Status   Log only
  FLAG_TX_PWR_LOW_ALARM    Status   Log only
  FLAG_TX_PWR_HIGH_WARN    Status   Ignore
  FLAG_TX_PWR_LOW_WARN     Status   Ignore

 * CMIS-only flags (all are module flags)
  FLAG_DATAPATH_FW_FAULT      Event?   Log only
  FLAG_MODULE_FW_FAULT        Event?   Log only
  FLAG_MODULE_STATE_CHANGE    Event    Currently ignored, module state is polled
  FLAG_DATAPATH_STATE_CHANGE  Event    Currently ignored, datapath state is
polled
  FLAG_AUX1_HIGH_ALARM        Status   Ignore
  FLAG_AUX1_LOW_ALARM         Status   Ignore
  FLAG_AUX1_HIGH_WARN         Status   Ignore
  FLAG_AUX1_LOW_WARN          Status   Ignore
  FLAG_AUX2_HIGH_ALARM        Status   Ignore
  FLAG_AUX2_LOW_ALARM         Status   Ignore
  FLAG_AUX2_HIGH_WARN         Status   Ignore
  FLAG_AUX2_LOW_WARN          Status   Ignore
  FLAG_VEND_HIGH_ALARM        Status   Ignore
  FLAG_VEND_LOW_ALARM         Status   Ignore
  FLAG_VEND_HIGH_WARN         Status   Ignore
  FLAG_VEND_LOW_WARN          Status   Ignore
  FLAG_AUX3_HIGH_ALARM        Status   Ignore
  FLAG_AUX3_LOW_ALARM         Status   Ignore
  FLAG_AUX3_HIGH_WARN         Status   Ignore
  FLAG_AUX3_LOW_WARN          Status   Ignore
*****************************************************************************/

static void qsfp_module_fsm_complete_update(bf_dev_id_t dev_id, int conn_id);

/* CMIS & SFF-8636 delay requirements */
typedef enum {
  CMISand8636_TMR_t_reset_init = 1,  // really only 2us
  CMISand8636_TMR_t_reset = 2000,    // all times in ms
  CMISand8636_TMR_Toff_LPMode = 300,
  CMISand8636_TMR_ton_txdis = 100,
  CMISand8636_TMR_toff_txdis = 400,
} sff8436_timer_val_t;

/* Module FSM states */
typedef enum {
  // QSFP unknown or unused on board
  QSFP_FSM_ST_IDLE = 0,
  // pseudo-states, assigned by qsfp scan timer to communicate
  // with qsfp fsm
  QSFP_FSM_ST_REMOVED,
  QSFP_FSM_ST_INSERTED,
  // states requiring fixed delays
  QSFP_FSM_ST_WAIT_T_RESET,      // 2000ms
  QSFP_FSM_ST_WAIT_TON_TXDIS,    //  100ms
  QSFP_FSM_ST_WAIT_TOFF_LPMODE,  //  300ms
  QSFP_FSM_ST_DETECTED,
  QSFP_FSM_ST_WAIT_TON_LPMODE,
  QSFP_FSM_ST_LPMODE,
  QSFP_FSM_ST_UPDATE,
  QSFP_FSM_ST_WAIT_UPDATE,  // update-specific delay
} qsfp_fsm_state_t;

static char *qsfp_fsm_st_to_str[] = {
    "QSFP_FSM_ST_IDLE            ",
    "QSFP_FSM_ST_REMOVED         ",
    "QSFP_FSM_ST_INSERTED        ",
    "QSFP_FSM_ST_WAIT_T_RESET    ",
    "QSFP_FSM_ST_WAIT_TON_TXDIS  ",
    "QSFP_FSM_ST_WAIT_TOFF_LPMODE",
    "QSFP_FSM_ST_DETECTED        ",
    "QSFP_FSM_ST_WAIT_TON_LPMODE ",
    "QSFP_FSM_ST_LPMODE          ",
    "QSFP_FSM_ST_UPDATE          ",
    "QSFP_FSM_ST_WAIT_UPDATE     ",
};

/* Channel FSM states */
typedef enum {
  QSFP_CH_FSM_ST_DISABLED = 0,
  QSFP_CH_FSM_ST_ENABLING,        // kick off enable sequence
  QSFP_CH_FSM_ST_ENA_CDR,         // 300ms (Luxtera PSM4)
  QSFP_CH_FSM_ST_ENA_OPTICAL_TX,  // 400ms, CMISand8636_TMR_toff_txdis
  QSFP_CH_FSM_ST_NOTIFY_ENABLED,
  QSFP_CH_FSM_ST_ENABLED,
  QSFP_CH_FSM_ST_DISABLING,  // assert TX_DISABLE
} qsfp_fsm_ch_en_state_t;

static char *qsfp_fsm_ch_en_st_to_str[] = {
    "QSFP_CH_FSM_ST_DISABLED",
    "QSFP_CH_FSM_ST_ENABLING",
    "QSFP_CH_FSM_ST_ENA_CDR",
    "QSFP_CH_FSM_ST_ENA_OPTICAL_TX",
    "QSFP_CH_FSM_ST_NOTIFY_ENABLED",
    "QSFP_CH_FSM_ST_ENABLED",
    "QSFP_CH_FSM_ST_DISABLING",
};

typedef enum qsfp_typ_t {
  QSFP_TYP_UNKNOWN = 0,
  QSFP_TYP_OPTICAL = 1,
  QSFP_TYP_COPPER = 2,
} qsfp_typ_t;

#define BYTE2_FLAT_MEM (1 << 2)
#define BYTE2_INTL (1 << 1)
#define BYTE2_DATA_NOT_READY (1 << 0)

typedef enum {
  FLAG_TYPE_EVENT,   // flag is edge triggered by an event, log positive trans
  FLAG_TYPE_STATUS,  // flag is level triggered by a status or state, log both
                     //    positive and negative transitions
} Sff_flag_type;

typedef enum {
  FLAG_ERROR,
  FLAG_INFO,
} Sff_flag_logtype;

typedef struct qsfp_flag_log_t {
  Sff_flag flag;
  Sff_flag_logtype logtype;
  int bitnum;          // bit number to log
  Sff_flag_type type;  // event or status
  char desc[50];
  int fval_prev[BF_PLAT_MAX_QSFP + 1];
  int fval_cur[BF_PLAT_MAX_QSFP + 1];
} qsfp_flag_log_t;

// this array should only contain flags that are logged
qsfp_flag_log_t sff8636_module_flag_log[] = {
    {
     FLAG_TEMP_HIGH_ALARM,
     FLAG_ERROR,
     0,
     FLAG_TYPE_STATUS,
     "FLAG_TEMP_HIGH_ALARM",
    },
    {
     FLAG_VCC_HIGH_ALARM,
     FLAG_ERROR,
     0,
     FLAG_TYPE_STATUS,
     "FLAG_VCC_HIGH_ALARM",
    },
    {
     FLAG_VCC_LOW_ALARM, FLAG_ERROR, 0, FLAG_TYPE_STATUS, "FLAG_VCC_LOW_ALARM",
    },
};

// this array should only contain flags that are logged
qsfp_flag_log_t cmis_module_flag_log[] = {
    {
     FLAG_TEMP_HIGH_ALARM,
     FLAG_ERROR,
     0,
     FLAG_TYPE_STATUS,
     "FLAG_TEMP_HIGH_ALARM",
    },
    {
     FLAG_VCC_HIGH_ALARM,
     FLAG_ERROR,
     0,
     FLAG_TYPE_STATUS,
     "FLAG_VCC_HIGH_ALARM",
    },
    {
     FLAG_VCC_LOW_ALARM, FLAG_ERROR, 0, FLAG_TYPE_STATUS, "FLAG_VCC_LOW_ALARM",
    },
    {
     FLAG_DATAPATH_FW_FAULT,
     FLAG_ERROR,
     0,
     FLAG_TYPE_EVENT,
     "FLAG_DATAPATH_FW_FAULT",
    },
    {
     FLAG_MODULE_FW_FAULT,
     FLAG_ERROR,
     0,
     FLAG_TYPE_EVENT,
     "FLAG_MODULE_FW_FAULT",
    },
    {
     FLAG_MODULE_STATE_CHANGE,
     FLAG_INFO,
     0,
     FLAG_TYPE_EVENT,
     "FLAG_MODULE_STATE_CHANGE",
    },
    {
     FLAG_DATAPATH_STATE_CHANGE,
     FLAG_INFO,
     0,
     FLAG_TYPE_EVENT,
     "FLAG_DATAPATH_STATE_CHANGE",
    },
};

// this array should only contain flags that are logged
qsfp_flag_log_t ln_flag_log[] = {
    {
     FLAG_TX_LOS, FLAG_INFO, 0, FLAG_TYPE_STATUS, "FLAG_TX_LOS",
    },
    {
     FLAG_RX_LOS, FLAG_INFO, 0, FLAG_TYPE_STATUS, "FLAG_RX_LOS",
    },
    {
     FLAG_TX_ADAPT_EQ_FAULT,
     FLAG_ERROR,
     0,
     FLAG_TYPE_EVENT,
     "FLAG_TX_ADAPT_EQ_FAULT",
    },
    {
     FLAG_TX_FAULT, FLAG_ERROR, 0, FLAG_TYPE_EVENT, "FLAG_TX_FAULT",
    },
    {
     FLAG_TX_LOL, FLAG_INFO, 0, FLAG_TYPE_STATUS, "FLAG_TX_LOL",
    },
    {
     FLAG_RX_LOL, FLAG_INFO, 0, FLAG_TYPE_STATUS, "FLAG_RX_LOL",
    },
    {
     FLAG_RX_PWR_HIGH_ALARM,
     FLAG_INFO,
     0,
     FLAG_TYPE_STATUS,
     "FLAG_RX_PWR_HIGH_ALARM",
    },
    {
     FLAG_RX_PWR_LOW_ALARM,
     FLAG_INFO,
     0,
     FLAG_TYPE_STATUS,
     "FLAG_RX_PWR_LOW_ALARM",
    },
    {
     FLAG_TX_BIAS_HIGH_ALARM,
     FLAG_INFO,
     0,
     FLAG_TYPE_STATUS,
     "FLAG_TX_BIAS_HIGH_ALARM",
    },
    {
     FLAG_TX_BIAS_LOW_ALARM,
     FLAG_INFO,
     0,
     FLAG_TYPE_STATUS,
     "FLAG_TX_BIAS_LOW_ALARM",
    },
    {
     FLAG_TX_PWR_HIGH_ALARM,
     FLAG_INFO,
     0,
     FLAG_TYPE_STATUS,
     "FLAG_TX_PWR_HIGH_ALARM",
    },
    {
     FLAG_TX_PWR_LOW_ALARM,
     FLAG_INFO,
     0,
     FLAG_TYPE_STATUS,
     "FLAG_TX_PWR_LOW_ALARM",
    },
};

// this struct is only used fr Tofino 1 systems
typedef struct qsfp_status_and_alarms_t {
  // Byte
  //  uint8_t identifier;     //  0. unused : identifier
  //  uint8_t status1;        //  1. unused : rev compliance
  uint8_t status2;         //  2. [2:0] = Flat_mem : IntL : Data_Not_Ready
  uint8_t los_ind;         //  3. [7:4] Tx LOS : [3:0] Rx LOS
  uint8_t eq_laser_fault;  //  4. [7:4] Tx Adapt Fault : [3:0] Tx laser fault
  uint8_t lol_ind;         //  5. [7:4] Tx LOL : [3:0] Rx LOL
  uint8_t temp_alarm;   //  6. [7:4] Temp alarm/warning : [0:0] init complete fl
  uint8_t vcc_alarm;    //  7. [7:4] Vcc alarm/warning
  uint8_t vendor_spec;  //  8. unused
  uint8_t rx_power_alarm12;  //  9. Rx power alarm/warning, Ch1 and Ch2
  uint8_t rx_power_alarm34;  // 10. Rx power alarm/warning, Ch3 and Ch4
  uint8_t tx_bias_alarm12;   // 11. Tx Bias  alarm/warning, Ch1 and Ch2
  uint8_t tx_bias_alarm34;   // 12. Tx Bias  alarm/warning, Ch3 and Ch4
  uint8_t tx_power_alarm12;  // 13. Rx power alarm/warning, Ch1 and Ch2
  uint8_t tx_power_alarm34;  // 14. Rx power alarm/warning, Ch3 and Ch4

} qsfp_status_and_alarms_t;

typedef struct qsfp_channel_fsm_t {
  qsfp_fsm_ch_en_state_t fsm_st;
  int substate;
  bool first_enable_after_reset;
  bool head_1st_ena_cdr;
  bool head_1st_tx_locked;
  bool head_1st_ena_opt_tx;
  bool head_1st_notify_enb;
  bool immediate_enable;  // used when retrying port init
  struct timespec next_fsm_run_time;
} qsfp_channel_fsm_t;

typedef struct dev_cfg_per_channel_t {
  bf_port_speed_t intf_speed;
  int host_intf_nlanes;
  int host_head_ch;  // host side head channel in this data path (CMIS only)
  int media_intf_nlanes;
  int media_head_ch;  // media side head channel in this data path (CMIS only)
  int media_ch;       // media side channel # corresponding to this host side ch
  bf_pltfm_encoding_type_t encoding;
} dev_cfg_per_channel_t;

typedef struct qsfp_state_t {
  qsfp_typ_t qsfp_type;
  uint8_t ch_cnt;        // number of host channels in the module
  uint8_t media_ch_cnt;  // number of media channels in the module (per bank)
  qsfp_fsm_state_t fsm_st;
  bool flat_mem;
  bool needs_hi_pwr_init;  // only Luxtera known to require this
  bool qsfp_quick_removed;
  struct timespec next_fsm_run_time;
  struct timespec next_flag_poll_time;
  int sync_ena_cdr;
  int sync_tx_locked;
  int sync_ena_opt_tx;
  int sync_notify_enb;

  qsfp_channel_fsm_t per_ch_fsm[CHANNEL_COUNT];
  dev_cfg_per_channel_t dev_cfg[CHANNEL_COUNT];

  /* structure used for coalescing writes to module
  * registers containing fields for multiple channels
  */
  struct {
    bool in_progress;
    int delay_ms;
    Sff_field field;
    int mask;
    uint8_t data;
  } wr_coalesce;
  qsfp_status_and_alarms_t status_and_alarms;
} qsfp_state_t;

qsfp_state_t qsfp_state[BF_PLAT_MAX_QSFP + 1] = {
    {QSFP_TYP_UNKNOWN,
     0,
     0,
     QSFP_FSM_ST_IDLE,
     false,
     false,
     false,
     {0, 0},
     {0, 0},
     0,
     0,
     0,
     0,
     {{0, 0, true, false, false, false, false, false, {0, 0}},
      {0, 0, true, false, false, false, false, false, {0, 0}},
      {0, 0, true, false, false, false, false, false, {0, 0}},
      {0, 0, true, false, false, false, false, false, {0, 0}},
      {0, 0, true, false, false, false, false, false, {0, 0}},
      {0, 0, true, false, false, false, false, false, {0, 0}},
      {0, 0, true, false, false, false, false, false, {0, 0}},
      {0, 0, true, false, false, false, false, false, {0, 0}},
      {0, 0, true, false, false, false, false, false, {0, 0}},
      {0, 0, true, false, false, false, false, false, {0, 0}},
      {0, 0, true, false, false, false, false, false, {0, 0}},
      {0, 0, true, false, false, false, false, false, {0, 0}},
      {0, 0, true, false, false, false, false, false, {0, 0}},
      {0, 0, true, false, false, false, false, false, {0, 0}},
      {0, 0, true, false, false, false, false, false, {0, 0}},
      {0, 0, true, false, false, false, false, false, {0, 0}}},
     {{0, 0, -1, 0},
      {0, 0, -1, 0},
      {0, 0, -1, 0},
      {0, 0, -1, 0},
      {0, 0, -1, 0},
      {0, 0, -1, 0},
      {0, 0, -1, 0},
      {0, 0, -1, 0},
      {0, 0, -1, 0},
      {0, 0, -1, 0},
      {0, 0, -1, 0},
      {0, 0, -1, 0},
      {0, 0, -1, 0},
      {0, 0, -1, 0},
      {0, 0, -1, 0},
      {0, 0, -1, 0}},
     {false, 0, 0, 0, 0},
     {0, 0xFF, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}}};

static qsfp_channel_fsm_t qsfp_channel_fsm_initial_state = {
    QSFP_CH_FSM_ST_DISABLED,
    0,
    true,
    false,
    false,
    false,
    false,
    false,
    {0, 0}};
static qsfp_status_and_alarms_t qsfp_alarms_initial_state = {
    0, 0xFF, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

// fwd ref
static void qsfp_fsm_reset_de_assert(int conn_id);
static void qsfp_fsm_lpmode_assert(int conn_id);
static int qsfp_num_media_lanes_get(int conn_id, int chnl_id);
static int qsfp_num_host_lanes_get(int conn_id, int chnl_id);

/****************************************************
*
****************************************************/
static int bf_fsm_qsfp_rd(
    unsigned int module, int page, int offset, int len, uint8_t *buf) {
  int rc;

  // bank 0 hard-coded for now
  rc = bf_qsfp_module_read(module, 0, page, offset, len, buf);

  // LOG_DEBUG("QSFP    %2d : Rd : pg=%2d : offset=%2d : len=%3d :
  // data[0]=%02x",
  //          module, page, offset, len, buf[0]);

  return rc;
}

/****************************************************
*
****************************************************/
static int bf_fsm_qsfp_wr(
    unsigned int module, int page, int offset, int len, uint8_t *buf) {
  int rc;

  // bank 0 hard-coded for now
  rc = bf_qsfp_module_write(module, 0, page, offset, len, buf);

  // LOG_DEBUG("QSFP    %2d : Wr : pg=%2d : offset=%2d : len=%3d :
  // data[0]=%02x",
  //          module, page, offset, len, buf[0]);

  return rc;
}

/****************************************************
* called by qsfp_fsm to set next time to process QSFP
****************************************************/
static void qsfp_fsm_next_run_time_set(struct timespec *next_run_time,
                                       int delay_ms) {
  struct timespec now;

  clock_gettime(CLOCK_MONOTONIC, &now);

  next_run_time->tv_sec = now.tv_sec + (delay_ms / 1000);
  next_run_time->tv_nsec =
      now.tv_nsec + (((delay_ms % 1000) * 1000000) % 1000000000);
  if (next_run_time->tv_nsec >= 1000000000) {
    next_run_time->tv_sec++;
    next_run_time->tv_nsec -= 1000000000;
  }
}

/****************************************************
* called by qsfp_scan to identify new QSFP to process
****************************************************/
void qsfp_fsm_inserted(int conn_id) {
  if (conn_id > BF_PLAT_MAX_QSFP) {
    LOG_ERROR(
        "QSFPMSM %s : %2d conn_id exceeed max supported\n", __func__, conn_id);
    return;
  }

  qsfp_fsm_state_t prev_st = qsfp_state[conn_id].fsm_st;

  qsfp_state[conn_id].fsm_st = QSFP_FSM_ST_INSERTED;
  qsfp_state[conn_id].next_fsm_run_time.tv_sec = 0;
  qsfp_state[conn_id].next_fsm_run_time.tv_nsec = 0;
  qsfp_fsm_next_run_time_set(&qsfp_state[conn_id].next_flag_poll_time, 0);

  qsfp_state[conn_id].status_and_alarms.los_ind = 0xFF;  // all LOS bits

  LOG_DEBUG("QSFPMSM %2d : %s --> QSFP_FSM_ST_INSERTED",
            conn_id,
            qsfp_fsm_st_to_str[prev_st]);
}

/****************************************************
* called by qsfp_scan to indicate QSFP no longer
* requires processing
****************************************************/
void qsfp_fsm_removed(int conn_id) {
  qsfp_fsm_state_t prev_st = qsfp_state[conn_id].fsm_st;

  qsfp_state[conn_id].next_fsm_run_time.tv_sec = 0;
  qsfp_state[conn_id].next_fsm_run_time.tv_nsec = 0;

  qsfp_state[conn_id].fsm_st = QSFP_FSM_ST_REMOVED;
  LOG_DEBUG("QSFPMSM %2d : %s --> QSFP_FSM_ST_REMOVED",
            conn_id,
            qsfp_fsm_st_to_str[prev_st]);
}

/****************************************************
* called to force all modules to low power mode
* (except copper cables, since they are inherently low power)
****************************************************/
void qsfp_fsm_force_all_lpmode() {
  int conn_id;

  for (conn_id = 1; conn_id <= bf_pm_num_ports_get(); conn_id++) {
    if ((qsfp_state[conn_id].fsm_st != QSFP_FSM_ST_IDLE) &&
        (qsfp_state[conn_id].fsm_st != QSFP_FSM_ST_REMOVED)) {
      if (qsfp_state[conn_id].qsfp_type != QSFP_TYP_COPPER) {
        qsfp_fsm_state_t prev_st = qsfp_state[conn_id].fsm_st;

        qsfp_state[conn_id].next_fsm_run_time.tv_sec = 0;
        qsfp_state[conn_id].next_fsm_run_time.tv_nsec = 0;

        qsfp_state[conn_id].fsm_st = QSFP_FSM_ST_WAIT_TON_LPMODE;
        LOG_DEBUG("QSFPMSM %2d : %s --> QSFP_FSM_ST_WAIT_TON_LPMODE",
                  conn_id,
                  qsfp_fsm_st_to_str[prev_st]);
      }
    }
  }
}

/****************************************************
* returns true when all (non-copper) conn_ids are in low power mode
* (except copper cables, since they are inherently low power)
****************************************************/
bool qsfp_fsm_query_all_lpmode() {
  int conn_id;
  bool all_complete = true;

  for (conn_id = 1; conn_id <= bf_pm_num_ports_get(); conn_id++) {
    if ((qsfp_state[conn_id].fsm_st != QSFP_FSM_ST_IDLE) &&
        (qsfp_state[conn_id].fsm_st != QSFP_FSM_ST_REMOVED)) {
      if (qsfp_state[conn_id].qsfp_type != QSFP_TYP_COPPER) {
        if (qsfp_state[conn_id].fsm_st != QSFP_FSM_ST_LPMODE) {
          all_complete = false;
          return all_complete;
        }
      }
    }
  }
  return all_complete;
}

/****************************************************
*
****************************************************/
/*static*/ void qsfp_needs_hi_pwr_init_set(int conn_id, bool needs) {
  qsfp_state[conn_id].needs_hi_pwr_init = needs;
}

/****************************************************
*
****************************************************/
bool qsfp_needs_hi_pwr_init(int conn_id) {
  return (qsfp_state[conn_id].needs_hi_pwr_init);
}

/****************************************************
* called by qsfp_fsm. If QSFP type has already been
* identified, returns known type. If not, attempts
* to determine type based on compliance and
* extended compliance fields.
* If any error determining type return -1
****************************************************/
static int qsfp_fsm_identify_type(int conn_id, bool *is_optical) {
  if (qsfp_state[conn_id].qsfp_type == QSFP_TYP_OPTICAL) {
    *is_optical = true;
    return 0;
  }
  if (qsfp_state[conn_id].qsfp_type == QSFP_TYP_COPPER) {
    *is_optical = false;
    return 0;
  }

  // see if flat_mem or paged
  qsfp_state[conn_id].flat_mem = bf_qsfp_is_flat_mem(conn_id);

  // if we get to this point, the qsfp_type is QSFP_TYP_UNKNOWN, so we need
  // to figure it out
  if (bf_qsfp_is_optical(conn_id)) {
    qsfp_state[conn_id].qsfp_type = QSFP_TYP_OPTICAL;
    qsfp_state[conn_id].ch_cnt = bf_qsfp_get_ch_cnt(conn_id);
    if (bf_qsfp_is_cmis(conn_id)) {
      qsfp_state[conn_id].media_ch_cnt = bf_qsfp_get_media_ch_cnt(conn_id);
    } else {
      qsfp_state[conn_id].media_ch_cnt = qsfp_state[conn_id].ch_cnt;
    }
    *is_optical = true;
  } else {
    qsfp_state[conn_id].qsfp_type = QSFP_TYP_COPPER;
    *is_optical = false;
    // since qsfp-fsm doesnot run on copper module, no
    // other information is necessary
  }

  return 0;
}

/****************************************************
* Determine if QSFP has any special requirements.
* Based on a priori knowledge of QSFP Vendor/PN
****************************************************/
static int qsfp_fsm_identify_model_requirements(int conn_id) {
  int rc;
  qsfp_vendor_info_t vendor;

  rc = bf_qsfp_get_vendor_info(conn_id, &vendor);
  if (!rc) {
    LOG_ERROR("QSFP    %2d : Error retrieving vendor info", conn_id);
    return -1;
  }

  if (strncmp(vendor.name, "LUXTERA", 7) == 0) {
    qsfp_needs_hi_pwr_init_set(conn_id, true);
  } else if (strncmp(vendor.name, "Alibaba", 7) == 0) {
    if (strncmp(vendor.part_number, "AB-QT0613P4n", 12) ==
        0) {  // JT 6/12/18 : from alibaba_co_v23.hex
      qsfp_needs_hi_pwr_init_set(
          conn_id,
          true);  // JT 6/12/18 : enable Alibaba-Luxtera detection and handling
    }
  } else if (strncmp(vendor.name, "Alibaba", 7) == 0) {
    if (strncmp(vendor.part_number, "AB-QT0613P4i03", 12) ==
        0) {  // JT 6/13/18 : from email Craig@Luxtera
      qsfp_needs_hi_pwr_init_set(
          conn_id,
          true);  // JT 6/13/18 : enable Alibaba-Luxtera detection and handling
    }
  } else if (strncmp(vendor.name, "JUNIPER-LUXTERA", 7) == 0) {
    if (strncmp(vendor.part_number, "LUX42604BO", 12) == 0) {  // JT 6/13/18
      qsfp_needs_hi_pwr_init_set(conn_id, true);               // JT 6/13/18
    }
  } else if (strncmp(vendor.name, "Arista Networks", 7) == 0) {
    if (strncmp(vendor.part_number, "QSFP-100G-PSM4", 12) == 0) {  // JT 6/13/18
      qsfp_needs_hi_pwr_init_set(conn_id, true);                   // JT 6/13/18
    }
  } else if (strncmp(vendor.name, "CISCO-LUXTERA", 7) == 0) {
    if (strncmp(vendor.part_number, "LUX42604BO", 12) == 0) {  // JT 6/13/18
      qsfp_needs_hi_pwr_init_set(conn_id, true);               // JT 6/13/18
    }
  } else if (strncmp(vendor.name, "CISCO-LUXTERA", 7) == 0) {
    if (strncmp(vendor.part_number, "LUX42604BOC", 12) == 0) {  // JT 6/13/18
      qsfp_needs_hi_pwr_init_set(conn_id, true);                // JT 6/13/18
    }
  }

  if (qsfp_needs_hi_pwr_init(conn_id)) {
    LOG_DEBUG("QSFP    %2d : %s : %s : requires hi-pwr init",
              conn_id,
              vendor.name,
              vendor.part_number);
  }
  return 0;
}

/****************************************************
* called by qsfp_fsm. If QSFP type has already been
* identified, returns known type. If not, attempts
* to determine type based on compliance and
* extended compliance fields.
****************************************************/
static bool qsfp_fsm_is_optical(int conn_id) {
  if (qsfp_state[conn_id].qsfp_type == QSFP_TYP_OPTICAL) {
    return true;
  }
  return false;
}

/*****************************************************************
*
*****************************************************************/
/*static*/ void qsfp_start_prbs31(bf_dev_id_t dev_id, int conn_id) {
  int ch;
  bf_pltfm_port_info_t port_info;
  bf_pltfm_ext_phy_cfg_t port_cfg;
  bool is_present;
  bf_dev_id_t dev_id_of_port = 0;

  LOG_DEBUG("QSFP    %2d : Start PRBS31", conn_id);
  bf_serdes_prbs_mode_set(dev_id, BF_PORT_PRBS_MODE_31);

  port_info.conn_id = conn_id;
  port_info.chnl_id = 0;
  bf_bd_cfg_port_has_rtmr(&port_info, &is_present);

  for (ch = 0; ch < 4; ch++) {
    bf_dev_port_t dev_port;
    bf_pal_front_port_handle_t port_hdl;
    bf_status_t rc;
    bf_pltfm_mac_lane_info_t mac_lane_info;

    port_hdl.conn_id = conn_id;
    port_hdl.chnl_id = ch;
    bf_pm_port_front_panel_port_to_dev_port_get(
        &port_hdl, &dev_id_of_port, &dev_port);
    dev_id = dev_id_of_port;

    if (!is_present) {
      bf_serdes_prbs_init(dev_id, dev_port, 0, 25);
    }

    // set Tx Eq settings
    port_info.conn_id = conn_id;
    port_info.chnl_id = ch;
    rc = bf_bd_cfg_port_mac_lane_info_get(
        &port_info, BF_PLTFM_QSFP_OPT, &mac_lane_info);

    LOG_DEBUG(
        "QSFP    %2d : ch[%d] : rc=%d : PN=%d : Atn=%d : Post=%d : Pre=%d",
        conn_id,
        ch,
        rc,
        mac_lane_info.tx_phy_pn_swap,
        mac_lane_info.tx_attn,
        mac_lane_info.tx_post,
        mac_lane_info.tx_pre);
    if (rc != BF_SUCCESS) {
      LOG_DEBUG("QSFP    %2d : ch[%d] : Error <%x> getting Tx EQ+P/N",
                conn_id,
                ch,
                rc);
      continue;
    }
    rc = bf_serdes_tx_drv_inv_set_allow_unassigned(
        dev_id, dev_port, 0, mac_lane_info.tx_phy_pn_swap);
    if (rc != BF_SUCCESS) {
      LOG_DEBUG(
          "QSFP    %2d : ch[%d] : Error <%x> setting Tx P/N", conn_id, ch, rc);
    }
    rc = bf_serdes_tx_drv_attn_set_allow_unassigned(dev_id,
                                                    dev_port,
                                                    0,
                                                    mac_lane_info.tx_attn,
                                                    mac_lane_info.tx_post,
                                                    mac_lane_info.tx_pre);
    if (rc != BF_SUCCESS) {
      LOG_DEBUG("QSFP    %2d : ch[%d] : Error <%x> Tx EQ", conn_id, ch, rc);
    }
    // Luxtera seems to "like" a certain setting during MZI init
    bf_serdes_tx_loop_bandwidth_set(
        dev_id, dev_port, 0, BF_SDS_TOF_TX_LOOP_BANDWIDTH_9MHZ);
  }
  // If this is a retimer port, set the retimer in 25g/optical mode
  port_info.conn_id = conn_id;
  port_info.chnl_id = 0;
  bf_bd_cfg_port_has_rtmr(&port_info, &is_present);
  if (is_present) {
    LOG_DEBUG("QSFP    %2d : Set retimer mode 25g+optical", conn_id);
    for (ch = 0; ch < 4; ch++) {
      port_info.conn_id = conn_id;
      port_info.chnl_id = ch;

      port_cfg.speed_cfg = BF_SPEED_25G;
      port_cfg.is_an_on = 0;
      port_cfg.is_optic = true;

      bf_pltfm_ext_phy_set_mode(&port_info, &port_cfg);
    }
  }
  if (is_present) {
    LOG_DEBUG("QSFP    %2d : Start retimer PRBS gen", conn_id);
    for (ch = 3; ch >= 0; ch--) {
      port_info.chnl_id = ch;
      bf_pltfm_platform_ext_phy_prbs_set(&port_info, 1, 0 /* PRBS9 prbs_mode*/);
    }
  }
}

/*****************************************************************
*
*****************************************************************/
static void qsfp_fsm_st_removed(bf_dev_id_t dev_id, int conn_id) {
  int ch;
  bf_dev_id_t dev_id_of_port = 0;

  // forget the page and bank number
  // invalidate_bankpage(conn_id);
  // reset type
  qsfp_state[conn_id].qsfp_type = QSFP_TYP_UNKNOWN;
  // reset alarm states
  qsfp_state[conn_id].status_and_alarms = qsfp_alarms_initial_state;
  // reset wr_coalesce state
  qsfp_module_fsm_complete_update(dev_id, conn_id);
  // set LPMODE GPIO to LPmode, for the next time a cable is inserted
  qsfp_fsm_lpmode_assert(conn_id);
  // deassert reset, in case module was removed while reset was asserted
  qsfp_fsm_reset_de_assert(conn_id);
  // fixme
  // clear "is_optical" indications on any ports the driver may have
  for (ch = 0; ch < 4; ch++) {
    bf_dev_port_t dev_port;
    bf_pal_front_port_handle_t port_hdl;

    port_hdl.conn_id = conn_id;
    port_hdl.chnl_id = ch;
    bf_pm_port_front_panel_port_to_dev_port_get(
        &port_hdl, &dev_id_of_port, &dev_port);
    dev_id = dev_id_of_port;

    bf_port_optical_xcvr_ready_set(dev_id, dev_port, false);
    // hack, really need # lanes on port
    // clear is_optical to driver
    bf_port_is_optical_xcvr_set(dev_id, dev_port, false);
    // assert LOS to driver
    // this wont matter if the next QSFP inserted is not optical
    bf_port_optical_los_set(dev_id, dev_port, true);
  }
}

/*****************************************************************
*
*****************************************************************/
static void qsfp_fsm_reset_de_assert(int conn_id) {
  int rc;

  LOG_DEBUG("QSFP    %2d : RESETL = false", conn_id);
  // de-assert resetL
  rc = bf_qsfp_reset(conn_id, false);
  if (rc != 0) {
    LOG_ERROR("QSFP    %2d : Error <%d> de-asserting resetL", conn_id, rc);
  }
}

/*****************************************************************
*
*****************************************************************/
static void qsfp_fsm_reset_assert(int conn_id) {
  int rc;

  LOG_DEBUG("QSFP    %2d : RESETL = true", conn_id);
  // assert resetL
  rc = bf_qsfp_reset(conn_id, true);
  if (rc != 0) {
    LOG_ERROR("QSFP    %2d : Error <%d> asserting resetL", conn_id, rc);
  }
}

/*****************************************************************
*
*****************************************************************/
static void qsfp_fsm_lpmode_de_assert(int conn_id) {
  int rc;

  LOG_DEBUG("QSFP    %2d : LPMODE = false", conn_id);
  // de-assert LPMODE
  rc = bf_qsfp_set_transceiver_lpmode(conn_id, false);
  if (rc != 0) {
    LOG_ERROR("QSFP    %2d : Error <%d> de-asserting LPMODE", conn_id, rc);
  }
}

/*****************************************************************
*
*****************************************************************/
static void qsfp_fsm_lpmode_assert(int conn_id) {
  int rc;

  LOG_DEBUG("QSFP    %2d : LPMODE = true", conn_id);
  // assert LPMODE
  rc = bf_qsfp_set_transceiver_lpmode(conn_id, true);
  if (rc != 0) {
    LOG_ERROR("QSFP    %2d : Error <%d> asserting LPMODE", conn_id, rc);
  }
}

/*****************************************************************
*
*****************************************************************/
void qsfp_deassert_all_reset_pins(void) {
  int conn_id;

  LOG_DEBUG("QSFP: Assert RESETL on all QSFPs");

  for (conn_id = 1; conn_id <= bf_pm_num_ports_get(); conn_id++) {
    if (bf_bd_is_this_port_internal(conn_id, 0)) {
      continue;
    }
    qsfp_fsm_reset_assert(conn_id);
  }
  bf_sys_usleep(CMISand8636_TMR_t_reset_init * 1000);  // takes micro-seconds

  LOG_DEBUG("QSFP: Assert LPMODE and De-assert RESETL on all QSFPs");

  for (conn_id = 1; conn_id <= bf_pm_num_ports_get(); conn_id++) {
    if (bf_bd_is_this_port_internal(conn_id, 0)) {
      continue;
    }
    // assert LP_MODE
    qsfp_fsm_lpmode_assert(conn_id);
    qsfp_fsm_reset_de_assert(conn_id);
  }
  LOG_DEBUG("QSFP: Wait 2 sec (t_reset) for QSFPs to initialize..");
  bf_sys_usleep(CMISand8636_TMR_t_reset * 1000);  // takes micro-seconds
}

/*****************************************************************
*
*****************************************************************/
static void qsfp_fsm_st_reset(bf_dev_id_t dev_id, int conn_id) {
  qsfp_fsm_reset_de_assert(conn_id);
}

/*****************************************************************
*
*****************************************************************/
static void qsfp_fsm_st_inserted(bf_dev_id_t dev_id, int conn_id) {
  // assert resetL
  qsfp_fsm_reset_assert(conn_id);

  // Luxtera modules require LPMODE de-asserted during init seq
  if (qsfp_needs_hi_pwr_init(conn_id)) {
    // de-assert LP_MODE
    qsfp_fsm_lpmode_de_assert(conn_id);
    // start PRBS31 on all 4 lanes
    qsfp_start_prbs31(dev_id, conn_id);
  } else {
    // assert LP_MODE
    qsfp_fsm_lpmode_assert(conn_id);
  }
}

/*****************************************************************
* Disables all TX lanes
*****************************************************************/
static void qsfp_fsm_st_tx_disable(bf_dev_id_t dev_id, int conn_id) {
  int chmask = (1 << qsfp_state[conn_id].media_ch_cnt) - 1;

  // Luxtera modules require TX_DISABLE de-asserted during init seq
  if (qsfp_needs_hi_pwr_init(conn_id)) {
    bf_qsfp_tx_disable(conn_id, chmask, false);
  } else {
    bf_qsfp_tx_disable(conn_id, chmask, true);
  }
}

/*****************************************************************
*
*****************************************************************/
void qsfp_lpmode_sw_set(bf_dev_id_t dev_id, int conn_id, bool lpmode) {
  bf_qsfp_set_pwr_ctrl(conn_id, lpmode);
}

/*****************************************************************
*
*****************************************************************/
static void qsfp_fsm_st_lp_mode_off(bf_dev_id_t dev_id, int conn_id) {
  qsfp_lpmode_sw_set(dev_id, conn_id, 0);
}

/*****************************************************************
* This routine is only used in Tofino 1 implementations
*****************************************************************/
static void qsfp_log_alarms(bf_dev_id_t dev_id,
                            int conn_id,
                            qsfp_status_and_alarms_t *old,
                            qsfp_status_and_alarms_t *new) {
  if (memcmp((char *)old, (char *)new, sizeof(*old)) != 0) {
    // log the raw fields to provide current state of all alarms
    LOG_DEBUG(
        "QSFP    %2d : Alarms (Bytes 2-14) : %02x %02x %02x %02x %02x"
        " %02x %02x %02x %02x %02x %02x %02x %02x",
        conn_id,
        new->status2,
        new->los_ind,
        new->eq_laser_fault,
        new->lol_ind,
        new->temp_alarm,
        new->vcc_alarm,
        new->vendor_spec,
        new->rx_power_alarm12,
        new->rx_power_alarm34,
        new->tx_bias_alarm12,
        new->tx_bias_alarm34,
        new->tx_power_alarm12,
        new->tx_power_alarm34);
    // now, log changes
    if (old->status2 != new->status2) {
      if ((old->status2 & BYTE2_INTL) != (new->status2 &BYTE2_INTL)) {
        LOG_DEBUG("QSFP    %2d : IntL : %d -> %d",
                  conn_id,
                  (old->status2 & BYTE2_INTL) >> 1,
                  (new->status2 &BYTE2_INTL) >> 1);
      }
      if ((old->status2 & BYTE2_DATA_NOT_READY) !=
          (new->status2 &BYTE2_DATA_NOT_READY)) {
        LOG_DEBUG("QSFP    %2d : Data_Not_Ready: %d -> %d",
                  conn_id,
                  (old->status2 & BYTE2_DATA_NOT_READY),
                  (new->status2 &BYTE2_DATA_NOT_READY));
      }
    }
    if (old->los_ind != new->los_ind) {
      LOG_DEBUG(
          "QSFP    %2d : [7:4] Tx LOS: %1x -> %1x : [3:0] Rx LOS: %1x -> %1x",
          conn_id,
          old->los_ind >> 4,
          new->los_ind >> 4,
          old->los_ind & 0xF,
          new->los_ind & 0xF);
    }
    if (old->eq_laser_fault != new->eq_laser_fault) {
      LOG_DEBUG(
          "QSFP    %2d : [7:4] Tx Adapt EQ Fault: %1x -> %1x : [3:0] Tx Laser "
          "fault: %1x -> %1x",
          conn_id,
          old->eq_laser_fault >> 4,
          new->eq_laser_fault >> 4,
          old->eq_laser_fault & 0xF,
          new->eq_laser_fault & 0xF);
    }
    if (old->lol_ind != new->lol_ind) {
      LOG_DEBUG(
          "QSFP    %2d : [7:4] Tx CDR LOL: %1x -> %1x : [3:0] Rx CDR LOL: %1x "
          "-> "
          "%1x",
          conn_id,
          old->lol_ind >> 4,
          new->lol_ind >> 4,
          old->lol_ind & 0xF,
          new->lol_ind & 0xF);
    }
    if (old->temp_alarm != new->temp_alarm) {
      LOG_DEBUG("QSFP    %2d : Temp Alarm/Warn: %02x -> %02x",
                conn_id,
                old->temp_alarm,
                new->temp_alarm);
    }
    if (old->vcc_alarm != new->vcc_alarm) {
      LOG_DEBUG("QSFP    %2d : Vcc Alarm/Warn: %02x -> %02x",
                conn_id,
                old->vcc_alarm,
                new->vcc_alarm);
    }
    if (old->vendor_spec != new->vendor_spec) {
      LOG_DEBUG("QSFP    %2d : Vendor Sepcific (Byte 8): %02x -> %02x",
                conn_id,
                old->vendor_spec,
                new->vendor_spec);
    }
    if (old->rx_power_alarm12 != new->rx_power_alarm12) {
      LOG_DEBUG("QSFP    %2d : Rx Power Alarm/Warn CH1/2: %02x -> %02x",
                conn_id,
                old->rx_power_alarm12,
                new->rx_power_alarm12);
    }
    if (old->rx_power_alarm34 != new->rx_power_alarm34) {
      LOG_DEBUG("QSFP    %2d : Rx Power Alarm/Warn CH3/4: %02x -> %02x",
                conn_id,
                old->rx_power_alarm34,
                new->rx_power_alarm34);
    }
    if (old->tx_bias_alarm12 != new->tx_bias_alarm12) {
      LOG_DEBUG("QSFP    %2d : Tx Bias Alarm/Warn CH1/2: %02x -> %02x",
                conn_id,
                old->tx_bias_alarm12,
                new->tx_bias_alarm12);
    }
    if (old->tx_bias_alarm34 != new->tx_bias_alarm34) {
      LOG_DEBUG("QSFP    %2d : Tx Bias Alarm/Warn CH3/4: %02x -> %02x",
                conn_id,
                old->tx_bias_alarm34,
                new->tx_bias_alarm34);
    }
    if (old->tx_power_alarm12 != new->tx_power_alarm12) {
      LOG_DEBUG("QSFP    %2d : Tx Power Alarm/Warn CH1/2: %02x -> %02x",
                conn_id,
                old->tx_power_alarm12,
                new->tx_power_alarm12);
    }
    if (old->tx_power_alarm34 != new->tx_power_alarm34) {
      LOG_DEBUG("QSFP    %2d : Tx Power Alarm/Warn CH3/4: %02x -> %02x",
                conn_id,
                old->tx_power_alarm34,
                new->tx_power_alarm34);
    }
  }
}

/* log all interesting module-level flags
 * Tofino 2 implementations only */
static void qsfp_log_module_flags(int conn_id) {
  int loopcnt, loopmax, flag_val;
  qsfp_flag_log_t *module_flag_log;

  if (bf_qsfp_is_cmis(conn_id)) {
    module_flag_log = cmis_module_flag_log;
    loopmax = sizeof(cmis_module_flag_log) / sizeof(qsfp_flag_log_t);
  } else {
    module_flag_log = sff8636_module_flag_log;
    loopmax = sizeof(sff8636_module_flag_log) / sizeof(qsfp_flag_log_t);
  }

  // get current flag values
  for (loopcnt = 0; loopcnt < loopmax; loopcnt++) {
    // move the last value read into the 'previous' variable
    module_flag_log[loopcnt].fval_prev[conn_id] =
        module_flag_log[loopcnt].fval_cur[conn_id];

    // get the current flag value from the cache
    flag_val = bf_qsfp_get_flag(conn_id,
                                module_flag_log[loopcnt].flag,
                                0,
                                module_flag_log[loopcnt].bitnum,
                                NULL);

    // log the new value, if appropriate
    if (flag_val >= 0) {
      module_flag_log[loopcnt].fval_cur[conn_id] = flag_val;
      if (flag_val > module_flag_log[loopcnt].fval_prev[conn_id]) {
        if (module_flag_log[loopcnt].logtype == FLAG_ERROR) {
          LOG_ERROR("QSFP    %2d : %s change (0 -> 1)",
                    conn_id,
                    module_flag_log[loopcnt].desc);
        } else {
          LOG_DEBUG("QSFP    %2d : %s change (0 -> 1)",
                    conn_id,
                    module_flag_log[loopcnt].desc);
        }
      } else if ((flag_val < module_flag_log[loopcnt].fval_prev[conn_id]) &&
                 (module_flag_log[loopcnt].type == FLAG_TYPE_STATUS)) {
        if (module_flag_log[loopcnt].logtype == FLAG_ERROR) {
          LOG_ERROR("QSFP    %2d : %s change (1 -> 0)",
                    conn_id,
                    module_flag_log[loopcnt].desc);
        } else {
          LOG_DEBUG("QSFP    %2d : %s change (1 -> 0)",
                    conn_id,
                    module_flag_log[loopcnt].desc);
        }
      }
    }
  }
}

/* clear all logged flags from the cache, in preparation for the next fetch
 * Tofino 2 implementations only */
static void clear_log_module_flags(int conn_id) {
  int loopcnt, loopmax;
  qsfp_flag_log_t *module_flag_log;

  if (bf_qsfp_is_cmis(conn_id)) {
    module_flag_log = cmis_module_flag_log;
    loopmax = sizeof(cmis_module_flag_log) / sizeof(qsfp_flag_log_t);
  } else {
    module_flag_log = sff8636_module_flag_log;
    loopmax = sizeof(sff8636_module_flag_log) / sizeof(qsfp_flag_log_t);
  }

  // get current flag values
  for (loopcnt = 0; loopcnt < loopmax; loopcnt++) {
    // clear the current flag value in the cache
    bf_qsfp_clear_flag(conn_id,
                       module_flag_log[loopcnt].flag,
                       0,
                       module_flag_log[loopcnt].bitnum,
                       false);
  }
}

/* log all interesting lane-specific flags
 * Tofino 2 implementations only */
static void qsfp_log_lane_flags(int conn_id) {
  int loopcnt, loopmax, flag_val, ln;
  bool logflag;

  loopmax = sizeof(ln_flag_log) / sizeof(qsfp_flag_log_t);

  // get current flag values
  for (loopcnt = 0; loopcnt < loopmax; loopcnt++) {
    logflag = false;

    // move the last value read into the 'previous' variable
    ln_flag_log[loopcnt].fval_prev[conn_id] =
        ln_flag_log[loopcnt].fval_cur[conn_id];

    // clear the pending value so we can OR in new values
    ln_flag_log[loopcnt].fval_cur[conn_id] = 0;

    for (ln = 0; ln < qsfp_state[conn_id].ch_cnt; ln++) {
      // get the current flag value for each lane from the cache
      flag_val =
          bf_qsfp_get_flag(conn_id, ln_flag_log[loopcnt].flag, ln, 0, NULL);

      // if set, add to bitmask for this flag
      if (flag_val >= 0) {
        ln_flag_log[loopcnt].fval_cur[conn_id] |= flag_val << ln;
        if (flag_val >
            ((ln_flag_log[loopcnt].fval_prev[conn_id] >> ln) & 0x1)) {
          logflag = true;
        } else if ((flag_val <
                    ((ln_flag_log[loopcnt].fval_prev[conn_id] >> ln) & 0x1)) &&
                   (ln_flag_log[loopcnt].type == FLAG_TYPE_STATUS)) {
          logflag = true;
        }
      }
    }

    if (logflag) {
      if (ln_flag_log[loopcnt].logtype == FLAG_ERROR) {
        LOG_ERROR("QSFP    %2d : %s change (0x%0*x -> 0x%0*x)",
                  conn_id,
                  ln_flag_log[loopcnt].desc,
                  (qsfp_state[conn_id].ch_cnt / 4),
                  ln_flag_log[loopcnt].fval_prev[conn_id],
                  (qsfp_state[conn_id].ch_cnt / 4),
                  ln_flag_log[loopcnt].fval_cur[conn_id]);
      } else {
        LOG_DEBUG("QSFP    %2d : %s change (0x%0*x -> 0x%0*x)",
                  conn_id,
                  ln_flag_log[loopcnt].desc,
                  (qsfp_state[conn_id].ch_cnt / 4),
                  ln_flag_log[loopcnt].fval_prev[conn_id],
                  (qsfp_state[conn_id].ch_cnt / 4),
                  ln_flag_log[loopcnt].fval_cur[conn_id]);
      }
    }
  }
}

/* clear all logged flags from the cache, in preparation for the next fetch
 * Tofino 2 implementations only */
static void clear_log_lane_flags(int conn_id, int ln) {
  int loopcnt, loopmax;

  loopmax = sizeof(ln_flag_log) / sizeof(qsfp_flag_log_t);

  // get current flag values
  for (loopcnt = 0; loopcnt < loopmax; loopcnt++) {
    // clear the current flag value in the cache
    bf_qsfp_clear_flag(conn_id, ln_flag_log[loopcnt].flag, ln, 0, false);
  }
}

/*****************************************************************
* This routine is only used in Tofino 1 implementations
*****************************************************************/
static void qsfp_fsm_check_alarms_after_unreset(bf_dev_id_t dev_id,
                                                int conn_id) {
  qsfp_status_and_alarms_t status_and_alarms;
  int rc;

  LOG_DEBUG("QSFP    %2d : Clear existing alarms..", conn_id);
  rc = bf_fsm_qsfp_rd(
      conn_id, 0, 2, sizeof(status_and_alarms), (uint8_t *)&status_and_alarms);
  if (rc) {
    LOG_ERROR("QSFP    %2d : Error <%d> reading status fields", conn_id, rc);
    return;  // can't trust the data, just leave
  }
  qsfp_log_alarms(dev_id,
                  conn_id,
                  &qsfp_state[conn_id].status_and_alarms,
                  &status_and_alarms);

  // copy new alarm state into out cached struct
  qsfp_state[conn_id].status_and_alarms = status_and_alarms;

  LOG_DEBUG("QSFP    %2d : Now check for current alarms..", conn_id);
  rc = bf_fsm_qsfp_rd(
      conn_id, 0, 2, sizeof(status_and_alarms), (uint8_t *)&status_and_alarms);
  if (rc) {
    LOG_ERROR("QSFP    %2d : Error <%d> reading status fields", conn_id, rc);
    return;  // can't trust the data, just leave
  }
  qsfp_log_alarms(dev_id,
                  conn_id,
                  &qsfp_state[conn_id].status_and_alarms,
                  &status_and_alarms);

  // copy new alarm state into out cached struct
  qsfp_state[conn_id].status_and_alarms = status_and_alarms;
}

/*****************************************************************
* This routine is only used in Tofino 1 implementations
*****************************************************************/
static int qsfp_fsm_poll_los(bf_dev_id_t dev_id, int conn_id) {
  qsfp_status_and_alarms_t status_and_alarms;
  int rc, ch;
  bf_dev_id_t dev_id_of_port = 0;

  // Donot move FSM until removal is hanlded
  if (qsfp_state[conn_id].qsfp_quick_removed) {
    return -1;
  }

  // read status and alarms (bytes2-14) onto stack. This is in case
  // there is an error on the read we dont corrupt the qsfp_state
  // structure. If no error, copy into struct
  rc = bf_fsm_qsfp_rd(
      conn_id, 0, 2, sizeof(status_and_alarms), (uint8_t *)&status_and_alarms);
  if (rc) {
    // Detect and latch the removal
    // state, so that can qsfp-scan handle the rest.
    if (!qsfp_state[conn_id].qsfp_quick_removed) {
      bool is_present = 0;
      bf_qsfp_detect_transceiver(conn_id, &is_present);
      if (!is_present) {
        qsfp_state[conn_id].qsfp_quick_removed = true;
        LOG_DEBUG("QSFP    %2d : Removal detected", conn_id);
        qsfp_state[conn_id].qsfp_quick_removed = true;
        bf_pm_qsfp_quick_removal_detected_set(conn_id, true);
        return -1;
      }
    }

    LOG_ERROR("QSFP    %2d : Error <%d> reading status fields", conn_id, rc);
    return 0;  // can't trust the data, just leave
  }
  qsfp_log_alarms(dev_id,
                  conn_id,
                  &qsfp_state[conn_id].status_and_alarms,
                  &status_and_alarms);

  // During HA, we really donot know when the config replay ends.
  // So simply push the current RX-los to the SDK in order for port-FSM
  // to continue after warm-init ends.
  //
  // If there is NO RX, SDK probably will detect link-down faster.
  //
  // If there is a RX and link is already UP, SDK will NOT act on this
  // los-flag. Hence we donot have to tie to link-up here or to port-FSM
  // states.

  // only want Rx LOS bits
  uint8_t rx_los_now, rx_los_before;
  uint8_t rx_lol_now, rx_lol_before;

  rx_los_now = status_and_alarms.los_ind & 0xF;
  rx_los_before = qsfp_state[conn_id].status_and_alarms.los_ind & 0xF;

  rx_lol_now = status_and_alarms.lol_ind & 0xF;
  rx_lol_before = qsfp_state[conn_id].status_and_alarms.lol_ind & 0xF;

  // read TX_DISABLE state (once)
  uint8_t byte_86 = 0;
  rc = bf_fsm_qsfp_rd(conn_id, 0, 86, 1, &byte_86);
  if (rc) {
    LOG_ERROR(
        "QSFP: %2d : Error <%d> reading TX_DISABLE (byte 86)", conn_id, rc);
  }
  for (ch = 0; ch < 4; ch++) {
    bf_dev_port_t dev_port;
    bf_status_t bf_status;
    bf_pal_front_port_handle_t port_hdl;
    bool los =
        (((rx_los_now >> ch) & 1) || ((rx_lol_now >> ch) & 1)) ? true : false;

    port_hdl.conn_id = conn_id;
    port_hdl.chnl_id = ch;
    bf_pm_port_front_panel_port_to_dev_port_get(
        &port_hdl, &dev_id_of_port, &dev_port);
    dev_id = dev_id_of_port;

    if (!bf_pm_intf_is_device_family_tofino(dev_id)) {
      if (((rx_los_now & (1 << ch)) ^ (rx_los_before & (1 << ch))) ||
          ((rx_lol_now & (1 << ch)) ^ (rx_lol_before & (1 << ch)))) {
        LOG_DEBUG(
            "QSFP    %2d : dev_port=%d : RXLOS=%d", conn_id, dev_port, los);
      }
      continue;
    }
    // hack, really need # lanes on port
    // TBD: Move optical-type to qsfp_detection_actions
    bf_port_is_optical_xcvr_set(dev_id, dev_port, true);
    if (byte_86 & (1 << ch)) {
      bf_port_optical_xcvr_ready_set(dev_id, dev_port, false);
    } else {
      bf_port_optical_xcvr_ready_set(dev_id, dev_port, true);
    }
    bf_status = bf_port_optical_los_set(dev_id, dev_port, los);
    if (bf_status == BF_SUCCESS) {
      if (((rx_los_now & (1 << ch)) ^ (rx_los_before & (1 << ch))) ||
          ((rx_lol_now & (1 << ch)) ^ (rx_lol_before & (1 << ch)))) {
        LOG_DEBUG("QSFP    %2d : dev_port=%d : LOS=%d", conn_id, dev_port, los);
      }
    }
  }
  // copy new alarm state into out cached struct
  qsfp_state[conn_id].status_and_alarms = status_and_alarms;
  return 0;
}

/*****************************************************************
* Bring all ch_fsms to DISABLED, then put the entire module in LPMode
* Skips passive copper cables
*****************************************************************/
static void qsfp_fsm_st_lp_mode_on(bf_dev_id_t dev_id,
                                   int conn_id,
                                   bool *alldown) {
  int cur_ch;

  *alldown = true;
  if (qsfp_state[conn_id].qsfp_type == QSFP_TYP_COPPER) {
    return;
  }

  for (cur_ch = 0; cur_ch < qsfp_state[conn_id].ch_cnt; cur_ch++) {
    // look at ch_fsm state, this tells us where we are in the process
    switch (qsfp_state[conn_id].per_ch_fsm[cur_ch].fsm_st) {
      case QSFP_CH_FSM_ST_DISABLING:
        // keep waiting
        *alldown = false;
        break;
      case QSFP_CH_FSM_ST_DISABLED:
        // this lane has completed
        break;
      default:
        qsfp_fsm_ch_disable(dev_id, conn_id, cur_ch);
        *alldown = false;
        break;
    }
  }

  if (*alldown) {  // all ch_fsms are in DISABLED for this conn_id
    if (bf_qsfp_is_cmis(conn_id)) {
      Module_State module_state;
      // look at module state to see where we are in the process
      if (bf_cmis_get_module_state(conn_id, &module_state) < 0) {
        LOG_ERROR("qsfp fsm: qsfp %d unable to get module state", conn_id);
        *alldown = false;
        return;
      }
      if (module_state == MODULE_ST_LOWPWR) {
        // we have completed
        return;
      } else if (module_state == MODULE_ST_PWRDN) {
        // we're working on coming down, be patient
        *alldown = false;
        return;
      }
      // fall through and put the module in low power mode
    }

    // enable lpmode
    if (qsfp_needs_hi_pwr_init(conn_id)) {
      qsfp_fsm_lpmode_assert(conn_id);
    } else {
      qsfp_lpmode_sw_set(dev_id, conn_id, 1);
    }
  }
}

/*****************************************************************
*
*****************************************************************/
static void qsfp_module_fsm_update(int conn_id) {
  Sff_field field = qsfp_state[conn_id].wr_coalesce.field;
  int mask = qsfp_state[conn_id].wr_coalesce.mask;
  uint8_t data = qsfp_state[conn_id].wr_coalesce.data;

  bf_qsfp_bitfield_rmw(conn_id, field, mask, data);
}

/*****************************************************************
*
*****************************************************************/
static void qsfp_module_fsm_complete_update(bf_dev_id_t dev_id, int conn_id) {
  qsfp_state[conn_id].wr_coalesce.in_progress = 0;
  qsfp_state[conn_id].wr_coalesce.delay_ms = 0;
  qsfp_state[conn_id].wr_coalesce.field = 0;
  qsfp_state[conn_id].wr_coalesce.mask = 0;
  qsfp_state[conn_id].wr_coalesce.data = 0;
}

/*****************************************************************
* Module FSM
*****************************************************************/
static void qsfp_module_fsm_run(bf_dev_id_t dev_id, int conn_id) {
  qsfp_fsm_state_t st = qsfp_state[conn_id].fsm_st;
  qsfp_fsm_state_t next_st = st;
  int rc, delay_ms = 0;
  bool is_optical = false, alldown;

  // LOG_DEBUG("QSFPMSM %2d : MSM RUN -- %s", conn_id, qsfp_fsm_st_to_str[st]);
  switch (st) {
    case QSFP_FSM_ST_IDLE:
      break;

    case QSFP_FSM_ST_REMOVED:
      // reset to initial conditions
      // resetL          <-- 1 # not reset
      // lp_mode         <-- 1 # low power mode
      qsfp_fsm_st_removed(dev_id, conn_id);

      next_st = QSFP_FSM_ST_IDLE;
      delay_ms = 0;
      qsfp_state[conn_id].qsfp_quick_removed = false;
      break;

    case QSFP_FSM_ST_INSERTED:
      // identify type.
      // If not optical got directly to DETECTED
      // else ensure initial conditions
      //   resetL          <-- 0 # assert reset
      //   lp_mode         <-- 1 # low power mode
      rc = qsfp_fsm_identify_type(conn_id, &is_optical);
      if ((rc != 0) || (qsfp_state[conn_id].qsfp_type == QSFP_TYP_UNKNOWN)) {
        // error identifying type. Stay in this state
        next_st = QSFP_FSM_ST_INSERTED;
        delay_ms = 1000;  // try again in 1sec
        break;
      }
      qsfp_state[conn_id].qsfp_quick_removed = false;
      if (is_optical) {
        int ch;

        // checks for QSFPs with "special" requirements
        qsfp_fsm_identify_model_requirements(conn_id);

        // set initial state for each of the channel FSMs
        for (ch = 0; ch < CHANNEL_COUNT; ch++) {
          qsfp_state[conn_id].per_ch_fsm[ch] = qsfp_channel_fsm_initial_state;
        }
        qsfp_fsm_st_inserted(dev_id, conn_id);

        next_st = QSFP_FSM_ST_WAIT_T_RESET;
        delay_ms = CMISand8636_TMR_t_reset_init;
      } else {
        if (bf_pm_intf_is_device_family_tofino(dev_id)) {
          qsfp_fsm_notify_bf_pltfm(dev_id, conn_id);
        }

        next_st = QSFP_FSM_ST_DETECTED;
        delay_ms = 0;
      }
      break;

    case QSFP_FSM_ST_WAIT_T_RESET:
      qsfp_fsm_st_reset(dev_id, conn_id);

      next_st = QSFP_FSM_ST_WAIT_TON_TXDIS;
      delay_ms = CMISand8636_TMR_t_reset;
      break;

    case QSFP_FSM_ST_WAIT_TON_TXDIS:
      // check for alarms during intialization sequence
      if (bf_pm_intf_is_device_family_tofino(dev_id)) {
        qsfp_fsm_check_alarms_after_unreset(dev_id, conn_id);
      }

      qsfp_fsm_st_tx_disable(dev_id, conn_id);

      /* if the module needs hi-pwr init then we actually de-asserted
      * TX_DISABLE above and need to wait for CMISand8636_TMR_toff_txdis
      * PLUS the Luxtera MZI init time of 3.1 seconds for a total of 3.5
      * seconds*/
      if (qsfp_needs_hi_pwr_init(conn_id)) {
        delay_ms =
            CMISand8636_TMR_toff_txdis + 3100;  // 3.5 sec for Luxtera MZI init;
      } else {
        bf_qsfp_dp_deactivate_all(conn_id);  // prevent datapaths from auto-
                                             // initing when we go into hp mode

        delay_ms = CMISand8636_TMR_ton_txdis;
      }
      next_st = QSFP_FSM_ST_WAIT_TOFF_LPMODE;
      break;

    case QSFP_FSM_ST_WAIT_TOFF_LPMODE:
      qsfp_fsm_st_lp_mode_off(dev_id, conn_id);

      next_st = QSFP_FSM_ST_DETECTED;
      delay_ms = CMISand8636_TMR_Toff_LPMode;

      if (bf_pm_intf_is_device_family_tofino(dev_id)) {
        qsfp_fsm_notify_bf_pltfm(dev_id, conn_id);
      }
      break;

    case QSFP_FSM_ST_DETECTED:
      // check LOS
      if (bf_pm_intf_is_device_family_tofino(dev_id)) {
        if (qsfp_fsm_is_optical(conn_id) &&
            (bf_qsfp_get_memmap_format(conn_id) == MMFORMAT_SFF8636)) {
          if (qsfp_fsm_poll_los(dev_id, conn_id) != 0) {
            if (qsfp_state[conn_id].qsfp_quick_removed) {
              // keep breaking
              break;
            }
          }
        }
      }

      next_st = QSFP_FSM_ST_DETECTED;
      delay_ms = 200;  // 200ms poll time
      break;

    case QSFP_FSM_ST_WAIT_TON_LPMODE:
      qsfp_fsm_st_lp_mode_on(dev_id, conn_id, &alldown);
      if (alldown) {
        next_st = QSFP_FSM_ST_LPMODE;
        delay_ms = 0;
      }
      break;

    case QSFP_FSM_ST_LPMODE:
      // park state to save power or shutdown system
      // Any exit should go to QSFP_FSM_ST_WAIT_TOFF_LPMODE for normal modules
      // and QSFP_FSM_ST_INSERTED for needs_hi_pwr_init modules
      break;

    case QSFP_FSM_ST_UPDATE:
      qsfp_module_fsm_update(conn_id);
      next_st = QSFP_FSM_ST_WAIT_UPDATE;
      delay_ms = qsfp_state[conn_id].wr_coalesce.delay_ms;
      break;

    case QSFP_FSM_ST_WAIT_UPDATE:
      qsfp_module_fsm_complete_update(dev_id, conn_id);
      next_st = QSFP_FSM_ST_DETECTED;
      delay_ms = 0;
      break;

    default:
      assert(0);
      break;
  }
  qsfp_state[conn_id].fsm_st = next_st;
  qsfp_fsm_next_run_time_set(&qsfp_state[conn_id].next_fsm_run_time, delay_ms);

  if (st != next_st) {
    LOG_DEBUG("QSFPMSM %2d : %s --> %s (%dms)",
              conn_id,
              qsfp_fsm_st_to_str[st],
              qsfp_fsm_st_to_str[next_st],
              delay_ms);
  }
}

/*****************************************************************
* Merge writes to the same memory location, so we can be more efficient but
* also to comply with synchroenicity rules for lanes in a data path
 Returns 0 = write merged
        !0 = different write in-progress, try later
*****************************************************************/
static int qsfp_fsm_coalesce_wr(bf_dev_id_t dev_id,
                                int conn_id,
                                int delay_ms,
                                Sff_field field,
                                int mask,
                                uint8_t data) {
  if (!qsfp_state[conn_id].wr_coalesce.in_progress) {
    qsfp_state[conn_id].wr_coalesce.in_progress = true;
    qsfp_state[conn_id].wr_coalesce.delay_ms = delay_ms;
    qsfp_state[conn_id].wr_coalesce.field = field;
    qsfp_state[conn_id].wr_coalesce.mask = mask;
    qsfp_state[conn_id].wr_coalesce.data = data;

    LOG_DEBUG("QSFPMSM %2d : %s --> %s (%dms)",
              conn_id,
              qsfp_fsm_st_to_str[qsfp_state[conn_id].fsm_st],
              qsfp_fsm_st_to_str[QSFP_FSM_ST_UPDATE],
              0);
    qsfp_state[conn_id].fsm_st = QSFP_FSM_ST_UPDATE;
    qsfp_fsm_next_run_time_set(&qsfp_state[conn_id].next_fsm_run_time, 0);
  } else {
    // check if this write can be coalesced with the current one
    if (qsfp_state[conn_id].wr_coalesce.field == field) {
      // merge this write with in-progress one
      // LOG_ERROR("QSFP    %2d : Coalesce field=%d : mask=%02x : data=%02x
      // : with : mask=%02x : data=%02x",
      //          conn_id, field, mask, data,
      //          qsfp_state[conn_id].wr_coalesce.mask,
      //          qsfp_state[conn_id].wr_coalesce.data);
      qsfp_state[conn_id].wr_coalesce.mask |= mask;
      qsfp_state[conn_id].wr_coalesce.data |= data;
    } else {
      return -1;  // try again later
    }
  }
  return 0;
}

/*****************************************************************
*
*****************************************************************/
void qsfp_fsm_ch_enable(bf_dev_id_t dev_id, int conn_id, int ch) {
  if (qsfp_state[conn_id].per_ch_fsm[ch].fsm_st != QSFP_CH_FSM_ST_ENABLING) {
    LOG_DEBUG(
        "QSFPCSM %2d/%d : %s --> %s (%dms)",
        conn_id,
        ch,
        qsfp_fsm_ch_en_st_to_str[qsfp_state[conn_id].per_ch_fsm[ch].fsm_st],
        qsfp_fsm_ch_en_st_to_str[QSFP_CH_FSM_ST_ENABLING],
        0);
    qsfp_state[conn_id].per_ch_fsm[ch].fsm_st = QSFP_CH_FSM_ST_ENABLING;
    qsfp_fsm_next_run_time_set(
        &qsfp_state[conn_id].per_ch_fsm[ch].next_fsm_run_time, 0);
  }
}

/*****************************************************************
*
*****************************************************************/
void qsfp_fsm_ch_disable(bf_dev_id_t dev_id, int conn_id, int ch) {
  if (qsfp_state[conn_id].per_ch_fsm[ch].fsm_st != QSFP_CH_FSM_ST_DISABLING) {
    LOG_DEBUG(
        "QSFPCSM %2d/%d : %s --> %s (%dms)",
        conn_id,
        ch,
        qsfp_fsm_ch_en_st_to_str[qsfp_state[conn_id].per_ch_fsm[ch].fsm_st],
        qsfp_fsm_ch_en_st_to_str[QSFP_CH_FSM_ST_DISABLING],
        0);
    qsfp_state[conn_id].per_ch_fsm[ch].fsm_st = QSFP_CH_FSM_ST_DISABLING;
    qsfp_fsm_next_run_time_set(
        &qsfp_state[conn_id].per_ch_fsm[ch].next_fsm_run_time, 0);
  }
}

/*****************************************************************
* Disables all lanes associated with a logical port, given a lane
* within that port, and marks them to immediately enable once
* they have completed DISABLING
*****************************************************************/
void qsfp_log_port_retry(bf_dev_id_t dev_id, int conn_id, int ch) {
  int cur_ch;
  for (cur_ch = 0; cur_ch < qsfp_state[conn_id].dev_cfg[ch].host_intf_nlanes;
       cur_ch++) {
    qsfp_state[conn_id].per_ch_fsm[cur_ch].immediate_enable = true;
    qsfp_fsm_ch_disable(
        dev_id, conn_id, qsfp_state[conn_id].dev_cfg[ch].host_head_ch + cur_ch);
  }
}

/*****************************************************************
* enable both TX and RX CDRs - only use for sff-8636 modules
*****************************************************************/
static int qsfp_fsm_ch_sff8636_enable_cdr(bf_dev_id_t dev_id,
                                          int conn_id,
                                          int ch) {
  int rc;
  // combine all CDR updates into one I2C transaction
  rc = qsfp_fsm_coalesce_wr(
      dev_id, conn_id, 300 /*300ms*/, CDR_CONTROL, (0x11 << ch), (0x11 << ch));
  return rc;  // non-0 means "try later"
}

#if 0 
/*****************************************************************
* disable both TX and RX CDRs - only use for sff-8636 modules
*****************************************************************/
static int qsfp_fsm_ch_sff8636_disable_cdr(bf_dev_id_t dev_id,
                                           int conn_id,
                                           int ch) {
  int rc;
  // combine all CDR updates into one I2C transaction
  rc = qsfp_fsm_coalesce_wr(dev_id,
                            conn_id,
                            300 /*300ms*/,
                            CDR_CONTROL,
                            (0x11 << ch),
                            (0x00 << ch));
  return rc;  // non-0 means "try later"
}
#endif

/*****************************************************************
* select the Application that matches the configured device speed
* only use for CMIS modules
*****************************************************************/
static int qsfp_fsm_ch_cmis_select_application(bf_dev_id_t dev_id,
                                               int conn_id,
                                               int ch) {
  int rc;
  int matching_ApSel;

  // lookup the Application that matches our configured speed
  matching_ApSel = bf_cmis_find_matching_Application(
      conn_id,
      qsfp_state[conn_id].dev_cfg[ch].intf_speed,
      qsfp_state[conn_id].dev_cfg[ch].host_intf_nlanes,
      qsfp_state[conn_id].dev_cfg[ch].host_head_ch /*,
          qsfp_state[conn_id].dev_cfg[ch].encoding */);

  if (!matching_ApSel) {
    LOG_ERROR("qsfp ch fsm: qsfp %d ch %d Matching Application not found",
              conn_id,
              ch);
    return -1;
  }

  // write the matching ApSel code to Staged Set 0
  rc = bf_cmis_select_Application(conn_id,
                                  ch,
                                  matching_ApSel,
                                  qsfp_state[conn_id].dev_cfg[ch].host_head_ch,
                                  0 /* no explict control*/);
  if (rc != 0) {
    LOG_ERROR(
        "qsfp ch fsm: qsfp %d ch %d Failed to select Application", conn_id, ch);
    return rc;
  }

  // now that we know the Application, we can figure out the media lanes
  return bf_cmis_get_Application_media_info(
      conn_id,
      matching_ApSel,
      ch,
      &qsfp_state[conn_id].dev_cfg[ch].media_ch,
      &qsfp_state[conn_id].dev_cfg[ch].media_head_ch,
      &qsfp_state[conn_id].dev_cfg[ch].media_intf_nlanes);
}

/*****************************************************************
* writes a 1 to Apply_DataPathInit for the the indicated channel
* only use for CMIS modules
*****************************************************************/
static int qsfp_fsm_ch_cmis_apply_dpinit(bf_dev_id_t dev_id,
                                         int conn_id,
                                         int ch) {
  int rc;

  // combine all Apply_DataPathInit updates into one I2C transaction
  rc = qsfp_fsm_coalesce_wr(dev_id,
                            conn_id,
                            0, /* delay handled in ch fsm */
                            APPLY_DATAPATHINIT_SS0,
                            (0x1 << ch),
                            (1 << ch));
  LOG_DEBUG("QSFP    %2d : Attempt to coalesce write 0x%02x, %s",
            conn_id,
            1 << ch,
            rc ? "no" : "yes");
  return rc;  // non-0 means "try later"
}

/*****************************************************************
* clears the DataPathDeinit bit for the indicated channel
* only use for CMIS modules
*****************************************************************/
static int qsfp_fsm_ch_cmis_datapath_init(bf_dev_id_t dev_id,
                                          int conn_id,
                                          int ch) {
  int rc;
  uint8_t major, newvalue;

  // unfortunately, the polarity of this register changed in CMIS 4.0
  bf_cmis_spec_rev_get(conn_id, &major, NULL);
  if (major < 4) {
    newvalue = 1;
  } else {
    newvalue = 0;
  }

  // combine all DataPathDeinit updates into one I2C transaction
  rc = qsfp_fsm_coalesce_wr(dev_id,
                            conn_id,
                            0, /* delay handled in ch fsm */
                            DATAPATH_DEINIT,
                            (0x1 << ch),
                            (newvalue << ch));
  LOG_DEBUG("QSFP    %2d : Attempt to coalesce write 0x%02x, %s",
            conn_id,
            newvalue << ch,
            rc ? "no" : "yes");
  return rc;  // non-0 means "try later"
}

/*****************************************************************
* sets the DataPathDeinit bit for the indicated channel
* only use for CMIS modules
*****************************************************************/
static int qsfp_fsm_ch_cmis_datapath_deinit(bf_dev_id_t dev_id,
                                            int conn_id,
                                            int ch) {
  int rc;
  uint8_t major, newvalue;

  // unfortunately, the polarity of this register changed in CMIS 4.0
  bf_cmis_spec_rev_get(conn_id, &major, NULL);
  if (major < 4) {
    newvalue = 0;
  } else {
    newvalue = 1;
  }

  // combine all DataPathDeinit updates into one I2C transaction
  rc = qsfp_fsm_coalesce_wr(dev_id,
                            conn_id,
                            0, /* delay handled in ch fsm */
                            DATAPATH_DEINIT,
                            (0x1 << ch),
                            (newvalue << ch));
  return rc;  // non-0 means "try later"
}

/*****************************************************************
* returns true if the DataPath state is DataPathInitialized for the
* current channel in the data path
* only use for CMIS modules
*****************************************************************/
static bool qsfp_fsm_ch_cmis_is_state_dpinitialized(bf_dev_id_t dev_id,
                                                    int conn_id,
                                                    int ch) {
  DataPath_State datapath_state;

  if (bf_cmis_get_datapath_state(conn_id, ch, &datapath_state) < 0) {
    LOG_ERROR("QSFPCSM %2d/%d : unable to get datapath state", conn_id, ch);
    return false;  // keep trying
  }
  if (datapath_state != DATAPATH_ST_INITIALIZED) {
    return false;
  }
  return true;
}

/*****************************************************************
* returns true if the DataPath state is DataPathActivated for all
* current channel in the data path
* only use for CMIS modules
*****************************************************************/
static bool qsfp_fsm_ch_cmis_is_state_dpactivated(bf_dev_id_t dev_id,
                                                  int conn_id,
                                                  int ch) {
  DataPath_State datapath_state;

  if (bf_cmis_get_datapath_state(conn_id, ch, &datapath_state) < 0) {
    LOG_ERROR("QSFPCSM %2d/%d : unable to get datapath state", conn_id, ch);
    return false;  // keep trying
  }
  if (datapath_state != DATAPATH_ST_ACTIVATED) {
    return false;
  }
  return true;
}

/*****************************************************************
* returns true if the DataPath state is DataPathDeactivated
* only use for CMIS modules
*****************************************************************/
static bool qsfp_fsm_ch_cmis_is_state_dpdeactivated(bf_dev_id_t dev_id,
                                                    int conn_id,
                                                    int ch) {
  DataPath_State datapath_state;
  if (bf_cmis_get_datapath_state(conn_id, ch, &datapath_state) < 0) {
    LOG_ERROR(
        "qsfp ch fsm: qsfp %d ch %d unable to get datapath state", conn_id, ch);
    return false;  // keep trying
  }

  if (datapath_state == DATAPATH_ST_DEACTIVATED) {
    return true;
  }
  return false;
}

/*****************************************************************
* This routine is only used in Tofino 1 implementations
*****************************************************************/
static void qsfp_fsm_ch_check_tx_cdr_lol(bf_dev_id_t dev_id,
                                         int conn_id,
                                         int ch,
                                         bool *ok,
                                         uint8_t *los_val,
                                         uint8_t *lol_val) {
  uint8_t byte_3, byte_5;
  bf_status_t rc;
  uint32_t lane_mask[] = {0, 1, 3, 0xF, 0xFF};
  int num_lanes = qsfp_num_host_lanes_get(conn_id, ch);
  uint32_t msk = lane_mask[num_lanes] << (ch + 4);

  rc = bf_fsm_qsfp_rd(conn_id, 0, 3, 1, &byte_3);
  if (rc) {
    LOG_ERROR("QSFP    %2d : Error <%d> reading TX LOS (byte 3)", conn_id, rc);
  }
  if (byte_3 & 0xF0 & msk) {  // only check L-TX1-4 LOS
    *ok = false;
  } else {
    *ok = true;
  }

  rc = bf_fsm_qsfp_rd(conn_id, 0, 5, 1, &byte_5);
  if (rc) {
    LOG_ERROR("QSFP    %2d : Error <%d> reading CDR LOL (byte 5)", conn_id, rc);
  }
  if (byte_5 & 0xF0 & msk) {  // only check L-TX1-4 LOL
    *ok = false;
  }
  *los_val = byte_3;
  *lol_val = byte_5;

  LOG_DEBUG(
      "QSFP    %2d : ch[%d] : TX LOS (byte 3) = %02x : TX CDR LOL (byte 5) = "
      "%02x : %s <num_lanes=%d : msk=%x>",
      conn_id,
      ch,
      byte_3,
      byte_5,
      *ok ? "[ok]" : "[fault present]",
      num_lanes,
      msk);
}

/*****************************************************************
* This routine is only used in Tofino 2 implementations
*****************************************************************/
static void qsfp_fsm_check_tx_lock(int conn_id, int ch, bool *ok) {
  int lol_flag, los_flag;

  // flags are logged in qsfp_fsm loop
  // flags are cleared at end of ch_fsm
  los_flag = bf_qsfp_get_flag(conn_id, FLAG_TX_LOS, ch, 0, NULL);
  if (los_flag != 0) {
    *ok = false;
    return;
  }

  lol_flag = bf_qsfp_get_flag(conn_id, FLAG_TX_LOL, ch, 0, NULL);
  if (lol_flag != 0) {
    *ok = false;
    return;
  }
  *ok = true;
}

/*****************************************************************
* enable optical transmitters
*****************************************************************/
static int qsfp_fsm_ch_enable_optical_tx(bf_dev_id_t dev_id,
                                         int conn_id,
                                         int ch,
                                         uint32_t delay_ms) {
  int rc;
  rc = qsfp_fsm_coalesce_wr(
      dev_id, conn_id, delay_ms, CHANNEL_TX_DISABLE, (0x1 << ch), (0x0 << ch));
  return rc;  // non-0 means "try later"
}

/*****************************************************************
* disable optical transmitters
*****************************************************************/
static int qsfp_fsm_ch_disable_optical_tx(bf_dev_id_t dev_id,
                                          int conn_id,
                                          int ch) {
  int rc;
  rc = qsfp_fsm_coalesce_wr(dev_id,
                            conn_id,
                            100 /*100ms*/,
                            CHANNEL_TX_DISABLE,
                            (0x1 << ch),
                            (0x1 << ch));
  return rc;  // non-0 means "try later"
}

/*****************************************************************
* This routine is only used in Tofino 1 implementations
*****************************************************************/
static void qsfp_fsm_ch_check_tx_optical_fault(bf_dev_id_t dev_id,
                                               int conn_id,
                                               int ch,
                                               bool clear_only,
                                               bool *ok,
                                               uint8_t *val) {
  uint8_t byte_4, byte, faults_and_alarms[16];
  bf_status_t rc;
  bool fault_or_alarm_set = false;
  uint32_t lane_mask[] = {0, 1, 3, 0xF};
  int num_lanes = qsfp_num_media_lanes_get(conn_id, ch);
  uint32_t msk = lane_mask[num_lanes] << ch;

  // read "Interrupt Flags" bytes, 3-15, most are faults and alarms
  rc = bf_fsm_qsfp_rd(conn_id, 0, 0, 15, faults_and_alarms);
  if (rc) {
    LOG_ERROR(
        "QSFP    %2d : Error <%d> reading faults and alarms", conn_id, rc);
  }
  if (clear_only) return;

  byte_4 = faults_and_alarms[4];

  // ignore some Rx faults and alarms
  faults_and_alarms[3] &= 0xF0;
  faults_and_alarms[5] &= 0xF0;
  faults_and_alarms[8] = 0;
  faults_and_alarms[9] = 0;
  faults_and_alarms[10] = 0;

  for (byte = 3; byte < 16; byte++) {
    if (faults_and_alarms[byte] != 0) {
      if (byte == 4) {
        fault_or_alarm_set = true;
      }
      LOG_DEBUG("QSFP    %2d : Page 0: Faults and alarms: Byte %2d: %02xh",
                conn_id,
                byte,
                faults_and_alarms[byte]);
    }
  }
  if (fault_or_alarm_set) {
    *ok = false;
    *val = byte_4;
    //    return;
  }

  if (byte_4 & 0x0F & msk) {  // only check L-TX1-4 Fault
    *ok = false;
  } else {
    *ok = true;
  }
  *val = byte_4;
}

/*****************************************************************
* This routine is only used in Tofino 2 implementations
*****************************************************************/
static void qsfp_fsm_check_tx_faults(int conn_id, int ch, bool *ok) {
  int tx_fault, tx_adapt_eq_fault;

  // flags are logged in qsfp_fsm loop
  // flags are cleared at end of ch_fsm
  tx_fault = bf_qsfp_get_flag(conn_id,
                              FLAG_TX_FAULT,
                              qsfp_state[conn_id].dev_cfg[ch].media_ch,
                              0,
                              NULL);
  if (tx_fault != 0) {
    *ok = false;
    return;
  }

  tx_adapt_eq_fault =
      bf_qsfp_get_flag(conn_id, FLAG_TX_ADAPT_EQ_FAULT, ch, 0, NULL);
  if (tx_adapt_eq_fault != 0) {
    *ok = false;
    return;
  }
  *ok = true;
}

/*****************************************************************
*
*****************************************************************/
void qsfp_fsm_ch_notify_ready(bf_dev_id_t dev_id, int conn_id, int ch) {
  bf_dev_port_t dev_port;
  bf_pal_front_port_handle_t port_hdl;
  bool los;
  bf_dev_id_t dev_id_of_port = 0;

  port_hdl.conn_id = conn_id;
  port_hdl.chnl_id = ch;
  bf_pm_port_front_panel_port_to_dev_port_get(
      &port_hdl, &dev_id_of_port, &dev_port);
  dev_id = dev_id_of_port;

  // notify of LOS state first
  los = ((qsfp_state[conn_id].status_and_alarms.los_ind & (1 << ch)) ||
         (qsfp_state[conn_id].status_and_alarms.lol_ind & (1 << ch)))
            ? true
            : false;
  bf_port_optical_los_set(dev_id, dev_port, los);
  bf_port_optical_xcvr_ready_set(dev_id, dev_port, true);

  LOG_DEBUG("QSFP    %2d : ch[%d] : Ready %s", conn_id, ch, los ? "<LOS>" : "");
}

/*****************************************************************
*
*****************************************************************/
void qsfp_fsm_ch_notify_not_ready(bf_dev_id_t dev_id, int conn_id, int ch) {
  bf_dev_port_t dev_port;
  bf_pal_front_port_handle_t port_hdl;
  bool los;
  bf_dev_id_t dev_id_of_port = 0;

  port_hdl.conn_id = conn_id;
  port_hdl.chnl_id = ch;
  bf_pm_port_front_panel_port_to_dev_port_get(
      &port_hdl, &dev_id_of_port, &dev_port);
  dev_id = dev_id_of_port;

  bf_port_optical_xcvr_ready_set(dev_id, dev_port, false);
  bf_port_optical_los_set(dev_id, dev_port, true);

  // tof2 uses a different logging mechanism
  if (bf_pm_intf_is_device_family_tofino(dev_id)) {
    los = (qsfp_state[conn_id].status_and_alarms.los_ind & (1 << ch)) ? true
                                                                      : false;
    LOG_DEBUG(
        "QSFP    %2d : ch[%d] : NOT Ready %s", conn_id, ch, los ? "<LOS>" : "");
  }
}

/*****************************************************************
*
*****************************************************************/
bool qsfp_fsm_ch_first_enable(bf_dev_id_t dev_id, int conn_id, int ch) {
  return qsfp_state[conn_id].per_ch_fsm[ch].first_enable_after_reset;
}

/*****************************************************************
*
*****************************************************************/
void qsfp_fsm_ch_first_enable_clear(bf_dev_id_t dev_id, int conn_id, int ch) {
  qsfp_state[conn_id].per_ch_fsm[ch].first_enable_after_reset = false;
}

/*****************************************************************
*
*****************************************************************/
static void qsfp_channel_fsm_run(bf_dev_id_t dev_id, int conn_id, int ch) {
  qsfp_fsm_ch_en_state_t st = qsfp_state[conn_id].per_ch_fsm[ch].fsm_st;
  qsfp_fsm_ch_en_state_t next_st = st;  // default to no transition
  MemMap_Format memmap_format = MMFORMAT_UNKNOWN;
  int wr_coalesce_err, delay_ms = 0;
  int substate = qsfp_state[conn_id].per_ch_fsm[ch].substate;
  int next_substate = substate;
  bool ok, discard_latched;
  uint8_t los_fault_val, lol_fault_val;
  uint8_t fault_val;
  int host_head_ch;
  int host_chmask_thisport;
  bool is_optical = false;

  host_head_ch = qsfp_state[conn_id].dev_cfg[ch].host_head_ch;
  /*
    LOG_DEBUG("QSFPCSM %2d/%d : CSM RUN -- %s.%d",
              conn_id,
              ch,
              qsfp_fsm_ch_en_st_to_str[st],
              substate);
   */
  // this mask has a 1 in each bit position corresponding to a lane in the
  // current port
  host_chmask_thisport =
      ((1 << qsfp_state[conn_id].dev_cfg[ch].host_intf_nlanes) - 1)
      << host_head_ch;

  switch (st) {
    case QSFP_CH_FSM_ST_DISABLED:
      // waiting to start enabling
      break;
    case QSFP_CH_FSM_ST_ENABLING:
      // initalize/re-initialize variables used in ch_fsm
      qsfp_state[conn_id].per_ch_fsm[ch].immediate_enable = false;
      qsfp_state[conn_id].per_ch_fsm[ch].head_1st_ena_cdr = false;
      qsfp_state[conn_id].sync_ena_cdr &= (0xFFFF ^ (1 << ch));  // unset
      qsfp_state[conn_id].per_ch_fsm[ch].head_1st_ena_opt_tx = false;
      qsfp_state[conn_id].sync_tx_locked &= (0xFFFF ^ (1 << ch));  // unset
      qsfp_state[conn_id].per_ch_fsm[ch].head_1st_tx_locked = false;
      qsfp_state[conn_id].sync_ena_opt_tx &= (0xFFFF ^ (1 << ch));  // unset
      qsfp_state[conn_id].per_ch_fsm[ch].head_1st_notify_enb = false;
      qsfp_state[conn_id].sync_notify_enb &= (0xFFFF ^ (1 << ch));  // unset

      if (bf_pm_intf_is_device_family_tofino(dev_id)) {
        // notify driver
        qsfp_fsm_ch_notify_not_ready(dev_id, conn_id, ch);

        // assume straight-through connections
        qsfp_state[conn_id].dev_cfg[ch].media_ch = ch;
        qsfp_state[conn_id].dev_cfg[ch].media_head_ch =
            qsfp_state[conn_id].dev_cfg[ch].host_head_ch;
        qsfp_state[conn_id].dev_cfg[ch].media_intf_nlanes =
            qsfp_state[conn_id].dev_cfg[ch].host_intf_nlanes;

        delay_ms = 300;  // ms
        next_st = QSFP_CH_FSM_ST_ENA_CDR;
        next_substate = 0;
        break;
      }

      // if the media is copper
      qsfp_fsm_identify_type(conn_id, &is_optical);
      if (!is_optical) {
        next_st = QSFP_CH_FSM_ST_ENABLED;
        delay_ms = 0;
        break;
      }

      // update the selected Application for CMIS modules
      if ((bf_qsfp_is_cmis(conn_id)) && (!qsfp_needs_hi_pwr_init(conn_id))) {
        // CMIS, software init
        qsfp_fsm_ch_cmis_select_application(dev_id, conn_id, ch);

        wr_coalesce_err = qsfp_fsm_ch_cmis_apply_dpinit(dev_id, conn_id, ch);
        if (wr_coalesce_err == 0) {  // successfully added to queue
          delay_ms = 0;              // wait in the next substate
          next_st = QSFP_CH_FSM_ST_ENA_CDR;
          next_substate = 0;
        }
      } else {
        // SFF-8636 - doesn't have Applications, nothing to do
        // CMIS, hardware init - nothing to do

        // assume straight-through connections
        qsfp_state[conn_id].dev_cfg[ch].media_ch = ch;
        qsfp_state[conn_id].dev_cfg[ch].media_head_ch =
            qsfp_state[conn_id].dev_cfg[ch].host_head_ch;
        qsfp_state[conn_id].dev_cfg[ch].media_intf_nlanes =
            qsfp_state[conn_id].dev_cfg[ch].host_intf_nlanes;

        delay_ms = 0;  // no need to wait after this
        next_st = QSFP_CH_FSM_ST_ENA_CDR;
        next_substate = 0;
      }
      break;
    case QSFP_CH_FSM_ST_ENA_CDR:
      // this state initializes the data path electronics in the module
      // SFF-8636 - enable the TX and RX CDRs, then wait for LOL to clear
      // CMIS (software init) - transition data path state to DataPathInit,
      //   where the module will initialize the data path and then transition
      //   the data path state to DataPathInitialized

      qsfp_state[conn_id].sync_ena_cdr |= 1 << ch;

      // this check makes sure all channel state machines go through this state
      // at the same time, starting wich the head channel
      if (((qsfp_state[conn_id].sync_ena_cdr & host_chmask_thisport) !=
           host_chmask_thisport) ||
          ((!qsfp_state[conn_id].per_ch_fsm[host_head_ch].head_1st_ena_cdr) &&
           (ch != host_head_ch))) {
        break;
      }
      qsfp_state[conn_id].per_ch_fsm[host_head_ch].head_1st_ena_cdr = true;
      switch (next_substate) {
        case 0:  // step 0 - initalize data path electronics
          if (!bf_qsfp_is_cmis(conn_id)) {
            // SFF-8636 - enable the CDR
            wr_coalesce_err =
                qsfp_fsm_ch_sff8636_enable_cdr(dev_id, conn_id, ch);
            if (wr_coalesce_err == 0) {  // successfully added to queue
              delay_ms = 300;            // wait 300 ms before checking LOL flag
              next_substate++;
            }  // else {
            // we'll try to add to the queue next time around
            //}
            break;
          } else if (!qsfp_needs_hi_pwr_init(conn_id)) {
            // CMIS, software init - clear applicable DataPathDeinit bits.
            //   This advances the modure DataPath state machine to
            //   DataPathInit, where it will start initializing the data path
            //   electronics
            wr_coalesce_err =
                qsfp_fsm_ch_cmis_datapath_init(dev_id, conn_id, ch);
            if (wr_coalesce_err == 0) {  // successfully added to queue
              delay_ms = 0;              // wait in the next substate
              next_substate++;
            }
            break;
          }
          // CMIS hardware init - nothing to do
          next_substate++;  // fall through
        case 1:             // Step 1 -  wait for initialization to complete
          memmap_format = bf_qsfp_get_memmap_format(conn_id);
          if (memmap_format == MMFORMAT_SFF8636) {
            if (bf_pm_intf_is_device_family_tofino(dev_id)) {
              // for some reason, need to read twice to clear latched faults
              // Tofino 1 flag handling method only
              qsfp_fsm_ch_check_tx_cdr_lol(dev_id,
                                           conn_id,
                                           ch,
                                           &discard_latched,
                                           &los_fault_val,
                                           &lol_fault_val);
              qsfp_fsm_ch_check_tx_cdr_lol(dev_id,
                                           conn_id,
                                           ch,
                                           &discard_latched,
                                           &los_fault_val,
                                           &lol_fault_val);
              // third time's the charm - see if the qsfp still sets lol or los
              qsfp_fsm_ch_check_tx_cdr_lol(
                  dev_id, conn_id, ch, &ok, &los_fault_val, &lol_fault_val);
              if (!ok) {
                LOG_DEBUG(
                    "QSFP    %2d : ch[%d] : TX LOS <%02x>, TX CDR LOL <%02x>. "
                    "retry..",
                    conn_id,
                    ch,
                    los_fault_val,
                    lol_fault_val);
              } else {
                delay_ms =
                    0;  // LOL and LOS are both unset, proceed immediately
                next_st = QSFP_CH_FSM_ST_ENA_OPTICAL_TX;
                next_substate = 0;
              }
            } else {  // tof2
              // get current los and lol flags for this lane
              qsfp_fsm_check_tx_lock(conn_id, ch, &ok);
              if (ok) {  // current lane has signal and cdr is locked
                // add lane to mask for port
                qsfp_state[conn_id].sync_tx_locked |= 1 << ch;

                // see if we are good across all lanes & we are on the head lane
                if (((qsfp_state[conn_id].sync_tx_locked &
                      host_chmask_thisport) != host_chmask_thisport) ||
                    ((!qsfp_state[conn_id]
                           .per_ch_fsm[host_head_ch]
                           .head_1st_tx_locked) &&
                     (ch != host_head_ch))) {
                  // not all lanes ready yet, keep waiting
                  break;
                }

                // entire logical port has sig det and cdr lock, advance states
                qsfp_state[conn_id]
                    .per_ch_fsm[host_head_ch]
                    .head_1st_tx_locked = true;
                delay_ms = 0;
                next_st = QSFP_CH_FSM_ST_ENA_OPTICAL_TX;
                next_substate = 0;
              }
            }
            break;
          } else if (!qsfp_needs_hi_pwr_init(conn_id)) {
            // CMIS, software init - wait for initialization to complete on all
            //   data path channels. The module is supposed to only report the
            //   new state when all channels in the data path have reached it

            if (memmap_format == MMFORMAT_CMIS3P0) {
              // CMIS 3.0 - exit state from DataPathInit is DataPathActivated
              if (qsfp_fsm_ch_cmis_is_state_dpactivated(dev_id, conn_id, ch)) {
                delay_ms = 0;  // we already waited, so we're ready to proceed
                next_st = QSFP_CH_FSM_ST_ENA_OPTICAL_TX;
                next_substate = 0;
              } else {
                LOG_DEBUG("QSFPCSM %2d/%d : Not all data path states %s %s",
                          conn_id,
                          ch,
                          "have reached DataPathActivated. ",
                          "Continuing to wait...");
              }
            } else {
              // CMIS 4.0+ - exit state from DataPathInit is DataPathInitialized
              if (qsfp_fsm_ch_cmis_is_state_dpinitialized(
                      dev_id, conn_id, ch)) {
                delay_ms = 0;  // we already waited, so we're ready to proceed
                next_st = QSFP_CH_FSM_ST_ENA_OPTICAL_TX;
                next_substate = 0;
              } else {
                LOG_DEBUG("QSFPCSM %2d/%d : Not all data path states %s %s",
                          conn_id,
                          ch,
                          "have reached DataPathInitialized. ",
                          "Continuing to wait...");
              }
            }
            break;
          }
          // CMIS hardware init - nothing to do, advance major state
          delay_ms = 0;
          next_st = QSFP_CH_FSM_ST_ENA_OPTICAL_TX;
          next_substate = 0;
          break;
      }
      break;  // QSFP_CH_FSM_ST_ENA_CDR
    case QSFP_CH_FSM_ST_ENA_OPTICAL_TX:
      // this state enables the optical transmitters for all module types

      qsfp_state[conn_id].sync_ena_opt_tx |= 1 << ch;

      // this check makes sure all channel state machines go through this state
      // at the same time, starting wich the head channel
      if (((qsfp_state[conn_id].sync_ena_opt_tx & host_chmask_thisport) !=
           host_chmask_thisport) ||
          ((!qsfp_state[conn_id]
                 .per_ch_fsm[host_head_ch]
                 .head_1st_ena_opt_tx) &&
           (ch != host_head_ch))) {
        break;
      }
      qsfp_state[conn_id].per_ch_fsm[host_head_ch].head_1st_ena_opt_tx = true;

      switch (next_substate) {
        case 0:  // step 0 - enable the transmitters
          // applies to all memory map formats
          wr_coalesce_err = qsfp_fsm_ch_enable_optical_tx(
              dev_id, conn_id, qsfp_state[conn_id].dev_cfg[ch].media_ch, 0);
          if (wr_coalesce_err == 0) {
            delay_ms = 0;  // wait in the next substate
            next_substate++;
          }
          break;
        case 1:  // step 1 - wait for the transmitters to be enabled
          memmap_format = bf_qsfp_get_memmap_format(conn_id);
          if ((memmap_format == MMFORMAT_SFF8636) ||
              (memmap_format == MMFORMAT_CMIS3P0)) {
            // CMIS state machine did not comprehend optics readiness until
            // CMIS4.0. Calculate the delay needed for the optics to stabilize
            if (qsfp_fsm_ch_first_enable(dev_id, conn_id, ch)) {
              delay_ms = 400 + 3100;  // 3.5 sec for Luxtera MZI init
              LOG_DEBUG(
                  "QSFPCSM %2d/%d : First enable after reset. Extra 3.1 sec "
                  "wait",
                  conn_id,
                  ch);
            } else {
              delay_ms = 400;  // ms
            }
            next_st = QSFP_CH_FSM_ST_NOTIFY_ENABLED;
            next_substate = 0;
            break;
          } else {  // CMIS 4.0+
            // wait for tx enable to complete on all data path channels. The
            // module is supposed to only report the new state when all channels
            // in the data path have reached it
            if (qsfp_fsm_ch_cmis_is_state_dpactivated(dev_id, conn_id, ch)) {
              delay_ms = 0;  // we already waited, so we're ready to proceed
              next_st = QSFP_CH_FSM_ST_NOTIFY_ENABLED;
              next_substate = 0;
            }
          }
          break;
      }
      break;  // QSFP_CH_FSM_ST_ENA_OPTICAL_TX
    case QSFP_CH_FSM_ST_NOTIFY_ENABLED:
      // this state does a check for faults and, if none found, notifies
      // the driver that the module is ready to bring the link up

      qsfp_state[conn_id].sync_notify_enb |= 1 << ch;

      // this check makes sure all channel state machines go through this state
      // at the same time, starting wich the head channel
      if (((qsfp_state[conn_id].sync_notify_enb & host_chmask_thisport) !=
           host_chmask_thisport) ||
          ((!qsfp_state[conn_id]
                 .per_ch_fsm[host_head_ch]
                 .head_1st_ena_opt_tx) &&
           (ch != host_head_ch))) {
        break;
      }
      qsfp_state[conn_id].per_ch_fsm[host_head_ch].head_1st_ena_opt_tx = true;

      qsfp_fsm_ch_first_enable_clear(dev_id, conn_id, ch);
      if (bf_pm_intf_is_device_family_tofino(dev_id)) {
        qsfp_fsm_ch_check_tx_optical_fault(dev_id,
                                           conn_id,
                                           ch,
                                           true /*clear only*/,
                                           &discard_latched,
                                           &fault_val);
        // for some reason, need to read twice to clear latched faults
        qsfp_fsm_ch_check_tx_optical_fault(dev_id,
                                           conn_id,
                                           ch,
                                           true /*clear only*/,
                                           &discard_latched,
                                           &fault_val);
        qsfp_fsm_ch_check_tx_optical_fault(
            dev_id, conn_id, ch, false /* log faults*/, &ok, &fault_val);
        if (!ok) {
          LOG_DEBUG("QSFP    %2d : ch[%d] : Optical TX FAULT <%02x>. retry..",
                    conn_id,
                    ch,
                    fault_val);
        }
      } else {  // tof 2
        qsfp_fsm_check_tx_faults(conn_id, ch, &ok);
        if (!ok) {
          LOG_DEBUG("QSFP    %2d : ch[%d] : FAULT in TX, retry..", conn_id, ch);
        }
      }

      if (!ok) {
        // we had a fault in the tx, take all lanes in the logical port back
        // through the ch_fsm, starting in ENABLING
        qsfp_log_port_retry(dev_id, conn_id, ch);
        next_st = QSFP_CH_FSM_ST_DISABLING;
      } else {  // ok
        if (bf_pm_intf_is_device_family_tofino(dev_id)) {
          // notify driver
          qsfp_fsm_ch_notify_ready(dev_id, conn_id, ch);
        }
        // else the driver stuff is handled in the pm_intf_fsm when it sees
        // all lanes are ENABLED

        delay_ms = 0;  // ms
        next_st = QSFP_CH_FSM_ST_ENABLED;
      }
      break;
    case QSFP_CH_FSM_ST_ENABLED:
      // waiting to start disabling
      next_substate = 0;
      break;
    case QSFP_CH_FSM_ST_DISABLING:
      // this state notifies the driver that the module is no longer ready,
      // disables the optical transmitters, and then deinitializes the
      // data path, if applicable
      switch (substate) {
        case 0:  // step 0 - notify driver
          if (bf_pm_intf_is_device_family_tofino(dev_id)) {
            qsfp_fsm_ch_notify_not_ready(dev_id, conn_id, ch);
          }
          delay_ms = 0;  // no need to wait here
          next_substate++;
          break;
        case 1:  // step 1 - disable optical TX
          wr_coalesce_err = qsfp_fsm_ch_disable_optical_tx(
              dev_id, conn_id, qsfp_state[conn_id].dev_cfg[ch].media_ch);
          if (wr_coalesce_err == 0) {
            delay_ms = 0;  // wait in the next substate
            next_substate++;
          }
          break;
        case 2:  // step 2 - wait for transmitters to be disabled
          memmap_format = bf_qsfp_get_memmap_format(conn_id);
          if (memmap_format == MMFORMAT_CMIS4P0) {
            if (qsfp_fsm_ch_cmis_is_state_dpinitialized(dev_id, conn_id, ch)) {
              delay_ms = 0;  // we already waited, so we're ready to proceed
              next_substate++;
            }
          } else {
            delay_ms = 100;  // wait 100ms for transmitters to be enabled
            next_substate++;
          }
          break;
        case 3:  // step 3 - deinitialize the data path
          if (bf_qsfp_is_cmis(conn_id)) {
            wr_coalesce_err =
                qsfp_fsm_ch_cmis_datapath_deinit(dev_id, conn_id, ch);
            if (wr_coalesce_err == 0) {  // successfully added to queue
              delay_ms = 0;              // wait in the next substate
              next_substate++;
            }
            break;
          }
          next_substate++;  // SFF-8636 - nothing to do
        // fall through
        case 4:  // step 4 - wait for data path to be deinitialized
          if (bf_qsfp_is_cmis(conn_id)) {
            // no pkb support, should be used for real CMIS modules
            if (!qsfp_fsm_ch_cmis_is_state_dpdeactivated(dev_id, conn_id, ch)) {
              break;  // keep waiting
            }
          }
          if (qsfp_state[conn_id].per_ch_fsm[ch].immediate_enable) {
            next_st = QSFP_CH_FSM_ST_ENABLING;
          } else {
            next_st = QSFP_CH_FSM_ST_DISABLED;
          }
          delay_ms = 0;  // we already waited, so we're ready to proceed
          next_substate = 0;
          break;
      }
      break;  // QSFP_CH_FSM_ST_DISABLING
    default:
      assert(0);
      break;
  }
  qsfp_state[conn_id].per_ch_fsm[ch].fsm_st = next_st;
  qsfp_state[conn_id].per_ch_fsm[ch].substate = next_substate;
  qsfp_fsm_next_run_time_set(
      &qsfp_state[conn_id].per_ch_fsm[ch].next_fsm_run_time, delay_ms);

  if ((st != next_st) || (substate != next_substate)) {
    LOG_DEBUG("QSFPCSM %2d/%d : %s.%d --> %s.%d (%dms)",
              conn_id,
              ch,
              qsfp_fsm_ch_en_st_to_str[st],
              substate,
              qsfp_fsm_ch_en_st_to_str[next_st],
              next_substate,
              delay_ms);
  }

  // clear the cache for all lane flags that we aren't ignoring
  if (!bf_pm_intf_is_device_family_tofino(dev_id)) {
    clear_log_lane_flags(conn_id, ch);
  }
}

// called by qsfp fsm timer to process QSFPs for a given device
bf_pltfm_status_t qsfp_fsm(bf_dev_id_t dev_id) {
  int conn_id;
#if MONITOR_TEMPS
  static struct timespec next_temp_poll_time = {0, 0};
#endif
  struct timespec now;

  clock_gettime(CLOCK_MONOTONIC, &now);

#if MONITOR_TEMPS
  if (next_temp_poll_time.tv_sec == 0) {
    qsfp_fsm_next_run_time_set(&next_temp_poll_time, 0);
  }
#endif

  extern int in_rtmr_init;
  if (in_rtmr_init) {
    LOG_DEBUG("ERROR: Both retimer init and qsfp fsm running");
    return BF_PLTFM_SUCCESS;
  }

  /* Note: the channel FSM is run before the module FSM so any coalesced
  * writes are performed immediately. Otherwise they would have to wait
  * one extra poll period.
  */
  for (conn_id = 1; conn_id <= bf_pm_num_ports_get(); conn_id++) {
    if (bf_bd_is_this_port_internal(conn_id, 0)) {
      continue;
    }
    if ((qsfp_state[conn_id].fsm_st != QSFP_FSM_ST_IDLE) &&
        (qsfp_state[conn_id].fsm_st != QSFP_FSM_ST_INSERTED) &&
        (qsfp_state[conn_id].fsm_st != QSFP_FSM_ST_WAIT_TON_LPMODE) &&
        (qsfp_state[conn_id].fsm_st != QSFP_FSM_ST_LPMODE) &&
        (qsfp_fsm_is_optical(conn_id)) &&
        (!bf_pm_intf_is_device_family_tofino(dev_id))) {
#if !POLL_QSFP_INT_FLAGS
      if (bf_pltfm_qsfp_get_interrupt_status(conn_id) == 0) {  // intl asserted
#else
      if ((qsfp_state[conn_id].next_flag_poll_time.tv_sec < now.tv_sec) ||
          ((qsfp_state[conn_id].next_flag_poll_time.tv_sec == now.tv_sec) &&
           (qsfp_state[conn_id].next_flag_poll_time.tv_nsec < now.tv_nsec))) {
        qsfp_fsm_next_run_time_set(&qsfp_state[conn_id].next_flag_poll_time,
                                   FLAG_POLL_INTERVAL_MS);
#endif
        bf_qsfp_refresh_flags(conn_id);
        // log interesting flags that have changed states
        qsfp_log_module_flags(conn_id);
        qsfp_log_lane_flags(conn_id);
      }
    }
    if (qsfp_state[conn_id].fsm_st == QSFP_FSM_ST_DETECTED) {
      int ch;

      if (!qsfp_fsm_is_optical(conn_id)) continue;

      for (ch = 0; ch < qsfp_state[conn_id].ch_cnt; ch++) {
        if (qsfp_state[conn_id].per_ch_fsm[ch].next_fsm_run_time.tv_sec >
            now.tv_sec)
          continue;
        if ((qsfp_state[conn_id].per_ch_fsm[ch].next_fsm_run_time.tv_sec ==
             now.tv_sec) &&
            (qsfp_state[conn_id].per_ch_fsm[ch].next_fsm_run_time.tv_nsec >
             now.tv_nsec))
          continue;
        // time to run this QSFP (channel) state
        qsfp_channel_fsm_run(dev_id, conn_id, ch);
      }
    }
  }
#if MONITOR_TEMPS
  if ((next_temp_poll_time.tv_sec < now.tv_sec) ||
      ((next_temp_poll_time.tv_sec == now.tv_sec) &&
       (next_temp_poll_time.tv_nsec < now.tv_nsec))) {
    qsfp_fsm_next_run_time_set(&next_temp_poll_time, TEMP_POLL_INTERVAL_MS);
    for (conn_id = 1; conn_id <= bf_pm_num_ports_get(); conn_id++) {
      if ((!bf_bd_is_this_port_internal(conn_id, 0)) &&
          qsfp_fsm_is_optical(conn_id)) {
        // bf_qsfp_get_temp_sensor(conn_id);
      }
    }
  }
#endif
  for (conn_id = 1; conn_id <= bf_pm_num_ports_get(); conn_id++) {
    if (bf_bd_is_this_port_internal(conn_id, 0)) {
      continue;
    }
    if (qsfp_state[conn_id].fsm_st != QSFP_FSM_ST_IDLE) {
      if (qsfp_state[conn_id].next_fsm_run_time.tv_sec > now.tv_sec) continue;
      if ((qsfp_state[conn_id].next_fsm_run_time.tv_sec == now.tv_sec) &&
          (qsfp_state[conn_id].next_fsm_run_time.tv_nsec > now.tv_nsec))
        continue;
      // time to run this QSFP (module) state
      qsfp_module_fsm_run(dev_id, conn_id);

      // clear the cache for all module flags that we aren't ignoring
      if ((!bf_pm_intf_is_device_family_tofino(dev_id)) &&
          qsfp_fsm_is_optical(conn_id)) {
        clear_log_module_flags(conn_id);
      }
    }
  }
  return BF_PLTFM_SUCCESS;
}

/*****************************************************************
*
*****************************************************************/
void qsfp_oper_info_get(int conn_id,
                        bool *present,
                        uint8_t *pg0_lower,
                        uint8_t *pg0_upper) {
  bf_status_t rc;

  if (!present || !pg0_lower || !pg0_upper) {
    return;
  }
  if ((qsfp_state[conn_id].qsfp_type != QSFP_TYP_OPTICAL) &&
      (qsfp_state[conn_id].qsfp_type != QSFP_TYP_COPPER)) {
    *present = false;
    return;
  }
  *present = true;

  rc = bf_fsm_qsfp_rd(conn_id, 0, 0, 128, pg0_lower);
  if (rc) {
    LOG_ERROR("QSFP    %2d : Error <%d> reading page 0 (lower)", conn_id, rc);
    *present = false;
    return;
  }

  rc = bf_fsm_qsfp_rd(conn_id, 0, 128, 128, pg0_upper);
  if (rc) {
    LOG_ERROR("QSFP    %2d : Error <%d> reading page 0 (upper)", conn_id, rc);
    *present = false;
    return;
  }
}

/*****************************************************************
*
*****************************************************************/
void qsfp_oper_info_get_pg3(int conn_id, bool *present, uint8_t *pg3) {
  bf_status_t rc;

  if (!present || !pg3) {
    return;
  }
  if (qsfp_state[conn_id].qsfp_type != QSFP_TYP_OPTICAL) {
    *present = false;
    return;
  }
  if (qsfp_state[conn_id].flat_mem) {
    *present = false;
    return;
  }

  *present = true;

  rc = bf_fsm_qsfp_rd(conn_id, 3, 128, 128, pg3);
  if (rc) {
    LOG_ERROR("QSFP    %2d : Error <%d> reading page 3", conn_id, rc);
    *present = false;
    return;
  }
}

/*****************************************************************
*
*****************************************************************/
void qsfp_luxtera_lpbk(int conn_id, bool near_lpbk) {
  uint8_t pwd[] = {0x4d, 0xe6, 0x40, 0xbb};
  uint8_t pg90 = 0x90;
  uint8_t lpbk_mode_elec_near[] = {0xf2, 0x00, 0x02, 0x55, 0x55};
  uint8_t lpbk_mode_opt_far[] = {0xf2, 0x00, 0x02, 0xaa, 0xaa};
  uint8_t byte80, byte81;
  int rc;

  rc = bf_fsm_qsfp_wr(conn_id, 0, 0x7b, 4, pwd);
  if (rc) {
    LOG_ERROR(
        "QSFP    %2d : Error <%d> Setting pwd for loopback mode", conn_id, rc);
    return;
  }

  if (rc) {
    LOG_ERROR("QSFP    %2d : Error <%d> Setting page 0x90 for loopback mode",
              conn_id,
              rc);
    return;
  }

  if (near_lpbk) {
    rc = bf_fsm_qsfp_wr(conn_id, pg90, 0x84, 5, lpbk_mode_elec_near);
  } else {
    rc = bf_fsm_qsfp_wr(conn_id, pg90, 0x84, 5, lpbk_mode_opt_far);
  }
  if (rc) {
    LOG_ERROR("QSFP    %2d : Error <%d> Setting loopback mode <%s>",
              conn_id,
              rc,
              near_lpbk ? "NEAR/ELEC." : "FAR/OPTICAL");
    return;
  }

  rc = bf_fsm_qsfp_rd(conn_id, pg90, 0x80, 1, &byte80);
  if (rc) {
    LOG_ERROR("QSFP    %2d : Error <%d> Reading byte 0x80 for loopback mode",
              conn_id,
              rc);
    return;
  }
  byte80 += 1;
  rc = bf_fsm_qsfp_wr(conn_id, pg90, 0x80, 1, &byte80);
  if (rc) {
    LOG_ERROR("QSFP    %2d : Error <%d> Setting byte 0x80 for loopback mode",
              conn_id,
              rc);
    return;
  }

  byte81 = 0x85;
  rc = bf_fsm_qsfp_wr(conn_id, pg90, 0x81, 1, &byte81);
  if (rc) {
    LOG_ERROR("QSFP    %2d : Error <%d> Setting byte 0x81 for loopback mode",
              conn_id,
              rc);
    return;
  }
  LOG_ERROR("QSFP    %2d : loopback mode <%s> set.",
            conn_id,
            near_lpbk ? "NEAR/ELEC." : "FAR/OPTICAL");
}

bool bf_pm_qsfp_is_luxtera(int conn_id) {
  if (qsfp_needs_hi_pwr_init(conn_id)) {
    return true;
  } else {
    return false;
  }
}

void bf_pm_qsfp_luxtera_state_capture(int conn_id, uint8_t arr_0x3k[0x3000]) {
  int rc;
  uint8_t dump_tag = 0xdd;
  int chunk = 0;
  uint8_t luxtera_pwd[] = {0x64, 0xb0, 0x05, 0xa2};

  // set dump tag
  rc = bf_fsm_qsfp_wr(conn_id, 0, 0x7A, 1, &dump_tag);
  if (rc) {
    LOG_ERROR("QSFP    %2d : Error <%d> Setting byte 0x81 for loopback mode",
              conn_id,
              rc);
  }

  // set luxtera password
  rc = bf_fsm_qsfp_wr(conn_id, 0, 0x7B, 4, luxtera_pwd);
  if (rc) {
    LOG_ERROR("QSFP    %2d : Error <%d> Setting byte 0x81 for loopback mode",
              conn_id,
              rc);
  }

  // wait 10 seconds for dump to complete
  sleep(10);

  for (chunk = 0; chunk < 0x3000 / 128; chunk++) {
    rc = bf_fsm_qsfp_rd(
        conn_id, 0, 0x0801d000 + (chunk * 128), 128, &arr_0x3k[(chunk * 128)]);
    if (rc) {
      LOG_ERROR("QSFP    %2d : Error <%d> Reading 128 bytes from %08x.",
                conn_id,
                rc,
                0x0801d000 + (chunk * 128));
    }
  }
}

/*****************************************************************
*
*****************************************************************/
void qsfp_state_ha_config_set(bf_dev_id_t dev_id, int conn_id) {
  bool is_optical = false;
  int chnl_id;

  qsfp_fsm_notify_bf_pltfm(dev_id, conn_id);

  qsfp_fsm_identify_type(conn_id, &is_optical);
  qsfp_state[conn_id].fsm_st = QSFP_FSM_ST_DETECTED;

  /* Make sure all channel state is disabled */
  for (chnl_id = 0; chnl_id < QSFP_NUM_CHN; chnl_id++) {
    qsfp_state[conn_id].per_ch_fsm[chnl_id].fsm_st = QSFP_CH_FSM_ST_ENABLED;
  }
  qsfp_fsm_next_run_time_set(&qsfp_state[conn_id].next_fsm_run_time, 0);
  qsfp_state[conn_id].status_and_alarms.status2 = 0;
}

void qsfp_state_ha_config_delete(int conn_id) {
  qsfp_state[conn_id].fsm_st = QSFP_FSM_ST_IDLE;
}

/*****************************************************************
*
*****************************************************************/
void qsfp_state_ha_enable_config_set(int conn_id, int chnl_id) {
  bool is_optical = qsfp_fsm_is_optical(conn_id);
  if (is_optical) {
    qsfp_state[conn_id].per_ch_fsm[chnl_id].fsm_st = QSFP_CH_FSM_ST_ENABLED;
    qsfp_fsm_next_run_time_set(
        &qsfp_state[conn_id].per_ch_fsm[chnl_id].next_fsm_run_time, 0);
    qsfp_state[conn_id].status_and_alarms.los_ind &= ~(1 << chnl_id);
  }
}

/*****************************************************************
*
*****************************************************************/
void qsfp_state_ha_enable_config_delete(int conn_id, int chnl_id) {
  qsfp_state[conn_id].per_ch_fsm[chnl_id].fsm_st = QSFP_CH_FSM_ST_DISABLED;
}

/*****************************************************************
* qsfp_num_media_lanes_get
*
* Utility fn to return number of lanes associate with a port
*****************************************************************/
static int qsfp_num_media_lanes_get(int conn_id, int chnl_id) {
  return qsfp_state[conn_id].dev_cfg[chnl_id].media_intf_nlanes;
}

/*****************************************************************
* qsfp_num_media_lanes_get
*
* Utility fn to return number of lanes associate with a port
*****************************************************************/
static int qsfp_num_host_lanes_get(int conn_id, int chnl_id) {
  return qsfp_state[conn_id].dev_cfg[chnl_id].host_intf_nlanes;
}

/*****************************************************************
* Updates dev_cfg structure inside qsfp_state for the specifed
* port and channel with the current configuration info
*****************************************************************/
int qsfp_fsm_update_cfg(int conn_id,
                        int host_first_ch,
                        bf_port_speed_t intf_speed,
                        int host_intf_nlanes,
                        bf_pltfm_encoding_type_t encoding) {
  int cur_ch;

  for (cur_ch = host_first_ch; cur_ch < (host_first_ch + host_intf_nlanes);
       cur_ch++) {
    qsfp_state[conn_id].dev_cfg[cur_ch].intf_speed = intf_speed;
    qsfp_state[conn_id].dev_cfg[cur_ch].host_intf_nlanes = host_intf_nlanes;
    qsfp_state[conn_id].dev_cfg[cur_ch].host_head_ch = host_first_ch;

    // we don't know the media lanes yet, these are determined when the port
    // is enabled and the Application is selected
    qsfp_state[conn_id].dev_cfg[cur_ch].media_intf_nlanes = 0;
    qsfp_state[conn_id].dev_cfg[cur_ch].media_head_ch = -1;
    qsfp_state[conn_id].dev_cfg[cur_ch].media_ch = -1;

    qsfp_state[conn_id].dev_cfg[cur_ch].encoding = encoding;
  }
  return 0;
}

/*****************************************************************
* clears the dev_cfg structure inside qsfp_state for the specifed
* port and channel
*****************************************************************/
int qsfp_fsm_deinit_cfg(int conn_id, int host_first_ch) {
  int cur_ch, host_intf_nlanes;
  host_intf_nlanes =
      qsfp_state[conn_id].dev_cfg[host_first_ch].host_intf_nlanes;
  for (cur_ch = host_first_ch; cur_ch < (host_first_ch + host_intf_nlanes);
       cur_ch++) {
    qsfp_state[conn_id].dev_cfg[cur_ch].intf_speed = 0;
    qsfp_state[conn_id].dev_cfg[cur_ch].host_intf_nlanes = 0;
    qsfp_state[conn_id].dev_cfg[cur_ch].host_head_ch = -1;
    qsfp_state[conn_id].dev_cfg[cur_ch].media_intf_nlanes = 0;
    qsfp_state[conn_id].dev_cfg[cur_ch].media_head_ch = -1;
    qsfp_state[conn_id].dev_cfg[cur_ch].media_ch = -1;
    qsfp_state[conn_id].dev_cfg[cur_ch].encoding = 0;
  }

  return 0;
}

int qsfp_fsm_get_enabled_mask(int conn_id, int first_ch) {
  int cur_ch, host_intf_nlanes;
  int enabled_mask = 0;

  // make sure all lanes are in QSFP_CH_FSM_ST_ENABLING
  host_intf_nlanes = qsfp_state[conn_id].dev_cfg[first_ch].host_intf_nlanes;
  for (cur_ch = first_ch; cur_ch < (first_ch + host_intf_nlanes); cur_ch++) {
    if (qsfp_state[conn_id].per_ch_fsm[cur_ch].fsm_st ==
        QSFP_CH_FSM_ST_ENABLED) {
      enabled_mask |= 1 << cur_ch;
    }
  }
  return enabled_mask;
}
