/*******************************************************************************
 * Copyright (c) 2015-2020 Barefoot Networks, Inc.
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * $Id: $
 *
 ******************************************************************************/
#include <stdio.h>
#include <unistd.h>

/* Module includes */
#include <bf_pltfm_types/bf_pltfm_types.h>
#include <bf_types/bf_types.h>
#include <bfsys/bf_sal/bf_sys_intf.h>
#include <bf_pltfm_chss_mgmt_intf.h>
#include <bf_pltfm_mgr/pltfm_mgr_handlers.h>

/* Local header includes */
#include "platform_priv.h"

static bf_pltfm_temperature_info_t peak;

uint8_t mntr_cntrl_hndlr = MNTR_DISABLE;
static uint8_t mntr_temp = MNTR_DISABLE;

static void check_pwr_supply(void) {
  bf_pltfm_pwr_supply_t pwr;
  bool info;
  /* check presence of power supply 1 */
  pwr = POWER_SUPPLY1;
  if (bf_pltfm_chss_mgmt_pwr_supply_prsnc_get(pwr, &info) != BF_PLTFM_SUCCESS) {
    LOG_ERROR("Error in reading power supply status\n");
  }

  if (!info) LOG_ALARM("POWER SUPPLY 1 not present \n");

  pwr = POWER_SUPPLY2;
  if (bf_pltfm_chss_mgmt_pwr_supply_prsnc_get(pwr, &info) != BF_PLTFM_SUCCESS) {
    LOG_ERROR("Error in reading power supply status\n");
  }

  if (!info) LOG_ALARM("POWER SUPPLY 2 not present \n");
}

static void check_tofino_temperature(void) {
  bf_dev_id_t dev_id = 0;
  int sensor = 0;
  bf_pltfm_switch_temperature_info_t temp_mC;
  bf_pltfm_status_t r;

  r = bf_pltfm_chss_mgmt_switch_temperature_get(dev_id, sensor, &temp_mC);

  if (r != BF_PLTFM_SUCCESS) {
    LOG_ERROR("Error in reading switch temperature\n");
  }

  if (temp_mC.main_sensor >= TOFINO_TMP_ALARM_RANGE) {
    LOG_ALARM("TOFINO MAIN TEMP SENOR above threshold value(%d): %d C\n",
              (TOFINO_TMP_ALARM_RANGE / 1000),
              (temp_mC.main_sensor / 1000));
    LOG_ALARM("=========================================\n");
    LOG_ALARM("        SHUTDOWN THE SYSTEM\n");
    LOG_ALARM("=========================================\n");
  }

  if (temp_mC.remote_sensor >= TOFINO_TMP_ALARM_RANGE) {
    LOG_ALARM("TOFINO REMOTE TEMP SENOR above threshold value(%d): %d C\n",
              (TOFINO_TMP_ALARM_RANGE / 1000),
              (temp_mC.remote_sensor / 1000));
    LOG_ALARM("=========================================\n");
    LOG_ALARM("        SHUTDOWN THE SYSTEM\n");
    LOG_ALARM("=========================================\n");
  }

  return;
}

static void check_chassis_temperature(void) {
  bf_pltfm_temperature_info_t t;

  if (bf_pltfm_chss_mgmt_temperature_get(&t) != BF_PLTFM_SUCCESS) {
    LOG_ERROR("Error in reading temperature \n");
    return;
  }

  if (t.tmp1 > CHASSIS_TMP_ALARM_RANGE) {
    LOG_ALARM("TMP1 in above alarm range :%f\n", t.tmp1);
  }

  if (t.tmp2 > CHASSIS_TMP_ALARM_RANGE) {
    LOG_ALARM("TMP2 in above alarm range :%f\n", t.tmp2);
  }

  if (t.tmp3 > CHASSIS_TMP_ALARM_RANGE) {
    LOG_ALARM("TMP3 in above alarm range :%f\n", t.tmp3);
  }

  if (t.tmp4 > CHASSIS_TMP_ALARM_RANGE) {
    LOG_ALARM("TMP4 in above alarm range :%f\n", t.tmp4);
  }

  if (t.tmp5 > CHASSIS_TMP_ALARM_RANGE) {
    LOG_ALARM("TMP5 in above alarm range :%f\n", t.tmp5);
  }

  if (t.tmp6 > CHASSIS_TMP_ALARM_RANGE) {
    LOG_ALARM("TMP6 in above alarm range :%f\n", t.tmp6);
  }

  if (t.tmp7 > CHASSIS_TMP_ALARM_RANGE) {
    LOG_ALARM("TMP7 in above alarm range :%f\n", t.tmp7);
  }

  if (t.tmp8 > CHASSIS_TMP_ALARM_RANGE) {
    LOG_ALARM("TMP8 in above alarm range :%f\n", t.tmp8);
  }

  if (t.tmp9 > CHASSIS_TMP_ALARM_RANGE) {
    LOG_ALARM("TMP9 in above alarm range :%f\n", t.tmp9);
  }

  /* TMP10 is assumed to be the ASIC temperature */
  if (t.tmp10 > ASIC_TMP_ALARM_RANGE) {
    LOG_ALARM("TMP10 in above alarm range :%f\n", t.tmp10);
  }

  if (t.tmp1 > peak.tmp1) {
    peak.tmp1 = t.tmp1;
  }

  if (t.tmp2 > peak.tmp2) {
    peak.tmp2 = t.tmp2;
  }

  if (t.tmp3 > peak.tmp3) {
    peak.tmp3 = t.tmp3;
  }

  if (t.tmp4 > peak.tmp4) {
    peak.tmp4 = t.tmp4;
  }

  if (t.tmp5 > peak.tmp5) {
    peak.tmp5 = t.tmp5;
  }

  if (t.tmp6 > peak.tmp6) {
    peak.tmp6 = t.tmp6;
  }

  if (t.tmp7 > peak.tmp7) {
    peak.tmp7 = t.tmp7;
  }

  if (t.tmp8 > peak.tmp8) {
    peak.tmp8 = t.tmp8;
  }

  if (t.tmp9 > peak.tmp9) {
    peak.tmp9 = t.tmp9;
  }

  if (t.tmp10 > peak.tmp10) {
    peak.tmp10 = t.tmp10;
  }
}

static void check_fantray(void) {
  bf_pltfm_fan_data_t fdata;

  if (bf_pltfm_chss_mgmt_fan_data_get(&fdata) != BF_PLTFM_SUCCESS) {
    LOG_ERROR("Error in reading temperature \n");
  }

  if (fdata.fantray_present) {
    LOG_ALARM("Fan tray not present \n");
  }
}

bf_pltfm_status_t bf_pltfm_temperature_get(
    bf_pltfm_temperature_info_t *tmp, bf_pltfm_temperature_info_t *peak_tmp) {
  if ((tmp == NULL) || (peak_tmp == tmp)) {
    LOG_ERROR("Invalid arguments passed \n");
    return BF_PLTFM_INVALID_ARG;
  }

  if (bf_pltfm_chss_mgmt_temperature_get(tmp) != BF_PLTFM_SUCCESS) {
    LOG_ERROR("Error in reading temperature \n");
    return BF_PLTFM_COMM_FAILED;
  }

  peak_tmp->tmp1 = peak.tmp1;
  peak_tmp->tmp2 = peak.tmp2;
  peak_tmp->tmp3 = peak.tmp3;
  peak_tmp->tmp4 = peak.tmp4;
  peak_tmp->tmp5 = peak.tmp5;
  peak_tmp->tmp6 = peak.tmp6;
  peak_tmp->tmp7 = peak.tmp7;
  peak_tmp->tmp8 = peak.tmp8;
  peak_tmp->tmp9 = peak.tmp9;
  peak_tmp->tmp10 = peak.tmp10;

  return BF_PLTFM_SUCCESS;
}

void bf_pltfm_temperature_monitor_enable(bool enable) {
  if (enable) {
    mntr_temp = MNTR_ENABLE;
  } else {
    mntr_temp = MNTR_DISABLE;
  }
  return;
}

static void run_health_mntr(void) {
  printf("Health monitor started \n");

  while (1) {
    if ((mntr_cntrl_hndlr == MNTR_ENABLE) || (mntr_temp == MNTR_ENABLE)) {
      check_chassis_temperature();
    }

    if (mntr_cntrl_hndlr == MNTR_ENABLE) {
      // Tofino temperature monitoring still needs to add FSM
      if (0) { // this info available thru check_chassis_temperature()
        check_tofino_temperature();
      }

      check_fantray();

      check_pwr_supply();
    }

    sleep(30);
  }

  // Never reaches here
  return;
}

/* Init function of thread */
void *health_mntr_init(void *arg) {
  (void)arg;
  run_health_mntr();

  return NULL;
}
