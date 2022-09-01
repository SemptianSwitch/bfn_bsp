/*******************************************************************************
 * Copyright (c) 2015-2020 Barefoot Networks, Inc.
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * $Id: $
 *
 ******************************************************************************/
/*!
 * @file bf_pltfm_chss_mgmt_tmp.c
 * @date
 *
 * API's for reading temperature from BMC
 *
 */

/* Standard includes */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <curl/curl.h>
#include <unistd.h>

/* Module includes */
#include <bf_pltfm_types/bf_pltfm_types.h>
#include <bf_pltfm_chss_mgmt_intf.h>

/* Local header includes */
#include "bf_pltfm_bd_eeprom.h"
#include "chss_mgmt_tmp.h"
#include "chss_mgmt.h"

static char sensors_out[BF_SNSR_OUT_BUF_SIZE];
static int idx = 0;

static bf_pltfm_board_id_t board_id;
static int tmp[11] = {0};

void tmp_call_back(void *p) {
  int i = 0;
  int j = 0;
  int k = 0;
  char str[10];
  char *ptr = p;
  int max_sensors = 0;

  if (ptr == NULL) {
    LOG_ERROR("NULL POINTER PASSED to call back function\n");
    return;
  }

  if ((board_id == BF_PLTFM_BD_ID_MAVERICKS_P0A) ||
      (board_id == BF_PLTFM_BD_ID_MAVERICKS_P0B) ||
      (board_id == BF_PLTFM_BD_ID_MAVERICKS_P0C)) {
    max_sensors = 11;
  } else {
    max_sensors = 7;
  }

  for (i = 0; i < 11; i++) tmp[i] = -1;

  i = 0;

  while (ptr[i] && ptr[i] != '[') {
    i++;
  }

  if (!ptr[i]) {
    return;
  }

  i++;
  while (ptr[i] && ptr[i] != ']' && (k < max_sensors)) {
    j = 0;
    while (ptr[i] != ',' && ptr[i] != ']') {
      str[j] = ptr[i];
      j++;
      i++;
    }
    str[j] = '\0';
    tmp[k] = atoi(str);
    k++;
    if (ptr[i] == ']') break;
    i++;
  }

  return;
}

bf_pltfm_status_t bf_pltfm_chss_mgmt_temperature_get(
    bf_pltfm_temperature_info_t *t) {
  char url[256];
  CURL *curl;
  bf_pltfm_status_t r;

  if (t == NULL) {
    LOG_ERROR("Invalid pointer passed \n");
    return BF_PLTFM_INVALID_ARG;
  }

  r = bf_pltfm_chss_mgmt_bd_type_get(&board_id);

  if (r != BF_PLTFM_SUCCESS) {
    LOG_ERROR("Board ID retrival failed \n");
    return BF_PLTFM_COMM_FAILED;
  }

  if ((board_id == BF_PLTFM_BD_ID_MAVERICKS_P0A) ||
      (board_id == BF_PLTFM_BD_ID_MAVERICKS_P0B) ||
      (board_id == BF_PLTFM_BD_ID_MAVERICKS_P0C)) {
    snprintf(url,
             sizeof(url),
             "http://[%s]:8080/api/sys/bmc/tmp/%s",
             bf_pltfm_bmc_server_addr_get(),
             "Mavericks");
  } else if ((board_id == BF_PLTFM_BD_ID_MONTARA_P0A) ||
             (board_id == BF_PLTFM_BD_ID_MONTARA_P0B) ||
             (board_id == BF_PLTFM_BD_ID_MONTARA_P0C) ||
             (board_id == BF_PLTFM_BD_ID_NEWPORT_P0B) ||
             (board_id == BF_PLTFM_BD_ID_NEWPORT_P0A)) {
    snprintf(url,
             sizeof(url),
             "http://[%s]:8080/api/sys/bmc/tmp/%s",
             bf_pltfm_bmc_server_addr_get(),
             "Montara");

  } else {
    LOG_ERROR("Invalid board type passed \n");
    return BF_PLTFM_COMM_FAILED;
  }

  curl = curl_easy_init();
  if (curl) {
    curl_easy_setopt(curl, CURLOPT_URL, url);
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, tmp_call_back);
    curl_easy_setopt(curl, CURLOPT_TIMEOUT, BF_PLTFM_CURL_TIMEOUT);
    curl_easy_perform(curl);

    /* always cleanup */
    curl_easy_cleanup(curl);
  } else {
    printf("CURL ERROR: Unable to use CURL\n");
    return BF_PLTFM_OBJECT_NOT_FOUND;
  }

  /* In case of error */
  if (tmp[0] != 0) {
    t->tmp1 = 0;
    t->tmp2 = 0;
    t->tmp3 = 0;
    t->tmp4 = 0;
    t->tmp5 = 0;
    t->tmp6 = 0;
    t->tmp7 = 0;
    t->tmp8 = 0;
    t->tmp9 = 0;
    t->tmp10 = 0;
    LOG_ERROR("Error returned from REST API with status %d \n", tmp[0]);
    return BF_PLTFM_COMM_FAILED;
  }

  t->tmp1 = (float)tmp[1] / 10;
  t->tmp2 = (float)tmp[2] / 10;
  t->tmp3 = (float)tmp[3] / 10;
  t->tmp4 = (float)tmp[4] / 10;
  t->tmp5 = (float)tmp[5] / 10;

  if ((board_id == BF_PLTFM_BD_ID_MONTARA_P0A) ||
      (board_id == BF_PLTFM_BD_ID_MONTARA_P0B) ||
      (board_id == BF_PLTFM_BD_ID_MONTARA_P0C) ||
      (board_id == BF_PLTFM_BD_ID_NEWPORT_P0B) ||
      (board_id == BF_PLTFM_BD_ID_NEWPORT_P0A)) {
    t->tmp6 = (float)tmp[6] / 10;
    t->tmp7 = 0;
    t->tmp8 = 0;
    t->tmp9 = 0;
    t->tmp10 = 0;
  } else {
    t->tmp6 = (float)tmp[6] / 10;
    t->tmp7 = (float)tmp[7] / 10;
    t->tmp8 = (float)tmp[8] / 10;
    t->tmp9 = (float)tmp[9] / 10;
    t->tmp10 = (float)tmp[10] / 10;
  }

  return BF_PLTFM_SUCCESS;
}

bf_pltfm_status_t bf_pltfm_chss_mgmt_switch_temperature_get(
    bf_dev_id_t dev_id, int sensor, bf_pltfm_switch_temperature_info_t *tmp) {
  uint32_t temp_mC = 0;
  uint32_t timeout = 0;
  bf_status_t r;

  if ((tmp == NULL) || (sensor != 0)) {
    LOG_ERROR("Invalid paramter NULL\n");
    return BF_PLTFM_INVALID_ARG;
  }

  if (BF_SUCCESS != bf_serdes_temperature_read_start(
                        dev_id, sensor, BF_SDS_MAIN_TEMP_SENSOR_CH)) {
    LOG_ERROR("Error in starting main temperature read of tofino\n");
    return BF_PLTFM_COMM_FAILED;
  }

  do {
    usleep(TEN_MILLISECOND);
    timeout = timeout + TEN_MILLISECOND;
    r = bf_serdes_temperature_read_get(
        dev_id, sensor, BF_SDS_MAIN_TEMP_SENSOR_CH, &temp_mC);
  } while ((r == BF_NOT_READY) && (timeout < HUNDRED_MILLISECOND));

  if (r != BF_SUCCESS) {
    LOG_ERROR("Error in getting main temperature from tofino\n");
    return BF_PLTFM_COMM_FAILED;
  }

  tmp->main_sensor = temp_mC;

  temp_mC = 0;
  timeout = 0;

  if (BF_SUCCESS != bf_serdes_temperature_read_start(
                        dev_id, sensor, BF_SDS_REMOTE_TEMP_SENSOR_0_CH)) {
    LOG_ERROR("Error in starting remote temperature read of tofino\n");
    return BF_PLTFM_COMM_FAILED;
  }

  bf_serdes_temperature_read_get(
      dev_id, sensor, BF_SDS_REMOTE_TEMP_SENSOR_0_CH, &temp_mC);

  do {
    usleep(TEN_MILLISECOND);
    timeout = timeout + TEN_MILLISECOND;
    r = bf_serdes_temperature_read_get(
        dev_id, sensor, BF_SDS_REMOTE_TEMP_SENSOR_0_CH, &temp_mC);
  } while ((r == BF_NOT_READY) && (timeout < HUNDRED_MILLISECOND));

  if (r != BF_SUCCESS) {
    LOG_ERROR("Error in getting remote temperature from tofino\n");
    return BF_PLTFM_COMM_FAILED;
  }

  tmp->remote_sensor = temp_mC;

  return BF_PLTFM_SUCCESS;
}

int sensors_call_back(char *ptr, int size, int nmemb, void *userdata) {
  int i = 0;

  if (ptr == NULL) {
    LOG_ERROR("NULL POINTER PASSED to call back function\n");
    return 0;
  }

  /* Check to see if this is first call to the callback routine. */
  /* Added to support multiple responses to curl request */
  if (idx == 0) {
    while (ptr[i] && ptr[i] != '[') {
      i++;
    }
    if (!ptr[i]) {
      return 0;
    }

    i++;
  }

  while (ptr[i] && ptr[i] != ']') {
    /* Check for hitting max buffer length */
    if (idx >= BF_SNSR_OUT_BUF_SIZE) {
      return 0;
    }

    sensors_out[idx] = ptr[i];
    i++;
    idx++;
  }
  return (size * nmemb);
}

bf_pltfm_status_t pltfm_mgr_sensor_out_get(const char *options,
                                           char *info,
                                           size_t info_size) {
  char url[256];

  CURL *curl;

  if (options && options[0]) {
    snprintf(url,
             sizeof(url),
             "http://[%s]:8080/api/sys/bmc/sensors/%s",
             bf_pltfm_bmc_server_addr_get(),
             options);
  } else {
    snprintf(url,
             sizeof(url),
             "http://[%s]:8080/api/sys/bmc/sensors",
             bf_pltfm_bmc_server_addr_get());
  }

  if (info == NULL) {
    LOG_ERROR("Invalid pointer passed \n");
    return BF_PLTFM_INVALID_ARG;
  }

  curl = curl_easy_init();
  if (curl) {
    /* Init static variables before curl request */
    idx = 0;
    memset(sensors_out, 0, sizeof(sensors_out));

    curl_easy_setopt(curl, CURLOPT_URL, (const char *)url);
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, sensors_call_back);
    curl_easy_setopt(curl, (CURLOPT_TIMEOUT), 20L);
    curl_easy_perform(curl);

    /* always cleanup */
    curl_easy_cleanup(curl);
  } else {
    printf("CURL ERROR: Unable to use CURL\n");
    return BF_PLTFM_OBJECT_NOT_FOUND;
  }

  strncpy(info, sensors_out, info_size);

  return BF_PLTFM_SUCCESS;
}
