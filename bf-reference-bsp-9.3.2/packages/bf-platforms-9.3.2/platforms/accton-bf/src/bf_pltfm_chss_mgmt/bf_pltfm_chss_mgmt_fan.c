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

/* Module includes */
#include <bf_pltfm_types/bf_pltfm_types.h>
#include <bf_pltfm_chss_mgmt_intf.h>

/* Local header includes */
#include "bf_pltfm_bd_eeprom.h"
#include "chss_mgmt.h"

static bf_pltfm_board_id_t board_id;

/* 42 for Max number elements passed from BMC REST API call */
static int farr[42];
static int fan_set_success;

void fan_call_back(void *p) {
  uint8_t i = 0;
  uint8_t j = 0;
  uint8_t k = 0;
  char str[10];
  char *ptr = p;
  uint8_t max_digits = 0;

  if (ptr == NULL) {
    LOG_ERROR("NULL POINTER PASSED to call back function\n");
    return;
  }

  if ((board_id == BF_PLTFM_BD_ID_MAVERICKS_P0A) ||
      (board_id == BF_PLTFM_BD_ID_MAVERICKS_P0B) ||
      (board_id == BF_PLTFM_BD_ID_MAVERICKS_P0C)) {
    /* number of elements in data passed
     * err + (4 digit for fan data) * Number of fans + fantray presence
     */
    max_digits = 1 + 10 * 4 + 1;
  } else if ((board_id == BF_PLTFM_BD_ID_NEWPORT_P0A) ||
             (board_id == BF_PLTFM_BD_ID_NEWPORT_P0B)) {
    max_digits = 1 + 6 * 4 + 1;
  } else {
    max_digits = 1 + 5 * 4 + 1;
  }

  for (i = 0; i < 42; i++) {
    farr[i] = 0;
  }

  i = 0;

  while (ptr[i] && ptr[i] != '[') {
    i++;
  }

  if (!ptr[i]) {
    return;
  }

  i++;
  while (ptr[i] && ptr[i] != ']' && (k < max_digits)) {
    j = 0;
    while (ptr[i] != ',' && ptr[i] != ']') {
      str[j] = ptr[i];
      j++;
      i++;
    }
    str[j] = '\0';
    farr[k] = atoi(str);
    k++;
    if (ptr[i] == ']') break;
    i++;
  }

  return;
}

bf_pltfm_status_t bf_pltfm_chss_mgmt_fan_data_get(bf_pltfm_fan_data_t *fdata) {
  char url[256];
  CURL *curl;
  bf_pltfm_status_t r;
  uint8_t max_fan = 0;
  uint8_t i;

  if (fdata == NULL) {
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
    max_fan = 10;
    snprintf(url,
             sizeof(url),
             "http://[%s]:8080/api/sys/bmc/fan/get/%s",
             bf_pltfm_bmc_server_addr_get(),
             "Mavericks");
  } else if ((board_id == BF_PLTFM_BD_ID_MONTARA_P0A) ||
             (board_id == BF_PLTFM_BD_ID_MONTARA_P0C) ||
             (board_id == BF_PLTFM_BD_ID_MONTARA_P0B)) {
    max_fan = 5;
    snprintf(url,
             sizeof(url),
             "http://[%s]:8080/api/sys/bmc/fan/get/%s",
             bf_pltfm_bmc_server_addr_get(),
             "Montara");
  } else if ((board_id == BF_PLTFM_BD_ID_NEWPORT_P0A) ||
             (board_id == BF_PLTFM_BD_ID_NEWPORT_P0B)) {
    max_fan = 6;
    snprintf(url,
             sizeof(url),
             "http://[%s]:8080/api/sys/bmc/fan/get/%s",
             bf_pltfm_bmc_server_addr_get(),
             "Newport");
  } else {
    LOG_ERROR("Invalid board type passed \n");
    return BF_PLTFM_COMM_FAILED;
  }

  curl = curl_easy_init();
  if (curl) {
    curl_easy_setopt(curl, CURLOPT_URL, url);
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, fan_call_back);
    curl_easy_setopt(curl, CURLOPT_TIMEOUT, BF_PLTFM_CURL_TIMEOUT);
    curl_easy_perform(curl);

    /* always cleanup */
    curl_easy_cleanup(curl);
  } else {
    printf("CURL ERROR: Unable to use CURL\n");
    return BF_PLTFM_OBJECT_NOT_FOUND;
  }

  /* In case of error */
  if (farr[0] != 0) {
    for (i = 0; i < 10; i++) {
      fdata->F[i].fan_num = 0;
      fdata->F[i].front_speed = 0;
      fdata->F[i].rear_speed = 0;
      fdata->F[i].percent = 0;
    }
    fdata->fantray_present = 0;
    LOG_ERROR("Error returned from REST API with status %d \n", farr[0]);
    return BF_PLTFM_COMM_FAILED;
  }

  for (i = 0; i < max_fan; i++) {
    fdata->F[i].fan_num = farr[4 * i + 1];
    fdata->F[i].front_speed = farr[4 * i + 2];
    fdata->F[i].rear_speed = farr[4 * i + 3];
    fdata->F[i].percent = farr[4 * i + 4];
  }

  fdata->fantray_present = farr[4 * i + 1];

  return BF_PLTFM_SUCCESS;
}

void fan_set_call_back(void *p) {
  char *ptr = p;
  int i = 0;
  int x = 0;

  while (ptr[i] && ptr[i] != '[') {
    i++;
  }

  if (!ptr[i]) {
    return;
  }

  i++;

  fan_set_success = 0;
  x = ptr[i] - '0';
  if (x != 0) {
    fan_set_success = x;
  }

  return;
}

bf_pltfm_status_t bf_pltfm_chss_mgmt_fan_speed_set(bf_pltfm_fan_info_t *fdata) {
  char url[256];
  CURL *curl;
  bf_pltfm_status_t r;
  char str[10] = {'\0'};

  if (fdata == NULL) {
    LOG_ERROR("Invalid pointer passed \n");
    return BF_PLTFM_INVALID_ARG;
  }

  r = bf_pltfm_chss_mgmt_bd_type_get(&board_id);

  if (r != BF_PLTFM_SUCCESS) {
    LOG_ERROR("Board ID retrival failed \n");
    return BF_PLTFM_COMM_FAILED;
  }

  snprintf(str, 10, "/%d/%d", fdata->fan_num, fdata->percent);

  if ((board_id == BF_PLTFM_BD_ID_MAVERICKS_P0A) ||
      (board_id == BF_PLTFM_BD_ID_MAVERICKS_P0B) ||
      (board_id == BF_PLTFM_BD_ID_MAVERICKS_P0C)) {
    snprintf(url,
             sizeof(url),
             "http://[%s]:8080/api/sys/bmc/fan/set/%s%s",
             bf_pltfm_bmc_server_addr_get(),
             "Mavericks",
             str);
  } else if ((board_id == BF_PLTFM_BD_ID_MONTARA_P0A) ||
             (board_id == BF_PLTFM_BD_ID_MONTARA_P0C) ||
             (board_id == BF_PLTFM_BD_ID_MONTARA_P0B)) {
    snprintf(url,
             sizeof(url),
             "http://[%s]:8080/api/sys/bmc/fan/set/%s%s",
             bf_pltfm_bmc_server_addr_get(),
             "Montara",
             str);
  } else if ((board_id == BF_PLTFM_BD_ID_NEWPORT_P0A) ||
             (board_id == BF_PLTFM_BD_ID_NEWPORT_P0B)) {
    snprintf(url,
             sizeof(url),
             "http://[%s]:8080/api/sys/bmc/fan/set/%s%s",
             bf_pltfm_bmc_server_addr_get(),
             "Newport",
             str);
  } else {
    LOG_ERROR("Invalid board type passed \n");
    return BF_PLTFM_COMM_FAILED;
  }

  curl = curl_easy_init();
  if (curl) {
    curl_easy_setopt(curl, CURLOPT_URL, url);
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, fan_set_call_back);
    curl_easy_setopt(curl, CURLOPT_TIMEOUT, BF_PLTFM_CURL_TIMEOUT);
    curl_easy_perform(curl);

    /* always cleanup */
    curl_easy_cleanup(curl);
  } else {
    printf("CURL ERROR: Unable to use CURL\n");
    return BF_PLTFM_OBJECT_NOT_FOUND;
  }

  if (fan_set_success != 0) {
    LOG_ERROR("Setting fan speed return with error 0x%x \n", fan_set_success);
    return BF_PLTFM_COMM_FAILED;
  }
  return BF_PLTFM_SUCCESS;
}
