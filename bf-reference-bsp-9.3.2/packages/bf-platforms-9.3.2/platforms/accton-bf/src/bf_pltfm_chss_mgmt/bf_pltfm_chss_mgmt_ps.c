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

/* 10 Number of elements passed by BMC REST API. It includes error and data */
#define PS_NUM_ELE 16
#define PS_DESC_LEN 32

static uint32_t ps[PS_NUM_ELE];
static char ps_model[PS_DESC_LEN];
static char ps_serial[PS_DESC_LEN];
static char ps_rev[PS_DESC_LEN];
static bool ps_presence;

static void ps_call_back(void *p) {
  int i = 0;
  int j = 0;
  int k = 0;
  char str[PS_DESC_LEN];
  char *ptr = p;

  if (ptr == NULL) {
    LOG_ERROR("NULL POINTER PASSED to call back function\n");
    return;
  }

  memset(ps_model, 0, sizeof(ps_model));
  memset(ps_serial, 0, sizeof(ps_serial));
  memset(ps_rev, 0, sizeof(ps_rev));

  for (i = 0; i < 10; i++) ps[i] = 0;

  i = 0;

  while (ptr[i] && ptr[i] != '[') {
    i++;
  }

  if (!ptr[i]) {
    return;
  }

  i++;
  while (ptr[i] && ptr[i] != ']') {
    j = 0;
    while (ptr[i] != ',' && ptr[i] != ']') {
      str[j] = ptr[i];
      j++;
      i++;
    }
    if (j >= PS_DESC_LEN) { /* sanity check */
      LOG_ERROR(
          "bad string len from BMC i %d j %d k %d 0x%02x\n", i, j, k, ptr[i]);
      return;
    }
    str[j] = '\0';
    if ((k < 9) || (k > 11)) {
      ps[k] = atoi(str);
    } else {
      switch (k) {
        case 9:
          strncpy(ps_model, str, sizeof(ps_model));
          ps_model[sizeof(ps_model) - 1] = '\0';
          break;
        case 10:
          strncpy(ps_serial, str, sizeof(ps_serial));
          ps_serial[sizeof(ps_serial) - 1] = '\0';
          break;
        case 11:
          strncpy(ps_rev, str, sizeof(ps_rev));
          ps_rev[sizeof(ps_rev) - 1] = '\0';
          break;
      }
    }
    k++;
    if (k >= PS_NUM_ELE) { /*  sanity check */
      LOG_ERROR(
          "bad num_elem from BMC i %d j %d k %d 0x%02x\n", i, j, k, ptr[i]);
      return;
    }
    if (ptr[i] == ']') break;
    i++;
  }

  return;
}

static void ps_presence_call_back(void *p) {
  int i = 0;
  char *ptr = p;

  ps_presence = false;

  i = 0;

  while (ptr[i] && ptr[i] != '[') {
    i++;
  }

  if (!ptr[i]) {
    return;
  }

  i++;

  if (ptr[i] == '0') {
    ps_presence = true;
  } else {
    ps_presence = false;
  }

  return;
}

bf_pltfm_status_t bf_pltfm_chss_mgmt_pwr_supply_prsnc_get(
    bf_pltfm_pwr_supply_t pwr, bool *info) {
  char url_pwr1[256];
  char url_pwr2[256];

  const char *url;
  CURL *curl;

  snprintf(url_pwr1,
           sizeof(url_pwr1),
           "http://[%s]:8080/api/sys/bmc/ps_feature/1/presence",
           bf_pltfm_bmc_server_addr_get());
  snprintf(url_pwr2,
           sizeof(url_pwr1),
           "http://[%s]:8080/api/sys/bmc/ps_feature/2/presence",
           bf_pltfm_bmc_server_addr_get());
  if (info == NULL) {
    LOG_ERROR("Invalid pointer passed \n");
    return BF_PLTFM_INVALID_ARG;
  }

  if ((pwr != POWER_SUPPLY1) && (pwr != POWER_SUPPLY2)) {
    LOG_ERROR("Invalid power supply number \n");
    return BF_PLTFM_INVALID_ARG;
  }

  if (pwr == POWER_SUPPLY1) {
    url = url_pwr1;
  } else {
    url = url_pwr2;
  }

  curl = curl_easy_init();
  if (curl) {
    curl_easy_setopt(curl, CURLOPT_URL, url);
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, ps_presence_call_back);
    curl_easy_setopt(curl, CURLOPT_TIMEOUT, BF_PLTFM_CURL_TIMEOUT);
    curl_easy_perform(curl);

    /* always cleanup */
    curl_easy_cleanup(curl);
  } else {
    printf("CURL ERROR: Unable to use CURL\n");
    return BF_PLTFM_OBJECT_NOT_FOUND;
  }

  *info = ps_presence;

  return BF_PLTFM_SUCCESS;
}
bf_pltfm_status_t bf_pltfm_chss_mgmt_pwr_supply_get(
    bf_pltfm_pwr_supply_t pwr, bf_pltfm_pwr_supply_info_t *info) {
  char url_pwr1[256];
  char url_pwr2[256];

  const char *url;
  CURL *curl;

  snprintf(url_pwr1,
           sizeof(url_pwr1),
           "http://[%s]:8080/api/sys/bmc/ps/1",
           bf_pltfm_bmc_server_addr_get());
  snprintf(url_pwr2,
           sizeof(url_pwr1),
           "http://[%s]:8080/api/sys/bmc/ps/2",
           bf_pltfm_bmc_server_addr_get());

  if (info == NULL) {
    LOG_ERROR("Invalid pointer passed \n");
    return BF_PLTFM_INVALID_ARG;
  }

  if ((pwr != POWER_SUPPLY1) && (pwr != POWER_SUPPLY2)) {
    LOG_ERROR("Invalid power supply number \n");
    return BF_PLTFM_INVALID_ARG;
  }

  if (pwr == POWER_SUPPLY1) {
    url = url_pwr1;
  } else {
    url = url_pwr2;
  }

  curl = curl_easy_init();
  if (curl) {
    curl_easy_setopt(curl, CURLOPT_URL, url);
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, ps_call_back);
    curl_easy_setopt(curl, CURLOPT_TIMEOUT, BF_PLTFM_CURL_TIMEOUT);
    curl_easy_perform(curl);

    /* always cleanup */
    curl_easy_cleanup(curl);
  } else {
    printf("CURL ERROR: Unable to use CURL\n");
    return BF_PLTFM_OBJECT_NOT_FOUND;
  }

  /* In case of error */
  if (ps[0] != 0) {
    info->vin = 0;              /* Input voltage in Volts */
    info->vout = 0;             /* Output voltage in Volts */
    info->iout = 0;             /* Output current in milli Amperes */
    info->pwr_out = 0;          /* Output power in milli watts */
    info->fspeed = 0;           /* Fan speed in RPM */
    info->ffault = false;       /* Fan fault TRUE/FALSE */
    info->presence = false;     /* Power supply present or not */
    info->load_sharing = false; /* load sharing TRUE/FALSE */
    memset(info->model, 0, sizeof(info->model));
    memset(info->serial, 0, sizeof(info->serial));
    memset(info->rev, 0, sizeof(info->rev));
    LOG_ERROR("Error returned from REST API with status %d \n", ps[0]);
    return BF_PLTFM_COMM_FAILED;
  }

  info->vin = ps[1];     /* Input voltage in Volts */
  info->vout = ps[2];    /* Output voltage in Volts */
  info->iout = ps[3];    /* Output current in milli Amperes */
  info->pwr_out = ps[4]; /* Output power in milli watts */
  info->fspeed = ps[5];  /* Fan speed in RPM */
  if (ps[6] == 0) {
    info->ffault = false; /* Fan fault TRUE/FALSE */
  } else {
    info->ffault = true;
  }
  if (ps[7] == 0) {
    info->presence = false; /* Power supply present or not */
  } else {
    info->presence = true;
  }
  if (ps[8] == 0) {
    info->load_sharing = false; /* load sharing TRUE/FALSE */
  } else {
    info->load_sharing = true;
  }
  strncpy(info->model, ps_model, sizeof(info->model));
  strncpy(info->serial, ps_serial, sizeof(info->serial));
  strncpy(info->rev, ps_rev, sizeof(info->rev));
  /* force NULL terminate above strings */
  info->model[sizeof(info->model) - 1] = '\0';
  info->serial[sizeof(info->serial) - 1] = '\0';
  info->rev[sizeof(info->rev) - 1] = '\0';

  return BF_PLTFM_SUCCESS;
}
