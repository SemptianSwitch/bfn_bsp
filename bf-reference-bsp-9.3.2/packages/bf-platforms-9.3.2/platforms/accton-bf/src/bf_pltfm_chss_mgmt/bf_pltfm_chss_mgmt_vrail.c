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

#define BF_MAX_VRAIL_CNT 17

static uint32_t ucd[BF_MAX_VRAIL_CNT];

void ucd_call_back(void *p) {
  int i = 0;
  int j = 0;
  int k = 0;
  char str[10];
  char *ptr = p;

  if (ptr == NULL) {
    LOG_ERROR("NULL POINTER PASSED to call back function\n");
    return;
  }

  for (i = 0; i < BF_MAX_VRAIL_CNT; i++) ucd[i] = 0;

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
    str[j] = '\0';
    ucd[k] = atoi(str);
    k++;
    if (ptr[i] == ']') break;
    i++;
  }

  return;
}

bf_pltfm_status_t bf_pltfm_chss_mgmt_pwr_rails_get(
    bf_pltfm_pwr_rails_info_t *pwr_rails) {
  char url[256];

  snprintf(url,
           sizeof(url),
           "http://[%s]:8080/api/sys/bmc/ucd",
           bf_pltfm_bmc_server_addr_get());
  CURL *curl;

  if (pwr_rails == NULL) {
    LOG_ERROR("Invalid pointer passed \n");
    return BF_PLTFM_INVALID_ARG;
  }

  curl = curl_easy_init();
  if (curl) {
    curl_easy_setopt(curl, CURLOPT_URL, url);
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, ucd_call_back);
    curl_easy_setopt(curl, CURLOPT_TIMEOUT, BF_PLTFM_CURL_TIMEOUT);
    curl_easy_perform(curl);

    /* always cleanup */
    curl_easy_cleanup(curl);
  } else {
    printf("CURL ERROR: Unable to use CURL\n");
    return BF_PLTFM_OBJECT_NOT_FOUND;
  }

  /* In case of error */
  if (ucd[0] != 0) {
    pwr_rails->vrail1 = 0;
    pwr_rails->vrail2 = 0;
    pwr_rails->vrail3 = 0;
    pwr_rails->vrail4 = 0;
    pwr_rails->vrail5 = 0;
    pwr_rails->vrail6 = 0;
    pwr_rails->vrail7 = 0;
    pwr_rails->vrail8 = 0;
    pwr_rails->vrail9 = 0;
    pwr_rails->vrail10 = 0;
    pwr_rails->vrail11 = 0;
    pwr_rails->vrail12 = 0;
    pwr_rails->vrail13 = 0;
    pwr_rails->vrail14 = 0;
    pwr_rails->vrail15 = 0;
    pwr_rails->vrail16 = 0;
    LOG_ERROR("Error returned from REST API with status %d \n", ucd[0]);
    return BF_PLTFM_COMM_FAILED;
  }

  pwr_rails->vrail1 = ucd[1];
  pwr_rails->vrail2 = ucd[2];
  pwr_rails->vrail3 = ucd[3];
  pwr_rails->vrail4 = ucd[4];
  pwr_rails->vrail5 = ucd[5];
  pwr_rails->vrail6 = ucd[6];
  pwr_rails->vrail7 = ucd[7];
  pwr_rails->vrail8 = ucd[8];
  pwr_rails->vrail9 = ucd[9];
  pwr_rails->vrail10 = ucd[10];
  pwr_rails->vrail11 = ucd[11];
  pwr_rails->vrail12 = ucd[12];
  pwr_rails->vrail13 = ucd[13];
  pwr_rails->vrail14 = ucd[14];
  pwr_rails->vrail15 = ucd[15];
  pwr_rails->vrail16 = ucd[16];

  return BF_PLTFM_SUCCESS;
}
