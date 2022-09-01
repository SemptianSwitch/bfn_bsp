/*******************************************************************************
 * Copyright (c) 2015-2020 Barefoot Networks, Inc.
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * $Id: $
 *
 ******************************************************************************/
/*!
 * @file bf_pltfm_eeprom.c
 * @date
 *
 * EEPROM parsing
 *
 */

/* Standard includes */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <curl/curl.h>
#include <bf_pltfm_chss_mgmt_intf.h>

/* Local header includes */
#include "bf_pltfm_bd_eeprom.h"

bf_pltfm_eeprom_t eeprom;

static bf_pltfm_board_id_t bd_id = BF_PLTFM_BD_ID_MAVERICKS_P0B_EMU;

static bf_pltfm_board_id_t board_id_nums[] = {BF_PLTFM_BD_ID_MAVERICKS_P0A,
                                              BF_PLTFM_BD_ID_MAVERICKS_P0B,
                                              BF_PLTFM_BD_ID_MAVERICKS_P0C,
                                              BF_PLTFM_BD_ID_MONTARA_P0A,
                                              BF_PLTFM_BD_ID_MONTARA_P0B,
                                              BF_PLTFM_BD_ID_MONTARA_P0C,
                                              BF_PLTFM_BD_ID_NEWPORT_P0A,
                                              BF_PLTFM_BD_ID_NEWPORT_P0B};

static char *board_id_strings[] = {
    "135-000011-01",  // Mavericks-P0A
    "015-000001-01",  // Mavericks-P0B
    "015-000001-02",  // Mavericks-P0C
    "135-000011-03",  // Montara-P0A
    "015-000003-01",  // Montara-P0B
    "015-000003-02",  // Montara-P0C, functionally same as Montara-P0B
    "015-000004-02",  // Newport-P0A
    "015-000004-03"   // Newport-P0B
};

static int board_id_set(char *ptr) {
  unsigned int i;

  if (ptr == NULL) {
    LOG_ERROR("EEPROM ERROR: Unable to read the board id EEPROM\n");
    return -1;
  }

  for (i = 0; i < (sizeof(board_id_strings) / sizeof(board_id_strings[0]));
       i++) {
    if (strcmp(board_id_strings[i], (char *)ptr) == 0) {
      bd_id = board_id_nums[i];
      LOG_DEBUG("EEPROM SUCCESS: Board type : %06x\n", bd_id);
      return 0;
    }
  }

  LOG_ERROR("EEPROM ERROR: Unknown Board type, Defaulting to Model\n");
  return -1;
}

/**
 * Parse Version in EEPROM data
 * @param ptr pointer to where data exists
 * @return status
 */
static int version(const char *index) {
  char str[6];
  uint8_t i = 0, j = 0;

  index = index + strlen("'Version':  ") - 1;

  while (i < 5) {
    if ((index[i] >= '0') && (index[i] <= '9')) {
      str[j++] = index[i];
    }
    i++;
  }
  str[j] = '\0';
  eeprom.bf_pltfm_version = atoi(str);

  LOG_DEBUG("Version: %d \n", eeprom.bf_pltfm_version);

  return 0;
}

/**
 * Parse product name in EEPROM data
 * @param ptr pointer to where data exists
 * @return status
 */
static int product_name(const char *index) {
  char str[BFN_EEPROM_F_PRODUCT_NAME + 1];
  uint8_t i = 0;

  index = index + strlen("Product Name': u'") - 1;

  while (i < BFN_EEPROM_F_PRODUCT_NAME) {
    if ((index[i] == '\'') || (index[i] == '\"')) break;

    str[i] = index[i];
    i++;
  }
  str[i] = '\0';
  strcpy(eeprom.bf_pltfm_product_name, str);

  LOG_DEBUG("Product Name: %s \n", eeprom.bf_pltfm_product_name);

  return 0;
}

/**
 * Parse product number in EEPROM data
 * @param ptr pointer to where data exists
 * @return status
 */
static int product_number(const char *index) {
  size_t len = 0;

  index = index + strlen("Product Part Number': u'") - 1;

  len = sizeof(eeprom.bf_pltfm_product_number) - 1;

  strncpy(eeprom.bf_pltfm_product_number, index, len);
  eeprom.bf_pltfm_product_number[len] = '\0';

  LOG_DEBUG("Product Number: %s \n", eeprom.bf_pltfm_product_number);

  return 0;
}

/**
 * Parse assembly at in EEPROM data
 * @param ptr pointer to where data exists
 * @return status
 */
static int assembly_at(const char *index) {
  size_t len = 0;

  index = index + strlen("Assembled At': u'") - 1;

  len = sizeof(eeprom.bf_pltfm_assembled) - 2;

  strncpy(eeprom.bf_pltfm_assembled, index, len);
  eeprom.bf_pltfm_assembled[len] = '\0';

  LOG_DEBUG("Assembled at: %s \n", eeprom.bf_pltfm_assembled);

  return 0;
}

/**
 * Parse board id in EEPROM data
 * @param ptr pointer to where data exists
 * @return status
 */
static int system_assembly_part_number(const char *index) {
  size_t len = 0;

  index = index + strlen("System Assembly Part Number': u'") - 1;

  len = sizeof(eeprom.bf_pltfm_assembly_number) - 2;

  strncpy(eeprom.bf_pltfm_assembly_number, index, len);
  eeprom.bf_pltfm_assembly_number[len] = '\0';

  if (board_id_set(eeprom.bf_pltfm_assembly_number)) {
    return -1;
  }

  return 0;
}

/**
 * Parse CRC8 in EEPROM data
 * @param ptr pointer to where data exists
 * @return status
 */
static int crc8(const char *index) {
  char str[5];
  uint8_t i = 0;

  index = index + strlen("CRC8': u'") - 1;

  while (i < 4) {
    if ((index[i] == '\'') || (index[i] == '\"')) break;

    str[i] = index[i];
    i++;
  }
  str[i] = '\0';
  eeprom.bf_pltfm_crc8 = (uint8_t)strtol(str, NULL, 0);

  LOG_DEBUG("CRC8: %d \n", eeprom.bf_pltfm_crc8);

  return 0;
}

/**
 * Parse Extended Mac addr size in EEPROM data
 * @param ptr pointer to where data exists
 * @return status
 */
static int ext_mac_size(const char *index) {
  char str[6];
  uint8_t i = 0, j = 0;

  index = index + strlen("Extended MAC Address Size':  ") - 1;

  while (i < 5) {
    if ((index[i] >= '0') && (index[i] <= '9')) {
      str[j++] = index[i];
    }
    i++;
  }
  str[j] = '\0';
  eeprom.bf_pltfm_mac_size = atoi(str);

  LOG_DEBUG("Extended MAC Address Size: %d \n", eeprom.bf_pltfm_mac_size);

  return 0;
}

/**
 * Parse Extended MAC Base address in EEPROM data
 * @param ptr pointer to where data exists
 * @return status
 */
static int ext_mac_base(const char *index) {
  uint8_t i = 0;
  char hex[3];

  index = index + strlen("Extended MAC Base': u'") - 1;

  for (; i < 6; i++) {
    hex[0] = index[0];
    hex[1] = index[1];
    hex[2] = '\0';
    index = index + 3;
    eeprom.bf_pltfm_mac_base[i] = (uint8_t)strtol(hex, NULL, 16);
  }

  LOG_DEBUG("Extended MAC Base %x:%x:%x:%x:%x:%x\n",
            eeprom.bf_pltfm_mac_base[0],
            eeprom.bf_pltfm_mac_base[1],
            eeprom.bf_pltfm_mac_base[2],
            eeprom.bf_pltfm_mac_base[3],
            eeprom.bf_pltfm_mac_base[4],
            eeprom.bf_pltfm_mac_base[5]);
  return 0;
}

/**
 * Parse BFN PCB Part Number in EEPROM data
 * @param ptr pointer to where data exists
 * @return status
 */
static int bfn_pcb_part_number(const char *index) {
  size_t len = 0;

  index = index + strlen("Facebook PCB Part Number': u'") - 1;

  len = sizeof(eeprom.bf_pltfm_pcb_number) - 2;

  strncpy(eeprom.bf_pltfm_pcb_number, index, len);
  eeprom.bf_pltfm_pcb_number[len] = '\0';

  LOG_DEBUG("BFN PCB Part Number %s \n", eeprom.bf_pltfm_pcb_number);

  return 0;
}

/**
 * Parse BFN PCBA Part Number in EEPROM data
 * @param ptr pointer to where data exists
 * @return status
 */
static int bfn_pcbA_part_number(const char *index) {
  size_t len = 0;

  index = index + strlen("Facebook PCBA Part Number': u'") - 1;

  len = sizeof(eeprom.bf_pltfm_pcba_number) - 2;

  strncpy(eeprom.bf_pltfm_pcba_number, index, len);
  eeprom.bf_pltfm_pcba_number[len] = '\0';

  LOG_DEBUG("BFN PCBA Part Number %s \n", eeprom.bf_pltfm_pcba_number);

  return 0;
}

/**
 * Parse Local MAC address in EEPROM data
 * @param ptr pointer to where data exists
 * @return status
 */
static int local_mac(const char *index) {
  uint8_t i = 0;
  char hex[3];

  index = index + strlen("Local MAC': u'") - 1;

  for (; i < 6; i++) {
    hex[0] = index[0];
    hex[1] = index[1];
    hex[2] = '\0';
    index = index + 3;
    eeprom.bf_pltfm_local_mac[i] = (uint8_t)strtol(hex, NULL, 16);
  }

  LOG_DEBUG("Local MAC %x:%x:%x:%x:%x:%x\n",
            eeprom.bf_pltfm_local_mac[0],
            eeprom.bf_pltfm_local_mac[1],
            eeprom.bf_pltfm_local_mac[2],
            eeprom.bf_pltfm_local_mac[3],
            eeprom.bf_pltfm_local_mac[4],
            eeprom.bf_pltfm_local_mac[5]);
  return 0;
}

/**
 * Parse Location on Fabric in EEPROM data
 * @param ptr pointer to where data exists
 * @return status
 */
static int location_fabric(const char *index) {
  char str[BFN_EEPROM_F_LOCATION + 1];
  uint8_t i = 0;

  index = index + strlen("Location on Fabric': u'") - 1;

  while (i < BFN_EEPROM_F_LOCATION) {
    if ((index[i] == '\'') || (index[i] == '\"')) break;

    str[i] = index[i];
    i++;
  }
  str[i] = '\0';
  strcpy(eeprom.bf_pltfm_location, str);

  LOG_DEBUG("Location on Fabric: %s\n", eeprom.bf_pltfm_location);

  return 0;
}

/**
 * Parse ODM PCBA Part Number in EEPROM data
 * @param ptr pointer to where data exists
 * @return status
 */
static int odm_pcba_part_number(const char *index) {
  size_t len = 0;

  index = index + strlen("ODM PCBA Part Number': u'") - 1;

  len = sizeof(eeprom.bf_pltfm_odm_pcba_number) - 1;

  strncpy(eeprom.bf_pltfm_odm_pcba_number, index, len);
  eeprom.bf_pltfm_odm_pcba_number[len] = '\0';

  LOG_DEBUG("ODM PCBA Part Number %s \n", eeprom.bf_pltfm_odm_pcba_number);

  return 0;
}

/**
 * Parse ODM PCBA Serial Number in EEPROM data
 * @param ptr pointer to where data exists
 * @return status
 */
static int odm_pcba_serial_number(const char *index) {
  size_t len = 0;

  index = index + strlen("ODM PCBA Serial Number': u'") - 1;

  len = sizeof(eeprom.bf_pltfm_odm_pcba_serial) - 3;

  strncpy(eeprom.bf_pltfm_odm_pcba_serial, index, len);
  eeprom.bf_pltfm_odm_pcba_serial[len] = '\0';

  LOG_DEBUG("ODM PCBA Serial Number %s \n", eeprom.bf_pltfm_odm_pcba_serial);

  return 0;
}

/**
 * Parse PCB Manufacturer in EEPROM data
 * @param ptr pointer to where data exists
 * @return status
 */
static int pcb_manufacturer(const char *index) {
  char str[BFN_EEPROM_F_PCB_MANUFACTURER + 1];
  uint8_t i = 0;

  index = index + strlen("PCB Manufacturer': u'") - 1;

  while (i < BFN_EEPROM_F_PCB_MANUFACTURER) {
    if ((index[i] == '\'') || (index[i] == '\"')) break;

    str[i] = index[i];
    i++;
  }
  str[i] = '\0';
  strcpy(eeprom.bf_pltfm_pcb_manufacturer, str);

  LOG_DEBUG("PCB Manufacturer: %s\n", eeprom.bf_pltfm_pcb_manufacturer);

  return 0;
}

/**
 * Parse Product Asset TAG in EEPROM data
 * @param ptr pointer to where data exists
 * @return status
 */
static int product_asset(const char *index) {
  char str[BFN_EEPROM_F_PRODUCT_ASSET + 1];
  uint8_t i = 0;

  index = index + strlen("Product Asset Tag': u'") - 1;

  while (i < BFN_EEPROM_F_PRODUCT_ASSET) {
    if ((index[i] == '\'') || (index[i] == '\"')) break;

    str[i] = index[i];
    i++;
  }
  str[i] = '\0';
  strcpy(eeprom.bf_pltfm_product_asset, str);

  LOG_DEBUG("Product Asset Tag: %s\n", eeprom.bf_pltfm_product_asset);

  return 0;
}

/**
 * Parse Production State in EEPROM data
 * @param ptr pointer to where data exists
 * @return status
 */
static int production_state(const char *index) {
  char str[4];
  uint8_t i = 0, j = 0;

  index = index + strlen("Product Production State':  ") - 1;
  while (i < 3) {
    if ((index[i] >= '0') && (index[i] <= '9')) {
      str[j++] = index[i];
    }
    i++;
  }
  str[j] = '\0';
  eeprom.bf_pltfm_production_state = atoi(str);

  LOG_DEBUG("Product Production State: %d \n",
            eeprom.bf_pltfm_production_state);

  return 0;
}

/**
 * Parse Product Serial number in EEPROM data
 * @param ptr pointer to where data exists
 * @return status
 */
static int product_serial_number(const char *index) {
  char str[BFN_EEPROM_F_PRODUCT_SERIAL + 1];
  uint8_t i = 0;

  index = index + strlen("Product Serial Number': u'") - 1;

  while (i < BFN_EEPROM_F_PRODUCT_SERIAL) {
    if ((index[i] == '\'') || (index[i] == '\"')) break;

    str[i] = index[i];
    i++;
  }
  str[i] = '\0';

  strcpy(eeprom.bf_pltfm_product_serial, str);

  LOG_DEBUG("Product Serial Number: %s \n", eeprom.bf_pltfm_product_serial);

  return 0;
}

/**
 * Parse Product Sub Version in EEPROM data
 * @param ptr pointer to where data exists
 * @return status
 */
static int product_subversion(const char *index) {
  char str[4];
  uint8_t i = 0, j = 0;

  index = index + strlen("Product Sub-Version':  ") - 1;

  while (i < 3) {
    if ((index[i] >= '0') && (index[i] <= '9')) {
      str[j++] = index[i];
    }
    i++;
  }
  str[j] = '\0';
  eeprom.bf_pltfm_product_subversion = atoi(str);

  LOG_DEBUG("Product Sub-Version: %d \n", eeprom.bf_pltfm_product_subversion);

  return 0;
}

/**
 * Parse Product Version in EEPROM data
 * @param ptr pointer to where data exists
 * @return status
 */
static int product_version(const char *index) {
  char str[4];
  uint8_t i = 0, j = 0;

  index = index + strlen("Product Version':  ") - 1;

  while (i < 3) {
    if ((index[i] >= '0') && (index[i] <= '9')) {
      str[j++] = index[i];
    }
    i++;
  }
  str[j] = '\0';
  eeprom.bf_pltfm_product_version = atoi(str);

  LOG_DEBUG("Product Version: %d \n", eeprom.bf_pltfm_product_version);

  return 0;
}

/**
 * Parse system manufacturer in EEPROM data
 * @param ptr pointer to where data exists
 * @return status
 */
static int system_manufacturer(const char *index) {
  size_t len = 0;

  index = index + strlen("System Manufacturer': u'") - 1;

  len = sizeof(eeprom.bf_pltfm_system_manufacturer) - 2;

  strncpy(eeprom.bf_pltfm_system_manufacturer, index, len);
  eeprom.bf_pltfm_system_manufacturer[len] = '\0';

  LOG_DEBUG("System Manufacturer: %s \n", eeprom.bf_pltfm_system_manufacturer);

  return 0;
}

/**
 * Parse system manufacturing date in EEPROM data
 * @param ptr pointer to where data exists
 * @return status
 */
static int system_manufacturing_date(const char *index) {
  size_t len = 0;

  index = index + strlen("System Manufacturing Date': u'") - 1;

  len = sizeof(eeprom.bf_pltfm_system_manufacturing_date) - 2;

  strncpy(eeprom.bf_pltfm_system_manufacturing_date, index, len);
  eeprom.bf_pltfm_system_manufacturing_date[len] = '\0';

  LOG_DEBUG("System Manufacturing Date: %s \n",
            eeprom.bf_pltfm_system_manufacturing_date);

  return 0;
}

/*
 * Example format of EEPROM JSON returned by REST API call
 *
 * u'Information': {u'Assembled At': u'JoyTech',
 *                  u'CRC8': u'0x0',
 *                  u'Extended MAC Address Size': 17751,
 *                  u'Extended MAC Base': u'8C:EA:1B:45:02:7C',
 *                  u'BFN PCB Part Number': u'142-000014-58',
 *                  u'BFN PCBA Part Number': u'142-000014-58',
 *                  u'Local MAC': u'8C:EA:1B:45:02:7B',
 *                  u'Location on Fabric': u'DGE100',
 *                  u'ODM PCBA Part Number': u'NP3zz7664001A',
 *                  u'ODM PCBA Serial Number': u'AG44022101',
 *                  u'PCB Manufacturer': u'ISU',
 *                  u'Product Asset Tag': u'44022101',
 *                  u'Product Name': u'Wedge100AC',
 *                  u'Product Part Number': u'20-001469',
 *                  u'Product Production State': 4,
 *                  u'Product Serial Number': u'AG44022101',
 *                  u'Product Sub-Version': 2,
 *                  u'Product Version': 1,
 *                  u'System Assembly Part Number': u'135-000011-02',
 *                  u'System Manufacturer': u'JoyTech',
 *                  u'System Manufacturing Date': u'11-18-16',
 *                  u'Version': 1},
 *
 */

/**
 * Parse EEPROM data received from OPENBMC
 * @param ptr pointer to json format data
 * @return status
 */
static int parse_eeprom_data(void *ptr) {
  const char *index = 0;
  const char *p = (const char *)ptr;

  if (ptr == NULL) {
    LOG_ERROR("Invalid pointer passed by curl callback");
    LOG_ERROR("EEPROM ERROR: Unknown Board type, Defaulting to Model");
    return -1;
  }

  LOG_DEBUG("EEPROM DATA FROM BMC \n %s", p);

  strncpy(eeprom_raw_data, (char *)ptr, EEPROM_RAW_DATA_SIZE - 1);
  eeprom_raw_data[EEPROM_RAW_DATA_SIZE - 1] = '\0';

  /* Assemble at */
  index = strstr(p, "Assembled At");
  if (index != NULL) {
    if (assembly_at(index)) {
      LOG_ERROR("FAILED to parse Assembled At\n");
    }
  } else {
    LOG_ERROR("FAILED to parse Assembled At\n");
  }

  /* CRC8 */
  index = strstr(p, "CRC8");
  if (index != NULL) {
    crc8(index);
  } else {
    LOG_ERROR("FAILED to parse CRC8\n");
  }

  /* Extended Mac address size */
  index = strstr(p, "Extended MAC Address Size");
  if (index != NULL) {
    if (ext_mac_size(index)) {
      LOG_ERROR("FAILED to parse Extended MAC Address Size\n");
    }
  } else {
    LOG_ERROR("FAILED to parse Extended MAC Address Size\n");
  }

  /* Extended Mac base address */
  index = strstr(p, "Extended MAC Base");
  if (index != NULL) {
    if (ext_mac_base(index)) {
      LOG_ERROR("FAILED to parse Extended MAC Base Address\n");
    }
  } else {
    LOG_ERROR("FAILED to parse Extended MAC Base Address\n");
  }

  /* BFN PCB Part Number*/
  index = strstr(p, "Facebook PCB Part Number");
  if (index != NULL) {
    bfn_pcb_part_number(index);
  } else {
    LOG_DEBUG("Unable to parse BFN PCB Part Number, Ignoring.\n");
  }

  /* BFN PCBA Part Number*/
  index = strstr(p, "Facebook PCBA Part Number");
  if (index != NULL) {
    bfn_pcbA_part_number(index);
  } else {
    LOG_DEBUG("Unable to parse BFN PCBA Part Number, Ignoring.\n");
  }

  /* Local Mac Address */
  index = strstr(p, "Local MAC");
  if (index != NULL) {
    local_mac(index);
  } else {
    LOG_ERROR("FAILED to parse Local MAC\n");
  }

  /* Location on Fabric */
  index = strstr(p, "Location on Fabric");
  if (index != NULL) {
    location_fabric(index);
  } else {
    LOG_ERROR("FAILED to parse Location on Fabric\n");
  }

  /* ODM PCBA Part Number*/
  index = strstr(p, "ODM PCBA Part Number");
  if (index != NULL) {
    odm_pcba_part_number(index);
  } else {
    LOG_ERROR("FAILED to parse ODM PCBA Part Number\n");
  }

  /* ODM PCBA Serial Number*/
  index = strstr(p, "ODM PCBA Serial Number");
  if (index != NULL) {
    odm_pcba_serial_number(index);
  } else {
    LOG_ERROR("FAILED to parse ODM PCBA Serial Number\n");
  }

  /* PCB Manufacturer */
  index = strstr(p, "PCB Manufacturer");
  if (index != NULL) {
    pcb_manufacturer(index);
  } else {
    LOG_ERROR("FAILED to parse PCB Manufacturer\n");
  }

  /* Product Asset tag */
  index = strstr(p, "Product Asset Tag");
  if (index != NULL) {
    product_asset(index);
  } else {
    LOG_ERROR("FAILED to parse Product Asset Tag\n");
  }

  /* Product Name */
  index = strstr(p, "Product Name");
  if (index != NULL) {
    product_name(index);
  } else {
    LOG_ERROR("FAILED to parse product name\n");
  }

  /* Product Part Number */
  index = strstr(p, "Product Part Number");
  if (index != NULL) {
    product_number(index);
  } else {
    LOG_ERROR("FAILED to parse product numbner\n");
  }

  /* Production State */
  index = strstr(p, "Product Production State");
  if (index != NULL) {
    production_state(index);
  } else {
    LOG_ERROR("FAILED to parse production state\n");
  }

  /* Product Serial Number */
  index = strstr(p, "Product Serial Number");
  if (index != NULL) {
    product_serial_number(index);
  } else {
    LOG_ERROR("FAILED to parse product serial number\n");
  }

  /* Product Sub-Version */
  index = strstr(p, "Product Sub-Version");
  if (index != NULL) {
    product_subversion(index);
  } else {
    LOG_ERROR("FAILED to parseproduct sub-version\n");
  }

  /* Product Version */
  index = strstr(p, "Product Version");
  if (index != NULL) {
    product_version(index);
  } else {
    LOG_ERROR("FAILED to parseproduct version\n");
  }

  /* Parse Board ID */
  index = strstr(p, "System Assembly Part Number");
  if (index != NULL) {
    if (system_assembly_part_number(index)) {
      LOG_ERROR("FAILED to parse BOARD ID\n");
    }
  } else {
    LOG_ERROR("FAILED to parse BOARD ID\n");
  }

  /* System Manufacturer */
  index = strstr(p, "System Manufacturer");
  if (index != NULL) {
    system_manufacturer(index);
  } else {
    LOG_ERROR("FAILED to parse system manufacturer\n");
  }

  /* System Manufacturing date */
  index = strstr(p, "System Manufacturing Date");
  if (index != NULL) {
    system_manufacturing_date(index);
  } else {
    LOG_ERROR("FAILED to parse system manufacturing date\n");
  }

  /* Version */
  index = strstr(p, "\"Version\":");
  if (index != NULL) {
    version(index);
  } else {
    LOG_ERROR("FAILED to parse version\n");
  }

  return strlen(p);
}

/**
 * Get EEPROM DATA from OPENBMC and populate eeprom struct
 * @return status
 */
bf_pltfm_status_t bf_pltfm_bd_type_init() {
  CURL *curl;
  CURLcode res;
  curl = curl_easy_init();
  char fruid_str[256];

  snprintf(fruid_str,
           sizeof(fruid_str),
           "http://[%s]:8080/api/sys/mb/fruid",
           bf_pltfm_bmc_server_addr_get());
  if (curl) {
    curl_easy_setopt(curl, CURLOPT_URL, fruid_str);
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, parse_eeprom_data);
#ifdef DEVICE_IS_ASIC
    int i = 0;
    while (i < 20) {
      res = curl_easy_perform(curl);
      if (res != CURLE_OK) {
        i++;
        bf_sys_sleep(1);
      } else {
        break;
      }
    }
    if (res != CURLE_OK) {
      LOG_ERROR("Error Getting the Board info from BMC. Hence exiting ****\n");
      exit(1);
    }
#endif
    /* always cleanup */
    curl_easy_cleanup(curl);
    (void)res;  // Keep the compiler happy
  } else {
    LOG_ERROR("CURL ERROR: Unable to use CURL\n");
    return BF_PLTFM_OBJECT_NOT_FOUND;
  }

  return BF_PLTFM_SUCCESS;
}

bf_pltfm_status_t bf_pltfm_bd_type_get(bf_pltfm_board_id_t *board_id) {
  *board_id = bd_id;
  return BF_PLTFM_SUCCESS;
}

bf_pltfm_status_t bf_pltfm_bd_eeprom_get(bf_pltfm_eeprom_t *ee_data) {
  memcpy(ee_data, &eeprom, sizeof(bf_pltfm_eeprom_t));
  return BF_PLTFM_SUCCESS;
}
