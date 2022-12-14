/*******************************************************************************
 * Copyright (c) 2015-2020 Barefoot Networks, Inc.
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * $Id: $
 *
 ******************************************************************************/
#ifndef _BF_PLTFM_CHSS_MGMT_H
#define _BF_PLTFM_CHSS_MGMT_H

#include <bf_pltfm_types/bf_pltfm_types.h>
#include <dvm/bf_drv_intf.h>
#include <port_mgr/bf_port_if.h>
#include <port_mgr/bf_serdes_if.h>
#include <port_mgr/port_mgr_intf.h>
#include <bfsys/bf_sal/bf_sys_intf.h>
#include <lld/lld_reg_if.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Initialize all the sub modules like EEPROM, Fan, etc.
 */
bf_pltfm_status_t bf_pltfm_chss_mgmt_init();

// Board EEPROM functions
#define BFN_EEPROM_F_MAGIC 2
#define BFN_EEPROM_F_VERSION 1
#define BFN_EEPROM_F_PRODUCT_NAME 12
#define BFN_EEPROM_F_PRODUCT_NUMBER 8
#define BFN_EEPROM_F_ASSEMBLY_NUMBER 12
#define BFN_EEPROM_F_BAREFOOT_PCBA_NUMBER 12
#define BFN_EEPROM_F_BAREFOOT_PCB_NUMBER 12
#define BFN_EEPROM_F_ODM_PCBA_NUMBER 13
#define BFN_EEPROM_F_ODM_PCBA_SERIAL 12
#define BFN_EEPROM_F_PRODUCT_STATE 1
#define BFN_EEPROM_F_PRODUCT_VERSION 1
#define BFN_EEPROM_F_PRODUCT_SUBVERSION 1
#define BFN_EEPROM_F_PRODUCT_SERIAL 12
#define BFN_EEPROM_F_PRODUCT_ASSET 12
#define BFN_EEPROM_F_SYSTEM_MANUFACTURER 8
#define BFN_EEPROM_F_SYSTEM_MANU_DATE 4
#define BFN_EEPROM_F_PCB_MANUFACTURER 8
#define BFN_EEPROM_F_ASSEMBLED 8
#define BFN_EEPROM_F_LOCAL_MAC 12
#define BFN_EEPROM_F_EXT_MAC_BASE 12
#define BFN_EEPROM_F_EXT_MAC_SIZE 2
#define BFN_EEPROM_F_LOCATION 8
#define BFN_EEPROM_F_CRC8 1

#define BF_PLTFM_SYS_MAC_ADDR_MAX 8

#define BF_SNSR_OUT_BUF_SIZE 4096

typedef struct bf_pltfm_eeprom_t {
  /* version number of the eeprom. Must be the first element */
  uint8_t bf_pltfm_version;

  /* Product Name */
  char bf_pltfm_product_name[BFN_EEPROM_F_PRODUCT_NAME + 1];

  /* Top Level 20 - Product Part Number: XX-XXXXXX */
  char bf_pltfm_product_number[BFN_EEPROM_F_PRODUCT_NUMBER + 2];

  /* System Assembly Part Number XXX-XXXXXX-XX */
  char bf_pltfm_assembly_number[BFN_EEPROM_F_ASSEMBLY_NUMBER + 3];

  /* BFN PCBA Part Number: XXX-XXXXXXX-XX */
  char bf_pltfm_pcba_number[BFN_EEPROM_F_BAREFOOT_PCBA_NUMBER + 3];

  /* BFN PCB Part Number: XXX-XXXXXXX-XX */
  char bf_pltfm_pcb_number[BFN_EEPROM_F_BAREFOOT_PCB_NUMBER + 3];

  /* ODM PCB Part Number: XXXXXXXXXXXX */
  char bf_pltfm_odm_pcba_number[BFN_EEPROM_F_ODM_PCBA_NUMBER + 1];

  /* ODM PCB Serial Number: XXXXXXXXXXXX */
  char bf_pltfm_odm_pcba_serial[BFN_EEPROM_F_ODM_PCBA_SERIAL + 1];

  /* Product Production State */
  uint8_t bf_pltfm_production_state;

  /* Product Version */
  uint8_t bf_pltfm_product_version;

  /* Product Sub Version */
  uint8_t bf_pltfm_product_subversion;

  /* Product Serial Number: XXXXXXXX */
  char bf_pltfm_product_serial[BFN_EEPROM_F_PRODUCT_SERIAL + 1];

  /* Product Asset Tag: XXXXXXXX */
  char bf_pltfm_product_asset[BFN_EEPROM_F_PRODUCT_ASSET + 1];

  /* System Manufacturer: XXXXXXXX */
  char bf_pltfm_system_manufacturer[BFN_EEPROM_F_SYSTEM_MANUFACTURER + 1];

  /* System Manufacturing Date: mm-dd-yy */
  char bf_pltfm_system_manufacturing_date[10];

  /* PCB Manufacturer: XXXXXXXXX */
  char bf_pltfm_pcb_manufacturer[BFN_EEPROM_F_PCB_MANUFACTURER + 1];

  /* Assembled At: XXXXXXXX */
  char bf_pltfm_assembled[BFN_EEPROM_F_ASSEMBLED + 1];

  /* Local MAC Address */
  uint8_t bf_pltfm_local_mac[6];

  /* Extended MAC Address */
  uint8_t bf_pltfm_mac_base[6];

  /* Extended MAC Address Size */
  uint16_t bf_pltfm_mac_size;

  /* Location on Fabric: "MAVERICKS", "MONTARA" */
  char bf_pltfm_location[BFN_EEPROM_F_LOCATION + 1];

  /* CRC8 */
  uint8_t bf_pltfm_crc8;
} bf_pltfm_eeprom_t;

bf_pltfm_status_t bf_pltfm_chss_mgmt_bd_type_get(bf_pltfm_board_id_t *board_id);
bf_pltfm_status_t bf_pltfm_bd_eeprom_get(bf_pltfm_eeprom_t *ee_data);

/*
 * Power Supply unit
 */

typedef enum bf_pltfm_pwr_supply_e {
  POWER_SUPPLY1 = 1,
  POWER_SUPPLY2 = 2
} bf_pltfm_pwr_supply_t;

typedef struct bf_pltfm_pwr_supply_info_t {
  uint32_t vin;      /* Input voltage in Volts */
  uint32_t vout;     /* Output voltage in Volts */
  uint32_t iout;     /* Output current in milli Amperes */
  uint32_t pwr_out;  /* Output power in milli watts */
  uint32_t fspeed;   /* Fan speed in RPM */
  bool ffault;       /* Fan fault TRUE/FALSE */
  bool presence;     /* Power supply present or not */
  bool load_sharing; /* load sharing TRUE/FALSE */
  char model[32];    /* Model number */
  char serial[32];   /* Serial number */
  char rev[32];      /* Revision number */
} bf_pltfm_pwr_supply_info_t;

bf_pltfm_status_t bf_pltfm_chss_mgmt_pwr_supply_get(
    bf_pltfm_pwr_supply_t pwr, bf_pltfm_pwr_supply_info_t *info);

bf_pltfm_status_t bf_pltfm_chss_mgmt_pwr_supply_prsnc_get(
    bf_pltfm_pwr_supply_t pwr, bool *info);

/* Temperature sensors */
typedef struct bf_pltfm_temperature_info_t {
  float tmp1;  /* tempearture of sensor 1 in C */
  float tmp2;  /* tempearture of sensor 2 in C */
  float tmp3;  /* tempearture of sensor 3 in C */
  float tmp4;  /* tempearture of sensor 4 in C */
  float tmp5;  /* tempearture of sensor 5 in C */
  float tmp6;  /* tempearture of sensor 6 in C */
  float tmp7;  /* tempearture of sensor 7 in C */
  float tmp8;  /* tempearture of sensor 8 in C */
  float tmp9;  /* tempearture of sensor 9 in C */
  float tmp10; /* tempearture of sensor 10 in C */
} bf_pltfm_temperature_info_t;

/* Switch temperature sensors */
typedef struct bf_pltfm_switch_temperature_info_t {
  uint32_t remote_sensor;
  uint32_t main_sensor;
} bf_pltfm_switch_temperature_info_t;

bf_pltfm_status_t bf_pltfm_chss_mgmt_temperature_get(
    bf_pltfm_temperature_info_t *tmp);

bf_pltfm_status_t bf_pltfm_chss_mgmt_switch_temperature_get(
    bf_dev_id_t dev_id, int sensor, bf_pltfm_switch_temperature_info_t *tmp);

/* Power Rails */
typedef struct bf_pltfm_pwr_rails_info_t {
  uint32_t vrail1;  /* Voltage of rail 1 in mV */
  uint32_t vrail2;  /* Voltage of rail 2 in mV */
  uint32_t vrail3;  /* Voltage of rail 3 in mV */
  uint32_t vrail4;  /* Voltage of rail 4 in mV */
  uint32_t vrail5;  /* Voltage of rail 5 in mV */
  uint32_t vrail6;  /* Voltage of rail 6 in mV */
  uint32_t vrail7;  /* Voltage of rail 7 in mV */
  uint32_t vrail8;  /* Voltage of rail 8 in mV */
  uint32_t vrail9;  /* Voltage of rail 9 in mV */
  uint32_t vrail10; /* Voltage of rail 10 in mV */
  uint32_t vrail11; /* Voltage of rail 11 in mV */
  uint32_t vrail12; /* Voltage of rail 12 in mV */
  uint32_t vrail13; /* Voltage of rail 13 in mV */
  uint32_t vrail14; /* Voltage of rail 14 in mV */
  uint32_t vrail15; /* Voltage of rail 15 in mV */
  uint32_t vrail16; /* Voltage of rail 16 in mV */
} bf_pltfm_pwr_rails_info_t;

bf_pltfm_status_t bf_pltfm_chss_mgmt_pwr_rails_get(
    bf_pltfm_pwr_rails_info_t *pwr_rails);

/* Fan tray presence */

typedef struct bf_pltfm_fan_info_t {
  uint32_t fan_num;     /* Fan number */
  uint32_t front_speed; /* Front fan speed */
  uint32_t rear_speed;  /* Rear fan speed */
  uint32_t percent;     /* Percentage of Max speed
                         * at which both fans are
                         * running */
} bf_pltfm_fan_info_t;

typedef struct bf_pltfm_fan_data_t {
  bf_pltfm_fan_info_t F[10]; /* 10 is Max number of fans*/
  uint8_t fantray_present;   /* Fan tray presence */
} bf_pltfm_fan_data_t;

bf_pltfm_status_t bf_pltfm_chss_mgmt_fan_data_get(bf_pltfm_fan_data_t *fdata);

bf_pltfm_status_t bf_pltfm_chss_mgmt_fan_speed_set(bf_pltfm_fan_info_t *fdata);

bf_pltfm_status_t bf_pltfm_chss_mgmt_sys_mac_addr_get(
    uint8_t *mac_info, uint8_t *num_sys_addresses);

bf_pltfm_status_t bf_pltfm_chss_mgmt_port_mac_addr_get(
    bf_pltfm_port_info_t *port_info, uint8_t *mac_info);

bf_pltfm_status_t bf_pltfm_test_core_set(void);

const char *bf_pltfm_bmc_server_addr_get(void);

const char *bf_pltfm_bmc_server_port_get(void);

bf_pltfm_status_t pltfm_mgr_sensor_out_get(const char *options,
                                           char *info,
                                           size_t info_size);

bf_pltfm_status_t bf_pltfm_temperature_get(
    bf_pltfm_temperature_info_t *tmp, bf_pltfm_temperature_info_t *peak_tmp);

#ifdef INC_PLTFM_UCLI
ucli_node_t *bf_pltfm_chss_mgmt_ucli_node_create(ucli_node_t *m);
#endif

#ifdef __cplusplus
}
#endif /* C++ */

#endif
