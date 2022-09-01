/*******************************************************************************
 * Copyright (c) 2015-2020 Barefoot Networks, Inc.
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * $Id: $
 *
 ******************************************************************************/
/*!
 * @file bf_pltfm_chss_mgmt_ucli.c
 * @date
 *
 * Unit testing cli for chss_mgmt
 *
 */

/* Standard includes */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include <signal.h>

/* Module includes */
#include <dvm/bf_dma_types.h>
#include <dvm/bf_drv_intf.h>
#include <bfutils/uCli/ucli.h>
#include <bfutils/uCli/ucli_argparse.h>
#include <bfutils/uCli/ucli_handler_macros.h>
#include <bf_pltfm_types/bf_pltfm_types.h>
#include <bf_pltfm_chss_mgmt_intf.h>
#include <bf_pltfm_bmc_tty.h>

/* Local includes */
#include "bf_pltfm_bd_eeprom.h"

static ucli_status_t bf_pltfm_rptr_ucli_ucli__chss_mgmt_ps_show__(
    ucli_context_t *uc) {
  UCLI_COMMAND_INFO(
      uc, "ps_show", 1, " Show <power supply number (1/2)> status");
  bf_pltfm_pwr_supply_t pwr;
  bf_pltfm_pwr_supply_info_t info;

  pwr = atoi(uc->pargs->args[0]);

  if (pwr != POWER_SUPPLY1 && pwr != POWER_SUPPLY2) {
    aim_printf(&uc->pvs,
               "Invalid power supply number passed. It should be 1 or 2 \n");
    return 0;
  }

  if (bf_pltfm_chss_mgmt_pwr_supply_get(pwr, &info) != BF_PLTFM_SUCCESS) {
    aim_printf(&uc->pvs, "Error in reading power supply status\n");
    return 0;
  }

  aim_printf(&uc->pvs, "Vin             %d V\n", info.vin);
  aim_printf(&uc->pvs, "Vout            %d V\n", info.vout);
  aim_printf(&uc->pvs, "Iout            %d mA\n", info.iout);
  aim_printf(&uc->pvs, "Power output    %d mW\n", info.pwr_out);
  aim_printf(&uc->pvs, "Fan speed       %d RPM\n", info.fspeed);
  aim_printf(
      &uc->pvs, "Fan Fault       %s \n", (info.ffault ? "true" : "false"));
  aim_printf(
      &uc->pvs, "Presence        %s \n", (info.presence ? "true" : "false"));
  aim_printf(&uc->pvs,
             "Load Sharing    %s \n",
             (info.load_sharing ? "true" : "false"));

  return 0;
}
static ucli_status_t bf_pltfm_rptr_ucli_ucli__chss_mgmt_vrail_show__(
    ucli_context_t *uc) {
  UCLI_COMMAND_INFO(uc, "vrail_show", 0, " Show all voltage rails reading");

  bf_pltfm_pwr_rails_info_t t;
  bf_pltfm_board_id_t board_id;
  bf_pltfm_status_t r;

  r = bf_pltfm_chss_mgmt_bd_type_get(&board_id);

  if (r != BF_PLTFM_SUCCESS) {
    LOG_ERROR("Board ID retrival failed \n");
    return BF_PLTFM_COMM_FAILED;
  }

  if (bf_pltfm_chss_mgmt_pwr_rails_get(&t) != BF_PLTFM_SUCCESS) {
    aim_printf(&uc->pvs, "Error in reading rails voltages\n");
    return 0;
  }

  if ((board_id == BF_PLTFM_BD_ID_MAVERICKS_P0A) ||
      (board_id == BF_PLTFM_BD_ID_MAVERICKS_P0B)) {
    aim_printf(&uc->pvs, "VDD12V         %d mV\n", t.vrail1);
    aim_printf(&uc->pvs, "VDD5V_IR       %d mV\n", t.vrail2);
    aim_printf(&uc->pvs, "VDD5V_stby     %d mV\n", t.vrail3);
    aim_printf(&uc->pvs, "VDD3_3V_iso    %d mV\n", t.vrail4);
    aim_printf(&uc->pvs, "VDD3_3V_stby   %d mV\n", t.vrail5);
    aim_printf(&uc->pvs, "VDD3_3V_lower  %d mV\n", t.vrail6);
    aim_printf(&uc->pvs, "VDD3_3V_uppoer %d mV\n", t.vrail7);
    aim_printf(&uc->pvs, "VDD2_5V_stby   %d mV\n", t.vrail8);
    aim_printf(&uc->pvs, "VDD1_8V_rptr   %d mV\n", t.vrail9);
    aim_printf(&uc->pvs, "VDD2_5V_tf     %d mV\n", t.vrail10);
    aim_printf(&uc->pvs, "VDD1_8V_stby   %d mV\n", t.vrail11);
    aim_printf(&uc->pvs, "VDD1_5V_stby   %d mV\n", t.vrail12);
    aim_printf(&uc->pvs, "VDD1_2V_stby   %d mV\n", t.vrail13);
    aim_printf(&uc->pvs, "VDD0_9V_anlg   %d mV\n", t.vrail14);
    aim_printf(&uc->pvs, "VDD_core       %d mV\n", t.vrail15);
  }

  if (board_id == BF_PLTFM_BD_ID_MAVERICKS_P0C) {
    aim_printf(&uc->pvs, "VDD12V         %d mV\n", t.vrail1);
    aim_printf(&uc->pvs, "VDD5V_IR       %d mV\n", t.vrail2);
    aim_printf(&uc->pvs, "VDD5V_stby     %d mV\n", t.vrail3);
    aim_printf(&uc->pvs, "VDD3_3V_iso    %d mV\n", t.vrail4);
    aim_printf(&uc->pvs, "VDD3_3V_stby   %d mV\n", t.vrail5);
    aim_printf(&uc->pvs, "VDD3_3V_lower  %d mV\n", t.vrail6);
    aim_printf(&uc->pvs, "VDD3_3V_uppoer %d mV\n", t.vrail7);
    aim_printf(&uc->pvs, "VDD2_5V_stby   %d mV\n", t.vrail8);
    aim_printf(&uc->pvs, "VDD1_8V_rt     %d mV\n", t.vrail9);
    aim_printf(&uc->pvs, "VDD2_5V_tf     %d mV\n", t.vrail10);
    aim_printf(&uc->pvs, "VDD1_8V_stby   %d mV\n", t.vrail11);
    aim_printf(&uc->pvs, "VDD1_5V_stby   %d mV\n", t.vrail12);
    aim_printf(&uc->pvs, "VDD1_2V_stby   %d mV\n", t.vrail13);
    aim_printf(&uc->pvs, "VDD0_9V_anlg   %d mV\n", t.vrail14);
    aim_printf(&uc->pvs, "VDD_core       %d mV\n", t.vrail15);
    aim_printf(&uc->pvs, "VDD1_0V_rt     %d mV\n", t.vrail16);
  }
  if ((board_id == BF_PLTFM_BD_ID_MONTARA_P0A) ||
      (board_id == BF_PLTFM_BD_ID_MONTARA_P0C) ||
      (board_id == BF_PLTFM_BD_ID_MONTARA_P0B)) {
    aim_printf(&uc->pvs, "VDD12V       %d mV\n", t.vrail1);
    aim_printf(&uc->pvs, "VDD5V_stby   %d mV\n", t.vrail2);
    aim_printf(&uc->pvs, "VDD3_3V_iso  %d mV\n", t.vrail3);
    aim_printf(&uc->pvs, "VDD3_3V      %d mV\n", t.vrail4);
    aim_printf(&uc->pvs, "VDD3_3V_stby %d mV\n", t.vrail5);
    aim_printf(&uc->pvs, "VDD2_5V_stby %d mV\n", t.vrail6);
    aim_printf(&uc->pvs, "VDD2_5V_tf   %d mV\n", t.vrail7);
    aim_printf(&uc->pvs, "VDD1_8V_stby %d mV\n", t.vrail8);
    aim_printf(&uc->pvs, "VDD1_5V_stby %d mV\n", t.vrail9);
    aim_printf(&uc->pvs, "VDD1_2V_stby %d mV\n", t.vrail10);
    aim_printf(&uc->pvs, "VDD0_9V_anlg %d mV\n", t.vrail11);
    aim_printf(&uc->pvs, "VDD_core     %d mV\n", t.vrail12);
  }
  if ((board_id == BF_PLTFM_BD_ID_NEWPORT_P0A) ||
      (board_id == BF_PLTFM_BD_ID_NEWPORT_P0B)) {
    aim_printf(&uc->pvs, "VDD12V         %d mV\n", t.vrail1);
    aim_printf(&uc->pvs, "VDD0_75V       %d mV\n", t.vrail2);
    aim_printf(&uc->pvs, "VDD5V_IR       %d mV\n", t.vrail3);
    aim_printf(&uc->pvs, "VDD5V_stby     %d mV\n", t.vrail4);
    aim_printf(&uc->pvs, "VDD3_3V        %d mV\n", t.vrail5);
    aim_printf(&uc->pvs, "VDD3_3V_iso    %d mV\n", t.vrail6);
    aim_printf(&uc->pvs, "VDD3_3V_stby   %d mV\n", t.vrail7);
    aim_printf(&uc->pvs, "VDD2_5V_stby   %d mV\n", t.vrail8);
    aim_printf(&uc->pvs, "VDD1_8V        %d mV\n", t.vrail9);
    aim_printf(&uc->pvs, "VDDA1_8V       %d mV\n", t.vrail10);
    aim_printf(&uc->pvs, "VDD1_8V_stby   %d mV\n", t.vrail11);
    aim_printf(&uc->pvs, "VDD1_5V_stby   %d mV\n", t.vrail12);
    aim_printf(&uc->pvs, "VDD1_2V        %d mV\n", t.vrail13);
    aim_printf(&uc->pvs, "VDD1_2V_stby   %d mV\n", t.vrail14);
    aim_printf(&uc->pvs, "VDD1_0V        %d mV\n", t.vrail15);
    aim_printf(&uc->pvs, "VDD_core       %d mV\n", t.vrail16);
  }
  return 0;
}

static ucli_status_t bf_pltfm_rptr_ucli_ucli__chss_mgmt_tmp_show__(
    ucli_context_t *uc) {
  UCLI_COMMAND_INFO(uc, "tmp_show", 0, " Show all temperature sensors reading");

  bf_pltfm_temperature_info_t t;
  bf_pltfm_board_id_t board_id;
  bf_pltfm_status_t r;

  r = bf_pltfm_chss_mgmt_bd_type_get(&board_id);

  if (r != BF_PLTFM_SUCCESS) {
    LOG_ERROR("Board ID retrival failed \n");
    return BF_PLTFM_COMM_FAILED;
  }

  if (bf_pltfm_chss_mgmt_temperature_get(&t) != BF_PLTFM_SUCCESS) {
    aim_printf(&uc->pvs, "Error in reading temperature \n");
    return 0;
  }

  aim_printf(&uc->pvs, "tmp1    %f C\n", t.tmp1);
  aim_printf(&uc->pvs, "tmp2    %f C\n", t.tmp2);
  aim_printf(&uc->pvs, "tmp3    %f C\n", t.tmp3);
  aim_printf(&uc->pvs, "tmp4    %f C\n", t.tmp4);
  aim_printf(&uc->pvs, "tmp5    %f C\n", t.tmp5);
  if ((board_id == BF_PLTFM_BD_ID_MAVERICKS_P0A) ||
      (board_id == BF_PLTFM_BD_ID_MAVERICKS_P0B) ||
      (board_id == BF_PLTFM_BD_ID_MAVERICKS_P0C)) {
    aim_printf(&uc->pvs, "tmp6    %f C\n", t.tmp6);
    aim_printf(&uc->pvs, "tmp7    %f C\n", t.tmp7);
    aim_printf(&uc->pvs, "tmp8    %f C\n", t.tmp8);
    aim_printf(&uc->pvs, "tmp9    %f C\n", t.tmp9);
    if (t.tmp10 <= 0) {
      aim_printf(&uc->pvs, "tmp10   Error (check BMC firmware version)\n");
    } else {
      aim_printf(&uc->pvs, "tmp10   %f C\n", t.tmp10);
    }
  } else if ((board_id == BF_PLTFM_BD_ID_MONTARA_P0A) ||
             (board_id == BF_PLTFM_BD_ID_MONTARA_P0B) ||
             (board_id == BF_PLTFM_BD_ID_MONTARA_P0C) ||
             (board_id == BF_PLTFM_BD_ID_NEWPORT_P0B) ||
             (board_id == BF_PLTFM_BD_ID_NEWPORT_P0A)) {
    if (t.tmp6 <= 0) {
      aim_printf(&uc->pvs, "tmp10   Error (check BMC firmware version)\n");
    } else {
      aim_printf(&uc->pvs, "tmp10   %f C\n", t.tmp6);
    }
  }

  return 0;
}

static ucli_status_t bf_pltfm_rptr_ucli_ucli__chss_mgmt_fan_show__(
    ucli_context_t *uc) {
  UCLI_COMMAND_INFO(uc, "fan_show", 0, " Show all fan speed data");
  int i;
  bf_pltfm_fan_data_t fdata;
  uint8_t max_fan = 5;
  bf_pltfm_board_id_t board_id;
  bf_pltfm_status_t r;

  r = bf_pltfm_chss_mgmt_bd_type_get(&board_id);

  if (r != BF_PLTFM_SUCCESS) {
    LOG_ERROR("Board ID retrival failed \n");
    return BF_PLTFM_COMM_FAILED;
  }

  if (bf_pltfm_chss_mgmt_fan_data_get(&fdata) != BF_PLTFM_SUCCESS) {
    aim_printf(&uc->pvs, "Error in reading temperature \n");
    return 0;
  }

  if (!fdata.fantray_present) {
    aim_printf(&uc->pvs, "Fan tray present \n");
  } else {
    aim_printf(&uc->pvs, "Fan tray not present \n");
    return 0;
  }

  if ((board_id == BF_PLTFM_BD_ID_MAVERICKS_P0A) ||
      (board_id == BF_PLTFM_BD_ID_MAVERICKS_P0B) ||
      (board_id == BF_PLTFM_BD_ID_MAVERICKS_P0C)) {
    max_fan = 10;
  } else if ((board_id == BF_PLTFM_BD_ID_MONTARA_P0A) ||
             (board_id == BF_PLTFM_BD_ID_MONTARA_P0C) ||
             (board_id == BF_PLTFM_BD_ID_MONTARA_P0B)) {
    max_fan = 5;
  } else if ((board_id == BF_PLTFM_BD_ID_NEWPORT_P0A) ||
             (board_id == BF_PLTFM_BD_ID_NEWPORT_P0B)) {
    max_fan = 6;
  } else {
    max_fan = 5;
  }

  aim_printf(&uc->pvs, "FAN  FRONT RPM  REAR RPM  MAX SPEED%%  \n");

  for (i = 0; i < max_fan; i++) {
    aim_printf(&uc->pvs,
               " %d       %d       %d        %d%% \n",
               fdata.F[i].fan_num,
               fdata.F[i].front_speed,
               fdata.F[i].rear_speed,
               fdata.F[i].percent);
  }

  return 0;
}

static ucli_status_t bf_pltfm_rptr_ucli_ucli__chss_mgmt_fan_speed_set__(
    ucli_context_t *uc) {
  UCLI_COMMAND_INFO(uc,
                    "fan_speed_set",
                    2,
                    " Set <Fan Num> speed in <%(0...100)> of max speed");
  bf_pltfm_fan_info_t fdata;
  uint8_t max_fan;
  bf_pltfm_board_id_t board_id;
  bf_pltfm_status_t r;

  r = bf_pltfm_chss_mgmt_bd_type_get(&board_id);

  if (r != BF_PLTFM_SUCCESS) {
    LOG_ERROR("Board ID retrival failed \n");
    return BF_PLTFM_COMM_FAILED;
  }

  if ((board_id == BF_PLTFM_BD_ID_MAVERICKS_P0A) ||
      (board_id == BF_PLTFM_BD_ID_MAVERICKS_P0B) ||
      (board_id == BF_PLTFM_BD_ID_MAVERICKS_P0C)) {
    max_fan = 10;
  } else if ((board_id == BF_PLTFM_BD_ID_MONTARA_P0A) ||
             (board_id == BF_PLTFM_BD_ID_MONTARA_P0C) ||
             (board_id == BF_PLTFM_BD_ID_MONTARA_P0B)) {
    max_fan = 5;
  } else if ((board_id == BF_PLTFM_BD_ID_NEWPORT_P0A) ||
             (board_id == BF_PLTFM_BD_ID_NEWPORT_P0B)) {
    max_fan = 6;
  } else {
    max_fan = 5;
  }

  fdata.fan_num = atoi(uc->pargs->args[0]);
  fdata.percent = atoi(uc->pargs->args[1]);
  fdata.front_speed = 0;
  fdata.rear_speed = 0;

  if (fdata.fan_num > max_fan) {
    aim_printf(&uc->pvs, "Invalid fan number \n");
    return 0;
  }

  if (bf_pltfm_chss_mgmt_fan_speed_set(&fdata) != BF_PLTFM_SUCCESS) {
    aim_printf(&uc->pvs, "Error in setting fan speed \n");
    return 0;
  }

  aim_printf(&uc->pvs, "Fan speed set successfull\n");
  return 0;
}

static ucli_status_t bf_pltfm_rptr_ucli_ucli__chss_mgmt_sys_mac_get__(
    ucli_context_t *uc) {
  UCLI_COMMAND_INFO(uc,
                    "sys_mac_get",
                    0,
                    "System Mac address and number of extended address");

  uint8_t mac_info[6] = {0};
  uint8_t num_sys_port = 0;

  if (bf_pltfm_chss_mgmt_sys_mac_addr_get(mac_info, &num_sys_port) !=
      BF_PLTFM_SUCCESS) {
    aim_printf(&uc->pvs, "Error in getting system mac addr \n");
    return 0;
  }

  aim_printf(&uc->pvs,
             "System Mac addr: %02x:%02x:%02x:%02x:%02x:%02x \n",
             mac_info[0],
             mac_info[1],
             mac_info[2],
             mac_info[3],
             mac_info[4],
             mac_info[5]);
  aim_printf(&uc->pvs, "Number of extended addr available %d\n", num_sys_port);

  return 0;
}

static ucli_status_t bf_pltfm_rptr_ucli_ucli__chss_mgmt_port_mac_get__(
    ucli_context_t *uc) {
  UCLI_COMMAND_INFO(
      uc, "port_mac_get", 2, "mac address of <conn_id> <channel>");

  uint8_t mac_info[6] = {0};
  bf_pltfm_port_info_t port;

  port.conn_id = atoi(uc->pargs->args[0]);
  port.chnl_id = atoi(uc->pargs->args[1]);

  if (bf_pltfm_chss_mgmt_port_mac_addr_get(&port, mac_info) !=
      BF_PLTFM_SUCCESS) {
    aim_printf(&uc->pvs, "Error in getting port mac addr \n");
    return 0;
  }

  aim_printf(&uc->pvs, "Port/channel:%d/%d  ", port.conn_id, port.chnl_id);
  aim_printf(&uc->pvs,
             "Port Mac addr: %02x:%02x:%02x:%02x:%02x:%02x \n",
             mac_info[0],
             mac_info[1],
             mac_info[2],
             mac_info[3],
             mac_info[4],
             mac_info[5]);
  return 0;
}

static ucli_status_t bf_pltfm_rptr_ucli_ucli__chss_mgmt_eeprom_data_get__(
    ucli_context_t *uc) {
  UCLI_COMMAND_INFO(uc, "eeprom_data", 0, "EEPROM parsed data");

  aim_printf(&uc->pvs, "Version: %d \n", eeprom.bf_pltfm_version);
  aim_printf(&uc->pvs, "Product Name: %s \n", eeprom.bf_pltfm_product_name);
  aim_printf(&uc->pvs, "Product Number: %s \n", eeprom.bf_pltfm_product_number);
  aim_printf(&uc->pvs, "Assembled at: %s \n", eeprom.bf_pltfm_assembled);
  aim_printf(&uc->pvs, "CRC8: 0x%x \n", eeprom.bf_pltfm_crc8);
  aim_printf(
      &uc->pvs, "Extended MAC Address Size: %d \n", eeprom.bf_pltfm_mac_size);
  aim_printf(&uc->pvs,
             "Extended MAC Base %x:%x:%x:%x:%x:%x\n",
             eeprom.bf_pltfm_mac_base[0],
             eeprom.bf_pltfm_mac_base[1],
             eeprom.bf_pltfm_mac_base[2],
             eeprom.bf_pltfm_mac_base[3],
             eeprom.bf_pltfm_mac_base[4],
             eeprom.bf_pltfm_mac_base[5]);
  aim_printf(&uc->pvs, "BFN PCB Part Number %s \n", eeprom.bf_pltfm_pcb_number);
  aim_printf(
      &uc->pvs, "BFN PCBA Part Number %s \n", eeprom.bf_pltfm_pcba_number);
  aim_printf(&uc->pvs,
             "Local MAC %x:%x:%x:%x:%x:%x\n",
             eeprom.bf_pltfm_local_mac[0],
             eeprom.bf_pltfm_local_mac[1],
             eeprom.bf_pltfm_local_mac[2],
             eeprom.bf_pltfm_local_mac[3],
             eeprom.bf_pltfm_local_mac[4],
             eeprom.bf_pltfm_local_mac[5]);
  aim_printf(&uc->pvs, "Location on Fabric: %s\n", eeprom.bf_pltfm_location);
  aim_printf(
      &uc->pvs, "ODM PCBA Part Number %s \n", eeprom.bf_pltfm_odm_pcba_number);
  aim_printf(&uc->pvs,
             "ODM PCBA Serial Number %s \n",
             eeprom.bf_pltfm_odm_pcba_serial);
  aim_printf(
      &uc->pvs, "PCB Manufacturer: %s\n", eeprom.bf_pltfm_pcb_manufacturer);
  aim_printf(
      &uc->pvs, "Product Asset Tag: %s\n", eeprom.bf_pltfm_product_asset);
  aim_printf(&uc->pvs,
             "Product Production State: %d \n",
             eeprom.bf_pltfm_production_state);
  aim_printf(
      &uc->pvs, "Product Serial Number: %s \n", eeprom.bf_pltfm_product_serial);
  aim_printf(&uc->pvs,
             "Product Sub-Version: %d \n",
             eeprom.bf_pltfm_product_subversion);
  aim_printf(
      &uc->pvs, "Product Version: %d \n", eeprom.bf_pltfm_product_version);
  aim_printf(&uc->pvs,
             "System Manufacturer: %s \n",
             eeprom.bf_pltfm_system_manufacturer);
  aim_printf(&uc->pvs,
             "System Manufacturing Date: %s \n",
             eeprom.bf_pltfm_system_manufacturing_date);
  aim_printf(&uc->pvs,
             "System Assembly Part Number: %s \n",
             eeprom.bf_pltfm_assembly_number);

  return 0;
}

static ucli_status_t bf_pltfm_rptr_ucli_ucli__chss_mgmt_tofino_tmp_show__(
    ucli_context_t *uc) {
  UCLI_COMMAND_INFO(uc, "tofino_tmp_show", 0, " Show tofino temperature");

  bf_dev_id_t dev_id = 0;
  int sensor = 0;
  bf_pltfm_switch_temperature_info_t temp_mC;
  bf_pltfm_board_id_t board_id;
  bf_pltfm_status_t r;

  r = bf_pltfm_chss_mgmt_bd_type_get(&board_id);

  if (r != BF_PLTFM_SUCCESS) {
    LOG_ERROR("Board ID retrival failed \n");
    return BF_PLTFM_COMM_FAILED;
  }

  if ((board_id == BF_PLTFM_BD_ID_NEWPORT_P0A) ||
      (board_id == BF_PLTFM_BD_ID_MAVERICKS_P0B_EMU)) {
    LOG_ERROR("info not available for this board\n");
    return BF_PLTFM_COMM_FAILED;
  }

  r = bf_pltfm_chss_mgmt_switch_temperature_get(dev_id, sensor, &temp_mC);

  if (r != BF_PLTFM_SUCCESS) {
    aim_printf(&uc->pvs, "Error in retrieving tofino temperature\n");
    return 0;
  }

  aim_printf(&uc->pvs,
             "TOFINO MAIN TEMP SENSOR: %d C\n",
             (temp_mC.main_sensor / 1000));

  aim_printf(&uc->pvs,
             "TOFINO REMOTE TEMP SENSOR %d C\n",
             (temp_mC.remote_sensor / 1000));

  return 0;
}

static ucli_status_t bf_pltfm_ucli_ucli__chss_mgmt_bmc_cmd_(
    ucli_context_t *uc) {
  UCLI_COMMAND_INFO(
      uc, "bmc_cmd", 2, "Execute BMC command <timeout in sec> <\"bmc cmd\">");

  char cmd_out[1024];
  char cmd_in[128];
  int ret;
  unsigned long timeout;

  timeout = atoi(uc->pargs->args[0]);
  snprintf(cmd_in, sizeof(cmd_in), "%s\r\n", uc->pargs->args[1]);
  cmd_in[sizeof(cmd_in) - 1] = '\0';
  ret = bmc_send_command_with_output(timeout, cmd_in, cmd_out, sizeof(cmd_out));

  if (ret != 0) {
    aim_printf(
        &uc->pvs,
        "Error in executing command \"%s\", displaying output data anyway\n",
        uc->pargs->args[0]);
  }

  aim_printf(&uc->pvs, "\n%s\n", cmd_out);

  return ret;
}

static ucli_command_handler_f bf_pltfm_chss_mgmt_ucli_ucli_handlers__[] = {
    bf_pltfm_rptr_ucli_ucli__chss_mgmt_sys_mac_get__,
    bf_pltfm_rptr_ucli_ucli__chss_mgmt_port_mac_get__,
    bf_pltfm_rptr_ucli_ucli__chss_mgmt_eeprom_data_get__,
    bf_pltfm_rptr_ucli_ucli__chss_mgmt_tmp_show__,
    bf_pltfm_rptr_ucli_ucli__chss_mgmt_tofino_tmp_show__,
    bf_pltfm_rptr_ucli_ucli__chss_mgmt_vrail_show__,
    bf_pltfm_rptr_ucli_ucli__chss_mgmt_ps_show__,
    bf_pltfm_rptr_ucli_ucli__chss_mgmt_fan_show__,
    bf_pltfm_rptr_ucli_ucli__chss_mgmt_fan_speed_set__,
    bf_pltfm_ucli_ucli__chss_mgmt_bmc_cmd_,
    NULL};

static ucli_module_t bf_pltfm_chss_mgmt_ucli_module__ = {
    "bf_pltfm_chss_mgmt_ucli",
    NULL,
    bf_pltfm_chss_mgmt_ucli_ucli_handlers__,
    NULL,
    NULL,
};

ucli_node_t *bf_pltfm_chss_mgmt_ucli_node_create(ucli_node_t *m) {
  ucli_node_t *n;
  ucli_module_init(&bf_pltfm_chss_mgmt_ucli_module__);
  n = ucli_node_create("chss_mgmt", m, &bf_pltfm_chss_mgmt_ucli_module__);
  ucli_node_subnode_add(n, ucli_module_log_node_create("chss_mgmt"));
  return n;
}
