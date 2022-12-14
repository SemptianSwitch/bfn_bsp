/*******************************************************************************
 * Copyright (c) 2015-2020 Barefoot Networks, Inc.
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * $Id: $
 *
 ******************************************************************************/
#include <iostream>

#include "pltfm_mgr_rpc.h"
#include <thrift/protocol/TBinaryProtocol.h>
#include <thrift/server/TSimpleServer.h>
#include <thrift/transport/TServerSocket.h>
#include <thrift/transport/TBufferTransports.h>
extern "C" {
#include <bf_pltfm_mgr/pltfm_mgr_handlers.h>
#include <bf_bd_cfg/bf_bd_cfg_intf.h>
#include <bf_pltfm_chss_mgmt_intf.h>
#include "bf_qsfp/bf_qsfp.h"
#include <bf_qsfp/sff.h>
}

using namespace ::apache::thrift;
using namespace ::apache::thrift::protocol;
using namespace ::apache::thrift::transport;
using namespace ::apache::thrift::server;

using namespace ::pltfm_mgr_rpc;

class pltfm_mgr_rpcHandler : virtual public pltfm_mgr_rpcIf {
 public:
  pltfm_mgr_rpcHandler() {}

  pltfm_mgr_status_t pltfm_mgr_dummy(const pltfm_mgr_device_t device) {
    pltfm_mgr_dummy_call();
    return 0;
  }

  void pltfm_mgr_sys_tmp_get(pltfm_mgr_sys_tmp_t &_return) {
    // pltfm_mgr_sys_tmp_t sys_tmp;
    bf_pltfm_status_t sts = BF_SUCCESS;
    bf_pltfm_temperature_info_t bf_sys_tmp;

    sts = bf_pltfm_chss_mgmt_temperature_get(&bf_sys_tmp);
    if (sts != BF_SUCCESS) {
      InvalidPltfmMgrOperation iop;
      iop.code = sts;
      throw iop;
    }

    _return.tmp1 = bf_sys_tmp.tmp1;
    _return.tmp2 = bf_sys_tmp.tmp2;
    _return.tmp3 = bf_sys_tmp.tmp3;
    _return.tmp4 = bf_sys_tmp.tmp4;
    _return.tmp5 = bf_sys_tmp.tmp5;
    _return.tmp6 = bf_sys_tmp.tmp6;
    _return.tmp7 = bf_sys_tmp.tmp7;
    _return.tmp8 = bf_sys_tmp.tmp8;
    _return.tmp9 = bf_sys_tmp.tmp9;
    _return.tmp10 = bf_sys_tmp.tmp10;

    // return sys_tmp;
  }

  void pltfm_mgr_sys_eeprom_get(pltfm_mgr_eeprom_t &_return) {
    // pltfm_mgr_eeprom_t eeprom;
    bf_pltfm_status_t sts = BF_SUCCESS;
    bf_pltfm_eeprom_t bf_eeprom;
    char str1[20];

    sts = bf_pltfm_bd_eeprom_get(&bf_eeprom);
    if (sts != BF_SUCCESS) {
      InvalidPltfmMgrOperation iop;
      iop.code = sts;
      throw iop;
    }

    _return.version = bf_eeprom.bf_pltfm_version;
    _return.prod_name =
        std::string(reinterpret_cast<char *>(bf_eeprom.bf_pltfm_product_name),
                    strlen(bf_eeprom.bf_pltfm_product_name));
    _return.prod_part_num =
        std::string(reinterpret_cast<char *>(bf_eeprom.bf_pltfm_product_number),
                    strlen(bf_eeprom.bf_pltfm_product_number));
    _return.sys_asm_part_num = std::string(
        reinterpret_cast<char *>(bf_eeprom.bf_pltfm_assembly_number),
        strlen(bf_eeprom.bf_pltfm_assembly_number));
    _return.bfn_pcba_part_num =
        std::string(reinterpret_cast<char *>(bf_eeprom.bf_pltfm_pcba_number),
                    strlen(bf_eeprom.bf_pltfm_pcba_number));
    _return.bfn_pcbb_part_num =
        std::string(reinterpret_cast<char *>(bf_eeprom.bf_pltfm_pcb_number),
                    strlen(bf_eeprom.bf_pltfm_pcb_number));
    _return.odm_pcba_part_num = std::string(
        reinterpret_cast<char *>(bf_eeprom.bf_pltfm_odm_pcba_number),
        strlen(bf_eeprom.bf_pltfm_odm_pcba_number));
    _return.odm_pcba_ser_num = std::string(
        reinterpret_cast<char *>(bf_eeprom.bf_pltfm_odm_pcba_serial),
        strlen(bf_eeprom.bf_pltfm_odm_pcba_serial));

    _return.prod_state = bf_eeprom.bf_pltfm_production_state;
    _return.prod_ver = bf_eeprom.bf_pltfm_product_version;
    _return.prod_sub_ver = bf_eeprom.bf_pltfm_product_subversion;

    _return.prod_ser_num =
        std::string(reinterpret_cast<char *>(bf_eeprom.bf_pltfm_product_serial),
                    strlen(bf_eeprom.bf_pltfm_product_serial));
    _return.prod_ast_tag =
        std::string(reinterpret_cast<char *>(bf_eeprom.bf_pltfm_product_asset),
                    strlen(bf_eeprom.bf_pltfm_product_asset));
    _return.sys_mfger = std::string(
        reinterpret_cast<char *>(bf_eeprom.bf_pltfm_system_manufacturer),
        strlen(bf_eeprom.bf_pltfm_system_manufacturer));
    _return.sys_mfg_date = std::string(
        reinterpret_cast<char *>(bf_eeprom.bf_pltfm_system_manufacturing_date),
        strlen(bf_eeprom.bf_pltfm_system_manufacturing_date));
    _return.pcb_mfger = std::string(
        reinterpret_cast<char *>(bf_eeprom.bf_pltfm_pcb_manufacturer),
        strlen(bf_eeprom.bf_pltfm_pcb_manufacturer));
    _return.assembled_at =
        std::string(reinterpret_cast<char *>(bf_eeprom.bf_pltfm_assembled),
                    strlen(bf_eeprom.bf_pltfm_assembled));

    memset(str1, 0, sizeof(str1));
    sprintf(str1,
            "%02x:%02x:%02x:%02x:%02x:%02x",
            bf_eeprom.bf_pltfm_local_mac[0],
            bf_eeprom.bf_pltfm_local_mac[1],
            bf_eeprom.bf_pltfm_local_mac[2],
            bf_eeprom.bf_pltfm_local_mac[3],
            bf_eeprom.bf_pltfm_local_mac[4],
            bf_eeprom.bf_pltfm_local_mac[5]);
    _return.loc_mac_addr.assign((const char *)(str1), sizeof(str1));
    memset(str1, 0, sizeof(str1));
    sprintf(str1,
            "%02x:%02x:%02x:%02x:%02x:%02x",
            bf_eeprom.bf_pltfm_mac_base[0],
            bf_eeprom.bf_pltfm_mac_base[1],
            bf_eeprom.bf_pltfm_mac_base[2],
            bf_eeprom.bf_pltfm_mac_base[3],
            bf_eeprom.bf_pltfm_mac_base[4],
            bf_eeprom.bf_pltfm_mac_base[5]);
    _return.ext_mac_addr.assign((const char *)(str1), sizeof(str1));

    _return.ext_mac_addr_size = bf_eeprom.bf_pltfm_mac_size;
    _return.location =
        std::string(reinterpret_cast<char *>(bf_eeprom.bf_pltfm_location),
                    strlen(bf_eeprom.bf_pltfm_location));
    _return.crc8 = bf_eeprom.bf_pltfm_crc8;

    // return eeprom;
  }

  bool pltfm_mgr_pwr_supply_present_get(const pltfm_mgr_ps_num_t ps_num) {
    bf_pltfm_status_t sts = BF_SUCCESS;
    bool ps_present = false;

    sts = bf_pltfm_chss_mgmt_pwr_supply_prsnc_get((bf_pltfm_pwr_supply_t)ps_num,
                                                  &ps_present);
    if (sts != BF_SUCCESS) {
      InvalidPltfmMgrOperation iop;
      iop.code = sts;
      throw iop;
    }

    return ps_present;
  }

  void pltfm_mgr_pwr_supply_info_get(pltfm_mgr_pwr_supply_info_t &_return,
                                     const pltfm_mgr_ps_num_t ps_num) {
    bf_pltfm_status_t sts = BF_SUCCESS;
    // pltfm_mgr_pwr_supply_info_t ps_info;
    bf_pltfm_pwr_supply_info_t bf_ps_info;

    sts = bf_pltfm_chss_mgmt_pwr_supply_get((bf_pltfm_pwr_supply_t)ps_num,
                                            &bf_ps_info);
    if (sts != BF_SUCCESS) {
      InvalidPltfmMgrOperation iop;
      iop.code = sts;
      throw iop;
    }

    _return.vin = bf_ps_info.vin;
    _return.vout = bf_ps_info.vout;
    _return.iout = bf_ps_info.iout;
    _return.pwr_out = bf_ps_info.pwr_out;
    _return.fspeed = bf_ps_info.fspeed;
    _return.ffault = bf_ps_info.ffault;
    _return.load_sharing = bf_ps_info.load_sharing;
    _return.model = bf_ps_info.model;
    _return.serial = bf_ps_info.serial;
    _return.rev = bf_ps_info.rev;

    // return ps_info;
  }

  void pltfm_mgr_pwr_rail_info_get(pltfm_mgr_pwr_rail_info_t &_return,
                                   const pltfm_mgr_ps_num_t pwr_supply_num) {
    bf_pltfm_status_t sts = BF_SUCCESS;
    // pltfm_mgr_pwr_rail_info_t pwr_rail_info;
    bf_pltfm_pwr_rails_info_t bf_pwr_rail_info;

    sts = bf_pltfm_chss_mgmt_pwr_rails_get(&bf_pwr_rail_info);
    if (sts != BF_SUCCESS) {
      InvalidPltfmMgrOperation iop;
      iop.code = sts;
      throw iop;
    }

    _return.vrail1 = bf_pwr_rail_info.vrail1;
    _return.vrail2 = bf_pwr_rail_info.vrail2;
    _return.vrail3 = bf_pwr_rail_info.vrail3;
    _return.vrail4 = bf_pwr_rail_info.vrail4;
    _return.vrail5 = bf_pwr_rail_info.vrail5;
    _return.vrail6 = bf_pwr_rail_info.vrail6;
    _return.vrail7 = bf_pwr_rail_info.vrail7;
    _return.vrail8 = bf_pwr_rail_info.vrail8;
    _return.vrail9 = bf_pwr_rail_info.vrail9;
    _return.vrail10 = bf_pwr_rail_info.vrail10;
    _return.vrail11 = bf_pwr_rail_info.vrail11;
    _return.vrail12 = bf_pwr_rail_info.vrail12;
    _return.vrail13 = bf_pwr_rail_info.vrail13;
    _return.vrail14 = bf_pwr_rail_info.vrail14;
    _return.vrail15 = bf_pwr_rail_info.vrail15;
    _return.vrail16 = bf_pwr_rail_info.vrail16;

    // return pwr_rail_info;
  }

  pltfm_mgr_status_t pltfm_mgr_fan_speed_set(
      const pltfm_mgr_fan_num fan_num, const pltfm_mgr_fan_percent percent) {
    bf_pltfm_status_t sts = BF_SUCCESS;
    bf_pltfm_fan_info_t bf_fan_info;

    bf_fan_info.fan_num = fan_num;
    bf_fan_info.percent = percent;

    sts = bf_pltfm_chss_mgmt_fan_speed_set(&bf_fan_info);
    if (sts != BF_SUCCESS) {
      InvalidPltfmMgrOperation iop;
      iop.code = sts;
      throw iop;
    }

    return 0;
  }

  void pltfm_mgr_fan_info_get(pltfm_mgr_fan_info_t &_return,
                              const pltfm_mgr_fan_num_t fan_num) {
    bf_pltfm_status_t sts = BF_SUCCESS;
    pltfm_mgr_fan_pct_t fan_percent;
    bf_pltfm_fan_data_t fan_data;

    sts = bf_pltfm_chss_mgmt_fan_data_get(&fan_data);
    if (sts != BF_SUCCESS) {
      InvalidPltfmMgrOperation iop;
      iop.code = sts;
      throw iop;
    }
    _return.fan_num = fan_data.F[fan_num - 1].fan_num;
    _return.front_rpm = fan_data.F[fan_num - 1].front_speed;
    _return.rear_rpm = fan_data.F[fan_num - 1].rear_speed;
    _return.percent = fan_data.F[fan_num - 1].percent;

    // return fan_percent;
  }

  bool pltfm_mgr_qsfp_presence_get(const int port_num) {
    bf_pltfm_status_t sts = BF_SUCCESS;
    bool mod_present;
    uint32_t lower_ports, upper_ports, cpu_ports;

    sts = bf_qsfp_detect_transceiver(port_num, &mod_present);
    if (sts != BF_SUCCESS) {
      InvalidPltfmMgrOperation iop;
      iop.code = sts;
      throw iop;
    }

    return mod_present;
  }

  void pltfm_mgr_qsfp_info_get(std::string &_return, const int port_num) {
    bf_pltfm_status_t sts = BF_SUCCESS;
    uint8_t buf[256];
    uint32_t i;
    char str[512];

    memset(buf, 0, sizeof(buf));
    sts = bf_qsfp_get_info(port_num, QSFP_PAGE0_LOWER, buf);
    if (sts != BF_SUCCESS) {
      InvalidPltfmMgrOperation iop;
      iop.code = sts;
      throw iop;
    }
    sts = bf_qsfp_get_info(port_num, QSFP_PAGE0_UPPER, (buf + 128));
    if (sts != BF_SUCCESS) {
      InvalidPltfmMgrOperation iop;
      iop.code = sts;
      throw iop;
    }
    for (i = 0; i < 128; i++) {
      sprintf(&(str[i * 2]), "%02x", buf[i]);
    }
    for (i = 128; i < 256; i++) {
      sprintf(&(str[i * 2]), "%02x", buf[i]);
    }
    _return.assign((const char *)(str), sizeof(str));
  }

  pltfm_mgr_max_port_t pltfm_mgr_qsfp_get_max_port(void) {
    return bf_bd_cfg_bd_num_port_get();
  }

  pltfm_mgr_status_t pltfm_mgr_qsfp_reset(const int port_num,
                                          const bool reset) {
    /* Not yet supported.  Need code from qsfp_mgnt branch */
    // bf_pltfm_qsfp_reset(port_num, reset);

    return 0;
  }

  bool pltfm_mgr_qsfp_lpmode_get(const int port_num) {
    bf_pltfm_status_t sts = BF_SUCCESS;
    uint32_t lower_ports;
    uint32_t upper_ports;
    uint32_t cpu_ports;
    bool lp_mode = false;

    sts =
        bf_qsfp_get_transceiver_lpmode(&lower_ports, &upper_ports, &cpu_ports);
    if (sts != BF_SUCCESS) {
      InvalidPltfmMgrOperation iop;
      iop.code = sts;
      throw iop;
    }

    if (port_num < 33) {
      lp_mode = ((lower_ports >> (port_num - 1)) & (0x1));
    } else if (port_num < 65) {
      lp_mode = ((upper_ports >> (port_num - 33)) & (0x1));
    } else if (port_num == 65) {
      lp_mode = cpu_ports;
    } else {
      InvalidPltfmMgrOperation iop;
      iop.code = sts;
      throw iop;
    }
    return lp_mode;
  }

  pltfm_mgr_status_t pltfm_mgr_qsfp_lpmode_set(const int port_num,
                                               const bool lpmode) {
    return bf_qsfp_set_transceiver_lpmode(port_num, (lpmode ? true : false));
  }

  void pltfm_mgr_sensor_info_get(std::string &_return,
                                 const std::string &options) {
    bf_pltfm_status_t sts = BF_SUCCESS;
    char buf[BF_SNSR_OUT_BUF_SIZE];

    sts = pltfm_mgr_sensor_out_get(&options[0u], buf, sizeof(buf));
    if (sts != BF_SUCCESS) {
      InvalidPltfmMgrOperation iop;
      iop.code = sts;
      throw iop;
    }

    _return.assign((const char *)(buf), sizeof(buf));
  }
};
