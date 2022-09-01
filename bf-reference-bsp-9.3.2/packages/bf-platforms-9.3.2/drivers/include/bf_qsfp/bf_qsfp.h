/*******************************************************************************
 * Copyright (c) 2015-2020 Barefoot Networks, Inc.
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * $Id: $
 *
 ******************************************************************************/

/*******************************************************************************
 *  the contents of this file is partially derived from
 *  https://github.com/facebook/fboss
 *
 *  Many changes have been applied all thru to derive the contents in this
 *  file.
 *  Please refer to the licensing information on the link proved above.
 *
 ******************************************************************************/
#ifndef _BF_QSFP_H
#define _BF_QSFP_H

#include <bf_qsfp/sff.h>
#ifdef INC_PLTFM_UCLI
#include <bfutils/uCli/ucli.h>
#endif

/* Allow the use in C++ code. */
#ifdef __cplusplus
extern "C" {
#endif

/* As per SFF-8436, QSFP+ 10 Gbs 4X PLUGGABLE TRANSCEIVER spec */

enum {
  /* Size of page read from QSFP via I2C */
  MAX_QSFP_PAGE_SIZE = 128,
  /* limit of memory address space in QSFP */
  MAX_QSFP_PAGE_SIZE_255 = 255,
  /* Maximum number of channels per module */
  CHANNEL_COUNT = 16,
  /* Maximum cable length reported */
  MAX_CABLE_LEN = 255,
};

typedef struct {
  bool low;
  bool high;
} qfsp_flag_t;

typedef struct {
  qfsp_flag_t warn;
  qfsp_flag_t alarm;
} qsfp_flags_level_t;

typedef struct {
  double value;
  qsfp_flags_level_t flags;
  struct {
    bool value;
    bool flags;
  } _isset;
} sff_sensor_t;

typedef struct {
  sff_sensor_t rx_pwr;
  sff_sensor_t tx_bias;
  sff_sensor_t tx_pwr;
} qsfp_channel_sensor_t;

typedef struct {
  uint32_t chn;
  qsfp_channel_sensor_t sensors;
} qsfp_channel_t;

typedef struct {
  sff_sensor_t temp;
  sff_sensor_t vcc;
} qsfp_global_sensor_t;

typedef struct {
  double low;
  double high;
} qfsp_threshold_t;

typedef struct {
  qfsp_threshold_t warn;
  qfsp_threshold_t alarm;
} qfsp_threshold_level_t;

typedef struct {
  qfsp_threshold_level_t temp;
  qfsp_threshold_level_t vcc;
  qfsp_threshold_level_t rx_pwr;
  qfsp_threshold_level_t tx_bias;
  qfsp_threshold_level_t tx_pwr;
} qsfp_alarm_threshold_t;

typedef struct {
  char name[17];
  char part_number[17];
  char rev[3];
  char serial_number[17];
  char date_code[9];
  uint8_t oui[3];
} qsfp_vendor_info_t;

typedef struct {
  bool is_cable_assy;
  int32_t xcvr_single_mode;
  int32_t xcvr_om5;
  int32_t xcvr_om4;
  int32_t xcvr_om3;
  int32_t xcvr_om2;
  int32_t xcvr_om1;
  float cable_assy;
  struct {
    bool xcvr_single_mode;
    bool xcvr_om5;
    bool xcvr_om4;
    bool xcvr_om3;
    bool xcvr_om2;
    bool xcvr_om1;
    bool cable_assy;
  } _isset;
} qsfp_cable_t;

typedef struct {
  uint8_t module_state;
  char module_state_str[15];
  bool lowPwr;
  bool forceLowPwr;
} qsfp_module_state_info_t;

typedef struct {
  uint8_t host_if_id;
  uint8_t media_if_id;
  char host_if_id_str[41];
  char media_if_id_str[41];
  uint8_t host_lane_cnt;
  uint8_t media_lane_cnt;
  uint8_t host_lane_assign_mask;
  uint8_t media_lane_assign_mask;
} qsfp_application_info_t;

typedef struct {
  uint8_t selected_ApSel_code;
  uint8_t data_path_id;
  bool explicit_control;
  bool adaptive_eq_en;
  //  uint8_t adaptive_input_eq_recall;  not currently needed in BF S/W
  uint8_t fixed_tx_input_eq;
  bool tx_cdr_en;
  bool rx_cdr_en;
  uint8_t rx_output_eq_pre;
  uint8_t rx_output_eq_post;
  uint8_t rx_output_ampl;
} qsfp_datapath_staged_set_info_t;  // per lane

typedef struct {
  DataPath_State datapath_state;
  char datapath_state_str[25];
  char datapath_state_str_short[5];
  bool data_path_deinit_bit;
} qsfp_datapath_state_info_t;  // per lane

typedef enum { qsfp } transceiver_type;

typedef struct {
  bool present;
  transceiver_type transceiver;
  int port;
  qsfp_global_sensor_t sensor;
  qsfp_alarm_threshold_t thresh;
  qsfp_vendor_info_t vendor;
  qsfp_cable_t cable;
  qsfp_channel_t chn[CHANNEL_COUNT];
  struct {
    bool sensor;
    bool vendor;
    bool cable;
    bool thresh;
    bool chn;
  } _isset;
} qsfp_transciever_info_t;

// Read/write vectors to be registered by platforms
typedef struct _bf_qsfp_vec_ {
  int (*pltfm_write)(unsigned int port,
                     uint8_t bank,
                     uint8_t page,
                     int offset,
                     int len,
                     uint8_t *buf,
                     uint32_t debug_flags,
                     void *);  // void for future/platform specific

  int (*pltfm_read)(unsigned int port,
                    uint8_t bank,
                    uint8_t page,
                    int offset,
                    int len,
                    uint8_t *buf,
                    uint32_t debug_flags,
                    void *);  // void for future/platform specific

} bf_qsfp_vec_t;

/* qsfp common APIs */
/* set custom power control through software reg */
void bf_qsfp_set_pwr_ctrl(int port, bool lpmode);
/* get interrupt status mask of QSFPs */
void bf_qsfp_get_transceiver_int(uint32_t *lower_ports,
                                 uint32_t *upper_ports,
                                 uint32_t *cpu_ports);
/* get presence status mask of QSFPs */
int bf_qsfp_get_transceiver_pres(uint32_t *lower_ports,
                                 uint32_t *upper_ports,
                                 uint32_t *cpu_ports);
/* get lpmode status mask of QSFPs */
int bf_qsfp_get_transceiver_lpmode(uint32_t *lower_ports,
                                   uint32_t *upper_ports,
                                   uint32_t *cpu_ports);
/* get presence status mask of QSFPs */
int bf_qsfp_set_transceiver_lpmode(int port, bool lpmode);
/* Performs a read-modify-write to a single-byte bitfield in qsfp memory */
int bf_qsfp_bitfield_rmw(int port,
                         Sff_field field,
                         uint8_t chmask,
                         uint8_t newdata);
/** read from a specific location in the memory map **/
int bf_qsfp_module_read(
    unsigned int port, int bank, int page, int offset, int len, uint8_t *buf);
/** write to a specific location in the memory map **/
int bf_qsfp_module_write(
    unsigned int port, int bank, int page, int offset, int len, uint8_t *buf);
/* detect a transceiver */
int bf_qsfp_detect_transceiver(int port, bool *is_present);
/* get transceiver information */
void bf_qsfp_get_transceiver_info(int port, qsfp_transciever_info_t *info);
/* force mark a transceiver present or not-present */
void bf_qsfp_set_present(int port, bool present);
// returns the spec revision, CMIS modules only
int bf_cmis_spec_rev_get(int port, uint8_t *major, uint8_t *minor);
/* return if a transceiver is present */
bool bf_qsfp_is_present(int port);
/* init qsfp module internal data structure */
int bf_qsfp_init();
/* deinit qsfp module internal data structure per port */
int bf_qsfp_deinit(int port);
/* set the number of qsfp ports, excluding CPU port */
void bf_qsfp_set_num(int num_ports);
/* get the number of qsfp ports, excluding CPU port */
int bf_qsfp_get_max_qsfp_ports(void);
/* if qsfp module supports pages */
bool bf_qsfp_has_pages(int port);
/* get per channel information of QSFP */
bool bf_qsfp_get_chan_sensor_data(int port, qsfp_channel_t *chn);
/* get cable information from QSFP memory */
bool bf_qsfp_get_cable_info(int port, qsfp_cable_t *cable);
/* return cmis module state information */
bool bf_qsfp_get_module_state_info(int port,
                                   qsfp_module_state_info_t *module_state_info);
/* return cmis datapath state information */
bool bf_qsfp_get_dp_state_info(int port,
                               int num_ch,
                               qsfp_datapath_state_info_t *dp_state_info);
/* return cmis datapath configuration in the specified control set */
bool bf_qsfp_get_dp_state_config(int port,
                                 int num_ch,
                                 qsfp_datapath_staged_set_info_t *set_cfg,
                                 Control_Sets cur_set);
/* returns the active and inactive firmware versions */
bool bf_qsfp_get_firmware_ver(int port,
                              uint8_t *active_ver_major,
                              uint8_t *active_ver_minor,
                              uint8_t *inactive_ver_major,
                              uint8_t *inactive_ver_minor);
/* returns the hardware version */
bool bf_qsfp_get_hardware_ver(int port,
                              uint8_t *hw_ver_major,
                              uint8_t *hw_ver_minor);
/* get module-level sensor information from QSFP memory */
bool bf_qsfp_get_module_sensor_info(int port, qsfp_global_sensor_t *info);
/* get various thresholds information from QSFP memory */
bool bf_qsfp_get_threshold_info(int port, qsfp_alarm_threshold_t *thresh);
/* return qsfp temp sensor information */
double bf_qsfp_get_temp_sensor(int port);
/* return module type identifier number */
int bf_qsfp_get_module_type(int port, uint8_t *module_type);
/* return module type identifier string */
bool bf_qsfp_get_module_type_str(int port,
                                 char *module_type_str,
                                 bool incl_hex);
/* get vendor information from QSFP memory */
bool bf_qsfp_get_vendor_info(int port, qsfp_vendor_info_t *vendor);
/* return QSFP cached info */
int bf_qsfp_get_cached_info(int port, int page, uint8_t *buf);
/* enable or disable a QSFP module transmitter */
int bf_qsfp_tx_disable(int port, int channel_mask, bool disable);
/* enable or disable a QSFP channel transmitter */
int bf_qsfp_tx_disable_single_lane(int port, int channel, bool disable);
/* is QSFP module transmitter disabled */
bool bf_qsfp_tx_is_disabled(int port, int channel);
/* deactivate all data paths */
int bf_qsfp_dp_deactivate_all(int port);
/* reset or unreset a QSFP module */
int bf_qsfp_reset(int port, bool reset);
/* return the requested media type string */
int bf_qsfp_get_media_type_str(int port, char *media_type_str);
/* get the number of advertsed Applications */
int bf_qsfp_get_application_count(int port);
/* get qsfp compliance code */
int bf_qsfp_get_eth_compliance(int port, Ethernet_compliance *code);
/* get qsfp extended compliance code */
int bf_qsfp_get_eth_ext_compliance(int port,
                                   Ethernet_extended_compliance *code);
/* get qsfp secondary extended compliance code */
int bf_qsfp_get_eth_secondary_compliance(int port,
                                         Ethernet_extended_compliance *code);
bool bf_qsfp_is_flat_mem(int port);
bool bf_qsfp_is_passive_cu(int port);
bool bf_qsfp_is_optical(int port);
uint8_t bf_qsfp_get_ch_cnt(int port);
uint8_t bf_qsfp_get_media_ch_cnt(int port);
bool bf_qsfp_get_spec_rev_str(int port, char *rev_str);
void bf_qsfp_debug_clear_all_presence_bits(void);
bool bf_qsfp_is_luxtera_optic(int conn_id);
int bf_qsfp_special_case_set(int port,
                             bf_pltfm_qsfp_type_t qsfp_type,
                             bool is_set);

#ifdef INC_PLTFM_UCLI
ucli_node_t *bf_qsfp_ucli_node_create(ucli_node_t *m);
#endif

int bf_qsfp_type_get(int port, bf_pltfm_qsfp_type_t *qsfp_type);
bool bf_qsfp_is_cable_assy(int port);
const char *bf_qsfp_get_conn_type_string(int port);
bool qsfp_needs_hi_pwr_init(int conn_id);

bool bf_qsfp_is_cmis(int port);
bool bf_qsfp_is_sff8636(int port);
MemMap_Format bf_qsfp_get_memmap_format(int port);
int bf_cmis_type_get(int port, bf_pltfm_qsfpdd_type_t *qsfp_type);
const char *bf_cmis_get_module_type_string(int port);
int bf_cmis_find_matching_Application(int port,
                                      bf_port_speed_t speed,
                                      int nlanes,
                                      int firstLane);
int bf_cmis_select_Application(int port,
                               int chnum,
                               uint8_t ApSel,
                               uint8_t datapath_ID,
                               bool explicit_control);
/* return the requested Application info */
int bf_qsfp_get_application(int port,
                            int ApSel,
                            qsfp_application_info_t *app_info);
int bf_cmis_get_datapath_state(int port,
                               int ch,
                               DataPath_State *datapath_state);

// Generic applies to all platforms
void bf_qsfp_soft_removal_set(int port, bool removed);
bool bf_qsfp_soft_removal_get(int port);

int bf_qsfp_field_read_onebank(int port,
                               Sff_field field,
                               int chmask,
                               int addl_offset,
                               int max_length,
                               uint8_t *data);

/* refresh the cache for all of the per-module and per-lane flags */
int bf_qsfp_refresh_flags(int port);
/* clears the local cached copy of the specified flag */
int bf_qsfp_clear_flag(int port, Sff_flag flag, int ln, int bit, bool clrcnt);
/* returns the last read value of the specified flag */
int bf_qsfp_get_flag(int port, Sff_flag flag, int ln, int bit, uint32_t *cnt);
int bf_cmis_get_Application_media_info(int port,
                                       int app,
                                       int host_lane,
                                       int *media_lane,
                                       int *media_firstLane,
                                       int *media_nLanes);
int bf_cmis_get_module_state(int port, Module_State *module_state);
void qsfp_fsm_force_all_lpmode();
bool qsfp_fsm_query_all_lpmode();
int bf_qsfp_vec_init(bf_qsfp_vec_t *vec);
void bf_qsfp_get_sff_eth_extended_code_description(int port,
                                                   char *ext_code_desc);
/* return QSFP info */
int bf_qsfp_get_info(int port, int page, uint8_t *buf);
/* Get reset state */
bool bf_qsfp_get_reset(int port);
#ifdef __cplusplus
}
#endif /* C++ */

#endif /* _BF_QSFP_H */
