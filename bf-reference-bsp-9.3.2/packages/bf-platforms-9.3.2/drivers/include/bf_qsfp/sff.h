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
#ifndef __SFF_H
#define __SFF_H

/* Allow the use in C++ code. */
#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  /* Shared CMIS, QSFP and SFP fields: */
  IDENTIFIER, /* Type of Transceiver */
  SPEC_REV,
  STATUS, /* Support flags for upper pages */
  SFF8636_LANE_STATUS_FLAGS,
  SFF8636_LANE_MONITOR_FLAGS,
  MODULE_FLAGS,
  TEMPERATURE_ALARMS,
  VCC_ALARMS, /* Voltage */
  CHANNEL_RX_PWR_ALARMS,
  CHANNEL_TX_BIAS_ALARMS,
  CHANNEL_TX_PWR_ALARMS,
  TEMPERATURE,
  VCC, /* Voltage */
  CHANNEL_RX_PWR,
  CHANNEL_TX_BIAS,
  CHANNEL_TX_PWR,
  CHANNEL_TX_DISABLE,
  POWER_CONTROL,
  CDR_CONTROL, /* sff-8636 only */
  ETHERNET_COMPLIANCE,
  PWR_CLASS,
  PWR_REQUIREMENTS,
  PAGE_SELECT_BYTE,
  CONN_TYPE,
  LENGTH_SM_KM, /* Single mode, in km */
  LENGTH_SM,    /* Single mode in 100m (not in QSFP) */
  LENGTH_OM5,
  LENGTH_OM4,
  LENGTH_OM3,
  LENGTH_OM2,
  LENGTH_OM1,
  LENGTH_CBLASSY,
  VENDOR_NAME,     /* QSFP Vendor Name (ASCII) */
  VENDOR_OUI,      /* QSFP Vendor IEEE company ID */
  PART_NUMBER,     /* Part NUmber provided by QSFP vendor (ASCII) */
  REVISION_NUMBER, /* Revision number */
  ETHERNET_EXTENDED_COMPLIANCE,  /* ethernet extended compliance code */
  ETHERNET_SECONDARY_COMPLIANCE, /* ethernet secondary ext compliance code */
  VENDOR_SERIAL_NUMBER,          /* Vendor Serial Number (ASCII) */
  MFG_DATE,                      /* Manufacturing date code */
  TEMPERATURE_THRESH,
  VCC_THRESH,
  RX_PWR_THRESH,
  TX_BIAS_THRESH,

  /* CMIS-specific fields */
  /* If these change, also update ApplicationAdversiting below */
  MODULE_STATE,
  FIRMWARE_VER_ACTIVE,
  MODULE_MEDIA_TYPE,
  APSEL1_ALL,
  APSEL1_HOST_ID,
  APSEL1_MEDIA_ID,
  APSEL1_LANE_COUNTS,
  APSEL1_HOST_LANE_OPTION_MASK,
  FIRMWARE_VER_INACTIVE,
  HARDWARE_VER,
  APSEL1_MEDIA_LANE_OPTION_MASK,  // this is one contiguous list
  APSEL9_ALL,
  APSEL9_HOST_ID,
  APSEL9_MEDIA_ID,
  APSEL9_LANE_COUNTS,
  APSEL9_HOST_LANE_OPTION_MASK,
  DATAPATH_DEINIT,
  APPLY_DATAPATHINIT_SS0,
  DATAPATH_CFG_ALL_STAGED_SET0,
  DATAPATH_CFG_ALL_STAGED_SET1,
  DATAPATH_CFG_ALL_ACTIVE_SET,
  APPLICATION_SELECT_LN1,
  DATAPATH_STATE_ALL,
  DATAPATH_STATE_LN1AND2,
  CMIS_LANE_FLAGS,

  /* SFP-specific Fields */
  /* 0xA0 Address Fields */
  EXT_IDENTIFIER,        /* Extended type of transceiver */
  CONNECTOR_TYPE,        /* Code for Connector Type */
  TRANSCEIVER_CODE,      /* Code for Electronic or optical capability */
  ENCODING_CODE,         /* High speed Serial encoding algo code */
  SIGNALLING_RATE,       /* nominal signalling rate */
  RATE_IDENTIFIER,       /* type of rate select functionality */
  TRANCEIVER_CAPABILITY, /* Code for Electronic or optical capability */
  WAVELENGTH,            /* laser wavelength */
  CHECK_CODE_BASEID,     /* Check code for the above fields */
  /* Extended options */
  ENABLED_OPTIONS, /* Indicates the optional transceiver signals enabled */
  UPPER_BIT_RATE_MARGIN,   /* upper bit rate margin */
  LOWER_BIT_RATE_MARGIN,   /* lower but rate margin */
  ENHANCED_OPTIONS,        /* Enhanced options implemented */
  SFF_COMPLIANCE,          /* revision number of SFF compliance */
  CHECK_CODE_EXTENDED_OPT, /* check code for the extended options */
  VENDOR_EEPROM,

  /* 0xA2 address Fields */
  /* Diagnostics */
  ALARM_THRESHOLD_VALUES,  /* diagnostic flag alarm and warning thresh values */
  EXTERNAL_CALIBRATION,    /* diagnostic calibration constants */
  CHECK_CODE_DMI,          /* Check code for base Diagnostic Fields */
  DIAGNOSTICS,             /* Diagnostic Monitor Data */
  STATUS_CONTROL,          /* Optional Status and Control bits */
  ALARM_WARN_FLAGS,        /* Diagnostic alarm and warning flag */
  EXTENDED_STATUS_CONTROL, /* Extended status and control bytes */
  /* General Purpose */
  VENDOR_MEM_ADDRESS, /* Vendor Specific memory address */
  USER_EEPROM,        /* User Writable NVM */
  VENDOR_CONTROL,     /* Vendor Specific Control */

  SFF_FIELD_MAX /* keep this the last */
} Sff_field;

/* any updates to this list also must update qsfp_flag_common in
 * bf_qsfp_comm.c */
typedef enum {
  /* Shared CMIS, QSFP and SFP fault/warning flags: */
  FLAG_TX_LOS,
  FLAG_RX_LOS,
  FLAG_TX_ADAPT_EQ_FAULT,
  FLAG_TX_FAULT,
  FLAG_TX_LOL,
  FLAG_RX_LOL,
  FLAG_TEMP_HIGH_ALARM,
  FLAG_TEMP_LOW_ALARM,
  FLAG_TEMP_HIGH_WARN,
  FLAG_TEMP_LOW_WARN,
  FLAG_VCC_HIGH_ALARM,
  FLAG_VCC_LOW_ALARM,
  FLAG_VCC_HIGH_WARN,
  FLAG_VCC_LOW_WARN,
  FLAG_VENDOR_SPECIFIC,
  FLAG_RX_PWR_HIGH_ALARM,
  FLAG_RX_PWR_LOW_ALARM,
  FLAG_RX_PWR_HIGH_WARN,
  FLAG_RX_PWR_LOW_WARN,
  FLAG_TX_BIAS_HIGH_ALARM,
  FLAG_TX_BIAS_LOW_ALARM,
  FLAG_TX_BIAS_HIGH_WARN,
  FLAG_TX_BIAS_LOW_WARN,
  FLAG_TX_PWR_HIGH_ALARM,
  FLAG_TX_PWR_LOW_ALARM,
  FLAG_TX_PWR_HIGH_WARN,
  FLAG_TX_PWR_LOW_WARN,

  /* CMIS-only flags */
  FLAG_DATAPATH_FW_FAULT,
  FLAG_MODULE_FW_FAULT,
  FLAG_MODULE_STATE_CHANGE,
  FLAG_DATAPATH_STATE_CHANGE,
  FLAG_AUX1_HIGH_ALARM,
  FLAG_AUX1_LOW_ALARM,
  FLAG_AUX1_HIGH_WARN,
  FLAG_AUX1_LOW_WARN,
  FLAG_AUX2_HIGH_ALARM,
  FLAG_AUX2_LOW_ALARM,
  FLAG_AUX2_HIGH_WARN,
  FLAG_AUX2_LOW_WARN,
  FLAG_VEND_HIGH_ALARM,
  FLAG_VEND_LOW_ALARM,
  FLAG_VEND_HIGH_WARN,
  FLAG_VEND_LOW_WARN,
  FLAG_AUX3_HIGH_ALARM,
  FLAG_AUX3_LOW_ALARM,
  FLAG_AUX3_HIGH_WARN,
  FLAG_AUX3_LOW_WARN,
  SFF_FLAG_MAX, /* keep this last on list */
} Sff_flag;

typedef enum {
  QSFP_BANKNA = -2,
  QSFP_BANKCH = -1,  // the bank number is based on the channel
  QSFP_BANK0 = 0,
  QSFP_BANK1,
  QSFP_BANK2,
  QSFP_BANK3
} Qsfp_bank;

typedef enum {
  QSFP_PAGE0_LOWER = -1,
  QSFP_PAGE0_UPPER,
  QSFP_PAGE1,
  QSFP_PAGE2,
  QSFP_PAGE3,
  QSFP_PAGE16 = 16,
  QSFP_PAGE17 = 17
} Qsfp_page;

// this strucutre describes the byte location of memory map fields
typedef struct {
  Qsfp_bank bank;
  Qsfp_page page;
  int offset;
  int length;
  bool in_cache;
} sff_field_info_t;

typedef struct {
  Sff_field field;
  sff_field_info_t info;
} sff_field_map_t;

typedef struct {
  Sff_flag flag;
  Sff_field field;
  int firstbyte;     // byte location of lane 1
  int firstbit;      // bit location of lane 1
  int numbits;       // number of bits, per lane if applicable
  bool perlaneflag;  // true if this flag is per lane
  int laneincr;      // bit count between lanes
  bool inbyteinv;    // true if lane order in a byte is opposite of bit order
} sff_flag_map_t;

typedef struct {
  Sff_field field;
  uint32_t value;
} sff_field_mult_t;

enum PowerControl {
  POWER_OVERRIDE = 1 << 0,       // SFF-8636
  POWER_SET = 1 << 1,            // SFF-8636
  HIGH_POWER_OVERRIDE = 1 << 2,  // SFF-8636
  FORCELOWPWR = 1 << 4,          // CMIS, all versions
  CMIS4_LOWPWR = 1 << 6,         // CMIS 4.0 and later
};

enum ExternalIdentifer {
  EXT_ID_SHIFT = 6,
  EXT_ID_MASK = 0xc0,
  EXT_ID_HI_POWER_MASK = 0x03,
};

/* following compliance codes are derived from SFF-8436 document */
typedef enum {
  COMPLIANCE_NONE = 0,
  ACTIVE_CABLE = 1 << 0,
  LR4_40GBASE = 1 << 1,
  SR4_40GBASE = 1 << 2,
  CR4_40GBASE = 1 << 3,
  SR_10GBASE = 1 << 4,
  LR_10GBASE = 1 << 5,
  LRM_10GBASE = 1 << 6,
  COMPLIANCE_RSVD = 1 << 7,
} Ethernet_compliance;

/* following complianbce codes are derived from SFF-8024 document */
typedef enum {
  EXT_COMPLIANCE_NONE = 0,
  AOC_100G_BER_5 = 0x01, /* 100G AOC or 25G AUI C2M AOC 5 * 10^^-5 BER */
  SR4_100GBASE = 0x02,   /* or SR-25GBASE */
  LR4_100GBASE = 0x03,   /* or LR-25GBASE */
  ER4_100GBASE = 0x04,   /* or ER-25GBASE */
  SR10_100GBASE = 0x05,
  CWDM4_100G = 0x06,
  PSM4_100G_SMF = 0x07,
  ACC_100G_BER_5 = 0x08, /* 100G ACC or 25G AUI C2M ACC 5 * 10^^-5 BER */
  // EXT_COMPLIANCE_OBSOLETE = 0x09,
  // EXT_COMPLIANCE_RSVD1 = 0x0A,
  CR4_100GBASE = 0x0B, /* or CR-25GBASE CA-L */
  CR_25GBASE_CA_S = 0x0C,
  CR_25GBASE_CA_N = 0x0D,
  EXT_COMPLIANCE_RSVD2 = 0x0E,
  EXT_COMPLIANCE_RSVD3 = 0x0F,
  ER4_40GBASE = 0x10,
  SR_10GBASE_4 = 0x11,
  PSM4_40G_SMF = 0x12,
  G959_P1I1_2D1 = 0x13, /* 10709 Mbd, 2 km, 1310nm SM */
  G959_P1S1_2D2 = 0x14, /* 10709 Mbd, 40 km, 1550nm SM */
  G959_P1L1_2D2 = 0x15, /* 10709 Mbd, 80 km, 1550nm SM */
  T_10BASE_SFI = 0x16,  /* 10BASE-T with SFI electrical interface */
  CLR4_100G = 0x17,
  AOC_100G_BER_12 = 0x18, /* 100G AOC or 25G AUI C2M AOC 10^^-12 BER */
  ACC_100G_BER_12 = 0x19, /* 100G ACC or 25G AUI C2M ACC 10^^-12 BER */
  DWDM2_100GE = 0x1A,     /* DMWM module using 1550nm, 80 km */
} Ethernet_extended_compliance;

// Commonly used QSFP variants
typedef enum {
  UNKNOWN = 0,
  QSFP = 0x0C,
  QSFP_PLUS = 0x0D,
  QSFP_28 = 0x11,
  QSFP_DD = 0x18,
  OSFP = 0x19,
  QSFP_CMIS = 0x1E,
} Module_identifier;

typedef enum {
  QSFP_CONN_TYPE_LC = 0x07,
  QSFP_CONN_TYPE_MTP = 0x0C,
  QSFP_CONN_TYPE_NO_SEPARABLE_CAB = 0x23,
} Connector_type;

// Media encoding type : Byte 85
typedef enum {
  MEDIA_TYPE_UNDEF = 0X00,
  MEDIA_TYPE_MMF = 0x01,
  MEDIA_TYPE_SMF = 0x02,
  MEDIA_TYPE_PASSIVE_CU = 0x03,
  MEDIA_TYPE_ACTIVE_CBL = 0x04,
} Media_type_enc_for_CMIS;

typedef enum {
  QSFPDD_400GBASE_AOC = 0x03,
  QSFPDD_400GBASE_SR8 = 0x10,
  QSFPDD_400GBASE_SR4 = 0x11,
  QSFPDD_400GBASE_FR8 = 0x1A,
  QSFPDD_400GBASE_LR8 = 0x1B,
  QSFPDD_400GBASE_DR4 = 0x1C,
  QSFPDD_400GBASE_FR4 = 0x1D,
  QSFPDD_400GBASE_LR4 = 0x1E,

  QSFPDD_200GBASE_DR4 = 0x17,
  QSFPDD_200GBASE_FR4 = 0x18,
  QSFPDD_200GBASE_LR4 = 0x19,
} Module_media_interface_code;

// memory map formats (IDENTIFIER_OFFSET field is used to determine this)
typedef enum {
  MMFORMAT_UNKNOWN,
  MMFORMAT_SFF8636,
  MMFORMAT_CMIS3P0,
  MMFORMAT_CMIS4P0,
} MemMap_Format;

typedef enum {
  MODULE_ST_LOWPWR = 1,
  MODULE_ST_PWRUP,
  MODULE_ST_READY,
  MODULE_ST_PWRDN,
  MODULE_ST_FAULT,
} Module_State;

typedef enum {
  DATAPATH_ST_DEACTIVATED = 1,
  DATAPATH_ST_INIT,
  DATAPATH_ST_DEINIT,
  DATAPATH_ST_ACTIVATED,
  DATAPATH_ST_TXTURNON,
  DATAPATH_ST_TXTURNOFF,
  DATAPATH_ST_INITIALIZED,
} DataPath_State;

typedef enum {
  STAGED_SET0 = 0,
  STAGED_SET1,
  ACTIVE_SET = 0XFF,
} Control_Sets;

#ifdef __cplusplus
}
#endif /* C++ */

#endif /* __SFF_H */
