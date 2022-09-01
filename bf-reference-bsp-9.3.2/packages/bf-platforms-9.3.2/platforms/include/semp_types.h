#ifndef __SEMP_TYPES_H__
#define __SEMP_TYPES_H__

#ifdef __KERNEL__

#include <linux/types.h>
#include <linux/string.h>
#include <linux/if.h>

#else /*User space*/

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <net/if.h>
#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0]))

#endif /*__KERNEL__*/
#define SEMP_PRODUCT_NAME_SZ    20
#define SEMP_OPTION_NAME_SZ     16
#define SEMP_BOARD_SN_SZ        32
#define SEMP_MOD_NAME_SZ        SEMP_OPTION_NAME_SZ
#define SEMP_MOD_DESC_SZ        128

#define SEMP_FUNC_NAME_SZ       SEMP_OPTION_NAME_SZ
#define SEMP_FUNC_DESC_SZ       128

#define SEMP_TRUE  1
#define SEMP_FALSE 0
#define MODNAM_LEN 32
#define VERSTR_LEN 32
#define LIBS_PER_MOD_MAX 10
#define SEMP_MOD_FUN_MAX 32

#define GEN_MODULE_TYPE(product_type, board_type)  ((product_type)<<16 | (board_type))
#define GET_PRODUCT_TYPE(module_type)              ((module_type) & 0xffff)

#define  ETH_COMP_FIBER    0
#define  ETH_COMP_COPPER   8

#ifndef LOCAL
#define LOCAL static
#endif

#define BIT_GET(_raw, _b) (!!((_raw) & BIT(_b)))

typedef unsigned short uint128_t[8];  //ip address type
typedef uint8_t semp_mac_t[6];
typedef unsigned short semp_vid_t;

typedef unsigned int semp_ip_t;
typedef unsigned char semp_ip6_t[16];

typedef unsigned char semp_ip4_t[4];

typedef unsigned int semp_port_t;
typedef unsigned short semp_l4_port_t;

typedef unsigned char semp_ip_proto_t;
typedef unsigned char semp_tcpflag_t;
typedef unsigned char semp_range_t;
typedef uint64_t  semp_inport_t;

typedef unsigned int semp_time_t;

//stypedef unsigned short semp_vid_t;

#define  ODD_EVEN_GET_SIZE(size) (((size) % 2) ? ((size) + 1): (size))


#define SEMP_IMG_TAIL_FLAG "CompanyImgTail" //semptian image tail flag string
typedef struct tail_dat {
    unsigned int tail_length;
    char flag_name[16];
}tail_date_t;


#define IOC_MAGIC  'c'
#define SEMP_CMD_CPLD_INIT    		       _IO(IOC_MAGIC, 0)
#define SEMP_CMD_CPLD_READ    		       _IO(IOC_MAGIC, 1)
#define SEMP_CMD_CPLD_WRITE    		       _IO(IOC_MAGIC, 2)
#define SEMP_CMD_CPLD_FLASH_READ           _IO(IOC_MAGIC, 3)
#define SEMP_CMD_CPLD_FLASH_WRITE          _IO(IOC_MAGIC ,4)
#define SEMP_CMD_CPLD_FLASH_ERASE          _IO(IOC_MAGIC ,5)
#define SEMP_CMD_CPLD_UPGRADE_FLAG         _IO(IOC_MAGIC ,6)
#define SEMP_CMD_CPLD_DEBUG_FLAG           _IO(IOC_MAGIC ,7)
#define SEMP_CMD_CPLD_UPGRADE_MODE         _IO(IOC_MAGIC ,8)
#define SEMP_CMD_CPLD_UPGRADE_TRIGGER      _IO(IOC_MAGIC ,9)



//#define _ADJ_OFF_(x) (x + 0x20)
#define _ADJ_OFF_(x) (x + 0x00)

/** For CPLD upgrade REGs definition*/
#define NT_CPLD_UPGRADE_REQ_REG                     _ADJ_OFF_(0x00)
#define NT_CPLD_UPGRADE_STATUS_REG                  _ADJ_OFF_(0x01)
#define NT_CPLD_UPGRADE_ORDER_REG                   _ADJ_OFF_(0x02)
#define NT_CPLD_UPGRADE_ADDR_15_0_REG               _ADJ_OFF_(0x03)
#define NT_CPLD_UPGRADE_ADDR_23_16_REG              _ADJ_OFF_(0x04)
#define NT_CPLD_UPGRADE_LENGTH_REG                  _ADJ_OFF_(0x05)
#define NT_CPLD_UPGRADE_STOP_GET_STATUS_REG         _ADJ_OFF_(0x06)
#define NT_CPLD_UPGRADE_FLASH_STATUS_REG            _ADJ_OFF_(0x07)
#define NT_CPLD_UPGRADE_VER_TRIGGER_REG             _ADJ_OFF_(0x08)
#define NT_CPLD_UPGRADE_POWER_OFF_REG               _ADJ_OFF_(0x09)
#define NT_CPLD_UPGRADE_RW_REQ_REG                  _ADJ_OFF_(0x10)
#define NT_CPLD_UPGRADE_ENTRY_ADDR_REG              _ADJ_OFF_(0x11)
#define NT_CPLD_UPGRADE_ENTRY_W_DATA_REG            _ADJ_OFF_(0x12)
#define NT_CPLD_UPGRADE_ENTRY_R_DATA_REG            _ADJ_OFF_(0x13)

/** For CPLD upgrade REGs definition*/

#define FC_CPLD_UPGRADE_REQ_REG             _ADJ_OFF_(0x00)
#define FC_CPLD_UPGRADE_STATUS_REG          _ADJ_OFF_(0x01)
#define FC_CPLD_UPGRADE_ORDER_REG           _ADJ_OFF_(0x02)
#define FC_CPLD_UPGRADE_ADDR_15_0_REG       _ADJ_OFF_(0x03)
#define FC_CPLD_UPGRADE_ADDR_23_16_REG      _ADJ_OFF_(0x04)
#define FC_CPLD_UPGRADE_LENGTH_REG          _ADJ_OFF_(0x05)
#define FC_CPLD_UPGRADE_STOP_GET_STATUS_REG _ADJ_OFF_(0x06)
#define FC_CPLD_UPGRADE_FLASH_STATUS_REG    _ADJ_OFF_(0x07)
#define FC_CPLD_UPGRADE_VER_TRIGGER_REG     _ADJ_OFF_(0x08)
#define FC_CPLD_UPGRADE_POWER_OFF_REG       _ADJ_OFF_(0x09)
#define FC_CPLD_UPGRADE_RAM_RW_REQ_REG      _ADJ_OFF_(0x10)
#define FC_CPLD_UPGRADE_RAM_ADDR_REG        _ADJ_OFF_(0x11)
#define FC_CPLD_UPGRADE_RAM_WDATE_REG       _ADJ_OFF_(0x12)
#define FC_CPLD_UPGRADE_RAM_RDATE_REG       _ADJ_OFF_(0x13)
#define FC_CPLD_UPGRADE_RW_REQ_REG          _ADJ_OFF_(0x10)
#define FC_CPLD_UPGRADE_ENTRY_ADDR_REG      _ADJ_OFF_(0x11)
#define FC_CPLD_UPGRADE_ENTRY_W_DATA_REG    _ADJ_OFF_(0x12)
#define FC_CPLD_UPGRADE_ENTRY_R_DATA_REG    _ADJ_OFF_(0x13)


#define FC_CPLD_UPGRADE_FLASH_PAGE_LENGTH 264

#define NT_CPLD_UPGRADE_SWITCH_REG                  0x1f

#define NT_CPLD_UPGRADE_FLAG_REG                    0x0ffe00


//#define CPLD_UPGRADE_SAFE_VERSION

#if defined(CPLD_UPGRADE_SAFE_VERSION)
#define NT_CPLD_UPGRADE_START_ADDR                  0x0
#else
#define NT_CPLD_UPGRADE_START_ADDR                  0x080000
#endif


typedef enum
{
    SE_SUCCESS = 0,
    SE_FAIL,        /*Unknow fail*/
    SE_NULL,        /*Pointer is NULL*/
    SE_MEMORY,      /*Not enough memory or memory error*/
    SE_TIMEOUT,     /*Time out*/
    SE_EXCEED,      /*Value exceed valid range*/
    SE_PARAM,       /*Unknow parameter error*/
    SE_EXIST,       /*Item have been exist, repeat add*/
    SE_OPEN,        /*Open file or buffer failed*/
    SE_READ,        /*Read file or buffer failed*/
    SE_WRITE,       /*Write file or buffer failed*/
    SE_FORMAT,      /*Parameter format not match*/
    SE_INIT,        /*Not init yet, or init failed*/
    SE_NOTFOUND,    /*Not found specified item*/
    SE_NOTSUPPORT,  /*Not support the item*/
    SE_NOTREADY,    /*Not ready yet*/
    SE_NOSYNC,      /*Have not been synchronized*/
    SE_NORESOURCE,  /*Have no resourece*/
    SE_MSG_QUE_SYNC,/*Message queue synchronized failed*/
    SE_OVERFLOW,    /*Overflow*/
    SE_MAX          /*Fail error count*/
} SERV;

#define SERV_MSG(e) \
( \
    ((e) & 0xff) == SE_SUCCESS      ? "SE_SUCCESS"      : \
    ((e) & 0xff) == SE_FAIL         ? "SE_FAIL"         : \
    ((e) & 0xff) == SE_NULL         ? "SE_NULL"         : \
    ((e) & 0xff) == SE_MEMORY       ? "SE_MEMORY"       : \
    ((e) & 0xff) == SE_TIMEOUT      ? "SE_TIMEOUT"      : \
    ((e) & 0xff) == SE_EXCEED       ? "SE_EXCEED"       : \
    ((e) & 0xff) == SE_PARAM        ? "SE_PARAM"        : \
    ((e) & 0xff) == SE_EXIST        ? "SE_EXIST"        : \
    ((e) & 0xff) == SE_OPEN         ? "SE_OPEN"         : \
    ((e) & 0xff) == SE_READ         ? "SE_READ"         : \
    ((e) & 0xff) == SE_WRITE        ? "SE_WRITE"        : \
    ((e) & 0xff) == SE_FORMAT       ? "SE_FORMAT"       : \
    ((e) & 0xff) == SE_INIT         ? "SE_INIT"         : \
    ((e) & 0xff) == SE_NOTFOUND     ? "SE_NOTFOUND"     : \
    ((e) & 0xff) == SE_NOTSUPPORT   ? "SE_NOTSUPPORT"   : \
    ((e) & 0xff) == SE_NOTREADY     ? "SE_NOTREADY"     : \
    ((e) & 0xff) == SE_NOSYNC       ? "SE_NOSYNC"       : \
    ((e) & 0xff) == SE_NORESOURCE   ? "SE_NORESOURCE"   : \
    ((e) & 0xff) == SE_MSG_QUE_SYNC ? "SE_MSG_QUE_SYNC" : \
    ((e) & 0xff) == SE_OVERFLOW     ? "SE_OVERFLOW"     : \
    "SE_MAX" \
)

#define XC3200AN_SIZE 264


typedef struct 
{
    unsigned int  reg_addr;         /* reg addr */
    unsigned int   data;  /* read/write data */
	unsigned int   cpld_index;  /* read/write data */
    int        data_len;            /*read/write data length */
	uint8_t    buff[XC3200AN_SIZE+10];
}cpld_ioctl_args_t;


//OTCS : Orthogonal case
typedef enum
{
    CASE_NULL,

    CASE_OTCS_21U_18S,
    CASE_OTCS_13U_16S,

    CASE_MAX,
} SEMP_CASE_TYPE_E;

static const char *semp_case_type_str_array[] =
{
    "CASS_NULL",/* 0 */

    "OTCS_21U_18S"
    "OTCS_13U_16S"

    "CASS_MAX"
};

typedef enum
{
    BOX_NULL,

    OTCS_23U_24S,
    OTCS_13U_16S,

    BOX_MAX,
} SEMP_BOX_TYPE_E;

typedef enum
{
    SEMP_CASE_SLOT_TYPE_INVALID,
    SEMP_CASE_SLOT_TYPE_CTRL,
    SEMP_CASE_SLOT_TYPE_DATA,
    SEMP_CASE_SLOT_TYPE_SWITCH,
} SEMP_CASE_SLOT_TYPE_E;

static inline char *semp_case_type_str(int type)
{
    int type_num = type;

    if ((type_num > CASE_MAX) || (type_num < CASE_NULL))
    {
        type_num = CASE_NULL;
    }

    return ((char*)(semp_case_type_str_array[type_num]));
}

static inline int semp_case_type_idx(char *name)
{
    int type_idx = CASE_NULL;

    if (!name)
        return CASE_NULL;

    while (type_idx < CASE_MAX)
    {
        if (!strncmp(semp_case_type_str_array[type_idx],
                     name,
                     strlen(semp_case_type_str_array[type_idx])))
        {
            break;
        }

        type_idx++;
    }

    if (type_idx >= CASE_MAX)
        type_idx = CASE_MAX;

    return type_idx;
}


typedef enum
{
    BOARD_NULL,

    /******Product: B6000 Start*******/
    AXES10G1,       /* 1 Application Switching Engine(Single 68xx CPU):whole board for MCBlade6000 product*/

    /******Product: FS9000 Start******/
    LE1008,         /* 2 Line Ethernet: 10GE, 8 ports per card */
    LI1008,         /* 3 Line Inline: base on LP1008, 10GE serial*/
    LP1008,         /* 4 Line POS: 10G, 8 ports per card, Rx POS, Tx GE*/
    LP4002N,        /* 5 Line POS: 40G New version card,2 ports per card,only rx*/
    LE1001,         /* 6 Line Ethernet: 100GE subcard, 1 port per card,*/
    PMI,            /* 7 Process Module I (only for FS9000I)*/
    PMII1210,       /* 8 Process Module II: 12 physical ports, handle 100G HiGi packets*/
    PMII2101,       /* 9 Process Module II: include tcam */
    IFC_PMII2101,   /* 10 Add for distinguish PM board ,2101 ,2100 and 4120*/
    IFC_PMII2100,   /* 11 */
    IFC_PMII4120,   /* 12 */
    PMII_IFC,       /* 13 Process Module II for Inline Flow Control */
    PMII_ULT,       /* 14 Process Module II: with TCAM */
    PMII_ULT_NT,    /* 15 Process Module II: without TCAM */
    PMIII0804,      /* 16 Process Module III*/
    CB2422,         /* 17 Carry Board: for FS9000 product orgin CB2422 = CB2021 */
    RE1008,         /* 18 Rear Eth: for RTM(Rear Transition Module) board for FS9000 product */
    LT1008,         /* 19 Line Timestamp: for the line stamp board for FS9000 product */
    CB2422_2,       /* 20 */
    CB2021_3,       /* 21 */

    /******Product: DC64 Start******/
    LC1001,         /* 22 Line Channelizing: for the whole board for DC64(Demux Channelize STM64) product */
    LS1004L,        /* 23 Line Serial: LAN+Eth serial for FS9000IV line board, 4 ports per card*/
    LS1004P,        /* 24 Line Serial: POS+Eth serial for FS9000ULT line board, 4 ports per card*/
    LS1004W,        /* 25 Line Serial: WAN serial for FS9000IV line board, 4 ports per card*/
    LE1008L,        /* 26 Line Ethernet: 10GE, 8 ports per card */
    LE1008W,        /* 27 Line Ethernet: 10GE, 8 ports per card */
    LP1008W,        /* 28 Line Ethernet: 10GW(WAN in and LAN out), 8 ports per card */
    CB2121,         /* 29 Carry Board*/
    RE1016,         /* 30 Rear Eth: for RTM(Rear Transition Module) board for FS9000 product*/
    LC1008,         /* 31 Line Channelizing: for the whole board for DC8064(Demux Channelize STM64) product */

    LS4002,         /* 32 */
    /*FC5000 SERIES */
    GF16XP4,        /* 33 */

    /*FC7000 SERIES*/
    FC7000_A010,     /* 34 */
    FC7000_AN010,    /* 35 */
    FC7000_AL4,      /* 36 */
    FC7000_AL6,      /* 37 */
    FC7000_A100,     /* 38 */
    FC7000_B010,     /* 39 */
    FC7000_A148,     /* 40 */
    FC7000_B148,     /* 41 */
    FC7000_A32,      /* 42 */
    FC7000_A32B,     /* 43 */
    FC7000_BH32,     /* 44 */
    FC7000_ZX32,     /* 45 */
    FC7000_COME_CN63XX, /* 46 */
    HPU_E600,           /* 47 */
    FC700A64,           /* 48 */
    FC7000_A106B,       /* 49 */
    FC7000_BH256,       /* 50 */
    FC7000_ZX106B,      /* 51 */

    /*MC7000 SERIES*/
    MC7000D,         /* 52 */
    MC7000D_V2_D012, /* 53 */
    MC7000D_V2_D024, /* 54 */

    /*AS100 SERIES*/
    SU1010,          /* 55 */
    RS1010,          /* 56 */

    /*NGIS SERIES*/
    PG4032,          /* 57: NGIS Line board */
    SU0960,          /* 58: NGIS Switch board */
    CT1000,          /* 59: NGIS Control board */

    /*NTM SERIES*/
    NTM_LC610_8G,    /* 60: NTM FTM, 4 GE optical and 4 GE electric interfaces */
    NTM_LC610_4X4G,  /* 61: NTM FTM, 4 10GE optical and 4 GE electric interfaces */
    NTM_LC610_4T,    /* 62: NTM FTM, 4 GE electric interfaces */
    NTM_LC610_4G,    /* 63: NTM FTM, 4 GE optical interfaces */
    NTM_LC610_4XG,   /* 64: NTM FTM, 4 10GE optical interfaces */
    NTM_RC610_4X,    /* 65: NTM RTM, 4 10GE optical interfaces */
    NTM_RC610_4G,    /* 66: NTM RTM, 4 GE electric interfaces */

    /* NGIS_10T */
    CB800,          /* 67: NGIS.10T CB board */
    IB3272,         /* 68: NGIS.10T Line board */
    SU1240,         /* 69: NGIS.10T Switch board */
    CTH100,         /* 70: NGIS.10T Control board */

    /* NGIS_10TM */
    PG3248,         /* 71: NGIS.10TM board */
    SFP10G12P,      /* 72 */
    SFP28G12P,      /* 73 */
    SFP100G6P,      /* 74 */

    /*NGIS_II */
    PG4032_240, /* 75: NGIS_II Control board */
    PG4032_280, /* 76: NGIS_II Control board */
    PG4032_360, /* 77: NGIS_II Control board */

    /*LE2024 SERIES*/
    LE2024,          /* 78 */
    RE1024,          /* 79 */

    /* NGIS_HPU */
    PM4800,        /* 80 */
    PM4800_SDH,    /* 81 */
    PM4800_GE,     /* 82 */
    CTH200,        /* 83 */

    /* NT_CME */
    CBN400,         /* 84 */
    PMN2436,        /* 85 */
    SUN6400,        /* 86 */
    PGN4800,        /* 87 */
    CBN410,         /* 88 */
    SUT3200,         /* 89 */
    PPP2436,        /* 90 */

    /* FC7000T */
    FC7000_T,
    FC7000_A32T,

    BOARD_MAX

} SEMP_BOARD_EM;

static const char *semp_board_type_str_array[] =
{
    "BOARD_NULL",/* 0 */
    "AXES10G1",  /* 1 */
    "LE1008",
    "LI1008",
    "LP1008",
    "LP4002N",
    "LE1001",
    "PMI",
    "PMII1210",
    "PMII2101",
    "IFC_PMII2101",/* 10 */
    "IFC_PMII2100",
    "IFC_PMII4120",
    "PMII_IFC",
    "PMII_ULT",
    "PMII_ULT_NT",
    "PMIII0804",
    "CB2021",
    "RE1008",
    "LT1008",
    "CB2021_2",   /* 20 */
    "CB2021_3",
    "LC1001",
    "LS1004L",
    "LS1004P",
    "LS1004W",
    "LE1008L",
    "LE1008W",
    "LP1008W",
    "CB2121",
    "RE1016",   /* 30 */
    "LC1008",
    "LS4002",
    "GF16XP4",

    /*FC7000 SERIES*/
    "FC7000_A010",
    "FC7000_AN010",
    "FC7000_AL4",
    "FC7000_AL6",
    "FC7000_A100",
    "FC7000_B010",
    "FC7000_A148", /* 40 */
    "FC7000_B148",
    "FC7000_A32",
    "FC7000_A32B",
    "BH32",
    "ZX32",
    "FC7000_COME_CN63XX",
    "HPU_E600",
    "FC700A64",
    "FC7000_A106B",
    "BH256",
    "ZX106B",
   

    /*MC7000 SERIES*/
    "MC7000D",          /* 45 */
    "MC7000D_V2_D012",  /* 46 */
    "MC7000D_V2_D024",  /* 47 */

    /*AS100 SERIES*/
    "SU1010",
    "RS1010",      /* 50 */

    /*NGIS SERIES*/
    "PG4032",           /* 50 */
    "SU0960",           /* 51 */
    "CT1000",           /* 52 */

    /*NTM SERIES */
    "NTM_LC610_8G",     /* 53 */
    "NTM_LC610_4X4G",   /* 54 */
    "NTM_LC610_4T",     /* 55 */
    "NTM_LC610_4G",     /* 56 */
    "NTM_LC610_4XG",    /* 57 */
    "NTM_RC610_4X",     /* 58 */
    "NTM_RC610_4G",     /* 59 */

    /* NGIS_10T */
    "CB800",            /* 60 */
    "IB3272",           /* 61 */
    "SU1240",           /* 62 */
    "CTH100",           /* 63 */

    "PG3248",           /* 64 */
    "SFP10G12P",        /* 65 */
    "SFP28G12P",        /* 66 */
    "SFP100G6P",        /* 67 */

    /*NGIS_II SERIES*/
    "PG4032_240",
    "PG4032_280",    /* 70 */
    "PG4032_360",

    /*LE2024 SERIES*/
    "LE2024",           /* 71 */
    "RE1024",           /* 72 */

    /*NGIS_HPU SERIES*/
    "PM4800",       /* 74 */
    "PM4800-SDH",
    "PM4800-GE",
    "CTH200",

    /*NT_CME*/
    "CBN400",
    "PMN2436",
    "SUN6400",
    "PGN4800",
    "CBN410",
    "SUT3200",
    "PPP2436",
    
    /* FC7000T */
    "FC7000_T",
    "FC7000_A32T",

    "BOARD_MAX"
};

static inline char *semp_board_type_str(int type)
{
    int type_num = type;

    if ((type_num > BOARD_MAX) || (type_num < BOARD_NULL))
    {
        type_num = BOARD_NULL;
    }

    return ((char*)(semp_board_type_str_array[type_num]));
}

static inline int semp_board_type_idx(char *name)
{
    int type_idx = BOARD_NULL;

    if (!name)
        return BOARD_NULL;

    while (type_idx < BOARD_MAX)
    {
        if (!strncmp(semp_board_type_str_array[type_idx],
                     name,
                     strlen(semp_board_type_str_array[type_idx])))
        {
            break;
        }

        type_idx++;
    }

    if (type_idx >= BOARD_MAX)
        type_idx = BOARD_NULL;

    return type_idx;
}


typedef enum
{
    SEMP_IFTYPE_NULL,
    SFI,
    ETH,
    LBK,
    SEMP_IFTYPE_MAX,
} SEMP_IFTYPE_EM;

typedef enum
{
    SPD_NULL,
    SPD_0M,
    SPD_10M,
    SPD_100M,
    SPD_155M,
    SPD_622M,
    SPD_1000M,
    SPD_2500M,
    SPD_10G,
    SPD_25G,
    SPD_40G,
    SPD_50G,
    SPD_100G,
    SPD_400G,
    SPD_MAX
} SEMP_SPD_EM;

static inline char *semp_port_spd_str(int type)
{
    char * spd_str[] =
    {
        "SPD_NULL",
        "SPD_0M",
        "SPD_10M",
        "SPD_100M",
        "SPD_155M",
        "SPD_622M",
        "SPD_1000M",
        "SPD_2500M",
        "SPD_10G",
        "SPD_25G",
        "SPD_40G",
        "SPD_50G",
        "SPD_100G",
        "SPD_400G",
    };

    if (type >= SPD_MAX)
    {
        return "";
    }

    return spd_str[type];
}

typedef enum
{
    SPD_0M_VAL      = 0,
    SPD_10M_VAL     = 10,
    SPD_100M_VAL    = 100,
    SPD_155M_VAL    = 155,
    SPD_622M_VAL    = 622,
    SPD_1000M_VAL   = 1000,
    SPD_2500M_VAL   = 2500,
    SPD_10G_VAL     = 10000,
    SPD_25G_VAL     = 25000,
    SPD_40G_VAL     = 40000,
    SPD_50G_VAL     = 50000,
    SPD_100G_VAL    = 100000,
    SPD_400G_VAL    = 400000
} SEMP_SPD_V;

//for fpga and sw load
typedef enum
{
    SEMP_PU_UNKONW,
    SEMP_LE1008,
    SEMP_LP1008,
    SEMP_LP4002,
    SEMP_LE1001,
    SEMP_LE4002,
    SEMP_RE1008,
    SEMP_PMI,
    SEMP_PMII1210,
    SEMP_PMIII0804,
    SEMP_FlowOS,
    SEMP_FlowAS,
    SEMP_PU_MAX,
} SEMP_PU_E;

/* #define LPORT_TYPE_LEN 4   ------   SEMP_PORT_TYPE_MAX  max value is 0xF */
/* Port type*/
typedef enum
{
    SEMP_PORT_TYPE_UNKNOWN,
    SEMP_PORT_TYPE_10M,
    SEMP_PORT_TYPE_100M,
    SEMP_PORT_TYPE_1GE,     //1G  鍏夊彛
    SEMP_PORT_TYPE_1GE_E,   //1G  鐢靛彛
    SEMP_PORT_TYPE_10GE,
    SEMP_PORT_TYPE_25GE,
    SEMP_PORT_TYPE_10GW,
    SEMP_PORT_TYPE_40GE,
    SEMP_PORT_TYPE_100GE,
    SEMP_PORT_TYPE_10GP,
    SEMP_PORT_TYPE_40GP,
    SEMP_PORT_TYPE_STM64,
    SEMP_PORT_TYPE_STM16,
    SEMP_PORT_TYPE_STM4,
    SEMP_PORT_TYPE_STM1,
    SEMP_PORT_TYPE_10GKR,
    SEMP_PORT_TYPE_10G_GFP,
    SEMP_PORT_TYPE_MAX,
} SEMP_PORT_TYPE_E;

typedef enum
{
    USER_PORT_TYPE_UNKNOWN,
    USER_PORT_TYPE_1G = 1000,
    USER_PORT_TYPE_10G = 10000,
    USER_PORT_TYPE_25G = 25000,
    USER_PORT_TYPE_40G = 40000,
    USER_PORT_TYPE_100G = 100000,
    USER_PORT_TYPE_MAX
}USER_PORT_TYPE_E;

static inline char *semp_port_type_str(int type)
{
    char * type_str[] =
    {
        "unknown",
        "10M",
        "100M",
        "1GE",
        "1GE_E",
        "10GE",
        "25GE",
        "10GW",
        "40GE",
        "100GE",
        "10GP",
        "40GP",
        "STM64",
        "STM16",
        "STM4",
        "STM1",
        "10GKR"
        "10G_GFP"
    };

    if (type >= SEMP_PORT_TYPE_MAX)
    {
        return "";
    }

    return type_str[type];
}

/* Subcards in the board */
typedef enum
{
    SEMP_SUBCARD_0 = 0,
    SEMP_SUBCARD_1,
    SEMP_SUBCARD_2,
    SEMP_SUBCARD_3,
    SEMP_SUBCARD_4,
    SEMP_SUBCARD_MAX,
} SEMP_SUBCARD_E;

typedef enum
{
    SEMP_OPTICAL_EN,
    SEMP_OPTICAL_TX_EN,
    SEMP_OPTICAL_TX_DIS,
} SEMP_OPTICAL_TYPE_E;

typedef enum
{
    SEMP_DUPLEX_HALF,
    SEMP_DUPLEX_FULL,
    SEMP_DUPLEX_MAX
} SEMP_DUPLEX_E;

typedef enum
{
    SEMP_FCS_32,
    SEMP_FCS_16,
    SEMP_FCS_MAX
} SEMP_FCS_E;

#define SEMP_PORT_LEARN_ARL      0x01       /* Learn SLF address. */
#define SEMP_PORT_LEARN_CPU      0x02       /* Copy SLF packet to CPU. */
#define SEMP_PORT_LEARN_FWD      0x04       /* Forward SLF packet */
#define SEMP_PORT_LEARN_PENDING  0x08       /* Mark learned SLF as pending */

typedef enum semp_port_discard_e
{
    SEMP_PORT_DISCARD_NONE  = 0,
    SEMP_PORT_DISCARD_ALL   = 1,
    SEMP_PORT_DISCARD_UNTAG = 2,
    SEMP_PORT_DISCARD_TAG   = 3,
    SEMP_PORT_DISCARD_INGRESS = 4,
    SEMP_PORT_DISCARD_EGRESS = 5,
    SEMP_PORT_DISCARD_COUNT = 6
} semp_port_discard_t;

typedef enum semp_linkscan_mode_e
{
    SEMP_LINKSCAN_MODE_NONE   = 0,
    SEMP_LINKSCAN_MODE_SW     = 1,
    SEMP_LINKSCAN_MODE_HW     = 2,
    SEMP_LINKSCAN_MODE_COUNT = 3
} semp_linkscan_mode_t;

typedef enum semp_port_encap_e
{
    SEMP_PORT_ENCAP_IEEE = 0,                /* IEEE 802.3 Ethernet-II  */
    SEMP_PORT_ENCAP_HIGIG = 1,               /* HIGIG Header mode       */
    SEMP_PORT_ENCAP_B5632 = 2,               /* BCM5632 Header mode     */
    SEMP_PORT_ENCAP_HIGIG2 = 3,              /* HIGIG2 Header mode      */
    SEMP_PORT_ENCAP_HIGIG2_LITE = 4,         /* HIGIG2 Header mode (Raptor style) */
    SEMP_PORT_ENCAP_HIGIG2_L2 = 5,           /* HIGIG2 Transport mode   */
    SEMP_PORT_ENCAP_HIGIG2_IP_GRE = 6,       /* HIGIG2 Tunnel mode   */
    SEMP_PORT_ENCAP_SBX = 7,                 /* SBX Header mode*/
    SEMP_PORT_ENCAP_PREAMBLE_SOP_ONLY = 8,   /* 1B preamble mode*/
    SEMP_PORT_ENCAP_COUNT                    /* last, please */
} semp_port_encap_t;


typedef enum semp_port_stp_e
{
    SEMP_PORT_STP_DISABLE   = 0,
    SEMP_PORT_STP_BLOCK     = 1,
    SEMP_PORT_STP_LISTEN    = 2,
    SEMP_PORT_STP_LEARN     = 3,
    SEMP_PORT_STP_FORWARD   = 4,
    SEMP_PORT_STP_COUNT = 5   /* last, please */
} semp_port_stp_t;

/* Modes for VLAN ingress/egress filtering. */
#define SEMP_PORT_VLAN_MEMBER_INGRESS    0x00000001
#define SEMP_PORT_VLAN_MEMBER_EGRESS     0x00000002

typedef enum
{
	SEMP_PORT_FEC_DISABLE = 0,
	SEMP_PORT_FEC_ENABLE = 1,
	SEMP_PORT_FEC_RS = 2,
}semp_port_fec_t;

/***************for ntm pkt type***************************/
typedef enum
{
    SEMP_PKT_NORMAL_IP = 0,
    SEMP_PKT_NON_IP    = 1,
    SEMP_PKT_IP_FRAGE  = 2,
    SEMP_PKT_GRE       = 3,
} SEMP_PKT_TYPE_EM;

/*****************end for ntm******************************/
typedef enum
{
    SEMP_PPP,
    SEMP_HDLC,
    SEMP_LINK_PROTO_MAX
} SEMP_LINK_PROTO_E;

typedef enum
{
    IPv4,
    IPv6
} SEMP_IP_EM;

typedef enum
{
    ICMP = 1,
    IGMP = 2,
    TCP  = 6,
    UDP  = 17,
    ICMPv6 = 58,
    SCTP = 132,
} SEMP_PROTO_EM;

typedef enum
{
    SEMP_IF_PPP = 0,         /* interface protocol PPP */
    SEMP_IF_HDLC,        /* interface protocol HDLC */
    SEMP_IF_ETH,         /* interface protocol ETH */
    SEMP_IF_POS,         /* interface protocol PPP */

    SEMP_IF_PROTO_NUM
}SEMP_IF_PROTO_TYPE_E;

typedef enum
{
    FIN = 1,
    SYN = 2,
    RST = 4,
    PSH = 8,
    ACK = 16,
    URG = 32,
} SEMP_TCPFLAG_EM;

typedef enum
{
    SEMP_STREAM_NONE = 0,
    SEMP_STREAM_RX = 1,
    SEMP_STREAM_TX = 2,
    SEMP_STREAM_RX_AND_TX = 3,
} SEMP_STREAM_DIR_EM;

typedef enum
{
    SEMP_PORT_NUM_8  = 8,
    SEMP_PORT_NUM_10 = 10,
    SEMP_PORT_NUM_16 = 16,
} SEMP_PORT_NUM_EM;

#define SEMP_IP_PROTO_COMMON_NUM    5

typedef enum
{
    SEMP_VERSION_NULL = 0,
    SEMP_VERSION_WORK,
    SEMP_VERSION_SAFE,
} SEMP_VERSION_TYPE_E;

/*Hisun */
typedef enum
{
    FC_ALL_NULL = 0,
    FC_CN6120_NO_CN78XX_NO_40G,       /*only cn6120+720G*/
    FC_CN6120_40G_NO_CN78XX,          /*only cn6120+1.28T*/
    FC_CN6120_CN78XX_NO_40G,          /*Without 40G*/
    FC_CN6120_CN78XX_40G,             /*all exist+1.28T*/
    FC_CN78XX_NO_40G_NO_CN6120,       /*only cn78xx+720G*/
    FC_CN78XX_40G_NO_CN6120,          /*Without cn6120+1.28T*/
    FC_CN6120_CN78XX_NO_40G_EXCARD,   /*cn6120+cn78+720G+exCard,none 40G*/
    FC_CN6120_CN78XX_40G_EXCARD,      /*cn6120+cn78+720G+exCard,have 40G*/
} SEMP_FC_PRODUCT_TYPE_E;

typedef enum
{
    FC_EX_CB_IS_PERSENT = 0,
    FC_EX_CB_DONOT_PERSENT,
} SEMP_FC_EX_BOARD_PERSENT_E;

/*CPLD SFP+ rate force tx enable  related definition*/
typedef enum
{
    SEMP_SFP_TX_DISABLE     =  0,   /*tx disable*/
    SEMP_SFP_TX_ENABLE      =  1,   /*tx enable*/
} SEMP_FC_SFP_TX_EN_E;

typedef enum
{
    SEMP_CFP4_TX_DISABLE     =  0,   /*tx disable*/
    SEMP_CFP4_TX_ENABLE      =  1,   /*tx enable*/
} SEMP_CFP4_TX_EN_E;

typedef enum
{
    SEMP_SFP_OPT_PROTECT_CLOSE    =  0,
    SEMP_SFP_OPT_PROTECT_OPEN     =  1,
} SEMP_SFP_OPT_PROTECT_E;

typedef enum
{
    SEMP_SFP_RATE_LE_4_5G   =  0,   /*less than 4.5G*/
    SEMP_SFP_RATE_GT_4_5G,          /*more than 4.5G*/
} SEMP_FC_SFP_RATE_E;

typedef enum
{
    SEMP_ALM_STATUS_ALARM   = 0,    /*alarm*/
    SEMP_ALM_STATUS_NORMAL,         /*Normal*/
} SEMP_FC_ALARM_STATE_E;

/*CPLD test led operation related definition*/
typedef enum
{
    SEMP_FC_TEST_LED1    =  0,
    SEMP_FC_TEST_LED2    =  1,
    SEMP_FC_TEST_LED3    =  2,
    SEMP_FC_TEST_MAX_LED =  3,
} SEMP_FC_TEST_LED_E;

typedef enum
{
    SEMP_EEPROM_RW = 0,          /*Read/write*/
    SEMP_EEPROM_PROCTECT,        /*Protection of state*/
} SEMP_FC_eeprom_protect_E;

typedef enum
{
    SEMP_CN6120_UART = 0,       /*cn6120 uart*/
    SEMP_CN78XX_UART,
} SEMP_FC_UART_SW_MODE_E;

/*get SFP+ info related definition*/
typedef enum
{
    SEMP_SFP_RX_SIG =  0,       /*Have received optical signal*/
    SEMP_SFP_RX_NO_SIG =  1,    /*Didn't receive optical signals*/
} SEMP_SFP_RX_LOS_SIG_E;

typedef enum
{
    SEMP_CFP4_RX_SIG =  0,       /*Have received optical signal*/
    SEMP_CFP4_RX_NO_SIG =  1,    /*Didn't receive optical signals*/
} SEMP_CFP4_RX_LOS_SIG_E;

typedef enum
{
    SEMP_CFP_LOW_PWR_MOD  =  0,
    SEMP_CFP_HIGH_PWR_MOD =  1,
} SEMP_CFP_PWR_MOD_E;

typedef enum
{
    SEMP_CFP_RST  =  0,
    SEMP_CFP_ENABLE =  1,
} SEMP_CFP_RST_E;

/*for qsfp28 100g mode info definition*/

typedef enum
{
    SEMP_QSFP_RESET  =  0,
    SEMP_QSFP_NORMAL =  1,
} SEMP_QSFP_RST_E;

typedef enum
{
    SEMP_QSFP_LOW_POWER  =  1,
    SEMP_QSFP_HIHG_POWER =  0,
} SEMP_QSFP_MODE_E;

typedef enum
{
    SEMP_QSFP_IIC_OPEN  =  0,
    SEMP_QSFP_IIC_CLOSE =  1,
} SEMP_QSFP_IIC_E;

/*end for qsfp28 100g mode definition*/

typedef enum
{
    SEMP_CFP_MODE_ENABLE  =  0,
    SEMP_CFP_MODE_DISABLE =  1,
} SEMP_CFP_MODE_E;

typedef enum
{
    SEMP_CFP_OK_SIG  =  5,
    SEMP_CFP_ALM_SIG =  4,
    SEMP_CFP_ABS_SIG =  3,
    SEMP_CFP_PRE_SIG =  2,
    SEMP_CFP_LOS_SIG =  1,
    SEMP_CFP_UP_SIG  =  0,
} SEMP_CFP_STAT_SIG_E;



typedef enum
{
    SEMP_RST_AS_RTC                = 0,
    SEMP_RST_AS_COME               = 1,
    SEMP_RST_AS_RTM                = 2,
    SEMP_RST_AS_MAX706             = 3,
    SEMP_RST_AS_FLASH              = 4,
    SEMP_RST_AS_DDR3_HIGH          = 5,
    SEMP_RST_AS_DDR3_LOW           = 6,
    SEMP_RST_AS_EMMC_FLASH         = 7,
    SEMP_RST_AS_P1016_PART         = 8,
    SEMP_RST_AS_P1016_GLOBAL       = 9,
    SEMP_RST_AS_IPMC               = 10,
    SEMP_RST_AS_88E1111            = 11,
    SEMP_RST_AS_BCM54980           = 12,
    SEMP_RST_AS_BCM56960_PCIE      = 13,
    SEMP_RST_AS_BCM56334           = 14,
    SEMP_RST_AS_BCM56960           = 15,
    SEMP_RST_AS_MAX,
} SEMP_AS_RESET_E;

typedef enum
{
    SEMP_LE_RST_RSV1            = 0,
    SEMP_LE_RST_COME_E          = 1,
    SEMP_LE_RST_RTM             = 2,
    SEMP_LE_RST_MAX706          = 3,
    SEMP_LE_RST_I210_SGMII      = 4,
    SEMP_LE_RST_I210_BASE0      = 5,
    SEMP_LE_RST_I210_BASE1      = 6,
    SEMP_LE_RST_I210_UPDATE     = 7,
    SEMP_LE_RST_QSFP0           = 8,
    SEMP_LE_RST_QSFP1           = 9,
    SEMP_LE_RST_IPMC            = 10,
    SEMP_LE_RST_88E_BASE0       = 11,
    SEMP_LE_RST_88E_BASE1       = 12,
    SEMP_LE_RST_NP8363_PCIE     = 13,
    SEMP_LE_RST_RSV2            = 14,
    SEMP_LE_RST_NP8363          = 15,
    SEMP_LE_RST_MAX,
} SEMP_LE_RESET_E;

typedef enum
{
    SEMP_CFP4_PORT_0         = 40,
    SEMP_CFP4_PORT_1         = 41,
    SEMP_CFP4_PORT_2         = 42,
    SEMP_CFP4_PORT_3         = 43,
} SEMP_FC_CFP4_PORT_ID_E;

typedef enum
{
    SEMP_WDT_CPU_CN6120      = 0,
    SEMP_WDT_CPU_CN78XX      = 1,
    SEMP_WDT_CPU_NULL,
} SEMP_FC_WTD_CPU_ID_E;

typedef enum
{
    SEMP_AS_WDT_CPU_CN6120      = 0,
    SEMP_AS_WDT_CPU_UBOOT       = 1,
    SEMP_AS_WDT_CPU_NULL,
} SEMP_AS_WTD_ID_E;

typedef enum
{
    SEMP_SYS_WDT_DISABLE        = 0,    /*SYS WDT close*/
    SEMP_SYS_WDT_ENABLE,                /*SYS WDT open*/
} SEMP_FC_WTD_STATUS_E;
/*End of Hisun*/
typedef struct
{
    uint8_t major;  /*fpga vertion type is 16-bit ,overflow risk*/
    uint8_t minor;  /*fpga vertion type is 16-bit ,overflow risk*/
    uint16_t build;
} semp_version_t;

#define SEMP_VER_MAX 8

typedef struct
{
    int num;                                /* Contain version number*/
    semp_version_t version[SEMP_VER_MAX];   /* Version info */
} semp_version_list_t;

typedef struct
{
    int num;                                /* Contain version number*/
    int version[SEMP_VER_MAX];   /* Version info */
} semp_version_type_t;

typedef struct
{
    uint8_t major;   /*fpga vertion type is 16-bit ,overflow risk*/
    uint16_t minor;  /*fpga vertion type is 16-bit ,overflow risk*/
} semp_hw_version_t;

typedef enum
{
    SEMP_NLP_SFP_LINK_UP    =  0,       /*nlp2342 sfp port link up*/
    SEMP_NLP_SFP_LINK_DOWN  =  1,       /*nlp2342 sfp port link down*/
} SEMP_NLP_SFP_LINK_SET_E;

typedef enum
{
    SEMP_BCM_SFP_LINK_UP    =  0,       /*sw56860 sfp port link up*/
    SEMP_BCM_SFP_LINK_DOWN  =  1,       /*sw56860 sfp port link down*/
} SEMP_BCM_SFP_LINK_SET_E;


typedef struct  /*Hisun*/
{
    int left_cb;         /*0x00=>percent; 0x01-0x11 =>Don't online*/
    int right_cb;
} bdd_ex_board_online_t;

/*flowos file headers:version*/
typedef struct
{
    uint8_t  major;
    uint8_t  minor;
    uint16_t build;
    uint8_t  reserved[4];
} semp_os_header_ver_t;

typedef struct
{
    uint16_t year;
    uint8_t  month;
    uint8_t  day;
    uint8_t  hour;
    uint8_t  minute;
    uint8_t  second;
    uint8_t  reserved;
} semp_os_header_time_t;

typedef struct
{
    uint8_t                 packet_flag[8];
    semp_os_header_ver_t    version;
    semp_os_header_time_t   time;
    uint8_t                 packet_len;
    uint8_t                 reserverd[3];
    uint32_t                file_len;
    uint8_t                 md5[32];
} semp_os_header_t;

typedef struct
{
    char *libname;
    char **version_string;
} semp_libver_t;

typedef struct
{
    char kmodnam[MODNAM_LEN];
    char kmodver[VERSTR_LEN];
} semp_kmodver_t;

#define SEMP_PROT_COMMON_NUM 5

typedef struct
{
    uint8_t idx;
    char name[SEMP_MOD_NAME_SZ];
    char desc[SEMP_MOD_DESC_SZ];
} semp_module_t;


typedef struct
{
    uint8_t idx;
    char name[SEMP_FUNC_NAME_SZ];
    char desc[SEMP_FUNC_DESC_SZ];
} semp_function_t;

typedef struct
{
    uint8_t val;
    char name[SEMP_OPTION_NAME_SZ];
    char sname[SEMP_OPTION_NAME_SZ];
} semp_u8_option_t;

typedef struct
{
    int position;
    uint8_t sn[SEMP_BOARD_SN_SZ];
} semp_board_sn_t;

typedef struct
{
    int position;
    uint8_t product_name[SEMP_PRODUCT_NAME_SZ];
} semp_product_name_t;


typedef struct
{
    unsigned short year:   11;
    unsigned short month:  4;
    unsigned short day:    5;
    unsigned short hour:   5;
    unsigned short min:    6;
    unsigned short sec:    6;
    unsigned short ms:     10;
    unsigned short us:     10;
    unsigned short ns:     10;
} semp_time_stamp_t;

#define SEMP_ADMIN_UID 0
#define SEMP_USERA_UID 1
#define SEMP_USERB_UID 2
#define SEMP_USERC_UID 3
#define SEMP_USERD_UID 4

typedef struct
{
    int pass;
    int fail;
    int error;
} semp_lookup_res_t;

typedef union
{
    uint8_t  v6[16];
    uint32_t v4[4];
    uint64_t v64[2];
} IP_ADDR_T;


#define IPv4_SIZE              4
#define IPv6_SIZE              16


typedef enum
{
    FPAG_STATUS_ONLINE = 0,
    FPAG_STATUS_OFFLINE,
    FPGA_STATUS_NUM
} FPAG_STATUS_E;

typedef struct
{
    int           fpgaId;
    FPAG_STATUS_E state;      /* 0:online  1:offline */
} fpga_online_t;

#ifndef ENABLE
#define ENABLE 1
#endif

#ifndef DISABLE
#define DISABLE 0
#endif

#ifndef FPGA_ENABLE
#define FPGA_ENABLE 0
#endif

#ifndef FPGA_DISABLE
#define FPGA_DISABLE 1
#endif

#ifndef NULL
#define NULL 0
#endif

/*CPLD led operation related definition*/
enum e_cpld_led_id
{
    CPLD_CB_MAX_LED =        4,
    CPLD_MASTER_SLAVE_LED =  3,
    CPLD_CB_LM_HS_LED =      2,
    CPLD_CB_LM_ALM_LED =     1,
    CPLD_CB_LM_RUN_LED =     0,
};



typedef enum
{
    FILTER_BIT_PROT = 0,           /* offset for protocol */
    FILTER_BIT_DP   = 1,           /* offset for destination port */
    FILTER_BIT_SP   = 2,           /* offset for source port */
    FILTER_BIT_DIP  = 3,           /* offset for destination ip */
    FILTER_BIT_SIP  = 4,           /* offset for source ip */
    FILTER_BIT_IPV6 = 6,           /* offset for IPv6 */
    FILTER_BIT_IPV4 = 7            /* offset for IPv4 */
}FILTER_BIT_OFFSET_E;

typedef struct
{
    uint32_t major_id;
    uint32_t logic_id;
    uint32_t major_speed;
    uint32_t minor_exist;    /*1:exist, 0:no exist*/
}semp_sw_port_info_t;

#define IPV4_FAMILY           (4)
#define IPV6_FAMILY           (6)

typedef struct net_addr_t
{
    char              family;     /* 地址类型: 4 for ipv4，6 for ipv6 */
    IP_ADDR_T         addr;       /* ip地址 */
    uint16_t          port;       /* ip地址对应端口 */
}NET_ADDR_T;

/*bit field protect macro(support little endian and big endian) for transplant*/
#if defined(__LITTLE_ENDIAN_BITFIELD) || defined(LITTLE_ENDIAN_BITFIELD)
#define SEMP_BITFIELD_2(a,b) a; b;
#define SEMP_BITFIELD_3(a,b,c) a; b; c;
#define SEMP_BITFIELD_4(a,b,c,d) a; b; c; d;
#define SEMP_BITFIELD_5(a,b,c,d,e) a; b; c; d; e;
#define SEMP_BITFIELD_6(a,b,c,d,e,f) a; b; c; d; e; f;
#define SEMP_BITFIELD_7(a,b,c,d,e,f,g) a; b; c; d; e; f; g;
#define SEMP_BITFIELD_8(a,b,c,d,e,f,g,h) a; b; c; d; e; f; g; h;
#define SEMP_BITFIELD_9(a,b,c,d,e,f,g,h,i) a; b; c; d; e; f; g; h; i;
#define SEMP_BITFIELD_10(a,b,c,d,e,f,g,h,i,j) a; b; c; d; e; f; g; h; i; j;
#define SEMP_BITFIELD_11(a,b,c,d,e,f,g,h,i,j,k) a; b; c; d; e; f; g; h; i; j; k;
#define SEMP_BITFIELD_12(a,b,c,d,e,f,g,h,i,j,k,l) a; b; c; d; e; f; g; h; i; j; k; \
                                             l;
#define SEMP_BITFIELD_13(a,b,c,d,e,f,g,h,i,j,k,l,m) a; b; c; d; e; f; g; h; i; j; \
                                             k; l; m;
#define SEMP_BITFIELD_14(a,b,c,d,e,f,g,h,i,j,k,l,m,n) a; b; c; d; e; f; g; h; i; \
                                                 j; k; l; m; n;
#define SEMP_BITFIELD_15(a,b,c,d,e,f,g,h,i,j,k,l,m,n,o) a; b; c; d; e; f; g; h; \
                                                   i; j; k; l; m; n; o;
#define SEMP_BITFIELD_16(a,b,c,d,e,f,g,h,i,j,k,l,m,n,o,p) a; b; c; d; e; f; g; h; \
                                                     i; j; k; l; m; n; o; p;
#define SEMP_BITFIELD_17(a,b,c,d,e,f,g,h,i,j,k,l,m,n,o,p,q) a; b; c; d; e; f; g; h; \
                                                     i; j; k; l; m; n; o; p; q;
#define SEMP_BITFIELD_18(a,b,c,d,e,f,g,h,i,j,k,l,m,n,o,p,q,r) a; b; c; d; e; f; g; h; \
                                                     i; j; k; l; m; n; o; p; q; r;
#define SEMP_BITFIELD_19(a,b,c,d,e,f,g,h,i,j,k,l,m,n,o,p,q,r,s) a; b; c; d; e; f; g; h; \
                                                     i; j; k; l; m; n; o; p; q; r; s;
#define SEMP_BITFIELD_20(a,b,c,d,e,f,g,h,i,j,k,l,m,n,o,p,q,r,s,t) a; b; c; d; e; f; g; h; \
                                                     i; j; k; l; m; n; o; p; q; r; s; t;
#define SEMP_BITFIELD_21(a,b,c,d,e,f,g,h,i,j,k,l,m,n,o,p,q,r,s,t,u) a; b; c; d; e; f; g; h; \
                                                     i; j; k; l; m; n; o; p; q; r; s; t; u;
#define SEMP_BITFIELD_22(a,b,c,d,e,f,g,h,i,j,k,l,m,n,o,p,q,r,s,t,u,v) a; b; c; d; e; f; g; h; \
                                                     i; j; k; l; m; n; o; p; q; r; s; t; u; v;
#else
#if defined(__BIG_ENDIAN_BITFIELD) || defined(BIG_ENDIAN_BITFIELD)

#define SEMP_BITFIELD_2(a,b) b; a;
#define SEMP_BITFIELD_3(a,b,c) c; b; a;
#define SEMP_BITFIELD_4(a,b,c,d) d; c; b; a;
#define SEMP_BITFIELD_5(a,b,c,d,e) e; d; c; b; a;
#define SEMP_BITFIELD_6(a,b,c,d,e,f) f; e; d; c; b; a;
#define SEMP_BITFIELD_7(a,b,c,d,e,f,g) g; f; e; d; c; b; a;
#define SEMP_BITFIELD_8(a,b,c,d,e,f,g,h) h; g; f; e; d; c; b; a;
#define SEMP_BITFIELD_9(a,b,c,d,e,f,g,h,i) i; h; g; f; e; d; c; b; a;
#define SEMP_BITFIELD_10(a,b,c,d,e,f,g,h,i,j) j; i; h; g; f; e; d; c; b; a;
#define SEMP_BITFIELD_11(a,b,c,d,e,f,g,h,i,j,k) k; j; i; h; g; f; e; d; c; b; a;
#define SEMP_BITFIELD_12(a,b,c,d,e,f,g,h,i,j,k,l) l; k; j; i; h; g; f; e; d; c; b; \
                                             a;
#define SEMP_BITFIELD_13(a,b,c,d,e,f,g,h,i,j,k,l,m) m; l; k; j; i; h; g; f; e; d; \
                                               c; b; a;
#define SEMP_BITFIELD_14(a,b,c,d,e,f,g,h,i,j,k,l,m,n) n; m; l; k; j; i; h; g; f; \
                                                 e; d; c; b; a;
#define SEMP_BITFIELD_15(a,b,c,d,e,f,g,h,i,j,k,l,m,n,o) o; n; m; l; k; j; i; h; g; \
                                                   f; e; d; c; b; a;
#define SEMP_BITFIELD_16(a,b,c,d,e,f,g,h,i,j,k,l,m,n,o,p) p; o; n; m; l; k; j; i; \
                                                     h; g; f; e; d; c; b; a;
#define SEMP_BITFIELD_17(a,b,c,d,e,f,g,h,i,j,k,l,m,n,o,p,q) q; p; o; n; m; l; k; j; i; \
                                                     h; g; f; e; d; c; b; a;
#define SEMP_BITFIELD_18(a,b,c,d,e,f,g,h,i,j,k,l,m,n,o,p,q,r) r; q; p; o; n; m; l; k; j; i; \
                                                     h; g; f; e; d; c; b; a;
#define SEMP_BITFIELD_19(a,b,c,d,e,f,g,h,i,j,k,l,m,n,o,p,q,r,s) s; r; q; p; o; n; m; l; k; j; i; \
                                                     h; g; f; e; d; c; b; a;
#define SEMP_BITFIELD_20(a,b,c,d,e,f,g,h,i,j,k,l,m,n,o,p,q,r,s,t) t; s; r; q; p; o; n; m; l; k; j; i; \
                                                     h; g; f; e; d; c; b; a;
#define SEMP_BITFIELD_21(a,b,c,d,e,f,g,h,i,j,k,l,m,n,o,p,q,r,s,t,u) u; t; s; r; q; p; o; n; m; l; k; j; i; \
                                                     h; g; f; e; d; c; b; a;
#define SEMP_BITFIELD_22(a,b,c,d,e,f,g,h,i,j,k,l,m,n,o,p,q,r,s,t,u,v) v; u; t; s; r; q; p; o; n; m; l; k; j; i; \
                                                     h; g; f; e; d; c; b; a;
#endif /*if defined(__BIG_ENDIAN_BITFIELD)*/
#endif /*if defined (__LITTLE_ENDIAN_BITFIELD)*/

#endif /* !__SEMP_TYPES_H__ */

