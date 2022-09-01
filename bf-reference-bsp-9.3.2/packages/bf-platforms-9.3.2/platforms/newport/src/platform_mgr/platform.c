/*******************************************************************************
 * Copyright (c) 2015-2020 Barefoot Networks, Inc.
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * $Id: $
 *
 ******************************************************************************/
#include <stdio.h>
#define __USE_GNU /* See feature_test_macros(7) */
#include <pthread.h>
#include <unistd.h>

#include <bfsys/bf_sal/bf_sys_intf.h>
#include <bf_pltfm_types/bf_pltfm_types.h>
#include <bf_switchd/bf_switchd.h>
#include <bf_bd_cfg/bf_bd_cfg_intf.h>
#include <bf_bd_cfg/bf_bd_cfg_porting.h>
#include <bf_pltfm_fpga.h>
#include <bf_pltfm_ext_phy.h>
#include <bf_pltfm_chss_mgmt_intf.h>
#include "platform_priv.h"

#include <bf_pltfm_spi.h>
#include <bf_led/bf_led.h>
#include <bf_pltfm.h>
#include <bf_pltfm_led.h>
#include <bf_newport_led.h>
#include <bf_pltfm_slave_i2c.h>
#include <bf_pltfm_lmk5318.h>
#include <bf_qsfp/bf_qsfp.h>
#include <bf_pltfm_qsfp.h>
#include <bf_pltfm_rtmr.h>
#include <bf_pltfm_bd_cfg.h>
#include <tcl_server.h>

#include <bfutils/uCli/ucli.h>
#include <bfutils/uCli/ucli_argparse.h>
#include <bfutils/uCli/ucli_handler_macros.h>

/* *** making this obsolete, use the presence of the file instead */
//#define USE_I2C_ACCESS

/* enable efuse-write if this file exists */
#define EFUSE_ENABLE_FILE "/efuse"
/* enable i2c-only image if this file exists */
#define I2C_ONLY_ENABLE_FILE "/i2c_only_enable"

#ifdef THRIFT_ENABLED
extern int add_platform_to_rpc_server(void *processor);
extern int rmv_platform_from_rpc_server(void *processor);
#endif

/* global */
static pltfm_mgr_info_t pltfm_mgr_info;

/* Start tcl-server */
void pltfm_mgr_start_tcl_server(void) {
  int ret = 0;
  /* Start the tcl-server thread */
  pthread_attr_t tclserver_t_attr;
  pthread_attr_init(&tclserver_t_attr);
  if ((ret = pthread_create(&(pltfm_mgr_info.tclserver_t_id),
                            &tclserver_t_attr,
                            tcl_server_init,
                            NULL)) != 0) {
    LOG_ERROR(
        "pltfm_mgr: ERROR: thread creation failed for tcl-server, ret=%d\n",
        ret);
  }
  pthread_setname_np(pltfm_mgr_info.tclserver_t_id, "bf_plmgr_tsrv");
}

/* Stop tcl-server */
void pltfm_mgr_stop_tcl_server(void) {
  pthread_attr_t tclserver_t_attr;
  int ret;

  if (pltfm_mgr_info.tclserver_t_id) {
    int err =
        pthread_getattr_np(pltfm_mgr_info.tclserver_t_id, &tclserver_t_attr);
    ret = pthread_cancel(pltfm_mgr_info.tclserver_t_id);
    if (ret != 0) {
      LOG_ERROR(
          "pltfm_mgr: ERROR: thread cancelation failed for tcl-server, "
          "ret=%d\n",
          ret);
    } else {
      pthread_join(pltfm_mgr_info.tclserver_t_id, NULL);
    }
    if (!err) {
      pthread_attr_destroy(&tclserver_t_attr);
    }
    pltfm_mgr_info.tclserver_t_id = 0;
  }
}

/*
 * 'TCL_ONLY' flag is to be used when you want to build the SDE to run on the
 * emulator. On the emulator we really only care about the tcl server and
 * thus build only the tcl server component of bf-platforms. The 'TCL_ONLY'
 * has not been propagated in the bf-drivers. Thus bf-switchd will still call
 * 'bf_pltfm_device_type_get' function which is actually defined in the chss-
 * mgmt module. However, since we don't compile that module in the 'TCL_ONLY'
 * option, we define a dummy version of the function under this. Also we
 * add placeholder functions for the thrift add and rmv services so that when
 * bf_switchd calls those, we don't do anything and simply return.
 */
#ifdef TCL_ONLY
int bf_pltfm_agent_rpc_server_thrift_service_add(void *processor) {
  (void)processor;
  return 0;
}

int bf_pltfm_agent_rpc_server_thrift_service_rmv(void *processor) {
  (void)processor;
  return 0;
}

/* platform-mgr thread exit callback API */
void bf_pltfm_platform_exit(void *arg) {
  (void)arg;
  pltfm_mgr_stop_tcl_server();
}

/* Initialize and start pltfm_mgrs server */
bf_status_t bf_pltfm_platform_init(bf_switchd_context_t *switchd_ctx) {
  memset(&pltfm_mgr_info, 0, sizeof(pltfm_mgr_info));
  pltfm_mgr_start_tcl_server();
  return BF_PLTFM_SUCCESS;
}

#else

static pthread_t health_mntr_t_id;
static ucli_node_t *bf_pltfm_ucli_node;
static bf_sys_rmutex_t newport_i2c_lock;

/*start Health Monitor */
static void pltfm_mgr_start_health_mntr(void) {
  int ret = 0;
  /* Start the tcl-server thread */
  pthread_attr_t health_mntr_t_attr;
  pthread_attr_init(&health_mntr_t_attr);
  if ((ret = pthread_create(
           &health_mntr_t_id, &health_mntr_t_attr, health_mntr_init, NULL)) !=
      0) {
    LOG_ERROR(
        "pltfm_mgr: ERROR: thread creation failed for health monitor, ret=%d\n",
        ret);
  } else {
    pthread_setname_np(health_mntr_t_id, "bf_plmgr_hmtr");
    LOG_DEBUG("pltfm_mgr: health monitor initialized\n");
  }
}

/* Stop Health Monitor */
static void pltfm_mgr_stop_health_mntr(void) {
  int ret;
  pthread_attr_t health_mntr_t_attr;

  if (health_mntr_t_id) {
    int err = pthread_getattr_np(health_mntr_t_id, &health_mntr_t_attr);
    ret = pthread_cancel(health_mntr_t_id);
    if (ret != 0) {
      LOG_ERROR(
          "pltfm_mgr: ERROR: thread cancellation failed for health monitor, "
          "ret=%d\n",
          ret);
    } else {
      pthread_join(health_mntr_t_id, NULL);
    }
    if (!err) {
      pthread_attr_destroy(&health_mntr_t_attr);
    }
    health_mntr_t_id = 0;
  }
}

#ifdef THRIFT_ENABLED
int bf_pltfm_agent_rpc_server_thrift_service_add(void *processor) {
  return add_platform_to_rpc_server(processor);
}

int bf_pltfm_agent_rpc_server_thrift_service_rmv(void *processor) {
  return rmv_platform_from_rpc_server(processor);
}
#endif

static bf_pltfm_status_t chss_mgmt_init() {
  bf_pltfm_status_t sts;
  sts = bf_pltfm_chss_mgmt_init();
  if (sts != BF_PLTFM_SUCCESS) {
    LOG_ERROR("pltfm_mgr: Chassis Mgmt library initialization failed\n");
    return sts;
  }
  return BF_PLTFM_SUCCESS;
}

static ucli_status_t pltfm_mgr_ucli_ucli__mntr__(ucli_context_t *uc) {
  uint8_t cntrl = 0;

  UCLI_COMMAND_INFO(
      uc, "monitoring_control", 1, " <0/1> 0: ENABLE , 1: DISABLE");

  cntrl = atoi(uc->pargs->args[0]);

  if (cntrl == MNTR_DISABLE) {
    mntr_cntrl_hndlr = MNTR_DISABLE;
    printf("Monitoring disabled \n");
  } else {
    mntr_cntrl_hndlr = MNTR_ENABLE;
    printf("Monitoring enabled \n");
  }

  return 0;
}

static ucli_command_handler_f bf_pltfm_mgr_ucli_ucli_handlers__[] = {
    pltfm_mgr_ucli_ucli__mntr__, NULL};

static ucli_module_t mavericks_pltfm_mgrs_ucli_module__ = {
    "pltfm_mgr_ucli", NULL, bf_pltfm_mgr_ucli_ucli_handlers__, NULL, NULL,
};

ucli_node_t *pltfm_mgrs_ucli_node_create(ucli_node_t *m) {
  ucli_node_t *n;
  ucli_module_init(&mavericks_pltfm_mgrs_ucli_module__);
  n = ucli_node_create("pltfm_mgr", m, &mavericks_pltfm_mgrs_ucli_module__);
  ucli_node_subnode_add(n, ucli_module_log_node_create("pltfm_mgr"));
  return n;
}

static ucli_command_handler_f bf_pltfm_ucli_ucli_handlers__[] = {NULL};

static ucli_module_t bf_pltfm_ucli_module__ = {
    "bf_pltfm_ucli", NULL, bf_pltfm_ucli_ucli_handlers__, NULL, NULL};

ucli_node_t *bf_pltfm_ucli_node_create(void) {
  ucli_node_t *n;
  ucli_module_init(&bf_pltfm_ucli_module__);
  n = ucli_node_create("bf_pltfm", NULL, NULL);
  ucli_node_subnode_add(n, ucli_module_log_node_create("bf_pltfm"));
  bf_bd_cfg_ucli_node_create(n);
  bf_pltfm_chss_mgmt_ucli_node_create(n);
  bf_qsfp_ucli_node_create(n);
  bf_pltfm_led_ucli_node_create(n);
  bf_pltfm_fpga_ucli_node_create(n);
  bf_pltfm_lmk5318_ucli_node_create(n);
  bf_pltfm_slave_i2c_ucli_node_create(n);
  bf_pltfm_spi_ucli_node_create(n);
  pltfm_mgrs_ucli_node_create(n);

  return n;
}

/* platform-mgr thread exit callback API */
void bf_pltfm_platform_exit(void *arg) {
  (void)arg;
  bf_pltfm_fpga_deinit((void *)0UL);
  pltfm_mgr_stop_health_mntr();
  bf_sys_rmutex_del(&newport_i2c_lock);
  pltfm_mgr_stop_tcl_server();
}

/* Initialize and start pltfm_mgrs server */
bf_status_t bf_pltfm_platform_init(bf_switchd_context_t *switchd_ctx) {
  int ret = 0;
  char fname[512];

  memset(&pltfm_mgr_info, 0, sizeof(pltfm_mgr_info));
  pltfm_mgr_start_tcl_server();
  (void)ret;

  bf_pltfm_board_id_t bd;

  if (bf_sys_rmutex_init(&newport_i2c_lock) != 0) {
    LOG_ERROR("pltfm_mgr: i2c lock init failed\n");
    return -1;
  }
  /* Initialize the Chassis Management library */
  ret = chss_mgmt_init();
  if (ret != BF_PLTFM_SUCCESS) {
    LOG_ERROR("pltfm_mgr: chassis mgmt library init failed\n");
    return -1;
  }
  bf_pltfm_chss_mgmt_bd_type_get(&bd);

  // Model run on newport returns NULL. Check me - TBD
  if (switchd_ctx->install_dir) {
    // If the file corresponding to baord-id does not exist,we return from here.
    snprintf(fname,
             sizeof(fname),
             "%s/share/platforms/board-maps/accton/board_lane_map_%d.json",
             switchd_ctx->install_dir,
             bd);
  } else {
    // assume absolute path
    snprintf(fname,
             sizeof(fname),
             "install/share/platforms/board-maps/accton/board_lane_map_%d.json",
             bd);
  }
  LOG_DEBUG("pltfm_mgr: parsing board-map %s\n", fname);
  if (pltfm_create_bd_map(fname) != 0) {
    LOG_ERROR("pltfm_mgr: parsing board-map %s failed\n", fname);
    return -1;
  }

  // We should check all supported models under newport and proceed - TBD
  if (bd != BF_PLTFM_BD_ID_MAVERICKS_P0B_EMU) {
    /* Initialize the FPGA device for the platform */
    ret = bf_pltfm_fpga_init((void *)0UL);
    if (ret == BF_PLTFM_SUCCESS) {
      int err = BF_PLTFM_SUCCESS;

      /* Insert ALL PLATFORM_INIT code here until board-type apis
         are implemented
      */
      /* perform a dummy access to sysCPLD  to work around a know issue where
       * the first access ito it after booting  fails
      */
      {
        uint8_t val, reg = 0x01; /* version register */
        bf_fpga_i2c_addr_read(0,
                              BF_NEWPORT_FPGA_BUS_CPLD,
                              0,
                              BF_NP_SYSCPLD_I2C_ADDR,
                              &reg,
                              &val,
                              1,
                              1);
        bf_sys_usleep(5000); /* provide settling time after a possible error */
      }

      /* enable writing to efuse */
      if (access(EFUSE_ENABLE_FILE, R_OK) == 0) {
        uint8_t val, reg[2], sz;
        reg[0] = 0x41;
        sz = 1;
        if (bf_fpga_i2c_addr_read(0,
                                  BF_NEWPORT_FPGA_BUS_CPLD,
                                  0,
                                  BF_NP_SYSCPLD_I2C_ADDR,
                                  reg,
                                  &val,
                                  sz,
                                  sz) != BF_PLTFM_SUCCESS) {
          LOG_ERROR("pltfm_mgr: cannot read sysCPLD register 0x%2x\n", reg[0]);
        }
        reg[1] = val & 0xFE; /*  turn off bit 0 that drives CORE_TAP_L */
        sz = 2;
        if (bf_fpga_i2c_write(0,
                              BF_NEWPORT_FPGA_BUS_CPLD,
                              0,
                              BF_NP_SYSCPLD_I2C_ADDR,
                              reg,
                              sz) != BF_PLTFM_SUCCESS) {
          LOG_ERROR("pltfm_mgr: cannot write sysCPLD register 0x%2x\n", reg[0]);
        }
      }

      /* Register spi module */
      if (bf_pltfm_spi_init(NULL)) {
        LOG_ERROR("Error in spi init \n");
        err |= BF_PLTFM_COMM_FAILED;
      }

      /* Register qsfp module */
      if (bf_pltfm_qsfp_init(NULL)) {
        LOG_ERROR("Error in qsfp init \n");
        err |= BF_PLTFM_COMM_FAILED;
      }

      /* Register  TI LMK05318 module */
      if (bf_pltfm_lmk5318_init(NULL)) {
        LOG_ERROR("Error in LMK5318 init \n");
        err |= BF_PLTFM_COMM_FAILED;
      }
      /* Register slave i2c module */
      if (bf_pltfm_slave_i2c_init(NULL)) {
        LOG_ERROR("Error in asic slave i2c init \n");
        err |= BF_PLTFM_COMM_FAILED;
      } else {
#if 1  // defined(USE_I2C_ACCESS) // Temporary, donot commit to master
        if (access(I2C_ONLY_ENABLE_FILE, R_OK) == 0) {
          /* Register the i2c reg rd/wr functions with switchd */
          LOG_DEBUG("switch program is using i2c access to the ASIC\n");
          bf_switchd_i2c_fn_reg(bf_pltfm_switchd_dir_i2c_rd,
                                bf_pltfm_switchd_dir_i2c_wr);
        }
#endif
      }

      if (err != BF_PLTFM_SUCCESS) {
        return -1;
      }

      /* Start Health Monitor */
      pltfm_mgr_start_health_mntr();
    } else {
      LOG_ERROR("Error in fpga initialization\n");
      return BF_PLTFM_COMM_FAILED;
    }
  }

  bf_pltfm_ucli_node = bf_pltfm_ucli_node_create();
  bf_drv_shell_register_ucli(bf_pltfm_ucli_node);

  return BF_PLTFM_SUCCESS;
}

bf_pltfm_status_t bf_pltfm_platform_port_init(bf_dev_id_t dev_id,
                                              bool warm_init) {
  (void)warm_init;
  bf_pltfm_board_id_t bd_id;
  bf_pltfm_chss_mgmt_bd_type_get(&bd_id);

  if (bd_id != BF_PLTFM_BD_ID_MAVERICKS_P0B_EMU) {
    /* Init led module */
    if (bf_pltfm_led_init(dev_id)) {
      LOG_ERROR("Error in led init dev_id %d\n", dev_id);
      return BF_PLTFM_COMM_FAILED;
    }
  }
  return BF_PLTFM_SUCCESS;
}

bool platform_is_hw(void) {
  bf_pltfm_status_t sts;
  bf_pltfm_board_id_t bd;
  sts = bf_pltfm_chss_mgmt_bd_type_get(&bd);
  if (sts == BF_PLTFM_SUCCESS && bd != BF_PLTFM_BD_ID_MAVERICKS_P0B_EMU) {
    return true;
  } else {
    return false;
  }
}

/* acquire lock to perform an endpoint i2c operation on a fpga i2c topology */
void bf_pltfm_i2c_lock() { bf_sys_rmutex_lock(&newport_i2c_lock); }

/* release lock after performing an i2c operation on fpga i2c topology */
void bf_pltfm_i2c_unlock() { bf_sys_rmutex_unlock(&newport_i2c_lock); }

#endif
