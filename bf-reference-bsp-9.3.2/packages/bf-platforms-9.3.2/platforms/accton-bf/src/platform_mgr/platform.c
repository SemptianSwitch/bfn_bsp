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

#include <bfsys/bf_sal/bf_sys_intf.h>
#include <bf_pltfm_types/bf_pltfm_types.h>
#include <bf_switchd/bf_switchd.h>
#include <bf_bd_cfg/bf_bd_cfg_intf.h>
#include <bf_bd_cfg/bf_bd_cfg_porting.h>
#include <bf_pltfm_cp2112_intf.h>
#include <bf_pltfm_rptr.h>
#include <bf_pltfm_ext_phy.h>
#include <bf_pltfm_chss_mgmt_intf.h>
#include "platform_priv.h"

#include <bf_pltfm_spi.h>
#include <bf_led/bf_led.h>
#include <bf_pltfm.h>
#include <bf_pltfm_led.h>
#include <bf_mav_led.h>
#include <bf_pltfm_slave_i2c.h>
#include <bf_pltfm_si5342.h>
#include <bf_qsfp/bf_qsfp.h>
#include <bf_pltfm_qsfp.h>
#include <bf_pltfm_rtmr.h>
#include <bf_pltfm_bd_cfg.h>
#include <tcl_server.h>

#include <bfutils/uCli/ucli.h>
#include <bfutils/uCli/ucli_argparse.h>
#include <bfutils/uCli/ucli_handler_macros.h>

//#define USE_I2C_ACCESS

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
    int err = pthread_getattr_np(pltfm_mgr_info.tclserver_t_id, &tclserver_t_attr);
    ret = pthread_cancel(pltfm_mgr_info.tclserver_t_id);
    if (ret != 0) {
      LOG_ERROR(
          "pltfm_mgr: ERROR: thread cancelation failed for tcl-server, "
          "ret=%d\n",ret);
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
static bf_sys_rmutex_t mav_i2c_lock;

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
          "pltfm_mgr: ERROR: thread cancelation failed for health monitor, "
          "ret=%d\n",ret);
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

static bf_pltfm_status_t cp2112_init() {
  bf_pltfm_status_t sts;
  sts = bf_pltfm_cp2112_init();
  if (sts != BF_PLTFM_SUCCESS) {
    LOG_ERROR("pltfm_mgr: CP2112 library initialization failed\n");
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
  bf_pltfm_cp2112_ucli_node_create(n);
  bf_pltfm_led_ucli_node_create(n);
  bf_qsfp_ucli_node_create(n);
  bf_pltfm_rptr_ucli_node_create(n);
  bf_pltfm_rtmr_ucli_node_create(n);
  bf_pltfm_si5342_ucli_node_create(n);
  bf_pltfm_slave_i2c_ucli_node_create(n);
  bf_pltfm_spi_ucli_node_create(n);
  pltfm_mgrs_ucli_node_create(n);

  return n;
}

/* platform-mgr thread exit callback API */
void bf_pltfm_platform_exit(void *arg) {
  (void)arg;

  bf_pltfm_board_id_t bd;
  bf_pltfm_chss_mgmt_bd_type_get(&bd);
  if (bd != BF_PLTFM_BD_ID_MAVERICKS_P0B_EMU) {
    if (bf_pltfm_cp2112_de_init()) {
      LOG_ERROR("pltfm_mgr: Error while de-initializing pltfm mgr");
    }
  }
  pltfm_mgr_stop_health_mntr();
  bf_sys_rmutex_del(&mav_i2c_lock);
  pltfm_mgr_stop_tcl_server();

}

/* Initialize and start pltfm_mgrs server */
bf_status_t bf_pltfm_platform_init(bf_switchd_context_t *switchd_ctx) {
  int ret = 0;

  memset(&pltfm_mgr_info, 0, sizeof(pltfm_mgr_info));
  pltfm_mgr_start_tcl_server();
  (void)ret;

  bf_pltfm_board_id_t bd;

  if (bf_sys_rmutex_init(&mav_i2c_lock) != 0) {
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
  if (bd != BF_PLTFM_BD_ID_MAVERICKS_P0B_EMU) {
    /* Initialize the CP2112 devices for the platform */
    ret = cp2112_init();
    if (ret == BF_PLTFM_SUCCESS) {
      int err = BF_PLTFM_SUCCESS;

      /* Insert ALL PLATFORM_INIT code here until board-type apis
         are implemented
      */
      /* set Tofino TEST_CORE_TAP signal */
      if (bf_pltfm_test_core_set() != BF_PLTFM_SUCCESS) {
        LOG_ERROR("Error in setting TEST_CORE_TAP\n");
        err |= BF_PLTFM_COMM_FAILED;
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

      /* Register silab 5342 module */
      if (bf_pltfm_si5342_init(NULL)) {
        LOG_ERROR("Error in SI5342 init \n");
        err |= BF_PLTFM_COMM_FAILED;
      }

      /* Register slave i2c module */
      if (bf_pltfm_slave_i2c_init(NULL)) {
        LOG_ERROR("Error in asic slave i2c init \n");
        err |= BF_PLTFM_COMM_FAILED;
      } else {
#if defined(USE_I2C_ACCESS)
        /* Register the i2c reg rd/wr functions with switchd */
        LOG_DEBUG("switch program is using i2c access to the ASIC\n");
        bf_switchd_i2c_fn_reg(bf_pltfm_switchd_dir_i2c_rd,
                              bf_pltfm_switchd_dir_i2c_wr);
#endif
      }
      /* Initialize repeater library */
      if ((bd == BF_PLTFM_BD_ID_MAVERICKS_P0A) ||
          (bd == BF_PLTFM_BD_ID_MAVERICKS_P0B)) {
        if (bf_pltfm_rptr_init()) {
          LOG_ERROR("Error while repeater library init \n");
          err |= BF_PLTFM_COMM_FAILED;
        }
      }

      /* Initialize repeater library */
      if (bd == BF_PLTFM_BD_ID_MAVERICKS_P0C) {
        bool is_in_ha =
            switchd_ctx->init_mode == BF_DEV_WARM_INIT_FAST_RECFG
                ? true
                : (switchd_ctx->init_mode == BF_DEV_WARM_INIT_HITLESS ? true
                                                                      : false);
        if (bf_pltfm_rtmr_pre_init(switchd_ctx->install_dir, is_in_ha)) {
          LOG_ERROR("Error while retimer library init \n");
          err |= BF_PLTFM_COMM_FAILED;
        }
      }

      if (err != BF_PLTFM_SUCCESS) {
        return -1;
      }

      /* Start Health Monitor */
      pltfm_mgr_start_health_mntr();
    } else {
      LOG_ERROR("Error in cp2112 initialization\n");
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

bf_pltfm_status_t bf_pltfm_platform_ext_phy_config_set(
    bf_dev_id_t dev_id, uint32_t conn, bf_pltfm_qsfp_type_t qsfp_type) {
  bf_status_t sts;

  bf_pltfm_board_id_t bd_id;
  bf_pltfm_chss_mgmt_bd_type_get(&bd_id);
  if ((bd_id == BF_PLTFM_BD_ID_MAVERICKS_P0B) ||
      (bd_id == BF_PLTFM_BD_ID_MAVERICKS_P0A)) {
    sts = bf_pltfm_rptr_config_set(dev_id, conn, qsfp_type);
    if (sts != BF_PLTFM_SUCCESS) {
      LOG_ERROR("Error: in setting rptr config for port %d at %s:%d",
                conn,
                __func__,
                __LINE__);
    }
  } else if (bd_id == BF_PLTFM_BD_ID_MAVERICKS_P0C) {
    /* nothing to do for retimers */
    sts = BF_PLTFM_SUCCESS;
  } else {
    sts = BF_PLTFM_OTHER;
  }
  return sts;
}

/* acquire lock to perform an endpoint i2c operation on a cp2112 topology */
void bf_pltfm_i2c_lock() {
  bf_sys_rmutex_lock(&mav_i2c_lock);
}

/* release lock after performing an endpoint i2c operation on a cp2112 topology */
void bf_pltfm_i2c_unlock() {
  bf_sys_rmutex_unlock(&mav_i2c_lock);
}

#endif

