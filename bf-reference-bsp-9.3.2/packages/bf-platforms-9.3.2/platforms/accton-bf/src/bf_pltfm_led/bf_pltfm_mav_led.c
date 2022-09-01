/*******************************************************************************
 * Copyright (c) 2015-2020 Barefoot Networks, Inc.
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * $Id: $
 *
 ******************************************************************************/
#include <string.h>
#include <bfsys/bf_sal/bf_sys_timer.h>
#include <dvm/bf_drv_intf.h>
#include <lld/lld_gpio_if.h>
#include <bf_pltfm_types/bf_pltfm_types.h>
#include <bf_bd_cfg/bf_bd_cfg_intf.h>
#include <bf_pltfm_cp2112_intf.h>
#include <bf_pltfm_chss_mgmt_intf.h>
#include <bf_pltfm_syscpld.h>
#include <bf_led/bf_led.h>
#include <bf_pltfm_bd_cfg.h>
#include <bf_pltfm_led.h>
#include <bf_mav_led.h>
#include <bfutils/uCli/ucli.h>
#include <bfutils/uCli/ucli_argparse.h>
#include <bfutils/uCli/ucli_handler_macros.h>

/**** MAVERICKS LED holding buffer organization ******
 * sysCPLD proxies as the LED driver. Each sysCPLD has got 64  8bit
 * registers. each register serves two Leds (each nibble = 1 Led). Each QSFP
 * has got 4 channels. Each channel has one LED. So, 32 ports require
 * 32*4 = 128 leds ->64 registers in sysCPLD.
 *
 * Tofino's state-out buffer is streamed out to 64 sysCPLD (+64 ) registers to
 * display the LED. so, our design of mapping of LEds contents in state-out
 * buffer is as follows:
 *
 *    byte    port/chn    port/chn
 *   _______________________________
 *
 *    0       2/0           1/0       |
 *    1       2/1           1/1       |
 *    2       2/2           1/2       |
 *    3       2/3           1/3       |
 *    4       4/0           3/0       |--upper 32 ports
 *    5       4/1           3/1       |
 *    6       4/2           3/2       |
 *    7       4/3           3/3       |
 *    8       6/0           5/0       |
 *                                    |
 *                                    |
 *                                    |
 *   63      32/3          31/3       |
 *
 *   64      34/0          33/0       |--lower 32 ports
 *   65      34/1          33/1       |
 *                                    |
 *  127      64/3          63/3       |
 *
 *   128     cpu/1        cpu/0       |--cpu port (irrespective of platform)
 *   129     cpu/3        cpu/2       |
 *
 * Note that  state-out buffer is organized to keep odd or even ports
 * contiguously together. A different organization can also be designed, but
 * that has to be implemented in the APIs accordingly. APIs in this module
 * assume above organization of state-out buffer
 */

/* The code also assumes that all i2c ports used for led belong to a single
 * ethgpio block in tofino.
 */

static int bf_pltfm_led_initialized = 0;

#define MAV_LED_PIN_PAIR_UPPER 0
#define MAV_LED_PIN_PAIR_LOWER 2

#define TIMEOUT_100MS 100
#define PERIOD_400KHZ_IN_10_NSEC               \
  0x2C /* value acquired from ASSIC simulation \
          */

/* cached value of current state-out buffer */
#define BF_LED_STATEOUT_BUF_SIZE 128

extern uint8_t st_out_buf[BF_LED_STATEOUT_BUF_SIZE];
extern uint8_t st_out_cpu[4]; /* 2 bytes: CPU led, 2 extra: for 32 bit access`*/

/* compile time, sanity check */
#if ((BF_LED_STATEOUT_BUF_SIZE + 4) > 256)
#error Tofino cannot take so large a buffer
#endif

/* cached value of current led display selection `*/
// static bool even_ports_selected = false;

/** platform port led subsystem initialization (for Mavericks)
 *
 *  @param chip_id
 *   chip_id
 *  @param upper
 *   cp2112 domain the leds belog to
 *  @return
 *   0 on success and -1 on error
 */
static int bf_pltfm_tof_i2c_led_init(int chip_id, bf_pltfm_cp2112_id_t upper) {
  bf_io_pin_pair_t idx;
  uint32_t offset;

  if (upper == CP2112_ID_1) {
    idx = MAV_LED_PIN_PAIR_UPPER;
    offset = 0;
  } else {
    idx = MAV_LED_PIN_PAIR_LOWER;
    offset = BF_LED_STATEOUT_BUF_SIZE / 2;
  }

  if (chip_id >= BF_MAX_DEV_COUNT || bf_pltfm_led_initialized) {
    return -1;
  }
  /* configure the gpio_pin_pair as i2c pins */
  bf_io_set_mode_i2c(chip_id, idx);
  /* set the i2c clk to TIMEOUT_100MS Khz (clk  period = 10000 nsec) */
  bf_i2c_set_clk(chip_id, idx, PERIOD_400KHZ_IN_10_NSEC);
  /* set i2c submode as stateout */
  bf_i2c_set_submode(chip_id, idx, BF_I2C_MODE_STATEOUT);
  /* initialize state out buffer  corresponding to the group of leds */
  memset(st_out_buf,
         BF_MAV_PORT_LED_OFF | (BF_MAV_PORT_LED_OFF << 4),
         sizeof(st_out_buf));
  memset(st_out_cpu,
         BF_MAV_PORT_LED_OFF | (BF_MAV_PORT_LED_OFF << 4),
         sizeof(st_out_cpu));
  /* write main port led bits into stateout buf */
  if (bf_write_stateout_buf(
          chip_id, idx, offset, st_out_buf, BF_LED_STATEOUT_BUF_SIZE / 2) !=
      BF_SUCCESS) {
    return -1;
  }

  /* write cpu port led bits into stateout buf */
  if (upper == CP2112_ID_1) {
    //    bf_sys_usleep(5000); /* wait until the previous i2c out is complete */

    if (bf_write_stateout_buf(
            chip_id, idx, BF_LED_STATEOUT_BUF_SIZE, st_out_cpu, 4) !=
        BF_SUCCESS) {
      return -1;
    }
  }

  return 0;
}

/* return the byte offset in st_out_buf corresponding to a given port and chn */
#define GET_BYTE_OFF_STOUT(port, chn) (((port - 1) / 2) * QSFP_NUM_CHN + chn)

/** Platform cpu port led set (for Mavericks) using Tofino state-out buffer
 *
 *  @param chip_id
 *   chip_id
 *  @param chn
 *   channel (0-3)
 *  @param val
 *   bit-wise or of BF_MAV_PORT_LED_RED, BF_MAV_PORT_LED_GREEN and
 *BF_MAV_PORT_LED_BLUE
 *  @return
 *   0 on success and -1 on error
 */
static int bf_pltfm_set_cpu_port_led(int chip_id, int chn, uint8_t val) {
  if (chn > QSFP_NUM_CHN) {
    return -1;
  }
  /* update cached value of state-out-buf */
  val &= 0xF; /* take only a nibble */
  if (chn == 0) {
    st_out_cpu[0] &= 0xF0;
    st_out_cpu[0] |= val;
  } else if (chn == 1) {
    st_out_cpu[0] &= 0x0F;
    st_out_cpu[0] |= (val << 4);
  } else if (chn == 2) {
    st_out_cpu[1] &= 0xF0;
    st_out_cpu[1] |= val;
  } else {
    st_out_cpu[1] &= 0x0F;
    st_out_cpu[1] |= (val << 4);
  }
  /* write to Tofino stetout buffer */
  if (bf_write_stateout_buf(chip_id,
                            MAV_LED_PIN_PAIR_UPPER,
                            BF_LED_STATEOUT_BUF_SIZE,
                            st_out_cpu,
                            4) != BF_SUCCESS) {
    return -1;
  }
  /* stream it out */
  if (bf_i2c_issue_stateout(chip_id,
                            MAV_LED_PIN_PAIR_UPPER,
                            (BF_MAV_SYSCPLD_I2C_ADDR >> 1),
                            BF_MAV_SYSCPLD_REG_CPU_LED_START,
                            1,
                            BF_LED_STATEOUT_BUF_SIZE,
                            BF_MAV_SYSCPLD_NUM_REG_LED) != BF_SUCCESS) {
    return -1;
  }
  return 0;
}

/** Platform port led set (for Mavericks) using Tofino state-out buffer
 *
 *  @param chip_id
 *   chip_id
 *  @param port
 *   port ( -1 for all the ports)
 *  @param led_col
 *   bit-wise or of BF_MAV_PORT_LED_RED, BF_MAV_PORT_LED_GREEN and
 *BF_MAV_PORT_LED_BLUE
 *  @param led_blink
 *   LED_BLINK or LED_NO_BLINK
 *  @return
 *   0 on success and -1 on error
 */
int bf_pltfm_port_led_by_tofino_set(int chip_id,
                                    bf_pltfm_port_info_t *port_info,
                                    int led_col,
                                    bf_led_blink_t led_blink) {
  bf_pltfm_status_t sts;
  bf_pltfm_board_id_t board_id;
  int st_out_off;
  int port;
  int num_ports = platform_num_ports_get();
  uint8_t val, st_out_byte;

  if (bf_pltfm_led_initialized == 0) {
    return -1;
  }
  sts = bf_pltfm_chss_mgmt_bd_type_get(&board_id);
  if (sts != BF_PLTFM_SUCCESS) {
    return -1;
  }

  val = led_col &
        (BF_MAV_PORT_LED_RED | BF_MAV_PORT_LED_GREEN | BF_MAV_PORT_LED_BLUE);
  if (led_blink & LED_BLINK) {
    val |= BF_MAV_PORT_LED_BLINK;
  }

  /* we need to update a nibble correponding to the port in state-out buf.
   * we can do only 32 bit PIO for state-out buf.
   * get the offset of 32 bit word containing the nibble.
   */

  if (!port_info) {
    val |= (val << 4);
    memset(st_out_buf, val, sizeof(st_out_buf));
    /* update the entire state-out buffer with the same value */
    if (bf_write_stateout_buf(chip_id,
                              MAV_LED_PIN_PAIR_UPPER,
                              0,
                              st_out_buf,
                              BF_LED_STATEOUT_BUF_SIZE) != BF_SUCCESS) {
      return -1;
    }
  } else {
    if (port_info->conn_id == (uint32_t)num_ports) {
      return bf_pltfm_set_cpu_port_led(chip_id, port_info->chnl_id, val);
    }
    if (port_info->conn_id <= 0 || port_info->conn_id > (uint32_t)num_ports ||
        port_info->chnl_id > 3) {
      return -1;
    }
    port = port_info->conn_id;
    st_out_off = GET_BYTE_OFF_STOUT(port_info->conn_id, port_info->chnl_id);
    st_out_byte = st_out_buf[st_out_off];
    if (port % 2) { /* odd ports occupy lower nibble */
      st_out_byte &= 0xF0;
      st_out_byte |= val;
    } else { /* even ports occupy upper nibble */
      st_out_byte &= 0x0F;
      st_out_byte |= (val << 4);
    }
    /* update the cached state-out buffer */
    st_out_buf[st_out_off] = st_out_byte;
    /* compute the 32 bit aligned offset containing the byte just changed */
    st_out_off &= ~0x3;
    /* update a corresponding nibble in the state-out buffer */
    if (bf_write_stateout_buf(chip_id,
                              MAV_LED_PIN_PAIR_UPPER,
                              st_out_off,
                              st_out_buf + st_out_off,
                              4) != BF_SUCCESS) {
      return -1;
    }
  }
  if ((board_id == BF_PLTFM_BD_ID_MONTARA_P0B) ||
      (board_id == BF_PLTFM_BD_ID_MONTARA_P0C) ||
      (board_id == BF_PLTFM_BD_ID_MONTARA_P0A)) {
    if (bf_i2c_issue_stateout(chip_id,
                              MAV_LED_PIN_PAIR_LOWER,
                              (BF_MAV_SYSCPLD_I2C_ADDR >> 1),
                              BF_MAV_SYSCPLD_REG_LED_START,
                              1,
                              0,
                              BF_LED_STATEOUT_BUF_SIZE / 2) != BF_SUCCESS) {
      return -1;
    }
  } else if ((board_id == BF_PLTFM_BD_ID_MAVERICKS_P0B) ||
             (board_id == BF_PLTFM_BD_ID_MAVERICKS_P0A) ||
             (board_id == BF_PLTFM_BD_ID_MAVERICKS_P0C)) {
    if (bf_i2c_issue_stateout(chip_id,
                              MAV_LED_PIN_PAIR_UPPER,
                              (BF_MAV_SYSCPLD_I2C_ADDR >> 1),
                              BF_MAV_SYSCPLD_REG_LED_START,
                              1,
                              0,
                              BF_LED_STATEOUT_BUF_SIZE / 2) != BF_SUCCESS) {
      return -1;
    }
    if (bf_i2c_issue_stateout(chip_id,
                              MAV_LED_PIN_PAIR_LOWER,
                              (BF_MAV_SYSCPLD_I2C_ADDR >> 1),
                              BF_MAV_SYSCPLD_REG_LED_START,
                              1,
                              BF_LED_STATEOUT_BUF_SIZE / 2,
                              BF_LED_STATEOUT_BUF_SIZE / 2) != BF_SUCCESS) {
      return -1;
    }
  }
  return 0;
}

static ucli_status_t bf_pltfm_ucli_ucli__led_set_tofino_(ucli_context_t *uc) {
  bf_dev_id_t dev = 0;
  int port, chn;
  int col;
  int blink;
  bf_led_blink_t led_blink;
  bf_pltfm_port_info_t port_info;
  int result;

  UCLI_COMMAND_INFO(
      uc, "led-set", 5, "led-set <dev> <port> <chn> 0x<col> <blink>");

  dev = atoi(uc->pargs->args[0]);
  port = atoi(uc->pargs->args[1]);
  chn = atoi(uc->pargs->args[2]);
  col = strtol(uc->pargs->args[3], NULL, 16);
  blink = atoi(uc->pargs->args[4]);

  aim_printf(&uc->pvs,
             "bf_pltfm_led: led-set <dev=%d> <port=%d> <chn=%d> <col=0x%x> "
             "<blink=%d>\n",
             dev,
             port,
             chn,
             col,
             blink);

  led_blink = blink ? LED_BLINK : LED_NO_BLINK;

  if (port == -1) {
    result = bf_pltfm_port_led_by_tofino_set(dev, NULL, col, led_blink);
  } else {
    port_info.conn_id = port;
    port_info.chnl_id = chn;
    result = bf_pltfm_port_led_by_tofino_set(dev, &port_info, col, led_blink);
  }
  if (result) {
    aim_printf(&uc->pvs, "error setting LED\n");
  } else {
    aim_printf(&uc->pvs, "LED set OK\n");
  }
  return 0;
}

/* set led thru sys cpld */
static ucli_status_t bf_pltfm_ucli_ucli__led_set_(ucli_context_t *uc) {
  bf_dev_id_t dev = 0;
  int port, chn;
  int col;
  int blink;
  bf_led_blink_t led_blink;
  bf_pltfm_port_info_t port_info;
  int result;

  UCLI_COMMAND_INFO(
      uc, "led-set-cpld", 5, "led-set-cpld <dev> <port> <chn> 0x<col> <blink>");

  dev = atoi(uc->pargs->args[0]);
  port = atoi(uc->pargs->args[1]);
  chn = atoi(uc->pargs->args[2]);
  col = strtol(uc->pargs->args[3], NULL, 16);
  blink = atoi(uc->pargs->args[4]);

  aim_printf(
      &uc->pvs,
      "bf_pltfm_led: led-set-cpld <dev=%d> <port=%d> <chn=%d> <col=0x%x> "
      "<blink=%d>\n",
      dev,
      port,
      chn,
      col,
      blink);

  led_blink = blink ? LED_BLINK : LED_NO_BLINK;

  if (port == -1) {
    result = bf_pltfm_port_led_by_cpld_set(dev, NULL, col, led_blink);
  } else {
    port_info.conn_id = port;
    port_info.chnl_id = chn;
    result = bf_pltfm_port_led_by_cpld_set(dev, &port_info, col, led_blink);
  }
  if (result) {
    aim_printf(&uc->pvs, "error setting LED\n");
  } else {
    aim_printf(&uc->pvs, "LED set OK\n");
  }
  return 0;
}

/* <auto.ucli.handlers.start> */
static ucli_command_handler_f bf_pltfm_led_ucli_ucli_handlers__[] = {
    bf_pltfm_ucli_ucli__led_set_tofino_, bf_pltfm_ucli_ucli__led_set_, NULL,
};

/* <auto.ucli.handlers.end> */
static ucli_module_t bf_pltfm_led_ucli_module__ = {
    "led_ucli", NULL, bf_pltfm_led_ucli_ucli_handlers__, NULL, NULL,
};

ucli_node_t *bf_pltfm_led_ucli_node_create(ucli_node_t *m) {
  ucli_node_t *n;
  ucli_module_init(&bf_pltfm_led_ucli_module__);
  n = ucli_node_create("led", m, &bf_pltfm_led_ucli_module__);
  ucli_node_subnode_add(n, ucli_module_log_node_create("led"));
  return n;
}

/* Init function  */
int bf_pltfm_led_init(int chip_id) {
  /* pin pair 0:1 go to upper CPLD, 2:3 go to lower CPLD */
  if (bf_pltfm_tof_i2c_led_init(chip_id, CP2112_ID_1)) {
    LOG_ERROR("error upper i2c_led_init\n");
    return -1;
  }
  if (bf_pltfm_tof_i2c_led_init(chip_id, CP2112_ID_2)) {
    LOG_ERROR("error lower i2c_led_init\n");
    return -1;
  }

  bf_pltfm_led_initialized = 1;

  return (bf_pltfm_led_cpld_init(chip_id));
}
