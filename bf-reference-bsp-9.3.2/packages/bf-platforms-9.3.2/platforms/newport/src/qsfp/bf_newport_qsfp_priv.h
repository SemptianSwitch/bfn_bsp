/*******************************************************************************
 * Copyright (c) 2015-2020 Barefoot Networks, Inc.
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * $Id: $
 *
 ******************************************************************************/

#ifndef _BF_NEWPORT_QSFP_H_
#define _BF_NEWPORT_QSFP_H_

#include <bf_pltfm_fpga.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

// Standard
#define QSFP_I2C_ADDR 0x50

#define NP_FPGA_QSFP_GPIO_I2CMUX_ADDR BF_NEWPORT_FPGA_MISC_MUX_ADDR
#define NP_FPGA_QSFP_GPIO_I2CBUS_ADDR BF_NEWPORT_FPGA_BUS_MISC
#define NP_FPGA_QSFP_I2CBUS_ADDR(port) ((port)-1)  // 0 to 31 for qsfp1 to 32
#define NP_MAX_NUM_OF_QSFP 32

#define NP_QSFP_WR_ACCESS_SLEEP_TIME 20000
#define NP_QSFP_RD_ACCESS_SLEEP_TIME 5000

// pca9548 channels on mux 0x74
#define GPIO_NP_GET_MUX_CHAN_FOR_QSFP_LP(port)    \
  ((port) > 16 ? BF_NEWPORT_FPGA_MUX_CHN_QSFP_LP1 \
               : BF_NEWPORT_FPGA_MUX_CHN_QSFP_LP0)

#define GPIO_NP_GET_MUX_CHAN_FOR_QSFP_PRSNT(port)    \
  ((port) > 16 ? BF_NEWPORT_FPGA_MUX_CHN_QSFP_PRSNT1 \
               : BF_NEWPORT_FPGA_MUX_CHN_QSFP_PRSNT0)

#define GPIO_NP_GET_MUX_CHAN_FOR_QSFP_INT(port)    \
  ((port) > 16 ? BF_NEWPORT_FPGA_MUX_CHN_QSFP_INT1 \
               : BF_NEWPORT_FPGA_MUX_CHN_QSFP_INT0)

#define GPIO_NP_GET_MUX_CHAN_FOR_QSFP_RST(port)    \
  ((port) > 16 ? BF_NEWPORT_FPGA_MUX_CHN_QSFP_RST1 \
               : BF_NEWPORT_FPGA_MUX_CHN_QSFP_RST0)

// pca9548 slave address
#define GPIO_NP_GET_I2CADDR_FOR_QSFP_LP(port)       \
  (((port) > 16) ? BF_NEWPORT_FPGA_I2CADDR_QSFP_LP1 \
                 : BF_NEWPORT_FPGA_I2CADDR_QSFP_LP0)

#define GPIO_NP_GET_I2CADDR_FOR_QSFP_PRSNT(port)       \
  (((port) > 16) ? BF_NEWPORT_FPGA_I2CADDR_QSFP_PRSNT1 \
                 : BF_NEWPORT_FPGA_I2CADDR_QSFP_PRSNT0)

#define GPIO_NP_GET_I2CADDR_FOR_QSFP_INT(port)       \
  (((port) > 16) ? BF_NEWPORT_FPGA_I2CADDR_QSFP_INT1 \
                 : BF_NEWPORT_FPGA_I2CADDR_QSFP_INT0)

#define GPIO_NP_GET_I2CADDR_FOR_QSFP_RST(port)       \
  (((port) > 16) ? BF_NEWPORT_FPGA_I2CADDR_QSFP_RST1 \
                 : BF_NEWPORT_FPGA_I2CADDR_QSFP_RST0)

typedef struct _qsfp_i2c_msg_ {
  uint8_t mux_addr;
  uint8_t mux_channel;
  int delay;
  bool mux_present;
} qsfp_i2c_msg_t;

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* !_BF_NEWPORT_QSFP_H_ */
