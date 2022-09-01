/*******************************************************************************
 * Copyright (c) 2015-2020 Barefoot Networks, Inc.
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * $Id: $
 *
 ******************************************************************************/

#ifndef _BF_PLTFM_SYSCPLD_H
#define _BF_PLTFM_SYSCPLD_H

/* Allow the use in C++ code. */
#ifdef __cplusplus
extern "C" {
#endif

#define BF_MAV_SYSCPLD_I2C_ADDR (0x32 << 1) /* syscpld i2c_addr on cp2112 */

/* syscpld registers */
#define BF_MAV_SYSCPLD_REG_LED_CTRL 0x3c
#define BF_MAV_SYSCPLD_TOF_LED_EN 0x2

#define BF_MAV_SYSCPLD_REG_LED_START 0x80
#define BF_MAV_SYSCPLD_NUM_REG_LED 0x40
#define BF_MAV_SYSCPLD_REG_QSFP_RST_START 0x34

#define BF_MAV_SYSCPLD_REG_CPU_LED_START 0x3E
#define BF_MAV_SYSCPLD_NUM_REG_CPU_LED 0x02

#define QSFP_SUBMODULE_RESET_REG(qsfp) \
  (BF_MAV_SYSCPLD_REG_QSFP_RST_START + (qsfp / 8))

#define QSFP_SUBMODULE_RESET_BIT(qsfp) (qsfp % 8)

#ifdef __cplusplus
}
#endif /* C++ */

#endif /* _BF_PLTFM_SYSCPLD_H */
