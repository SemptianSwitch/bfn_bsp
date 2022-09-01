/*******************************************************************************
 * Copyright (c) 2015-2020 Barefoot Networks, Inc.
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * $Id: $
 *
 ******************************************************************************/

#ifndef _BF_MAV_LED_H
#define _BF_MAV_LED_H
#ifdef INC_PLTFM_UCLI
#include <bfutils/uCli/ucli.h>
#endif

/* Allow the use in C++ code. */
#ifdef __cplusplus
extern "C" {
#endif

/* LED colors can be bit-wise ORed */
#define BF_MAV_PORT_LED_OFF 0x0
#define BF_MAV_PORT_LED_RED 0x4
#define BF_MAV_PORT_LED_GREEN 0x2
#define BF_MAV_PORT_LED_BLUE 0x1
#define BF_MAV_PORT_LED_BLINK 0x8

/* display port led using tofino stateout buffer */
int bf_pltfm_port_led_by_tofino_set(int chip_id,
                                    bf_pltfm_port_info_t *port_info,
                                    int led_col,
                                    bf_led_blink_t led_blink);
int bf_pltfm_port_led_by_cpld_set(int chip_id,
                                  bf_pltfm_port_info_t *port_info,
                                  int led_col,
                                  bf_led_blink_t led_blink);
int bf_pltfm_led_cpld_init(int chip_id);

#ifdef INC_PLTFM_UCLI
ucli_node_t *bf_pltfm_led_ucli_node_create(ucli_node_t *m);
#endif

#ifdef __cplusplus
}
#endif /* C++ */

#endif /* _BF_MAV_LED_H */
