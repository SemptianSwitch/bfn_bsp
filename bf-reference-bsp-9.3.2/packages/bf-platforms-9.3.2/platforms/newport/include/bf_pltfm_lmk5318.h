/*******************************************************************************
 * Copyright (c) 2015-2020 Barefoot Networks, Inc.
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * $Id: $
 *
 ******************************************************************************/
#ifndef _BF_PLTFM_SI5342_H
#define _BF_PLTFM_SI5342_H

#include <bfutils/uCli/ucli.h>

/* Allow the use in C++ code. */
#ifdef __cplusplus
extern "C" {
#endif

#define BF_NEWPORT_LMK5318_ADDR 0x64

int bf_pltfm_lmk5318_init(void *arg);
int bf_pltfm_lmk5318_i2c_wr(int chip_id,
                            uint16_t reg,
                            uint8_t *data,
                            size_t cnt);
int bf_pltfm_lmk5318_i2c_rd(int chip_id,
                            uint16_t reg,
                            uint8_t *data,
                            size_t cnt);
ucli_node_t *bf_pltfm_lmk5318_ucli_node_create(ucli_node_t *m);

#ifdef __cplusplus
}
#endif /* C++ */

#endif /* _BF_PLTFM_SI5342_H */
