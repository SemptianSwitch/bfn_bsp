/*******************************************************************************
 * Copyright (c) 2015-2020 Barefoot Networks, Inc.
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * $Id: $
 *
 ******************************************************************************/
#ifndef BF_PLTFM_DIAG_VER_H
#define BF_PLTFM_DIAG_VER_H

#include "bf_pltfm_diag_bld_ver.h"

#define BF_PLTFM_DIAG_REL_VER "2"
#define BF_PLTFM_DIAG_VER BF_PLTFM_DIAG_REL_VER "-" BF_PLTFM_DIAG_BLD_VER

#define BF_PLTFM_DIAG_INTERNAL_VER \
  BF_PLTFM_DIAG_VER "(" BF_PLTFM_DIAG_GIT_VER ")"

#endif /* BF_PLTFM_DIAG_VER_H */
