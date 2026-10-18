/* SPDX-License-Identifier: BSD-3-Clause */

/*
 * Copyright 2022 NXP
 */
#ifndef __LLCP_API_H
#define __LLCP_API_H

#include <stdint.h>

#define LLCP_REG_R 0
#define LLCP_REG_W 1

void vLlcpRegRW(uint16_t *val, uint16_t addr, uint8_t rw);
#endif /* ifndef __LLCP_API_H */
