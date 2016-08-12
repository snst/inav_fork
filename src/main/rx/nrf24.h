/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "rx/rx.h"

typedef enum {
    NRF24RX_V202_250K = 0,
    NRF24RX_V202_1M,
    NRF24RX_SYMA_X,
    NRF24RX_SYMA_X5C,
    NRF24RX_CX10,
    NRF24RX_CX10A,
    NRF24RX_H8_3D,
    NRF24RX_INAV,
    NRF24RX_PROTOCOL_COUNT
} nrf24_protocol_t;

typedef enum {
    NRF24_RECEIVED_NONE = 0,
    NRF24_RECEIVED_BIND,
    NRF24_RECEIVED_DATA
} nrf24_received_t;

// RC channels in AETR order
typedef enum {
    NRF24_ROLL = 0,
    NRF24_PITCH,
    NRF24_THROTTLE,
    NRF24_YAW,
    NRF24_AUX1,
    NRF24_AUX2,
    NRF24_AUX3,
    NRF24_AUX4,
    NRF24_AUX5,
    NRF24_AUX6,
    NRF24_AUX7,
    NRF24_AUX8,
    NRF24_AUX9,
    NRF24_AUX10,
    NRF24_AUX11,
    NRF24_AUX12,
    NRF24_AUX13,
    NRF24_AUX14
} nrf24_AETR_t;

// RC channels as used by deviation
#define RC_CHANNEL_RATE        NRF24_AUX1
#define RC_CHANNEL_FLIP        NRF24_AUX2
#define RC_CHANNEL_PICTURE     NRF24_AUX3
#define RC_CHANNEL_VIDEO       NRF24_AUX4
#define RC_CHANNEL_HEADLESS    NRF24_AUX5
#define RC_CHANNEL_RTH         NRF24_AUX6 // return to home

bool rxNrf24DataReceived(void);
struct rxConfig_s;
struct rxRuntimeConfig_s;
bool rxNrf24Init(nfr24l01_spi_type_e spiType, const struct rxConfig_s *rxConfig, struct rxRuntimeConfig_s *rxRuntimeConfig, rcReadRawDataPtr *callback);
