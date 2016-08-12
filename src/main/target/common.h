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

#define I2C1_OVERCLOCK false
#define I2C2_OVERCLOCK false

#define USE_SERVOS
#define SERIAL_RX
#define USE_CLI

#if (FLASH_SIZE <= 64)
#define SKIP_TASK_STATISTICS
#define SKIP_CLI_COMMAND_HELP
#else
#define BLACKBOX
#define GPS
#define GPS_PROTO_NMEA
#define GPS_PROTO_UBLOX
#define GPS_PROTO_I2C_NAV
#define GPS_PROTO_NAZA

#define TELEMETRY
#define TELEMETRY_FRSKY
#define TELEMETRY_HOTT
#define TELEMETRY_SMARTPORT
#define TELEMETRY_LTM
#endif

#if (FLASH_SIZE > 128)
#define DISPLAY
#define DISPLAY_ARMED_BITMAP
#define TELEMETRY_MAVLINK
#else
#define SKIP_CLI_COMMAND_HELP
#define SKIP_RX_MSP
#define DISABLE_UNCOMMON_MIXERS
#endif

