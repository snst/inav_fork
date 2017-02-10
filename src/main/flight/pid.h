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

#include "config/parameter_group.h"
#include "fc/runtime_config.h"

#define GYRO_SATURATION_LIMIT   1800        // 1800dps
#define PID_SUM_LIMIT_MIN       100
#define PID_SUM_LIMIT_MAX       1000
#define PID_SUM_LIMIT_DEFAULT   500
#define YAW_P_LIMIT_MIN 100                 // Maximum value for yaw P limiter
#define YAW_P_LIMIT_MAX 500                 // Maximum value for yaw P limiter
#define YAW_P_LIMIT_DEFAULT 300             // Default value for yaw P limiter

#define FW_ITERM_THROW_LIMIT_DEFAULT 165
#define FW_ITERM_THROW_LIMIT_MIN 0
#define FW_ITERM_THROW_LIMIT_MAX 500

#define AXIS_ACCEL_MIN_LIMIT    50

#define MAG_HOLD_ERROR_LPF_FREQ 2

typedef enum {
    /* PID              MC      FW  */
    PID_ROLL,       //   +       +
    PID_PITCH,      //   +       +
    PID_YAW,        //   +       +
    PID_POS_Z,      //   +       +
    PID_POS_XY,     //   +       +
    PID_VEL_XY,     //   +       n/a
    PID_SURFACE,    //   n/a     n/a
    PID_LEVEL,      //   +       +
    PID_HEADING,    //   +       +
    PID_VEL_Z,      //   +       n/a
    PID_ITEM_COUNT
} pidIndex_e;

typedef struct pid8_s {
    uint8_t P;
    uint8_t I;
    uint8_t D;
} pid8_t;

typedef struct pidBank_s {
    pid8_t  pid[PID_ITEM_COUNT];
} pidBank_t;

typedef struct pidProfile_s {
    pidBank_t bank_fw;
    pidBank_t bank_mc;

    uint16_t dterm_soft_notch_hz;           // Dterm Notch frequency
    uint16_t dterm_soft_notch_cutoff;       // Dterm Notch Cutoff frequency
    uint8_t dterm_lpf_hz;                   // (default 17Hz, Range 1-50Hz) Used for PT1 element in PID1, PID2 and PID5

    uint8_t yaw_pterm_lpf_hz;               // Used for filering Pterm noise on noisy frames
    uint8_t acc_soft_lpf_hz;                // Set the Low Pass Filter factor for ACC. Reducing this value would reduce ACC noise (visible in GUI), but would increase ACC lag time. Zero = no filter
    uint8_t yaw_lpf_hz;
    uint16_t yaw_p_limit;

    uint16_t rollPitchItermIgnoreRate;      // Experimental threshold for ignoring iterm for pitch and roll on certain rates
    uint16_t yawItermIgnoreRate;            // Experimental threshold for ignoring iterm for yaw on certain rates

    uint32_t axisAccelerationLimitYaw;          // Max rate of change of yaw angular rate setpoint (deg/s^2 = dps/s)
    uint32_t axisAccelerationLimitRollPitch;    // Max rate of change of roll/pitch angular rate setpoint (deg/s^2 = dps/s)

    int16_t max_angle_inclination[ANGLE_INDEX_COUNT];       // Max possible inclination (roll and pitch axis separately

    float dterm_setpoint_weight;

    uint16_t fixedWingItermThrowLimit;
    uint16_t pidSumLimit;
} pidProfile_t;

PG_DECLARE_PROFILE(pidProfile_t, pidProfile);

static inline const pidBank_t * pidBank() { return STATE(FIXED_WING) ? &pidProfile()->bank_fw : &pidProfile()->bank_mc; }
static inline pidBank_t * pidBankMutable() { return STATE(FIXED_WING) ? &pidProfileMutable()->bank_fw : &pidProfileMutable()->bank_mc; }

extern int16_t axisPID[];
extern int32_t axisPID_P[], axisPID_I[], axisPID_D[], axisPID_Setpoint[];

void pidInit(void);

#ifdef USE_DTERM_NOTCH
bool pidInitFilters(void);
#endif

void pidResetErrorAccumulators(void);

struct controlRateConfig_s;
struct motorConfig_s;
struct rxConfig_s;

void schedulePidGainsUpdate(void);
void updatePIDCoefficients(void);
void pidController(void);

float pidRateToRcCommand(float rateDPS, uint8_t rate);
int16_t pidAngleToRcCommand(float angleDeciDegrees, int16_t maxInclination);

enum {
    MAG_HOLD_DISABLED = 0,
    MAG_HOLD_UPDATE_HEADING,
    MAG_HOLD_ENABLED
};

void updateMagHoldHeading(int16_t heading);
void resetMagHoldHeading(int16_t heading);
int16_t getMagHoldHeading();
