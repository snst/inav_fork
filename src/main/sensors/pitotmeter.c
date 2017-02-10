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

#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include "platform.h"

#include "build/debug.h"

#include "common/maths.h"

#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"

#include "drivers/logging.h"
#include "drivers/pitotmeter.h"
#include "drivers/pitotmeter_ms4525.h"

#include "fc/config.h"
#include "fc/runtime_config.h"

#include "sensors/pitotmeter.h"
#include "sensors/sensors.h"

pitot_t pitot;

#ifdef PITOT

static float pitotPressureZero = 0;
static uint16_t calibratingP = 0;      // pitot calibration = get new zero pressure value
static float pitotPressure = 0;
static float pitotTemperature = 0;
static float CalibratedAirspeed = 0;

PG_REGISTER_WITH_RESET_TEMPLATE(pitotmeterConfig_t, pitotmeterConfig, PG_PITOTMETER_CONFIG, 0);

#ifdef PITOT
#define PITOT_HARDWARE_DEFAULT    PITOT_AUTODETECT
#else
#define PITOT_HARDWARE_DEFAULT    PITOT_NONE
#endif
PG_RESET_TEMPLATE(pitotmeterConfig_t, pitotmeterConfig,
    .pitot_hardware = PITOT_HARDWARE_DEFAULT,
    .use_median_filtering = 1,
    .pitot_noise_lpf = 0.6f,
    .pitot_scale = 1.00f
);

bool pitotDetect(pitotDev_t *dev, uint8_t pitotHardwareToUse)
{
    pitotSensor_e pitotHardware = PITOT_NONE;
    requestedSensors[SENSOR_INDEX_PITOT] = pitotHardwareToUse;

    switch (pitotHardwareToUse) {
        case PITOT_AUTODETECT:
        case PITOT_MS4525:
#ifdef USE_PITOT_MS4525
            if (ms4525Detect(dev)) {
                pitotHardware = PITOT_MS4525;
                break;
            }
#endif
            /* If we are asked for a specific sensor - break out, otherwise - fall through and continue */
            if (pitotHardwareToUse != PITOT_AUTODETECT) {
                break;
            }

        case PITOT_FAKE:
#ifdef USE_PITOT_FAKE
            if (fakePitotDetect(&pitot)) {
                pitotHardware = PITOT_FAKE;
                break;
            }
#endif
            /* If we are asked for a specific sensor - break out, otherwise - fall through and continue */
            if (pitotHardwareToUse != PITOT_AUTODETECT) {
                break;
            }

        case PITOT_NONE:
            pitotHardware = PITOT_NONE;
            break;
    }

    addBootlogEvent6(BOOT_EVENT_PITOT_DETECTION, BOOT_EVENT_FLAGS_NONE, pitotHardware, 0, 0, 0);

    if (pitotHardware == PITOT_NONE) {
        sensorsClear(SENSOR_PITOT);
        return false;
    }

    detectedSensors[SENSOR_INDEX_PITOT] = pitotHardware;
    sensorsSet(SENSOR_PITOT);
    return true;
}

bool pitotInit(void)
{
    if (!pitotDetect(&pitot.dev, pitotmeterConfig()->pitot_hardware)) {
        return false;
    }
    return true;
}

bool pitotIsCalibrationComplete(void)
{
    return calibratingP == 0;
}

void pitotSetCalibrationCycles(uint16_t calibrationCyclesRequired)
{
    calibratingP = calibrationCyclesRequired;
}

static bool pitotReady = false;

#define PRESSURE_SAMPLES_MEDIAN 3

static int32_t applyPitotmeterMedianFilter(int32_t newPressureReading)
{
    static int32_t barometerFilterSamples[PRESSURE_SAMPLES_MEDIAN];
    static int currentFilterSampleIndex = 0;
    static bool medianFilterReady = false;
    int nextSampleIndex;

    nextSampleIndex = (currentFilterSampleIndex + 1);
    if (nextSampleIndex == PRESSURE_SAMPLES_MEDIAN) {
        nextSampleIndex = 0;
        medianFilterReady = true;
    }

    barometerFilterSamples[currentFilterSampleIndex] = newPressureReading;
    currentFilterSampleIndex = nextSampleIndex;

    if (medianFilterReady)
        return quickMedianFilter3(barometerFilterSamples);
    else
        return newPressureReading;
}

typedef enum {
    PITOTMETER_NEEDS_SAMPLES = 0,
    PITOTMETER_NEEDS_CALCULATION,
} pitotmeterState_e;


bool pitotIsReady(void) {
    return pitotReady;
}

uint32_t pitotUpdate(void)
{
    static pitotmeterState_e state = PITOTMETER_NEEDS_SAMPLES;

    switch (state) {
        default:
        case PITOTMETER_NEEDS_SAMPLES:
            pitot.dev.start();
            state = PITOTMETER_NEEDS_CALCULATION;
            return pitot.dev.delay;
        break;

        case PITOTMETER_NEEDS_CALCULATION:
            pitot.dev.get();
            pitot.dev.calculate(&pitotPressure, &pitotTemperature);
            if (pitotmeterConfig()->use_median_filtering) {
                pitotPressure = applyPitotmeterMedianFilter(pitotPressure);
            }
            state = PITOTMETER_NEEDS_SAMPLES;
           return pitot.dev.delay;
        break;
    }
}

#define P0          101325.0f           // standard pressure
#define CCEXPONENT  0.2857142857f       // exponent of compressibility correction 2/7
#define CASFACTOR   760.8802669f        // sqrt(5) * speed of sound at standard
#define TASFACTOR   0.05891022589f      // 1/sqrt(T0)

static void performPitotCalibrationCycle(void)
{
    pitotPressureZero -= pitotPressureZero / 8;
    pitotPressureZero += pitotPressure / 8;
    calibratingP--;
}

int32_t pitotCalculateAirSpeed(void)
{
    if (pitotIsCalibrationComplete()) {
        const float CalibratedAirspeed_tmp = pitotmeterConfig()->pitot_scale * CASFACTOR * sqrtf(powf(fabsf(pitotPressure - pitotPressureZero) / P0 + 1.0f, CCEXPONENT) - 1.0f);
        CalibratedAirspeed = CalibratedAirspeed * pitotmeterConfig()->pitot_noise_lpf + CalibratedAirspeed_tmp * (1.0f - pitotmeterConfig()->pitot_noise_lpf); // additional LPF to reduce baro noise
        float TrueAirspeed = CalibratedAirspeed * TASFACTOR * sqrtf(pitotTemperature);

        pitot.airSpeed = TrueAirspeed*100;
        //debug[0] = (int16_t)(CalibratedAirspeed*100);
        //debug[1] = (int16_t)(TrueAirspeed*100);
        //debug[2] = (int16_t)((pitotTemperature-273.15f)*100);
        //debug[3] = AirSpeed;
    } else {
        performPitotCalibrationCycle();
        pitot.airSpeed = 0;
    }
    return pitot.airSpeed;
}

bool pitotIsHealthy(void)
{
    return true;
}

#endif /* PITOT */
