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
#include <string.h>
#include <math.h>

#include <platform.h>

#include "build/build_config.h"

#include "common/maths.h"
#include "common/utils.h"

#include "config/feature.h"
#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"

#include "drivers/io.h"
#include "drivers/logging.h"
#include "drivers/sonar_hcsr04.h"
#include "drivers/sonar_srf10.h"
#include "drivers/sonar_i2c.h"
#include "drivers/rangefinder.h"

#include "fc/config.h"
#include "fc/runtime_config.h"

#include "sensors/sensors.h"
#include "sensors/rangefinder.h"
#include "sensors/battery.h"

rangefinder_t rangefinder;

#ifdef SONAR
PG_REGISTER_WITH_RESET_TEMPLATE(rangefinderConfig_t, rangefinderConfig, PG_RANGEFINDER_CONFIG, 0);

PG_RESET_TEMPLATE(rangefinderConfig_t, rangefinderConfig,
    .rangefinder_hardware = RANGEFINDER_NONE,
);

const rangefinderHardwarePins_t * sonarGetHardwarePins(void)
{
    static rangefinderHardwarePins_t rangefinderHardwarePins;

#if defined(SONAR_PWM_TRIGGER_PIN)
    // If we are using softserial, parallel PWM or ADC current sensor, then use motor pins for sonar, otherwise use RC pins
    if (feature(FEATURE_SOFTSERIAL)
            || feature(FEATURE_RX_PARALLEL_PWM )
            || (feature(FEATURE_CURRENT_METER) && batteryConfig()->currentMeterType == CURRENT_SENSOR_ADC)) {
        rangefinderHardwarePins.triggerTag = IO_TAG(SONAR_TRIGGER_PIN_PWM);
        rangefinderHardwarePins.echoTag = IO_TAG(SONAR_ECHO_PIN_PWM);
    } else {
        rangefinderHardwarePins.triggerTag = IO_TAG(SONAR_TRIGGER_PIN);
        rangefinderHardwarePins.echoTag = IO_TAG(SONAR_ECHO_PIN);
    }
#elif defined(SONAR_TRIGGER_PIN)
    rangefinderHardwarePins.triggerTag = IO_TAG(SONAR_TRIGGER_PIN);
    rangefinderHardwarePins.echoTag = IO_TAG(SONAR_ECHO_PIN);
#else
#error Sonar not defined for target
#endif
    return &rangefinderHardwarePins;
}

/*
 * Detect which rangefinder is present
 */
static bool rangefinderDetect(rangefinderDev_t * dev, uint8_t rangefinderHardwareToUse)
{
    rangefinderType_e rangefinderHardware = RANGEFINDER_NONE;
    requestedSensors[SENSOR_INDEX_RANGEFINDER] = rangefinderHardwareToUse;

    switch (rangefinderHardwareToUse) {
#ifdef USE_SONAR_SR04
    case RANGEFINDER_HCSR04:
            {
                const rangefinderHardwarePins_t *sonarHardwarePins = sonarGetHardwarePins();
                if (hcsr04Detect(dev, sonarHardwarePins)) {   // FIXME: Do actual detection if HC-SR04 is plugged in
                    rangefinderHardware = RANGEFINDER_HCSR04;
                }
            }
        break;
#endif
#ifdef USE_SONAR_SRF10
    case RANGEFINDER_SRF10:
            if (srf10Detect(dev)) {
                rangefinderHardware = RANGEFINDER_SRF10;
            }
        break;
#endif
#ifdef USE_SONAR_I2C
    case RANGEFINDER_I2C:
            if (sonarI2cDetect(dev)) {
                rangefinderHardware = RANGEFINDER_I2C;
			}
        break;
#endif    

        case RANGEFINDER_NONE:
            rangefinderHardware = RANGEFINDER_NONE;
            break;
    }

    addBootlogEvent6(BOOT_EVENT_RANGEFINDER_DETECTION, BOOT_EVENT_FLAGS_NONE, rangefinderHardware, 0, 0, 0);

    if (rangefinderHardware == RANGEFINDER_NONE) {
        sensorsClear(SENSOR_SONAR);
        return false;
}

    detectedSensors[SENSOR_INDEX_RANGEFINDER] = rangefinderHardware;
    sensorsSet(SENSOR_SONAR);
    return true;
}

bool rangefinderInit(void)
{
    if (!rangefinderDetect(&rangefinder.dev, rangefinderConfig()->rangefinder_hardware)) {
        return false;
    }

    rangefinder.dev.init();
    rangefinder.rawAltitude = RANGEFINDER_OUT_OF_RANGE;
    rangefinder.calculatedAltitude = RANGEFINDER_OUT_OF_RANGE;
    rangefinder.maxTiltCos = cos_approx(DECIDEGREES_TO_RADIANS(rangefinder.dev.detectionConeExtendedDeciDegrees / 2.0f));

    return true;
}

static int32_t applyMedianFilter(int32_t newReading)
{
    #define DISTANCE_SAMPLES_MEDIAN 5
    static int32_t filterSamples[DISTANCE_SAMPLES_MEDIAN];
    static int filterSampleIndex = 0;
    static bool medianFilterReady = false;

    if (newReading > RANGEFINDER_OUT_OF_RANGE) {// only accept samples that are in range
        filterSamples[filterSampleIndex] = newReading;
        ++filterSampleIndex;
        if (filterSampleIndex == DISTANCE_SAMPLES_MEDIAN) {
            filterSampleIndex = 0;
            medianFilterReady = true;
        }
    }
    return medianFilterReady ? quickMedianFilter5(filterSamples) : newReading;
}

/*
 * This is called periodically by the scheduler
 */
void rangefinderUpdate(void)
{
    if (rangefinder.dev.update) {
        rangefinder.dev.update();
	}
}

/**
 * Get the last distance measured by the sonar in centimeters. When the ground is too far away, RANGEFINDER_OUT_OF_RANGE is returned.
 */
int32_t rangefinderRead(void)
{
    if (rangefinder.dev.read) {
        const int32_t distance = rangefinder.dev.read();
        if (distance >= 0) {
            rangefinder.rawAltitude = applyMedianFilter(distance);
        }
        else {
            rangefinder.rawAltitude = RANGEFINDER_OUT_OF_RANGE;
        }
    }
    else {
        rangefinder.rawAltitude = RANGEFINDER_OUT_OF_RANGE;
    }

    return rangefinder.rawAltitude;
}

/**
 * Apply tilt correction to the given raw sonar reading in order to compensate for the tilt of the craft when estimating
 * the altitude. Returns the computed altitude in centimeters.
 *
 * When the ground is too far away or the tilt is too large, RANGEFINDER_OUT_OF_RANGE is returned.
 */
int32_t rangefinderCalculateAltitude(int32_t rangefinderDistance, float cosTiltAngle)
{
    // calculate sonar altitude only if the ground is in the sonar cone
    if (cosTiltAngle < rangefinder.maxTiltCos || rangefinderDistance == RANGEFINDER_OUT_OF_RANGE) {
        rangefinder.calculatedAltitude = RANGEFINDER_OUT_OF_RANGE;
    } else {
        rangefinder.calculatedAltitude = rangefinderDistance * cosTiltAngle;
    }
    return rangefinder.calculatedAltitude;
}

/**
 * Get the latest altitude that was computed by a call to rangefinderCalculateAltitude(), or RANGEFINDER_OUT_OF_RANGE if sonarCalculateAltitude
 * has never been called.
 */
int32_t rangefinderGetLatestAltitude(void)
{
    return rangefinder.calculatedAltitude;
}

bool rangefinderIsHealthy(void)
{
    return true;
}
#endif

