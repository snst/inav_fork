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


/*
 * Detect which rangefinder is present
 */
rangefinderType_e rangefinderDetect(void)
{
    rangefinderType_e rangefinderType = RANGEFINDER_NONE;
    if (feature(FEATURE_SONAR)) {
        // the user has set the sonar feature, so assume they have an HC-SR04 plugged in,
        // since there is no way to detect it
        rangefinderType = RANGEFINDER_I2C;
    }
#ifdef USE_SONAR_SRF10
    if (srf10_detect()) {
        // if an SFR10 sonar rangefinder is detected then use it in preference to the assumed HC-SR04
        rangefinderType = RANGEFINDER_SRF10;
    }
#endif

    addBootlogEvent6(BOOT_EVENT_RANGEFINDER_DETECTION, BOOT_EVENT_FLAGS_NONE, rangefinderType, 0, 0, 0);

    requestedSensors[SENSOR_INDEX_RANGEFINDER] = rangefinderType;   // FIXME: Make rangefinder type selectable from CLI
    detectedSensors[SENSOR_INDEX_RANGEFINDER] = rangefinderType;

    if (rangefinderType != RANGEFINDER_NONE) {
        sensorsSet(SENSOR_SONAR);
    }

    return rangefinderType;
}

static const sonarHcsr04Hardware_t *sonarGetHardwareConfigurationForHCSR04(currentSensor_e currentSensor)
{
#if defined(SONAR_PWM_TRIGGER_PIN)
#endif
#if !defined(UNIT_TEST)
#endif
#if defined(UNIT_TEST)
   UNUSED(currentSensor);
   return 0;
#elif defined(SONAR_PWM_TRIGGER_PIN)
    // If we are using softserial, parallel PWM or ADC current sensor, then use motor pins for sonar, otherwise use RC pins
    if (feature(FEATURE_SOFTSERIAL)
            || feature(FEATURE_RX_PARALLEL_PWM )
            || (feature(FEATURE_CURRENT_METER) && currentSensor == CURRENT_SENSOR_ADC)) {
        sonarHcsr04Hardware.triggerTag = IO_TAG(SONAR_TRIGGER_PIN_PWM);
        sonarHcsr04Hardware.echoTag = IO_TAG(SONAR_ECHO_PIN_PWM);
    } else {
        sonarHcsr04Hardware.triggerTag = IO_TAG(SONAR_TRIGGER_PIN);
        sonarHcsr04Hardware.echoTag = IO_TAG(SONAR_ECHO_PIN);
    }
#elif defined(SONAR_TRIGGER_PIN)
    UNUSED(currentSensor);
    sonarHcsr04Hardware.triggerTag = IO_TAG(SONAR_TRIGGER_PIN);
    sonarHcsr04Hardware.echoTag = IO_TAG(SONAR_ECHO_PIN);
#else
#error Sonar not defined for target
#endif
    return &sonarHcsr04Hardware;
}

STATIC_UNIT_TESTED void rangefinderSetFunctionPointers(rangefinderType_e rangefinderType)
{

    switch (rangefinderType) {
    default:
    case RANGEFINDER_NONE:
        break;
#ifdef USE_SONAR_SR04
    case RANGEFINDER_HCSR04:
        rangefinderFunctionPointers.init = hcsr04_init;
        rangefinderFunctionPointers.update = hcsr04_start_reading;
        rangefinderFunctionPointers.read = hcsr04_get_distance;
        break;
#endif
#ifdef USE_SONAR_SRF10
    case RANGEFINDER_SRF10:
        rangefinderFunctionPointers.init = srf10_init;
        rangefinderFunctionPointers.update = srf10_start_reading;
        rangefinderFunctionPointers.read = srf10_get_distance;
        break;
#endif
#ifdef USE_SONAR_I2C
    case RANGEFINDER_I2C:
        rangefinderFunctionPointers.init = sonar_i2c_init;
        rangefinderFunctionPointers.update = sonar_i2c_start_reading;
        rangefinderFunctionPointers.read = sonar_i2c_get_distance;
        break;
#endif    
	}
}

/*
 * Get the HCSR04 sonar hardware configuration.
 * NOTE: sonarInit() must be subsequently called before using any of the sonar functions.
 */
/*
const sonarHcsr04Hardware_t *sonarGetHardwareConfiguration(currentSensor_e currentSensor)
{
    // Return the configuration for the HC-SR04 hardware.
    // Unfortunately the I2C bus is not initialised at this point
    // so cannot detect if another sonar device is present
    return sonarGetHardwareConfigurationForHCSR04(currentSensor);
}
*/
void rangefinderInit(rangefinderType_e rangefinderType)
{
    calculatedAltitude = RANGEFINDER_OUT_OF_RANGE;
#ifdef USE_SONAR_SR04
    if (rangefinderType == RANGEFINDER_HCSR04) {
        hcsr04_set_sonar_hardware();
    }
#endif	
#ifndef UNIT_TEST
    rangefinderSetFunctionPointers(rangefinderType);
#endif

    rangefinder_t rangefinder;
    if (rangefinderType == RANGEFINDER_NONE) {
        memset(&rangefinder, 0, sizeof(rangefinder));
    } else {
        rangefinderFunctionPointers.init(&rangefinder);
    }
    rangefinderMaxRangeCm = rangefinder.maxRangeCm;
    rangefinderCfAltCm = rangefinderMaxRangeCm / 2;
    rangefinderMaxTiltDeciDegrees =  rangefinder.detectionConeExtendedDeciDegrees / 2;
    rangefinderMaxTiltCos = cos_approx(rangefinderMaxTiltDeciDegrees / 10.0f * RAD);
    rangefinderMaxAltWithTiltCm = rangefinderMaxRangeCm * rangefinderMaxTiltCos;
}


static int32_t applyMedianFilter(int32_t newReading)
{
    #define DISTANCE_SAMPLES_MEDIAN 5
    static int32_t filterSamples[DISTANCE_SAMPLES_MEDIAN];
    static int filterSampleIndex = 0;

    filterSamples[filterSampleIndex] = newReading;
	filterSampleIndex = (filterSampleIndex + 1) % DISTANCE_SAMPLES_MEDIAN;
    return quickMedianFilter5(filterSamples);
}

/*
 * This is called periodically by the scheduler
 */
void rangefinderUpdate(void)
{
    if (rangefinderFunctionPointers.update) {
        rangefinderFunctionPointers.update();
        int32_t distance = rangefinderFunctionPointers.read();
		if(distance>rangefinderMaxRangeCm)
		{
			distance = RANGEFINDER_OUT_OF_RANGE;
		}
        readAlt = applyMedianFilter(distance);
	}
}

/**
 * Get the last distance measured by the sonar in centimeters. When the ground is too far away, RANGEFINDER_OUT_OF_RANGE is returned.
 */
int32_t rangefinderRead(void)
{
	return readAlt;
	/*
    if (rangefinderFunctionPointers.read) {
        const int32_t distance = rangefinderFunctionPointers.read();
        return applyMedianFilter(distance);
    }
    return 0;*/
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

