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

#include <platform.h>

#if defined(SONAR) && defined(USE_SONAR_I2C)

#include "build/build_config.h"
#include "build/debug.h"

#include "system.h"
#include "exti.h"
#include "io.h"
#include "gpio.h"
#include "nvic.h"
#include "bus_i2c.h"


#include "drivers/rangefinder.h"
#include "drivers/sonar_i2c.h"

#define HCSR04_MAX_RANGE_CM 200 // 4m, from HC-SR04 spec sheet
#define HCSR04_DETECTION_CONE_DECIDEGREES 300 // recommended cone angle30 degrees, from HC-SR04 spec sheet
#define HCSR04_DETECTION_CONE_EXTENDED_DECIDEGREES 450 // in practice 45 degrees seems to work well


STATIC_UNIT_TESTED volatile int32_t sonar_distance = -1;
//static uint32_t lastMeasurementAt;

#define I2C_GPS_ADDRESS     0x20 //7 bits       
#define I2C_GPS_STATUS      01
#define I2C_GPS_DATA        02
#define I2C_SONAR_DATA      03

#ifndef SONAR_I2C_INSTANCE
#define SONAR_I2C_INSTANCE I2C_DEVICE
#endif

bool sonar_i2c_module_detect(void)
{
    int16_t distance;
    bool ack = i2cRead(SONAR_I2C_INSTANCE, I2C_GPS_ADDRESS, I2C_SONAR_DATA, sizeof(distance), (uint8_t *)&distance);  
//	debug[1] = ack ? 1 : 2;
//	ack = true;
    return ack;
}

void sonar_i2c_init(struct rangefinder_s *rangeFinder)
{
    rangeFinder->maxRangeCm = HCSR04_MAX_RANGE_CM;
    rangeFinder->detectionConeDeciDegrees = HCSR04_DETECTION_CONE_DECIDEGREES;
    rangeFinder->detectionConeExtendedDeciDegrees = HCSR04_DETECTION_CONE_EXTENDED_DECIDEGREES;
}

void sonar_i2c_start_reading(void)
{
    int16_t distance;
    bool ack = i2cRead(SONAR_I2C_INSTANCE, I2C_GPS_ADDRESS, I2C_SONAR_DATA, sizeof(distance), (uint8_t *)&distance);  
	sonar_distance = ack ? distance : RANGEFINDER_OUT_OF_RANGE;
//	debug[2] = sonar_distance;
//	debug[1]++;
}

/**
 * Get the distance that was measured by the last pulse, in centimeters.
 */
int32_t sonar_i2c_get_distance(void)
{
    return sonar_distance;
}
#endif
