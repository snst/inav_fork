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

#include "build/build_config.h"


#include "gps_i2cnav.h"

#include "gpio.h"
#include "system.h"
#include "bus_i2c.h"

#ifndef GPS_I2C_INSTANCE
#define GPS_I2C_INSTANCE I2C_DEVICE
#endif



#define I2C_GPS_ADDRESS    0x20
#define I2C_GPS_STATUS_00      01
#define I2C_GPS_DATA      02
#define I2C_SONAR_DATA    03

typedef struct 
{
  uint8_t    new_data:1;
  uint8_t    gps2dfix:1;
  uint8_t    gps3dfix:1;
  uint8_t    reserved:1;
  uint8_t    numsats:4;
} __attribute__ ((__packed__))  STATUS_REGISTER;


typedef struct 
{
    uint32_t              lat;
    uint32_t              lon;
    uint16_t              ground_speed;             // ground speed from gps m/s*100
    uint16_t              ground_course;            // GPS ground course
    int16_t               altitude;                 // gps altitude
    uint16_t              hdop;
    uint32_t              time;
    STATUS_REGISTER       status;                   // 0x00  status register
} __attribute__ ((__packed__))  I2C_REGISTERS;


#if 0
#define I2C_GPS_ADDRESS               0x20 //7 bits   

#define I2C_GPS_STATUS_00             00    //(Read only)
  #define I2C_GPS_STATUS_NEW_DATA       0x01  // New data is available (after every GGA frame)
  #define I2C_GPS_STATUS_2DFIX          0x02  // 2dfix achieved
  #define I2C_GPS_STATUS_3DFIX          0x04  // 3dfix achieved
  #define I2C_GPS_STATUS_NUMSATS        0xF0  // Number of sats in view
#define I2C_GPS_REG_VERSION           03   // Version of the I2C_NAV SW uint8_t
#define I2C_GPS_LOCATION              07    // current location 8 byte (lat, lon) int32_t
#define I2C_GPS_GROUND_SPEED          31    // GPS ground speed in m/s*100 (uint16_t)      (Read Only)
#define I2C_GPS_ALTITUDE              33    // GPS altitude in meters (uint16_t)           (Read Only)
#define I2C_GPS_GROUND_COURSE         35    // GPS ground course (uint16_t)
#define I2C_GPS_TIME                  39    // UTC Time from GPS in hhmmss.sss * 100 (uint32_t)(unneccesary precision) (Read Only)
#endif

bool i2cnavGPSModuleDetect(void)
{
    bool ack;
    uint8_t i2cGpsStatus;

    ack = i2cRead(GPS_I2C_INSTANCE, I2C_GPS_ADDRESS, I2C_GPS_STATUS_00, 1, &i2cGpsStatus); /* status register */

    if (ack) 
        return true;

    return false;
}

void i2cnavGPSModuleRead(gpsDataI2CNAV_t * gpsMsg)
{
    bool ack;
    I2C_REGISTERS i2cGpsData;
	uint8_t* data = (uint8_t*) &i2cGpsData;
    
    gpsMsg->flags.newData = 0;
    gpsMsg->flags.fix3D = 0;
    gpsMsg->flags.gpsOk = 0;
    
    ack = i2cRead(GPS_I2C_INSTANCE, I2C_GPS_ADDRESS, I2C_GPS_DATA, sizeof(I2C_REGISTERS), data); 

    if (!ack)
	{
        return;
	}

    gpsMsg->flags.gpsOk = 1;
	gpsMsg->flags.newData = i2cGpsData.status.new_data;
    gpsMsg->numSat = i2cGpsData.status.numsats;
	gpsMsg->flags.fix3D = i2cGpsData.status.gps3dfix;
	gpsMsg->latitude = i2cGpsData.lat;
	gpsMsg->longitude = i2cGpsData.lon;
	gpsMsg->speed = i2cGpsData.ground_speed;
	gpsMsg->ground_course = i2cGpsData.ground_course;
	gpsMsg->altitude = i2cGpsData.altitude;
	gpsMsg->hdop = i2cGpsData.hdop;
	//	debug[2] = i2cGpsData.time;
}



#if 0
void i2cnavGPSModuleRead(gpsDataI2CNAV_t * gpsMsg)
{
    gpsMsg->flags.newData = 0;
    gpsMsg->flags.fix3D = 0;
    gpsMsg->flags.gpsOk = 0;

    uint8_t i2cGpsStatus;
    bool ack = i2cRead(GPS_I2C_INSTANCE, I2C_GPS_ADDRESS, I2C_GPS_STATUS_00, 1, &i2cGpsStatus); /* status register */

    if (!ack)
        return;

    gpsMsg->flags.gpsOk = 1;
    gpsMsg->numSat = i2cGpsStatus >> 4;

    if (i2cGpsStatus & I2C_GPS_STATUS_3DFIX) {
        gpsMsg->flags.fix3D = 1;

        if (i2cGpsStatus & I2C_GPS_STATUS_NEW_DATA) {   
            i2cRead(GPS_I2C_INSTANCE, I2C_GPS_ADDRESS, I2C_GPS_LOCATION,      4, (uint8_t*)&gpsMsg->latitude);
            i2cRead(GPS_I2C_INSTANCE, I2C_GPS_ADDRESS, I2C_GPS_LOCATION + 4,  4, (uint8_t*)&gpsMsg->longitude);
            i2cRead(GPS_I2C_INSTANCE, I2C_GPS_ADDRESS, I2C_GPS_GROUND_SPEED,  2, (uint8_t*)&gpsMsg->speed);
            i2cRead(GPS_I2C_INSTANCE, I2C_GPS_ADDRESS, I2C_GPS_GROUND_COURSE, 2, (uint8_t*)&gpsMsg->ground_course);
            i2cRead(GPS_I2C_INSTANCE, I2C_GPS_ADDRESS, I2C_GPS_ALTITUDE,      2, (uint8_t*)&gpsMsg->altitude);

            gpsMsg->hdop = 0;

            gpsMsg->flags.newData = 1;
        }
    }
}
#endif