/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 *       AP_Compass_ZERO.cpp - Arduino Library for ZERO I2C magnetometer
 *       Code by Jordi Muñoz and Jose Julio. DIYDrones.com
 *
 *       Sensor is conected to I2C port
 *       Sensor is initialized in Continuos mode (10Hz)
 *
 */

// AVR LibC Includes
#include <AP_Math.h>
#include <AP_HAL.h>

#include "AP_Compass_ZERO.h"

extern const AP_HAL::HAL& hal;




// accumulate a reading from the magnetometer
void AP_Compass_ZERO::accumulate(void)
{
    _motor_offset[0].zero();
    _field[0].zero();
}



// Public Methods //////////////////////////////////////////////////////////////
bool
AP_Compass_ZERO::init()
{
    hal.console->println("AP_Compass_ZERO\n");
	read();

    return true;
}

// Read Sensor data
bool AP_Compass_ZERO::read()
{


    last_update = hal.scheduler->micros(); // record time of update

    
    _motor_offset[0].zero();
    _field[0].zero();

    _healthy[0] = true;

    return true;
}
