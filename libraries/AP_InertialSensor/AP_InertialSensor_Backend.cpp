// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
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

#include <AP_InertialSensor.h>
extern const AP_HAL::HAL& hal;

AP_InertialSensor_Backend::AP_InertialSensor_Backend(AP_InertialSensor &_imu, AP_InertialSensor::IMU_State &_state):
    //port(_port),
    imu(_imu),
    state(_state)
{
}

void
AP_InertialSensor_Backend::init(AP_InertialSensor::Sample_rate sample_rate)
{
    state._product_id = _init_sensor(sample_rate);

    //TODO check scaling (maybe there can be an IMU with more than one accelerometer)
    //for (uint8_t i=0; i<imu.get_accel_count(); i++) {
        if (state._accel_scale.get().is_zero()) {
            state._accel_scale.set(Vector3f(1,1,1));
        }
    //}

}

void
AP_InertialSensor_Backend::init_accel()
{
    //TODO maybe there can be an IMU with more than a accelerometer
    uint8_t num_accels = min(imu.get_accel_count(), INS_MAX_INSTANCES);
    uint8_t flashcount = 0;
    Vector3f prev;
    Vector3f accel_offset;
    float total_change;
    float max_offset;

    //memset(max_offset, 0, sizeof(max_offset));
    //memset(total_change, 0, sizeof(total_change));

    // cold start
    hal.scheduler->delay(100);

    hal.console->print_P(PSTR("Init Accel"));

    // flash leds to tell user to keep the IMU still
    AP_Notify::flags.initialising = true;

    // clear accelerometer offsets and scaling
    //for (uint8_t k=0; k<num_accels; k++) {
        state._accel_offset = Vector3f(0,0,0);
        state._accel_scale = Vector3f(1,1,1);

        // initialise accel offsets to a large value the first time
        // this will force us to calibrate accels at least twice
        accel_offset = Vector3f(500, 500, 500);
    //}

    // loop until we calculate acceptable offsets
    while (true) {
        // get latest accelerometer values
        _update();

        for (uint8_t k=0; k<num_accels; k++) {
            // store old offsets
            prev = accel_offset;

            // get new offsets
            accel_offset = state._accel;
        }

        // We take some readings...
        for(int8_t i = 0; i < 50; i++) {

            hal.scheduler->delay(20);
            _update();

            // low pass filter the offsets
           // for (uint8_t k=0; k<num_accels; k++) {
                accel_offset = accel_offset * 0.9f + state._accel * 0.1f;
            //}

            // display some output to the user
            if(flashcount >= 10) {
                hal.console->print_P(PSTR("*"));
                flashcount = 0;
            }
            flashcount++;
        }

        //for (uint8_t k=0; k<num_accels; k++) {
            // null gravity from the Z accel
            accel_offset.z += GRAVITY_MSS;

            total_change = 
                fabsf(prev.x - accel_offset.x) + 
                fabsf(prev.y - accel_offset.y) + 
                fabsf(prev.z - accel_offset.z);
            max_offset = (accel_offset.x > accel_offset.y) ? accel_offset.x : accel_offset.y;
            max_offset = (max_offset > accel_offset.z) ? max_offset : accel_offset.z;
        //}

        //uint8_t num_converged = 0;
        //for (uint8_t k=0; k<num_accels; k++) {
            if (total_change <= AP_INERTIAL_SENSOR_ACCEL_TOT_MAX_OFFSET_CHANGE && 
                max_offset <= AP_INERTIAL_SENSOR_ACCEL_MAX_OFFSET) {
                //num_converged++;
                break;
            }
        //}

        //if (num_converged == num_accels) break;

        hal.scheduler->delay(500);
    }

    // set the global accel offsets
    //for (uint8_t k=0; k<num_accels; k++) {
        state._accel_offset = accel_offset;
    //}

    imu._save_parameters();
    // stop flashing the leds
    AP_Notify::flags.initialising = false;

    hal.console->print_P(PSTR(" "));

}

