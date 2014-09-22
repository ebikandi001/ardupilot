/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
#include "AP_InertialSensor_VRBRAIN.h"

const extern AP_HAL::HAL& hal;

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

#include <drivers/drv_accel.h>
#include <drivers/drv_gyro.h>
#include <drivers/drv_hrt.h>

#include <AP_Notify.h>

AP_InertialSensor_VRBRAIN::AP_InertialSensor_VRBRAIN(AP_InertialSensor &_imu, AP_InertialSensor::IMU_State &_state):
    AP_InertialSensor_Backend(_imu, _state),
    _last_get_sample_timestamp(0),
    _sample_time_usec(0)
{
}



uint16_t AP_InertialSensor_VRBRAIN::_init_sensor( AP_InertialSensor::Sample_rate sample_rate )
{
    // assumes max 2 instances
    _accel_fd = open(ACCEL_DEVICE_PATH, O_RDONLY);
    //_accel_fd = open(ACCEL_DEVICE_PATH "1", O_RDONLY);
    //_accel_fd = open(ACCEL_DEVICE_PATH "2", O_RDONLY);
    _gyro_fd = open(GYRO_DEVICE_PATH, O_RDONLY);
    //_gyro_fd = open(GYRO_DEVICE_PATH "1", O_RDONLY);
    //_gyro_fd = open(GYRO_DEVICE_PATH "2", O_RDONLY);

    _num_accel_instances = 1;//0;
    _num_gyro_instances = 1;//0;
    /*for (uint8_t i=0; i<INS_MAX_INSTANCES; i++) {
        if (_accel_fd[i] >= 0) {
            _num_accel_instances = i+1;
        }
        if (_gyro_fd[i] >= 0) {
            _num_gyro_instances = i+1;
        }
    }    
	if (_num_accel_instances == 0) {
        hal.scheduler->panic("Unable to open accel device " ACCEL_DEVICE_PATH);
    }
	if (_num_gyro_instances == 0) {
        hal.scheduler->panic("Unable to open gyro device " GYRO_DEVICE_PATH);
    }*/

    switch (sample_rate) {
    case AP_InertialSensor::RATE_50HZ:
        _default_filter_hz = 15;
        _sample_time_usec = 20000;
        break;
    case AP_InertialSensor::RATE_100HZ:
        _default_filter_hz = 30;
        _sample_time_usec = 10000;
        break;
    case AP_InertialSensor::RATE_200HZ:
        _default_filter_hz = 30;
        _sample_time_usec = 5000;
        break;
    case AP_InertialSensor::RATE_400HZ:
    default:
        _default_filter_hz = 30;
        _sample_time_usec = 2500;
        break;
    }

    _set_filter_frequency(state._mpu6000_filter);




    return AP_PRODUCT_ID_VRBRAIN;

}

/*
  set the filter frequency
 */
void AP_InertialSensor_VRBRAIN::_set_filter_frequency(uint8_t filter_hz)
{
    if (filter_hz == 0) {
        filter_hz = _default_filter_hz;
    }
   // for (uint8_t i=0; i<_num_gyro_instances; i++) {
        ioctl(_gyro_fd,  GYROIOCSLOWPASS,  filter_hz);
   // }
   // for (uint8_t i=0; i<_num_accel_instances; i++) {
        ioctl(_accel_fd, ACCELIOCSLOWPASS, filter_hz);
   // }
}

/*================ AP_INERTIALSENSOR PUBLIC INTERFACE ==================== */

// multi-device interface
bool AP_InertialSensor_VRBRAIN::get_gyro_health(uint8_t instance) const
{
    if (_sample_time_usec == 0 || _last_get_sample_timestamp == 0) {
        // not initialised yet, show as healthy to prevent scary GCS
        // warnings
        return true;
    }
    if (instance >= _num_gyro_instances) {
        return false;
    }

    if ((_last_get_sample_timestamp - _last_gyro_timestamp) > 2*_sample_time_usec) {
        // gyros have not updated
        return false;
    }
    return true;
}

uint8_t AP_InertialSensor_VRBRAIN::get_gyro_count(void) const
{
    return _num_gyro_instances;
}

bool AP_InertialSensor_VRBRAIN::get_accel_health(uint8_t k) const
{
    if (_sample_time_usec == 0 || _last_get_sample_timestamp == 0) {
        // not initialised yet, show as healthy to prevent scary GCS
        // warnings
        return true;
    }
    if (k >= _num_accel_instances) {
        return false;
    }

    if ((_last_get_sample_timestamp - _last_accel_timestamp[k]) > 2*_sample_time_usec) {
        // accels have not updated
        return false;
    }
    if (fabsf(state._accel.x) > 30 && fabsf(state._accel.y) > 30 && fabsf(state._accel.z) > 30 &&
        (state._previous_accel - state._accel).length() < 0.01f) {
        // unchanging accel, large in all 3 axes. This is a likely
        // accelerometer failure of the LSM303d
        return false;
    }
    return true;
    
}

uint8_t AP_InertialSensor_VRBRAIN::get_accel_count(void) const
{
    return _num_accel_instances;
}

bool AP_InertialSensor_VRBRAIN::_update(void)
{
    if (!wait_for_sample(100)) {
        return false;
    }

    // get the latest sample from the sensor drivers
    _get_sample();


    //for (uint8_t k=0; k<_num_accel_instances; k++) {
        state._previous_accel = state._accel;
        state._accel = _accel_in;
        state._accel.rotate(state._board_orientation);
        state._accel.x *= state._accel_scale.get().x;
        state._accel.y *= state._accel_scale.get().y;
        state._accel.z *= state._accel_scale.get().z;
        state._accel   -= state._accel_offset;
    //}

    //for (uint8_t k=0; k<_num_gyro_instances; k++) {
        state._gyro = _gyro_in;
        state._gyro.rotate(state._board_orientation);
        state._gyro -= state._gyro_offset;
    //}

    if (_last_filter_hz != state._mpu6000_filter) {
        _set_filter_frequency(state._mpu6000_filter);
        _last_filter_hz = state._mpu6000_filter;
    }

    _have_sample_available = false;

    return true;
}

float AP_InertialSensor_VRBRAIN::get_delta_time(void) const
{
    return _sample_time_usec * 1.0e-6f;
}

float AP_InertialSensor_VRBRAIN::get_gyro_drift_rate(void)
{
    // assume 0.5 degrees/second/minute
    return ToRad(0.5/60);
}

void AP_InertialSensor_VRBRAIN::_get_sample(void)
{
    //for (uint8_t i=0; i<_num_accel_instances; i++) {
        struct accel_report	accel_report;
        while (_accel_fd != -1 && 
               ::read(_accel_fd, &accel_report, sizeof(accel_report)) == sizeof(accel_report) &&
               accel_report.timestamp != _last_accel_timestamp) {        
            _accel_in = Vector3f(accel_report.x, accel_report.y, accel_report.z);
            _last_accel_timestamp = accel_report.timestamp;
        }
    //}
    //for (uint8_t i=0; i<_num_gyro_instances; i++) {
        struct gyro_report	gyro_report;
        while (_gyro_fd != -1 && 
               ::read(_gyro_fd, &gyro_report, sizeof(gyro_report)) == sizeof(gyro_report) &&
               gyro_report.timestamp != _last_gyro_timestamp) {        
            _gyro_in = Vector3f(gyro_report.x, gyro_report.y, gyro_report.z);
            _last_gyro_timestamp = gyro_report.timestamp;
        }
    //}
    _last_get_sample_timestamp = hal.scheduler->micros64();
}

bool AP_InertialSensor_VRBRAIN::_sample_available(void)
{
    uint64_t tnow = hal.scheduler->micros64();
    while (tnow - _last_sample_timestamp > _sample_time_usec) {
        _have_sample_available = true;
        _last_sample_timestamp += _sample_time_usec;
    }
    return _have_sample_available;
}

bool AP_InertialSensor_VRBRAIN::wait_for_sample(uint16_t timeout_ms)
{
    if (_sample_available()) {
        return true;
    }
    uint32_t start = hal.scheduler->millis();
    while ((hal.scheduler->millis() - start) < timeout_ms) {
        uint64_t tnow = hal.scheduler->micros64();
        // we spin for the last timing_lag microseconds. Before that
        // we yield the CPU to allow IO to happen
        const uint16_t timing_lag = 400;
        if (_last_sample_timestamp + _sample_time_usec > tnow+timing_lag) {
            hal.scheduler->delay_microseconds(_last_sample_timestamp + _sample_time_usec - (tnow+timing_lag));
        }
        if (_sample_available()) {
            return true;
        }
    }
    return false;
}

/**
   try to detect bad accel/gyro sensors
 */
bool AP_InertialSensor_VRBRAIN::healthy(void) const
{
    return get_gyro_health(0) && get_accel_health(0);
}

uint8_t AP_InertialSensor_VRBRAIN::_get_primary_gyro(void) const
{
    for (uint8_t i=0; i<_num_gyro_instances; i++) {
        if (get_gyro_health(i)) return i;
    }    
    return 0;
}

uint8_t AP_InertialSensor_VRBRAIN::get_primary_accel(void) const
{
    for (uint8_t i=0; i<_num_accel_instances; i++) {
        if (get_accel_health(i)) return i;
    }    
    return 0;
}

#endif // CONFIG_HAL_BOARD

