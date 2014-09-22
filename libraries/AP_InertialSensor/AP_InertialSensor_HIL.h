/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef __AP_INERTIAL_SENSOR_STUB_H__
#define __AP_INERTIAL_SENSOR_STUB_H__

#include <AP_Progmem.h>
#include "AP_InertialSensor.h"

class AP_InertialSensor_HIL : public AP_InertialSensor_Backend
{
public:

    AP_InertialSensor_HIL(AP_InertialSensor &_imu, AP_InertialSensor::IMU_State &_state);

    /* Concrete implementation of AP_InertialSensor functions: */
    bool            _update();
    float	        get_delta_time() const;
    float           get_gyro_drift_rate(void);
    bool            wait_for_sample(uint16_t timeout_ms);
    void            set_accel(uint8_t instance, const Vector3f &accel);
    void            set_gyro(uint8_t instance, const Vector3f &gyro);
    bool            get_gyro_health(uint8_t instance) const;
    bool            get_accel_health(uint8_t instance) const;
    uint8_t         get_gyro_count(void) const;
    uint8_t         get_accel_count(void) const;

private:
    bool            _sample_available();
    uint16_t        _init_sensor( AP_InertialSensor::Sample_rate sample_rate );
    uint32_t         _sample_period_usec;
    uint32_t        _last_sample_usec;
    uint32_t        _last_accel_usec;
    uint32_t        _last_gyro_usec;
};

#endif // __AP_INERTIAL_SENSOR_STUB_H__
