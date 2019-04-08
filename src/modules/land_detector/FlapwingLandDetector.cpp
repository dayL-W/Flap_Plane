/****************************************************************************
 *
 *   Copyright (c) 2013-2016 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file FlapwingLandDetector.cpp
 *
 * @author Johan Jansen <jnsn.johan@gmail.com>
 * @author Lorenz Meier <lorenz@px4.io>
 * @author Julian Oes <julian@oes.ch>
 */

#include "FlapwingLandDetector.h"

#include <cmath>

#include <px4_config.h>
#include <px4_defines.h>

namespace land_detector
{

FlapwingLandDetector::FlapwingLandDetector()
{
        _paramHandle.maxVelocity = param_find("LNDFW_VEL_XY_MAX");
        _paramHandle.maxClimbRate = param_find("LNDFW_VEL_Z_MAX");
        _paramHandle.maxAirSpeed = param_find("LNDFW_AIRSPD_MAX");
        _paramHandle.maxIntVelocity = param_find("LNDFW_VELI_MAX");

        // Use Trigger time when transitioning from in-air (false) to landed (true) / ground contact (true).
        _landed_hysteresis.set_hysteresis_time_from(false, LANDED_TRIGGER_TIME_US);

        _landed_hysteresis.set_hysteresis_time_from(true, FLYING_TRIGGER_TIME_US);
}

void FlapwingLandDetector::_initialize_topics()
{
        _airspeedSub = orb_subscribe(ORB_ID(airspeed));
        _local_pos_sub = orb_subscribe(ORB_ID(vehicle_local_position));
        _sensor_bias_sub = orb_subscribe(ORB_ID(sensor_bias));
        _manual_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
}

void FlapwingLandDetector::_update_topics()
{
        _orb_update(ORB_ID(airspeed), _airspeedSub, &_airspeed);
        _orb_update(ORB_ID(sensor_bias), _sensor_bias_sub, &_sensors);
        _orb_update(ORB_ID(vehicle_local_position), _local_pos_sub, &_local_pos);
        _orb_update(ORB_ID(manual_control_setpoint), _manual_sub, &_manual);
}

void FlapwingLandDetector::_update_params()
{
        param_get(_paramHandle.maxVelocity, &_params.maxVelocity);
        param_get(_paramHandle.maxClimbRate, &_params.maxClimbRate);
        param_get(_paramHandle.maxAirSpeed, &_params.maxAirSpeed);
        param_get(_paramHandle.maxIntVelocity, &_params.maxIntVelocity);
}

float FlapwingLandDetector::_get_max_altitude()
{
        // TODO
        // This means no altitude limit as the limit
        // is always current position plus 10000 meters
        return roundf(-_local_pos.z + 10000);
}

bool FlapwingLandDetector::_get_landed_state()
{
        // only trigger flight conditions if we are armed
        if (!_arming.armed) {
                return true;
        }

        bool landDetected = false;
        bool flyDetected = false;
        if (hrt_elapsed_time(&_local_pos.timestamp) < 500 * 1000) {

                // horizontal velocity
                float val = 0.97f * _velocity_xy_filtered + 0.03f * sqrtf(_local_pos.vx * _local_pos.vx + _local_pos.vy *
                                _local_pos.vy);

                if (PX4_ISFINITE(val)) {
                        _velocity_xy_filtered = val;
                }

                // vertical velocity
                val = 0.99f * _velocity_z_filtered + 0.01f * fabsf(_local_pos.vz);

                if (PX4_ISFINITE(val)) {
                        _velocity_z_filtered = val;
                }
                if(fabs(_airspeed.true_airspeed_m_s)<100)
                    _airspeed_filtered = 0.95f * _airspeed_filtered + 0.05f * _airspeed.true_airspeed_m_s;

                // a leaking lowpass prevents biases from building up, but
                // gives a mostly correct response for short impulses
                const float acc_hor = sqrtf(_sensors.accel_x * _sensors.accel_x +
                                            _sensors.accel_y * _sensors.accel_y);
                _accel_horz_lp = _accel_horz_lp * 0.8f + acc_hor * 0.18f;

                _manual_throttle = _manual.z;
                // crude land detector for Flapwing
                flyDetected = (_velocity_xy_filtered > 0.7f
                               || _velocity_z_filtered > 0.9f
                               || _airspeed_filtered > 20.0f
                               || _accel_horz_lp > 14.0f)
                               && _manual_throttle > 0.15f;
                landDetected  =! flyDetected;

        } else {
                // Control state topic has timed out and we need to assume we're landed.
                _velocity_xy_filtered = 0;
                _velocity_z_filtered = 0;
                _airspeed_filtered = 0;
                _accel_horz_lp = 0;
                landDetected = true;
        }

        return landDetected;
}

} // namespace land_detector
