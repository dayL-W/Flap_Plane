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
 * @file FlapwingLandDetector.h
 * Land detector implementation for Flapwing.
 *
 * @author Johan Jansen <jnsn.johan@gmail.com>
 * @author Morten Lysgaard <morten@lysgaard.no>
 * @author Julian Oes <julian@oes.ch>
 */

#pragma once

#include <drivers/drv_hrt.h>
#include <uORB/topics/airspeed.h>
#include <uORB/topics/sensor_bias.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/manual_control_setpoint.h>

#include "LandDetector.h"

using namespace time_literals;

namespace land_detector
{

class FlapwingLandDetector final : public LandDetector
{
public:
        FlapwingLandDetector();

protected:
        void _initialize_topics() override;
        void _update_params() override;
        void _update_topics() override;

        bool _get_landed_state() override;
        float _get_max_altitude() override;

private:

        /** Time in us that landing conditions have to hold before triggering a land. */
        static constexpr hrt_abstime LANDED_TRIGGER_TIME_US = 2_s;
        static constexpr hrt_abstime FLYING_TRIGGER_TIME_US = 0_us;

        struct {
                param_t maxVelocity;
                param_t maxClimbRate;
                param_t maxAirSpeed;
                param_t maxIntVelocity;
        } _paramHandle{};

        struct {
                float maxVelocity;
                float maxClimbRate;
                float maxAirSpeed;
                float maxIntVelocity;
        } _params{};

        int _airspeedSub{-1};
        int _sensor_bias_sub{-1};
        int _local_pos_sub{-1};
        int _manual_sub{-1};

        airspeed_s _airspeed{};
        sensor_bias_s _sensors{};
        vehicle_local_position_s _local_pos{};
        manual_control_setpoint_s _manual{};

        float _velocity_xy_filtered{0.0f};
        float _velocity_z_filtered{0.0f};
        float _airspeed_filtered{0.0f};
        float _accel_horz_lp{0.0f};
        float _manual_throttle{0.0f};
};

} // namespace land_detector
