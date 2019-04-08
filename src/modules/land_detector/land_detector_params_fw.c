/****************************************************************************
 *
 *   Copyright (c) 2014-2017 PX4 Development Team. All rights reserved.
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
walk
max_vxy: 0.894
max_vz: 0.457
max_veli: 10.59

simulation fly
max_vxy: 1.977
max_vz: 0.390
max_veli: 9.017
*/
/**
 * Fixedwing max horizontal velocity
 *
 * Maximum horizontal velocity allowed in the landed state (m/s)
 *
 * @unit m/s
 * @min 0.5
 * @max 10
 * @decimal 1
 * walk max vxy: 0.85
 * @group Land Detector
 */
PARAM_DEFINE_FLOAT(LNDFW_VEL_XY_MAX, 1.8f);

/**
 * Fixedwing max climb rate
 *
 * Maximum vertical velocity allowed in the landed state (m/s up and down)
 *
 * @unit m/s
<<<<<<< HEAD
 * @min 0.1
=======
 * @min 1
>>>>>>> dev
 * @max 20
 * @decimal 1
 * walk max vz: 0.18
 * @group Land Detector
 */
<<<<<<< HEAD
PARAM_DEFINE_FLOAT(LNDFW_VEL_Z_MAX, 3.0f);
=======
PARAM_DEFINE_FLOAT(LNDFW_VEL_Z_MAX, 1.0f);
>>>>>>> dev

/**
 * Fixedwing max horizontal acceleration
 *
 * Maximum horizontal (x,y body axes) acceleration allowed in the landed state (m/s^2)
 *
 * @unit m/s^2
 * @min 2
 * @max 15
 * @decimal 1
 * walk max: 7.9
 * @group Land Detector
 */
<<<<<<< HEAD
PARAM_DEFINE_FLOAT(LNDFW_XYACC_MAX, 8.0f);
=======
PARAM_DEFINE_FLOAT(LNDFW_VELI_MAX, 10.0f);
>>>>>>> dev

/**
 * Airspeed max
 *
 * Maximum airspeed allowed in the landed state (m/s)
 *
 * @unit m/s
 * @min 4
 * @max 20
 * @decimal 1
 *
 * @group Land Detector
 */
PARAM_DEFINE_FLOAT(LNDFW_AIRSPD_MAX, 4.00f);
