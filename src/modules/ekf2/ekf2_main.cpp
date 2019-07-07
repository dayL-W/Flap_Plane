/****************************************************************************
 *
 *   Copyright (c) 2015-2017 PX4 Development Team. All rights reserved.
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
 * @file ekf2_main.cpp
 * Implementation of the attitude and position estimator.
 *
 * @author Roman Bapst
 */

#include <cfloat>

#include <drivers/drv_hrt.h>
#include <ecl/EKF/ekf.h>
#include <mathlib/mathlib.h>
#include <px4_defines.h>
#include <px4_module.h>
#include <px4_module_params.h>
#include <px4_posix.h>
#include <px4_tasks.h>
#include <px4_time.h>
#include <uORB/Publication.hpp>
#include <uORB/topics/airspeed.h>
#include <uORB/topics/distance_sensor.h>
#include <uORB/topics/ekf2_innovations.h>
#include <uORB/topics/ekf2_timestamps.h>
#include <uORB/topics/estimator_status.h>
#include <uORB/topics/optical_flow.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_bias.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/sensor_selection.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/wind_estimate.h>
#include <uORB/topics/landing_target_pose.h>
#include <uORB/topics/vehicle_air_data.h>
#include <uORB/topics/vehicle_magnetometer.h>

using math::constrain;

extern "C" __EXPORT int ekf2_main(int argc, char *argv[]);

class Ekf2 final : public ModuleBase<Ekf2>, public ModuleParams
{
public:
	Ekf2();
	~Ekf2() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static Ekf2 *instantiate(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::run() */
	void run() override;

	void set_replay_mode(bool replay) { _replay_mode = replay; }

	static void	task_main_trampoline(int argc, char *argv[]);

	int print_status() override;

private:
	int getRangeSubIndex(const int *subs); ///< get subscription index of first downward-facing range sensor

	template<typename Param>
	void update_mag_bias(Param &mag_bias_param, int axis_index);

	bool publish_wind_estimate(const hrt_abstime &timestamp);

	const Vector3f get_vel_body_wind();

	bool 	_replay_mode = false;			///< true when we use replay data from a log

	// time slip monitoring
	uint64_t _integrated_time_us = 0;	///< integral of gyro delta time from start (uSec)
	uint64_t _start_time_us = 0;		///< system time at EKF start (uSec)
	int64_t _last_time_slip_us = 0;		///< Last time slip (uSec)

	// Initialise time stamps used to send sensor data to the EKF and for logging
	uint8_t _invalid_mag_id_count = 0;	///< number of times an invalid magnetomer device ID has been detected

	// Used to down sample magnetometer data
	float _mag_data_sum[3] = {};			///< summed magnetometer readings (Gauss)
	uint64_t _mag_time_sum_ms = 0;		///< summed magnetoemter time stamps (mSec)
	uint8_t _mag_sample_count = 0;		///< number of magnetometer measurements summed during downsampling
	uint32_t _mag_time_ms_last_used =
		0;	///< time stamp of the last averaged magnetometer measurement sent to the EKF (mSec)

	// Used to down sample barometer data
	float _balt_data_sum = 0.0f;			///< summed pressure altitude readings (m)
	uint64_t _balt_time_sum_ms = 0;		///< summed pressure altitude time stamps (mSec)
	uint8_t _balt_sample_count = 0;		///< number of barometric altitude measurements summed
	uint32_t _balt_time_ms_last_used =
		0;	///< time stamp of the last averaged barometric altitude measurement sent to the EKF (mSec)

	// Used to check, save and use learned magnetometer biases
	hrt_abstime _last_magcal_us = 0;	///< last time the EKF was operating a mode that estimates magnetomer biases (uSec)
	hrt_abstime _total_cal_time_us = 0;	///< accumulated calibration time since the last save

	float _last_valid_mag_cal[3] = {};	///< last valid XYZ magnetometer bias estimates (mGauss)
	bool _valid_cal_available[3] = {};	///< true when an unsaved valid calibration for the XYZ magnetometer bias is available
	float _last_valid_variance[3] = {};	///< variances for the last valid magnetometer XYZ bias estimates (mGauss**2)

	// Used to filter velocity innovations during pre-flight checks
	bool _preflt_horiz_fail = false;	///< true if preflight horizontal innovation checks are failed
	bool _preflt_vert_fail = false;		///< true if preflight vertical innovation checks are failed
	bool _preflt_fail = false;		///< true if any preflight innovation checks are failed
	Vector2f _vel_ne_innov_lpf = {};	///< Preflight low pass filtered NE axis velocity innovations (m/sec)
	float _vel_d_innov_lpf = {};		///< Preflight low pass filtered D axis velocity innovations (m/sec)
	float _hgt_innov_lpf = 0.0f;		///< Preflight low pass filtered height innovation (m)
	float _yaw_innov_magnitude_lpf = 0.0f;	///< Preflight low pass filtered yaw innovation magntitude (rad)

	static constexpr float _innov_lpf_tau_inv = 0.2f;	///< Preflight low pass filter time constant inverse (1/sec)
	static constexpr float _vel_innov_test_lim =
        1.5f;	///< Maximum permissible velocity innovation to pass pre-flight checks (m/sec)
	static constexpr float _hgt_innov_test_lim =
        3.0f;	///< Maximum permissible height innovation to pass pre-flight checks (m)
	static constexpr float _nav_yaw_innov_test_lim =
        0.5f;	///< Maximum permissible yaw innovation to pass pre-flight checks when aiding inertial nav using NE frame observations (rad)
	static constexpr float _yaw_innov_test_lim =
        1.0f;	///< Maximum permissible yaw innovation to pass pre-flight checks when not aiding inertial nav using NE frame observations (rad)
	const float _vel_innov_spike_lim = 2.0f * _vel_innov_test_lim;	///< preflight velocity innovation spike limit (m/sec)
	const float _hgt_innov_spike_lim = 2.0f * _hgt_innov_test_lim;	///< preflight position innovation spike limit (m)

	int _airdata_sub{-1};
	int _airspeed_sub{-1};
	int _ev_att_sub{-1};
	int _ev_pos_sub{-1};
	int _gps_sub{-1};
	int _landing_target_pose_sub{-1};
	int _magnetometer_sub{-1};
	int _optical_flow_sub{-1};
	int _params_sub{-1};
	int _sensor_selection_sub{-1};
	int _sensors_sub{-1};
	int _status_sub{-1};
	int _vehicle_land_detected_sub{-1};

	// because we can have several distance sensor instances with different orientations
	int _range_finder_subs[ORB_MULTI_MAX_INSTANCES];
	int _range_finder_sub_index = -1; // index for downward-facing range finder subscription

	orb_advert_t _att_pub{nullptr};
	orb_advert_t _wind_pub{nullptr};
	orb_advert_t _estimator_status_pub{nullptr};
	orb_advert_t _estimator_innovations_pub{nullptr};
	orb_advert_t _ekf2_timestamps_pub{nullptr};
	orb_advert_t _sensor_bias_pub{nullptr};

	uORB::Publication<vehicle_local_position_s> _vehicle_local_position_pub;
	uORB::Publication<vehicle_global_position_s> _vehicle_global_position_pub;

	Ekf _ekf;

	parameters *_params;	///< pointer to ekf parameter struct (located in _ekf class instance)

    DEFINE_PARAMETERS(
		(ParamExtInt<px4::params::EKF2_MIN_OBS_DT>)
        _obs_dt_min_ms,	///< Maximmum time delay of any sensor used to increse buffer length to handle large timing jitter (mSec)
		(ParamExtFloat<px4::params::EKF2_MAG_DELAY>)
		_mag_delay_ms,	///< magnetometer measurement delay relative to the IMU (mSec)
		(ParamExtFloat<px4::params::EKF2_BARO_DELAY>)
		_baro_delay_ms,	///< barometer height measurement delay relative to the IMU (mSec)
		(ParamExtFloat<px4::params::EKF2_GPS_DELAY>) _gps_delay_ms,	///< GPS measurement delay relative to the IMU (mSec)
		(ParamExtFloat<px4::params::EKF2_OF_DELAY>)
		_flow_delay_ms,	///< optical flow measurement delay relative to the IMU (mSec) - this is to the middle of the optical flow integration interval
		(ParamExtFloat<px4::params::EKF2_RNG_DELAY>)
		_rng_delay_ms,	///< range finder measurement delay relative to the IMU (mSec)
		(ParamExtFloat<px4::params::EKF2_ASP_DELAY>)
		_airspeed_delay_ms,	///< airspeed measurement delay relative to the IMU (mSec)
		(ParamExtFloat<px4::params::EKF2_EV_DELAY>)
		_ev_delay_ms,	///< off-board vision measurement delay relative to the IMU (mSec)
		(ParamExtFloat<px4::params::EKF2_AVEL_DELAY>)
		_auxvel_delay_ms,	///< auxillary velocity measurement delay relative to the IMU (mSec)

		(ParamExtFloat<px4::params::EKF2_GYR_NOISE>)
		_gyro_noise,	///< IMU angular rate noise used for covariance prediction (rad/sec)
		(ParamExtFloat<px4::params::EKF2_ACC_NOISE>)
		_accel_noise,	///< IMU acceleration noise use for covariance prediction (m/sec**2)

		// process noise
		(ParamExtFloat<px4::params::EKF2_GYR_B_NOISE>)
		_gyro_bias_p_noise,	///< process noise for IMU rate gyro bias prediction (rad/sec**2)
		(ParamExtFloat<px4::params::EKF2_ACC_B_NOISE>)
		_accel_bias_p_noise,///< process noise for IMU accelerometer bias prediction (m/sec**3)
		(ParamExtFloat<px4::params::EKF2_MAG_E_NOISE>)
		_mage_p_noise,	///< process noise for earth magnetic field prediction (Gauss/sec)
		(ParamExtFloat<px4::params::EKF2_MAG_B_NOISE>)
		_magb_p_noise,	///< process noise for body magnetic field prediction (Gauss/sec)
		(ParamExtFloat<px4::params::EKF2_WIND_NOISE>)
		_wind_vel_p_noise,	///< process noise for wind velocity prediction (m/sec**2)
		(ParamExtFloat<px4::params::EKF2_TERR_NOISE>) _terrain_p_noise,	///< process noise for terrain offset (m/sec)
		(ParamExtFloat<px4::params::EKF2_TERR_GRAD>)
		_terrain_gradient,	///< gradient of terrain used to estimate process noise due to changing position (m/m)

		(ParamExtFloat<px4::params::EKF2_GPS_V_NOISE>)
		_gps_vel_noise,	///< minimum allowed observation noise for gps velocity fusion (m/sec)
		(ParamExtFloat<px4::params::EKF2_GPS_P_NOISE>)
		_gps_pos_noise,	///< minimum allowed observation noise for gps position fusion (m)
		(ParamExtFloat<px4::params::EKF2_NOAID_NOISE>)
		_pos_noaid_noise,	///< observation noise for non-aiding position fusion (m)
		(ParamExtFloat<px4::params::EKF2_BARO_NOISE>) _baro_noise,	///< observation noise for barometric height fusion (m)
		(ParamExtFloat<px4::params::EKF2_BARO_GATE>)
		_baro_innov_gate,	///< barometric height innovation consistency gate size (STD)
		(ParamExtFloat<px4::params::EKF2_GPS_P_GATE>)
		_posNE_innov_gate,	///< GPS horizontal position innovation consistency gate size (STD)
		(ParamExtFloat<px4::params::EKF2_GPS_V_GATE>) _vel_innov_gate,	///< GPS velocity innovation consistency gate size (STD)
		(ParamExtFloat<px4::params::EKF2_TAS_GATE>) _tas_innov_gate,	///< True Airspeed innovation consistency gate size (STD)

		// control of magnetometer fusion
		(ParamExtFloat<px4::params::EKF2_HEAD_NOISE>)
		_mag_heading_noise,	///< measurement noise used for simple heading fusion (rad)
		(ParamExtFloat<px4::params::EKF2_MAG_NOISE>)
		_mag_noise,		///< measurement noise used for 3-axis magnetoemeter fusion (Gauss)
		(ParamExtFloat<px4::params::EKF2_EAS_NOISE>) _eas_noise,		///< measurement noise used for airspeed fusion (m/sec)
		(ParamExtFloat<px4::params::EKF2_BETA_GATE>)
		_beta_innov_gate, ///< synthetic sideslip innovation consistency gate size (STD)
		(ParamExtFloat<px4::params::EKF2_BETA_NOISE>) _beta_noise,	///< synthetic sideslip noise (rad)
		(ParamExtFloat<px4::params::EKF2_MAG_DECL>) _mag_declination_deg,///< magnetic declination (degrees)
		(ParamExtFloat<px4::params::EKF2_HDG_GATE>)
		_heading_innov_gate,///< heading fusion innovation consistency gate size (STD)
		(ParamExtFloat<px4::params::EKF2_MAG_GATE>)
		_mag_innov_gate,	///< magnetometer fusion innovation consistency gate size (STD)
		(ParamExtInt<px4::params::EKF2_DECL_TYPE>)
		_mag_decl_source,	///< bitmask used to control the handling of declination data
		(ParamExtInt<px4::params::EKF2_MAG_TYPE>)
		_mag_fuse_type,	///< integer used to specify the type of magnetometer fusion used
		(ParamExtFloat<px4::params::EKF2_MAG_ACCLIM>)
		_mag_acc_gate,	///< integer used to specify the type of magnetometer fusion used
		(ParamExtFloat<px4::params::EKF2_MAG_YAWLIM>)
		_mag_yaw_rate_gate,	///< yaw rate threshold used by mode select logic (rad/sec)

		(ParamExtInt<px4::params::EKF2_GPS_CHECK>)
		_gps_check_mask,	///< bitmask used to control which GPS quality checks are used
		(ParamExtFloat<px4::params::EKF2_REQ_EPH>) _requiredEph,	///< maximum acceptable horiz position error (m)
		(ParamExtFloat<px4::params::EKF2_REQ_EPV>) _requiredEpv,	///< maximum acceptable vert position error (m)
		(ParamExtFloat<px4::params::EKF2_REQ_SACC>) _requiredSacc,	///< maximum acceptable speed error (m/s)
		(ParamExtInt<px4::params::EKF2_REQ_NSATS>) _requiredNsats,	///< minimum acceptable satellite count
		(ParamExtFloat<px4::params::EKF2_REQ_GDOP>) _requiredGDoP,	///< maximum acceptable geometric dilution of precision
		(ParamExtFloat<px4::params::EKF2_REQ_HDRIFT>) _requiredHdrift,	///< maximum acceptable horizontal drift speed (m/s)
		(ParamExtFloat<px4::params::EKF2_REQ_VDRIFT>) _requiredVdrift,	///< maximum acceptable vertical drift speed (m/s)

		// measurement source control
		(ParamExtInt<px4::params::EKF2_AID_MASK>)
		_fusion_mode,		///< bitmasked integer that selects which of the GPS and optical flow aiding sources will be used
		(ParamExtInt<px4::params::EKF2_HGT_MODE>) _vdist_sensor_type,	///< selects the primary source for height data
		(ParamExtInt<px4::params::EKF2_NOAID_TOUT>)
		_valid_timeout_max,	///< maximum lapsed time from last fusion of measurements that constrain drift before the EKF will report the horizontal nav solution invalid (uSec)

		// range finder fusion
		(ParamExtFloat<px4::params::EKF2_RNG_NOISE>) _range_noise,	///< observation noise for range finder measurements (m)
		(ParamExtFloat<px4::params::EKF2_RNG_SFE>) _range_noise_scaler, ///< scale factor from range to range noise (m/m)
		(ParamExtFloat<px4::params::EKF2_RNG_GATE>)
		_range_innov_gate,	///< range finder fusion innovation consistency gate size (STD)
		(ParamExtFloat<px4::params::EKF2_MIN_RNG>) _rng_gnd_clearance,	///< minimum valid value for range when on ground (m)
		(ParamExtFloat<px4::params::EKF2_RNG_PITCH>) _rng_pitch_offset,	///< range sensor pitch offset (rad)
		(ParamExtInt<px4::params::EKF2_RNG_AID>)
		_rng_aid,		///< enables use of a range finder even if primary height source is not range finder
		(ParamExtFloat<px4::params::EKF2_RNG_A_VMAX>)
		_rng_aid_hor_vel_max,	///< maximum allowed horizontal velocity for range aid (m/s)
		(ParamExtFloat<px4::params::EKF2_RNG_A_HMAX>)
		_rng_aid_height_max,	///< maximum allowed absolute altitude (AGL) for range aid (m)
		(ParamExtFloat<px4::params::EKF2_RNG_A_IGATE>)
		_rng_aid_innov_gate,	///< gate size used for innovation consistency checks for range aid fusion (STD)

		// vision estimate fusion
		(ParamFloat<px4::params::EKF2_EVP_NOISE>)
		_ev_pos_noise,	///< default position observation noise for exernal vision measurements (m)
		(ParamFloat<px4::params::EKF2_EVA_NOISE>)
		_ev_ang_noise,	///< default angular observation noise for exernal vision measurements (rad)
		(ParamExtFloat<px4::params::EKF2_EV_GATE>)
		_ev_innov_gate,	///< external vision position innovation consistency gate size (STD)

		// optical flow fusion
		(ParamExtFloat<px4::params::EKF2_OF_N_MIN>)
		_flow_noise,	///< best quality observation noise for optical flow LOS rate measurements (rad/sec)
		(ParamExtFloat<px4::params::EKF2_OF_N_MAX>)
		_flow_noise_qual_min,	///< worst quality observation noise for optical flow LOS rate measurements (rad/sec)
		(ParamExtInt<px4::params::EKF2_OF_QMIN>) _flow_qual_min,	///< minimum acceptable quality integer from  the flow sensor
		(ParamExtFloat<px4::params::EKF2_OF_GATE>)
		_flow_innov_gate,	///< optical flow fusion innovation consistency gate size (STD)

		// sensor positions in body frame
		(ParamExtFloat<px4::params::EKF2_IMU_POS_X>) _imu_pos_x,		///< X position of IMU in body frame (m)
		(ParamExtFloat<px4::params::EKF2_IMU_POS_Y>) _imu_pos_y,		///< Y position of IMU in body frame (m)
		(ParamExtFloat<px4::params::EKF2_IMU_POS_Z>) _imu_pos_z,		///< Z position of IMU in body frame (m)
		(ParamExtFloat<px4::params::EKF2_GPS_POS_X>) _gps_pos_x,		///< X position of GPS antenna in body frame (m)
		(ParamExtFloat<px4::params::EKF2_GPS_POS_Y>) _gps_pos_y,		///< Y position of GPS antenna in body frame (m)
		(ParamExtFloat<px4::params::EKF2_GPS_POS_Z>) _gps_pos_z,		///< Z position of GPS antenna in body frame (m)
		(ParamExtFloat<px4::params::EKF2_RNG_POS_X>) _rng_pos_x,		///< X position of range finder in body frame (m)
		(ParamExtFloat<px4::params::EKF2_RNG_POS_Y>) _rng_pos_y,		///< Y position of range finder in body frame (m)
		(ParamExtFloat<px4::params::EKF2_RNG_POS_Z>) _rng_pos_z,		///< Z position of range finder in body frame (m)
		(ParamExtFloat<px4::params::EKF2_OF_POS_X>)
		_flow_pos_x,	///< X position of optical flow sensor focal point in body frame (m)
		(ParamExtFloat<px4::params::EKF2_OF_POS_Y>)
		_flow_pos_y,	///< Y position of optical flow sensor focal point in body frame (m)
		(ParamExtFloat<px4::params::EKF2_OF_POS_Z>)
		_flow_pos_z,	///< Z position of optical flow sensor focal point in body frame (m)
		(ParamExtFloat<px4::params::EKF2_EV_POS_X>) _ev_pos_x,		///< X position of VI sensor focal point in body frame (m)
		(ParamExtFloat<px4::params::EKF2_EV_POS_Y>) _ev_pos_y,		///< Y position of VI sensor focal point in body frame (m)
		(ParamExtFloat<px4::params::EKF2_EV_POS_Z>) _ev_pos_z,		///< Z position of VI sensor focal point in body frame (m)

		// control of airspeed and sideslip fusion
		(ParamFloat<px4::params::EKF2_ARSP_THR>)
		_arspFusionThreshold, 	///< A value of zero will disabled airspeed fusion. Any positive value sets the minimum airspeed which will be used (m/sec)
		(ParamInt<px4::params::EKF2_FUSE_BETA>) _fuseBeta,		///< Controls synthetic sideslip fusion, 0 disables, 1 enables

		// output predictor filter time constants
		(ParamExtFloat<px4::params::EKF2_TAU_VEL>)
		_tau_vel,		///< time constant used by the output velocity complementary filter (sec)
		(ParamExtFloat<px4::params::EKF2_TAU_POS>)
		_tau_pos,		///< time constant used by the output position complementary filter (sec)

		// IMU switch on bias parameters
		(ParamExtFloat<px4::params::EKF2_GBIAS_INIT>) _gyr_bias_init,	///< 1-sigma gyro bias uncertainty at switch on (rad/sec)
		(ParamExtFloat<px4::params::EKF2_ABIAS_INIT>)
		_acc_bias_init,	///< 1-sigma accelerometer bias uncertainty at switch on (m/sec**2)
		(ParamExtFloat<px4::params::EKF2_ANGERR_INIT>)
		_ang_err_init,	///< 1-sigma tilt error after initial alignment using gravity vector (rad)

		// EKF saved XYZ magnetometer bias values
		(ParamFloat<px4::params::EKF2_MAGBIAS_X>) _mag_bias_x,		///< X magnetometer bias (mGauss)
		(ParamFloat<px4::params::EKF2_MAGBIAS_Y>) _mag_bias_y,		///< Y magnetometer bias (mGauss)
		(ParamFloat<px4::params::EKF2_MAGBIAS_Z>) _mag_bias_z,		///< Z magnetometer bias (mGauss)
		(ParamInt<px4::params::EKF2_MAGBIAS_ID>) _mag_bias_id,		///< ID of the magnetometer sensor used to learn the bias values
		(ParamFloat<px4::params::EKF2_MAGB_VREF>)
		_mag_bias_saved_variance, ///< Assumed error variance of previously saved magnetometer bias estimates (mGauss**2)
		(ParamFloat<px4::params::EKF2_MAGB_K>)
		_mag_bias_alpha,	///< maximum fraction of the learned magnetometer bias that is saved at each disarm

		// EKF accel bias learning control
		(ParamExtFloat<px4::params::EKF2_ABL_LIM>) _acc_bias_lim,	///< Accelerometer bias learning limit (m/s**2)
		(ParamExtFloat<px4::params::EKF2_ABL_ACCLIM>)
		_acc_bias_learn_acc_lim,	///< Maximum IMU accel magnitude that allows IMU bias learning (m/s**2)
		(ParamExtFloat<px4::params::EKF2_ABL_GYRLIM>)
		_acc_bias_learn_gyr_lim,	///< Maximum IMU gyro angular rate magnitude that allows IMU bias learning (m/s**2)
		(ParamExtFloat<px4::params::EKF2_ABL_TAU>)
		_acc_bias_learn_tc,	///< Time constant used to inhibit IMU delta velocity bias learning (sec)

		// Multi-rotor drag specific force fusion
		(ParamExtFloat<px4::params::EKF2_DRAG_NOISE>)
		_drag_noise,	///< observation noise variance for drag specific force measurements (m/sec**2)**2
		(ParamExtFloat<px4::params::EKF2_BCOEF_X>) _bcoef_x,		///< ballistic coefficient along the X-axis (kg/m**2)
		(ParamExtFloat<px4::params::EKF2_BCOEF_Y>) _bcoef_y,		///< ballistic coefficient along the Y-axis (kg/m**2)

		// Corrections for static pressure position error where Ps_error = Ps_meas - Ps_truth
		// Coef = Ps_error / Pdynamic, where Pdynamic = 1/2 * density * TAS**2
		(ParamFloat<px4::params::EKF2_ASPD_MAX>) _aspd_max,		///< upper limit on airspeed used for correction  (m/s**2)
		(ParamFloat<px4::params::EKF2_PCOEF_XP>)
		_K_pstatic_coef_xp,	///< static pressure position error coefficient along the positive X body axis
		(ParamFloat<px4::params::EKF2_PCOEF_XN>)
		_K_pstatic_coef_xn,	///< static pressure position error coefficient along the negative X body axis
		(ParamFloat<px4::params::EKF2_PCOEF_Y>)
		_K_pstatic_coef_y,	///< static pressure position error coefficient along the Y body axis
		(ParamFloat<px4::params::EKF2_PCOEF_Z>)
		_K_pstatic_coef_z	///< static pressure position error coefficient along the Z body axis
	)

};

Ekf2::Ekf2():
	ModuleParams(nullptr),
	_vehicle_local_position_pub(ORB_ID(vehicle_local_position)),
	_vehicle_global_position_pub(ORB_ID(vehicle_global_position)),
	_params(_ekf.getParamHandle()),
	_obs_dt_min_ms(_params->sensor_interval_min_ms),
	_mag_delay_ms(_params->mag_delay_ms),
	_baro_delay_ms(_params->baro_delay_ms),
	_gps_delay_ms(_params->gps_delay_ms),
	_flow_delay_ms(_params->flow_delay_ms),
	_rng_delay_ms(_params->range_delay_ms),
	_airspeed_delay_ms(_params->airspeed_delay_ms),
	_ev_delay_ms(_params->ev_delay_ms),
	_auxvel_delay_ms(_params->auxvel_delay_ms),
	_gyro_noise(_params->gyro_noise),
	_accel_noise(_params->accel_noise),
	_gyro_bias_p_noise(_params->gyro_bias_p_noise),
	_accel_bias_p_noise(_params->accel_bias_p_noise),
	_mage_p_noise(_params->mage_p_noise),
	_magb_p_noise(_params->magb_p_noise),
	_wind_vel_p_noise(_params->wind_vel_p_noise),
	_terrain_p_noise(_params->terrain_p_noise),
	_terrain_gradient(_params->terrain_gradient),
	_gps_vel_noise(_params->gps_vel_noise),
	_gps_pos_noise(_params->gps_pos_noise),
	_pos_noaid_noise(_params->pos_noaid_noise),
	_baro_noise(_params->baro_noise),
	_baro_innov_gate(_params->baro_innov_gate),
	_posNE_innov_gate(_params->posNE_innov_gate),
	_vel_innov_gate(_params->vel_innov_gate),
	_tas_innov_gate(_params->tas_innov_gate),
	_mag_heading_noise(_params->mag_heading_noise),
	_mag_noise(_params->mag_noise),
	_eas_noise(_params->eas_noise),
	_beta_innov_gate(_params->beta_innov_gate),
	_beta_noise(_params->beta_noise),
	_mag_declination_deg(_params->mag_declination_deg),
	_heading_innov_gate(_params->heading_innov_gate),
	_mag_innov_gate(_params->mag_innov_gate),
	_mag_decl_source(_params->mag_declination_source),
	_mag_fuse_type(_params->mag_fusion_type),
	_mag_acc_gate(_params->mag_acc_gate),
	_mag_yaw_rate_gate(_params->mag_yaw_rate_gate),
	_gps_check_mask(_params->gps_check_mask),
	_requiredEph(_params->req_hacc),
	_requiredEpv(_params->req_vacc),
	_requiredSacc(_params->req_sacc),
	_requiredNsats(_params->req_nsats),
	_requiredGDoP(_params->req_gdop),
	_requiredHdrift(_params->req_hdrift),
	_requiredVdrift(_params->req_vdrift),
	_fusion_mode(_params->fusion_mode),
	_vdist_sensor_type(_params->vdist_sensor_type),
	_valid_timeout_max(_params->valid_timeout_max),
	_range_noise(_params->range_noise),
	_range_noise_scaler(_params->range_noise_scaler),
	_range_innov_gate(_params->range_innov_gate),
	_rng_gnd_clearance(_params->rng_gnd_clearance),
	_rng_pitch_offset(_params->rng_sens_pitch),
	_rng_aid(_params->range_aid),
	_rng_aid_hor_vel_max(_params->max_vel_for_range_aid),
	_rng_aid_height_max(_params->max_hagl_for_range_aid),
	_rng_aid_innov_gate(_params->range_aid_innov_gate),
	_ev_innov_gate(_params->ev_innov_gate),
	_flow_noise(_params->flow_noise),
	_flow_noise_qual_min(_params->flow_noise_qual_min),
	_flow_qual_min(_params->flow_qual_min),
	_flow_innov_gate(_params->flow_innov_gate),
	_imu_pos_x(_params->imu_pos_body(0)),
	_imu_pos_y(_params->imu_pos_body(1)),
	_imu_pos_z(_params->imu_pos_body(2)),
	_gps_pos_x(_params->gps_pos_body(0)),
	_gps_pos_y(_params->gps_pos_body(1)),
	_gps_pos_z(_params->gps_pos_body(2)),
	_rng_pos_x(_params->rng_pos_body(0)),
	_rng_pos_y(_params->rng_pos_body(1)),
	_rng_pos_z(_params->rng_pos_body(2)),
	_flow_pos_x(_params->flow_pos_body(0)),
	_flow_pos_y(_params->flow_pos_body(1)),
	_flow_pos_z(_params->flow_pos_body(2)),
	_ev_pos_x(_params->ev_pos_body(0)),
	_ev_pos_y(_params->ev_pos_body(1)),
	_ev_pos_z(_params->ev_pos_body(2)),
	_tau_vel(_params->vel_Tau),
	_tau_pos(_params->pos_Tau),
	_gyr_bias_init(_params->switch_on_gyro_bias),
	_acc_bias_init(_params->switch_on_accel_bias),
	_ang_err_init(_params->initial_tilt_err),
	_acc_bias_lim(_params->acc_bias_lim),
	_acc_bias_learn_acc_lim(_params->acc_bias_learn_acc_lim),
	_acc_bias_learn_gyr_lim(_params->acc_bias_learn_gyr_lim),
	_acc_bias_learn_tc(_params->acc_bias_learn_tc),
	_drag_noise(_params->drag_noise),
	_bcoef_x(_params->bcoef_x),
	_bcoef_y(_params->bcoef_y)
{
	_airdata_sub = orb_subscribe(ORB_ID(vehicle_air_data));
	_airspeed_sub = orb_subscribe(ORB_ID(airspeed));
	_ev_att_sub = orb_subscribe(ORB_ID(vehicle_vision_attitude));
	_ev_pos_sub = orb_subscribe(ORB_ID(vehicle_vision_position));
	_gps_sub = orb_subscribe(ORB_ID(vehicle_gps_position));
	_landing_target_pose_sub = orb_subscribe(ORB_ID(landing_target_pose));
	_magnetometer_sub = orb_subscribe(ORB_ID(vehicle_magnetometer));
	_optical_flow_sub = orb_subscribe(ORB_ID(optical_flow));
	_params_sub = orb_subscribe(ORB_ID(parameter_update));
	_sensor_selection_sub = orb_subscribe(ORB_ID(sensor_selection));
	_sensors_sub = orb_subscribe(ORB_ID(sensor_combined));
	_status_sub = orb_subscribe(ORB_ID(vehicle_status));
	_vehicle_land_detected_sub = orb_subscribe(ORB_ID(vehicle_land_detected));

	for (unsigned i = 0; i < ORB_MULTI_MAX_INSTANCES; i++) {
		_range_finder_subs[i] = orb_subscribe_multi(ORB_ID(distance_sensor), i);
	}

	// initialise parameter cache
	updateParams();
}

Ekf2::~Ekf2()
{
	orb_unsubscribe(_airdata_sub);
	orb_unsubscribe(_airspeed_sub);
	orb_unsubscribe(_ev_att_sub);
	orb_unsubscribe(_ev_pos_sub);
	orb_unsubscribe(_gps_sub);
	orb_unsubscribe(_landing_target_pose_sub);
	orb_unsubscribe(_magnetometer_sub);
	orb_unsubscribe(_optical_flow_sub);
	orb_unsubscribe(_params_sub);
	orb_unsubscribe(_sensor_selection_sub);
	orb_unsubscribe(_sensors_sub);
	orb_unsubscribe(_status_sub);
	orb_unsubscribe(_vehicle_land_detected_sub);

	for (unsigned i = 0; i < ORB_MULTI_MAX_INSTANCES; i++) {
		orb_unsubscribe(_range_finder_subs[i]);
		_range_finder_subs[i] = -1;
	}
}

int Ekf2::print_status()
{
	PX4_INFO("local position OK %s", (_ekf.local_position_is_valid()) ? "yes" : "no");
	PX4_INFO("global position OK %s", (_ekf.global_position_is_valid()) ? "yes" : "no");
	PX4_INFO("time slip: %" PRId64 " us", _last_time_slip_us);
	return 0;
}

template<typename Param>
void Ekf2::update_mag_bias(Param &mag_bias_param, int axis_index)
{
	if (_valid_cal_available[axis_index]) {

		// calculate weighting using ratio of variances and update stored bias values
		const float weighting = constrain(_mag_bias_saved_variance.get() / (_mag_bias_saved_variance.get() +
						  _last_valid_variance[axis_index]), 0.0f, _mag_bias_alpha.get());
		const float mag_bias_saved = mag_bias_param.get();

		_last_valid_mag_cal[axis_index] = weighting * _last_valid_mag_cal[axis_index] + mag_bias_saved;

		mag_bias_param.set(_last_valid_mag_cal[axis_index]);
		mag_bias_param.commit_no_notification();

		_valid_cal_available[axis_index] = false;
	}
}

void Ekf2::run()
{
	bool imu_bias_reset_request = false;

	px4_pollfd_struct_t fds[1] = {};
	fds[0].fd = _sensors_sub;
	fds[0].events = POLLIN;

	// initialize data structures outside of loop
	// because they will else not always be
	// properly populated
	sensor_combined_s sensors = {};
	vehicle_land_detected_s vehicle_land_detected = {};
	vehicle_status_s vehicle_status = {};
	sensor_selection_s sensor_selection = {};

	while (!should_exit()) {
		int ret = px4_poll(fds, sizeof(fds) / sizeof(fds[0]), 1000);

		if (!(fds[0].revents & POLLIN)) {
			// no new data
			continue;
		}

		if (ret < 0) {
			// Poll error, sleep and try again
			usleep(10000);
			continue;

		} else if (ret == 0) {
			// Poll timeout or no new data, do nothing
			continue;
		}

		bool params_updated = false;
		orb_check(_params_sub, &params_updated);

		if (params_updated) {
			// read from param to clear updated flag
			parameter_update_s update;
			orb_copy(ORB_ID(parameter_update), _params_sub, &update);
			updateParams();
		}

		orb_copy(ORB_ID(sensor_combined), _sensors_sub, &sensors);

		// ekf2_timestamps (using 0.1 ms relative timestamps)
		ekf2_timestamps_s ekf2_timestamps = {};
		ekf2_timestamps.timestamp = sensors.timestamp;

		ekf2_timestamps.airspeed_timestamp_rel = ekf2_timestamps_s::RELATIVE_TIMESTAMP_INVALID;
		ekf2_timestamps.distance_sensor_timestamp_rel = ekf2_timestamps_s::RELATIVE_TIMESTAMP_INVALID;
		ekf2_timestamps.gps_timestamp_rel = ekf2_timestamps_s::RELATIVE_TIMESTAMP_INVALID;
		ekf2_timestamps.optical_flow_timestamp_rel = ekf2_timestamps_s::RELATIVE_TIMESTAMP_INVALID;
		ekf2_timestamps.vehicle_air_data_timestamp_rel = ekf2_timestamps_s::RELATIVE_TIMESTAMP_INVALID;
		ekf2_timestamps.vehicle_magnetometer_timestamp_rel = ekf2_timestamps_s::RELATIVE_TIMESTAMP_INVALID;
		ekf2_timestamps.vision_attitude_timestamp_rel = ekf2_timestamps_s::RELATIVE_TIMESTAMP_INVALID;
		ekf2_timestamps.vision_position_timestamp_rel = ekf2_timestamps_s::RELATIVE_TIMESTAMP_INVALID;

		// update all other topics if they have new data

		bool vehicle_status_updated = false;

		orb_check(_status_sub, &vehicle_status_updated);

		if (vehicle_status_updated) {
			if (orb_copy(ORB_ID(vehicle_status), _status_sub, &vehicle_status) == PX4_OK) {
				// only fuse synthetic sideslip measurements if conditions are met
				_ekf.set_fuse_beta_flag(!vehicle_status.is_rotary_wing && (_fuseBeta.get() == 1));

				// let the EKF know if the vehicle motion is that of a fixed wing (forward flight only relative to wind)
				_ekf.set_is_fixed_wing(!vehicle_status.is_rotary_wing);
			}
		}

		bool sensor_selection_updated = false;

		orb_check(_sensor_selection_sub, &sensor_selection_updated);

		// Always update sensor selction first time through if time stamp is non zero
		if (sensor_selection_updated || (sensor_selection.timestamp == 0)) {
			sensor_selection_s sensor_selection_prev = sensor_selection;

			if (orb_copy(ORB_ID(sensor_selection), _sensor_selection_sub, &sensor_selection) == PX4_OK) {
				if ((sensor_selection_prev.timestamp > 0) && (sensor_selection.timestamp > sensor_selection_prev.timestamp)) {
					if (sensor_selection.accel_device_id != sensor_selection_prev.accel_device_id) {
						PX4_WARN("accel id changed, resetting IMU bias");
						imu_bias_reset_request = true;
					}

					if (sensor_selection.gyro_device_id != sensor_selection_prev.gyro_device_id) {
						PX4_WARN("gyro id changed, resetting IMU bias");
						imu_bias_reset_request = true;
					}
				}
			}
		}

		// attempt reset until successful
		if (imu_bias_reset_request) {
			imu_bias_reset_request = !_ekf.reset_imu_bias();
		}

		// in replay mode we are getting the actual timestamp from the sensor topic
		hrt_abstime now = 0;

		if (_replay_mode) {
			now = sensors.timestamp;

		} else {
			now = hrt_absolute_time();
		}

		// push imu data into estimator
		float gyro_integral[3];
		float gyro_dt = sensors.gyro_integral_dt / 1.e6f;
		gyro_integral[0] = sensors.gyro_rad[0] * gyro_dt;
		gyro_integral[1] = sensors.gyro_rad[1] * gyro_dt;
		gyro_integral[2] = sensors.gyro_rad[2] * gyro_dt;

		float accel_integral[3];
		float accel_dt = sensors.accelerometer_integral_dt / 1.e6f;
		accel_integral[0] = sensors.accelerometer_m_s2[0] * accel_dt;
		accel_integral[1] = sensors.accelerometer_m_s2[1] * accel_dt;
		accel_integral[2] = sensors.accelerometer_m_s2[2] * accel_dt;

		_ekf.setIMUData(now, sensors.gyro_integral_dt, sensors.accelerometer_integral_dt, gyro_integral, accel_integral);

		// read mag data
		bool magnetometer_updated = false;
		orb_check(_magnetometer_sub, &magnetometer_updated);

		if (magnetometer_updated) {
			vehicle_magnetometer_s magnetometer;

			if (orb_copy(ORB_ID(vehicle_magnetometer), _magnetometer_sub, &magnetometer) == PX4_OK) {
				// Reset learned bias parameters if there has been a persistant change in magnetometer ID
				// Do not reset parmameters when armed to prevent potential time slips casued by parameter set
				// and notification events
				// Check if there has been a persistant change in magnetometer ID
				if (sensor_selection.mag_device_id != 0 && sensor_selection.mag_device_id != _mag_bias_id.get()) {
					if (_invalid_mag_id_count < 200) {
						_invalid_mag_id_count++;
					}

				} else {
					if (_invalid_mag_id_count > 0) {
						_invalid_mag_id_count--;
					}
				}

				if ((vehicle_status.arming_state != vehicle_status_s::ARMING_STATE_ARMED) && (_invalid_mag_id_count > 100)) {
					// the sensor ID used for the last saved mag bias is not confirmed to be the same as the current sensor ID
					// this means we need to reset the learned bias values to zero
					_mag_bias_x.set(0.f);
					_mag_bias_x.commit_no_notification();
					_mag_bias_y.set(0.f);
					_mag_bias_y.commit_no_notification();
					_mag_bias_z.set(0.f);
					_mag_bias_z.commit_no_notification();
					_mag_bias_id.set(sensor_selection.mag_device_id);
					_mag_bias_id.commit();

					_invalid_mag_id_count = 0;

					PX4_INFO("Mag sensor ID changed to %i", _mag_bias_id.get());
				}

				// If the time last used by the EKF is less than specified, then accumulate the
				// data and push the average when the specified interval is reached.
				_mag_time_sum_ms += magnetometer.timestamp / 1000;
				_mag_sample_count++;
				_mag_data_sum[0] += magnetometer.magnetometer_ga[0];
				_mag_data_sum[1] += magnetometer.magnetometer_ga[1];
				_mag_data_sum[2] += magnetometer.magnetometer_ga[2];
				uint32_t mag_time_ms = _mag_time_sum_ms / _mag_sample_count;

				if (mag_time_ms - _mag_time_ms_last_used > _params->sensor_interval_min_ms) {
					const float mag_sample_count_inv = 1.0f / _mag_sample_count;
					// calculate mean of measurements and correct for learned bias offsets
					float mag_data_avg_ga[3] = {_mag_data_sum[0] *mag_sample_count_inv - _mag_bias_x.get(),
								    _mag_data_sum[1] *mag_sample_count_inv - _mag_bias_y.get(),
								    _mag_data_sum[2] *mag_sample_count_inv - _mag_bias_z.get()
								   };

					_ekf.setMagData(1000 * (uint64_t)mag_time_ms, mag_data_avg_ga);

					_mag_time_ms_last_used = mag_time_ms;
					_mag_time_sum_ms = 0;
					_mag_sample_count = 0;
					_mag_data_sum[0] = 0.0f;
					_mag_data_sum[1] = 0.0f;
					_mag_data_sum[2] = 0.0f;
				}

				ekf2_timestamps.vehicle_magnetometer_timestamp_rel = (int16_t)((int64_t)magnetometer.timestamp / 100 -
						(int64_t)ekf2_timestamps.timestamp / 100);
			}
		}

		// read baro data
		bool airdata_updated = false;
		orb_check(_airdata_sub, &airdata_updated);

		if (airdata_updated) {
			vehicle_air_data_s airdata;

			if (orb_copy(ORB_ID(vehicle_air_data), _airdata_sub, &airdata) == PX4_OK) {
				// If the time last used by the EKF is less than specified, then accumulate the
				// data and push the average when the specified interval is reached.
				_balt_time_sum_ms += airdata.timestamp / 1000;
				_balt_sample_count++;
				_balt_data_sum += airdata.baro_alt_meter;
				uint32_t balt_time_ms = _balt_time_sum_ms / _balt_sample_count;

				if (balt_time_ms - _balt_time_ms_last_used > (uint32_t)_params->sensor_interval_min_ms) {
					// take mean across sample period
					float balt_data_avg = _balt_data_sum / (float)_balt_sample_count;

					_ekf.set_air_density(airdata.rho);

					// calculate static pressure error = Pmeas - Ptruth
					// model position error sensitivity as a body fixed ellipse with different scale in the positive and negtive X direction
					const float max_airspeed_sq = _aspd_max.get() * _aspd_max.get();
					float K_pstatic_coef_x;

					const Vector3f vel_body_wind = get_vel_body_wind();

					if (vel_body_wind(0) >= 0.0f) {
						K_pstatic_coef_x = _K_pstatic_coef_xp.get();

					} else {
						K_pstatic_coef_x = _K_pstatic_coef_xn.get();
					}

					const float x_v2 = fminf(vel_body_wind(0) * vel_body_wind(0), max_airspeed_sq);
					const float y_v2 = fminf(vel_body_wind(1) * vel_body_wind(1), max_airspeed_sq);
					const float z_v2 = fminf(vel_body_wind(2) * vel_body_wind(2), max_airspeed_sq);

					const float pstatic_err = 0.5f * airdata.rho *
								  (K_pstatic_coef_x * x_v2) + (_K_pstatic_coef_y.get() * y_v2) + (_K_pstatic_coef_z.get() * z_v2);

					// correct baro measurement using pressure error estimate and assuming sea level gravity
					balt_data_avg += pstatic_err / (airdata.rho * CONSTANTS_ONE_G);

					// push to estimator
					_ekf.setBaroData(1000 * (uint64_t)balt_time_ms, balt_data_avg);

					_balt_time_ms_last_used = balt_time_ms;
					_balt_time_sum_ms = 0;
					_balt_sample_count = 0;
					_balt_data_sum = 0.0f;
				}

				ekf2_timestamps.vehicle_air_data_timestamp_rel = (int16_t)((int64_t)airdata.timestamp / 100 -
						(int64_t)ekf2_timestamps.timestamp / 100);
			}
		}

		// read gps data if available
		bool gps_updated = false;
		orb_check(_gps_sub, &gps_updated);

		if (gps_updated) {
			vehicle_gps_position_s gps;

			if (orb_copy(ORB_ID(vehicle_gps_position), _gps_sub, &gps) == PX4_OK) {
				struct gps_message gps_msg;
				gps_msg.time_usec = gps.timestamp;
				gps_msg.lat = gps.lat;
				gps_msg.lon = gps.lon;
				gps_msg.alt = gps.alt;
				gps_msg.fix_type = gps.fix_type;
				gps_msg.eph = gps.eph;
				gps_msg.epv = gps.epv;
				gps_msg.sacc = gps.s_variance_m_s;
				gps_msg.vel_m_s = gps.vel_m_s;
				gps_msg.vel_ned[0] = gps.vel_n_m_s;
				gps_msg.vel_ned[1] = gps.vel_e_m_s;
				gps_msg.vel_ned[2] = gps.vel_d_m_s;
				gps_msg.vel_ned_valid = gps.vel_ned_valid;
				gps_msg.nsats = gps.satellites_used;
				//TODO: add gdop to gps topic
				gps_msg.gdop = 0.0f;

				_ekf.setGpsData(gps.timestamp, &gps_msg);

				ekf2_timestamps.gps_timestamp_rel = (int16_t)((int64_t)gps.timestamp / 100 - (int64_t)ekf2_timestamps.timestamp / 100);
			}
		}

		bool airspeed_updated = false;
		orb_check(_airspeed_sub, &airspeed_updated);

		if (airspeed_updated) {
			airspeed_s airspeed;

			if (orb_copy(ORB_ID(airspeed), _airspeed_sub, &airspeed) == PX4_OK) {
				// only set airspeed data if condition for airspeed fusion are met
				if ((_arspFusionThreshold.get() > FLT_EPSILON) && (airspeed.true_airspeed_m_s > _arspFusionThreshold.get())) {

					const float eas2tas = airspeed.true_airspeed_m_s / airspeed.indicated_airspeed_m_s;
					_ekf.setAirspeedData(airspeed.timestamp, airspeed.true_airspeed_m_s, eas2tas);
				}

				ekf2_timestamps.airspeed_timestamp_rel = (int16_t)((int64_t)airspeed.timestamp / 100 -
						(int64_t)ekf2_timestamps.timestamp / 100);
			}
		}

		bool optical_flow_updated = false;

		orb_check(_optical_flow_sub, &optical_flow_updated);

		if (optical_flow_updated) {
			optical_flow_s optical_flow;

			if (orb_copy(ORB_ID(optical_flow), _optical_flow_sub, &optical_flow) == PX4_OK) {
				flow_message flow;
				flow.flowdata(0) = optical_flow.pixel_flow_x_integral;
				flow.flowdata(1) = optical_flow.pixel_flow_y_integral;
				flow.quality = optical_flow.quality;
				flow.gyrodata(0) = optical_flow.gyro_x_rate_integral;
				flow.gyrodata(1) = optical_flow.gyro_y_rate_integral;
				flow.gyrodata(2) = optical_flow.gyro_z_rate_integral;
				flow.dt = optical_flow.integration_timespan;

				if (PX4_ISFINITE(optical_flow.pixel_flow_y_integral) &&
				    PX4_ISFINITE(optical_flow.pixel_flow_x_integral)) {

					_ekf.setOpticalFlowData(optical_flow.timestamp, &flow);
				}

				// Save sensor limits reported by the optical flow sensor
				_ekf.set_optical_flow_limits(optical_flow.max_flow_rate, optical_flow.min_ground_distance,
							     optical_flow.max_ground_distance);

				ekf2_timestamps.optical_flow_timestamp_rel = (int16_t)((int64_t)optical_flow.timestamp / 100 -
						(int64_t)ekf2_timestamps.timestamp / 100);
			}
		}

		if (_range_finder_sub_index >= 0) {
			bool range_finder_updated = false;

			orb_check(_range_finder_subs[_range_finder_sub_index], &range_finder_updated);

			if (range_finder_updated) {
				distance_sensor_s range_finder;

				if (orb_copy(ORB_ID(distance_sensor), _range_finder_subs[_range_finder_sub_index], &range_finder) == PX4_OK) {
					// check if distance sensor is within working boundaries
					if (range_finder.min_distance >= range_finder.current_distance ||
					    range_finder.max_distance <= range_finder.current_distance) {
						// use rng_gnd_clearance if on ground
						if (_ekf.get_in_air_status()) {
							range_finder_updated = false;

						} else {
							range_finder.current_distance = _rng_gnd_clearance.get();
						}
					}

					_ekf.setRangeData(range_finder.timestamp, range_finder.current_distance);

					// Save sensor limits reported by the rangefinder
					_ekf.set_rangefinder_limits(range_finder.min_distance, range_finder.max_distance);

					ekf2_timestamps.distance_sensor_timestamp_rel = (int16_t)((int64_t)range_finder.timestamp / 100 -
							(int64_t)ekf2_timestamps.timestamp / 100);
				}
			}

		} else {
			_range_finder_sub_index = getRangeSubIndex(_range_finder_subs);
		}

		// get external vision data
		// if error estimates are unavailable, use parameter defined defaults
		bool vision_position_updated = false;
		bool vision_attitude_updated = false;
		orb_check(_ev_pos_sub, &vision_position_updated);
		orb_check(_ev_att_sub, &vision_attitude_updated);

		if (vision_position_updated || vision_attitude_updated) {
			// copy both attitude & position if either updated, we need both to fill a single ext_vision_message
			vehicle_attitude_s ev_att = {};
			orb_copy(ORB_ID(vehicle_vision_attitude), _ev_att_sub, &ev_att);

			vehicle_local_position_s ev_pos = {};
			orb_copy(ORB_ID(vehicle_vision_position), _ev_pos_sub, &ev_pos);

			ext_vision_message ev_data;
			ev_data.posNED(0) = ev_pos.x;
			ev_data.posNED(1) = ev_pos.y;
			ev_data.posNED(2) = ev_pos.z;
			matrix::Quatf q(ev_att.q);
			ev_data.quat = q;

			// position measurement error from parameters. TODO : use covariances from topic
			ev_data.posErr = fmaxf(_ev_pos_noise.get(), fmaxf(ev_pos.eph, ev_pos.epv));
			ev_data.angErr = _ev_ang_noise.get();

			// only set data if all positions and velocities are valid
			if (ev_pos.xy_valid && ev_pos.z_valid && ev_pos.v_xy_valid && ev_pos.v_z_valid) {
				// use timestamp from external computer, clocks are synchronized when using MAVROS
				_ekf.setExtVisionData(vision_position_updated ? ev_pos.timestamp : ev_att.timestamp, &ev_data);
			}

			ekf2_timestamps.vision_position_timestamp_rel = (int16_t)((int64_t)ev_pos.timestamp / 100 -
					(int64_t)ekf2_timestamps.timestamp / 100);
			ekf2_timestamps.vision_attitude_timestamp_rel = (int16_t)((int64_t)ev_att.timestamp / 100 -
					(int64_t)ekf2_timestamps.timestamp / 100);
		}

		bool vehicle_land_detected_updated = false;
		orb_check(_vehicle_land_detected_sub, &vehicle_land_detected_updated);

		if (vehicle_land_detected_updated) {
			if (orb_copy(ORB_ID(vehicle_land_detected), _vehicle_land_detected_sub, &vehicle_land_detected) == PX4_OK) {
				_ekf.set_in_air_status(!vehicle_land_detected.landed);
			}
		}

		// use the landing target pose estimate as another source of velocity data
		bool landing_target_pose_updated = false;
		orb_check(_landing_target_pose_sub, &landing_target_pose_updated);

		if (landing_target_pose_updated) {
			landing_target_pose_s landing_target_pose;

			if (orb_copy(ORB_ID(landing_target_pose), _landing_target_pose_sub, &landing_target_pose) == PX4_OK) {
				// we can only use the landing target if it has a fixed position and  a valid velocity estimate
				if (landing_target_pose.is_static && landing_target_pose.rel_vel_valid) {
					// velocity of vehicle relative to target has opposite sign to target relative to vehicle
					float velocity[2] = {-landing_target_pose.vx_rel, -landing_target_pose.vy_rel};
					float variance[2] = {landing_target_pose.cov_vx_rel, landing_target_pose.cov_vy_rel};
					_ekf.setAuxVelData(landing_target_pose.timestamp, velocity, variance);
				}
			}
		}

		// run the EKF update and output
		const bool updated = _ekf.update();

		if (updated) {

			// integrate time to monitor time slippage
			if (_start_time_us == 0) {
				_start_time_us = now;
				_last_time_slip_us = 0;

			} else if (_start_time_us > 0) {
				_integrated_time_us += sensors.gyro_integral_dt;
				_last_time_slip_us = (now - _start_time_us) - _integrated_time_us;
			}

			matrix::Quatf q;
			_ekf.copy_quaternion(q.data());

			// In-run bias estimates
			float gyro_bias[3];
			_ekf.get_gyro_bias(gyro_bias);

			{
				// generate vehicle attitude quaternion data
				vehicle_attitude_s att;
				att.timestamp = now;

				q.copyTo(att.q);
				_ekf.get_quat_reset(&att.delta_q_reset[0], &att.quat_reset_counter);

				att.rollspeed = sensors.gyro_rad[0] - gyro_bias[0];
				att.pitchspeed = sensors.gyro_rad[1] - gyro_bias[1];
				att.yawspeed = sensors.gyro_rad[2] - gyro_bias[2];

				// publish vehicle attitude data
				if (_att_pub == nullptr) {
					_att_pub = orb_advertise(ORB_ID(vehicle_attitude), &att);

				} else {
					orb_publish(ORB_ID(vehicle_attitude), _att_pub, &att);
				}
			}

			// generate vehicle local position data
			vehicle_local_position_s &lpos = _vehicle_local_position_pub.get();

			lpos.timestamp = now;

			// Position of body origin in local NED frame
			float position[3];
			_ekf.get_position(position);
			const float lpos_x_prev = lpos.x;
			const float lpos_y_prev = lpos.y;
			lpos.x = (_ekf.local_position_is_valid()) ? position[0] : 0.0f;
			lpos.y = (_ekf.local_position_is_valid()) ? position[1] : 0.0f;
			lpos.z = position[2];

			// Velocity of body origin in local NED frame (m/s)
			float velocity[3];
			_ekf.get_velocity(velocity);
			lpos.vx = velocity[0];
			lpos.vy = velocity[1];
			lpos.vz = velocity[2];

			// vertical position time derivative (m/s)
			_ekf.get_pos_d_deriv(&lpos.z_deriv);

			// Acceleration of body origin in local NED frame
			float vel_deriv[3];
			_ekf.get_vel_deriv_ned(vel_deriv);
			lpos.ax = vel_deriv[0];
			lpos.ay = vel_deriv[1];
			lpos.az = vel_deriv[2];

			// TODO: better status reporting
			lpos.xy_valid = _ekf.local_position_is_valid() && !_preflt_horiz_fail;
			lpos.z_valid = !_preflt_vert_fail;
			lpos.v_xy_valid = _ekf.local_position_is_valid() && !_preflt_horiz_fail;
			lpos.v_z_valid = !_preflt_vert_fail;

			// Position of local NED origin in GPS / WGS84 frame
			map_projection_reference_s ekf_origin;
			uint64_t origin_time;

			// true if position (x,y,z) has a valid WGS-84 global reference (ref_lat, ref_lon, alt)
			const bool ekf_origin_valid = _ekf.get_ekf_origin(&origin_time, &ekf_origin, &lpos.ref_alt);
			lpos.xy_global = ekf_origin_valid;
			lpos.z_global = ekf_origin_valid;

			if (ekf_origin_valid && (origin_time > lpos.ref_timestamp)) {
				lpos.ref_timestamp = origin_time;
				lpos.ref_lat = ekf_origin.lat_rad * 180.0 / M_PI; // Reference point latitude in degrees
				lpos.ref_lon = ekf_origin.lon_rad * 180.0 / M_PI; // Reference point longitude in degrees
			}

			// The rotation of the tangent plane vs. geographical north
			matrix::Eulerf euler(q);
			lpos.yaw = euler.psi();

			lpos.dist_bottom_valid = _ekf.get_terrain_valid();

			float terrain_vpos;
			_ekf.get_terrain_vert_pos(&terrain_vpos);
			lpos.dist_bottom = terrain_vpos - lpos.z; // Distance to bottom surface (ground) in meters

			// constrain the distance to ground to _rng_gnd_clearance
			if (lpos.dist_bottom < _rng_gnd_clearance.get()) {
				lpos.dist_bottom = _rng_gnd_clearance.get();
			}

			lpos.dist_bottom_rate = -lpos.vz; // Distance to bottom surface (ground) change rate

			_ekf.get_ekf_lpos_accuracy(&lpos.eph, &lpos.epv);
			_ekf.get_ekf_vel_accuracy(&lpos.evh, &lpos.evv);

			// get state reset information of position and velocity
			_ekf.get_posD_reset(&lpos.delta_z, &lpos.z_reset_counter);
			_ekf.get_velD_reset(&lpos.delta_vz, &lpos.vz_reset_counter);
			_ekf.get_posNE_reset(&lpos.delta_xy[0], &lpos.xy_reset_counter);
			_ekf.get_velNE_reset(&lpos.delta_vxy[0], &lpos.vxy_reset_counter);

			// get control limit information
			_ekf.get_ekf_ctrl_limits(&lpos.vxy_max, &lpos.vz_max, &lpos.hagl_min, &lpos.hagl_max);

			// convert NaN to INFINITY
			if (!PX4_ISFINITE(lpos.vxy_max)) {
				lpos.vxy_max = INFINITY;
			}

			if (!PX4_ISFINITE(lpos.vz_max)) {
				lpos.vz_max = INFINITY;
			}

			if (!PX4_ISFINITE(lpos.hagl_min)) {
				lpos.hagl_min = INFINITY;
			}

			if (!PX4_ISFINITE(lpos.hagl_max)) {
				lpos.hagl_max = INFINITY;
			}

			// publish vehicle local position data
			_vehicle_local_position_pub.update();

			if (_ekf.global_position_is_valid() && !_preflt_fail) {
				// generate and publish global position data
				vehicle_global_position_s &global_pos = _vehicle_global_position_pub.get();

				global_pos.timestamp = now;

				if (fabsf(lpos_x_prev - lpos.x) > FLT_EPSILON || fabsf(lpos_y_prev - lpos.y) > FLT_EPSILON) {
					map_projection_reproject(&ekf_origin, lpos.x, lpos.y, &global_pos.lat, &global_pos.lon);
				}

				global_pos.lat_lon_reset_counter = lpos.xy_reset_counter;

				global_pos.alt = -lpos.z + lpos.ref_alt; // Altitude AMSL in meters

				// global altitude has opposite sign of local down position
				global_pos.delta_alt = -lpos.delta_z;

				global_pos.vel_n = lpos.vx; // Ground north velocity, m/s
				global_pos.vel_e = lpos.vy; // Ground east velocity, m/s
				global_pos.vel_d = lpos.vz; // Ground downside velocity, m/s

				global_pos.yaw = lpos.yaw; // Yaw in radians -PI..+PI.

				_ekf.get_ekf_gpos_accuracy(&global_pos.eph, &global_pos.epv);

				global_pos.dead_reckoning = _ekf.inertial_dead_reckoning();

				global_pos.terrain_alt_valid = lpos.dist_bottom_valid;

				if (global_pos.terrain_alt_valid) {
					global_pos.terrain_alt = lpos.ref_alt - terrain_vpos; // Terrain altitude in m, WGS84

				} else {
					global_pos.terrain_alt = 0.0f; // Terrain altitude in m, WGS84
				}

				global_pos.dead_reckoning = _ekf.inertial_dead_reckoning(); // True if this position is estimated through dead-reckoning

				_vehicle_global_position_pub.update();
			}

			{
				// publish all corrected sensor readings and bias estimates after mag calibration is updated above
				float accel_bias[3];
				_ekf.get_accel_bias(accel_bias);

				sensor_bias_s bias;

				bias.timestamp = now;

				bias.accel_x = sensors.accelerometer_m_s2[0] - accel_bias[0];
				bias.accel_y = sensors.accelerometer_m_s2[1] - accel_bias[1];
				bias.accel_z = sensors.accelerometer_m_s2[2] - accel_bias[2];

				bias.gyro_x_bias = gyro_bias[0];
				bias.gyro_y_bias = gyro_bias[1];
				bias.gyro_z_bias = gyro_bias[2];

				bias.accel_x_bias = accel_bias[0];
				bias.accel_y_bias = accel_bias[1];
				bias.accel_z_bias = accel_bias[2];

				bias.mag_x_bias = _last_valid_mag_cal[0];
				bias.mag_y_bias = _last_valid_mag_cal[1];
				bias.mag_z_bias = _last_valid_mag_cal[2];

				if (_sensor_bias_pub == nullptr) {
					_sensor_bias_pub = orb_advertise(ORB_ID(sensor_bias), &bias);

				} else {
					orb_publish(ORB_ID(sensor_bias), _sensor_bias_pub, &bias);
				}
			}
		}

		// publish estimator status
		estimator_status_s status;
		status.timestamp = now;
		_ekf.get_state_delayed(status.states);
		_ekf.get_covariances(status.covariances);
		_ekf.get_gps_check_status(&status.gps_check_fail_flags);
		_ekf.get_control_mode(&status.control_mode_flags);
		_ekf.get_filter_fault_status(&status.filter_fault_flags);
		_ekf.get_innovation_test_status(&status.innovation_check_flags, &status.mag_test_ratio,
						&status.vel_test_ratio, &status.pos_test_ratio,
						&status.hgt_test_ratio, &status.tas_test_ratio,
						&status.hagl_test_ratio, &status.beta_test_ratio);

		status.pos_horiz_accuracy = _vehicle_local_position_pub.get().eph;
		status.pos_vert_accuracy = _vehicle_local_position_pub.get().epv;
		_ekf.get_ekf_soln_status(&status.solution_status_flags);
		_ekf.get_imu_vibe_metrics(status.vibe);
		status.time_slip = _last_time_slip_us / 1e6f;
		status.nan_flags = 0.0f; // unused
		status.health_flags = 0.0f; // unused
		status.timeout_flags = 0.0f; // unused
		status.pre_flt_fail = _preflt_fail;

		if (_estimator_status_pub == nullptr) {
			_estimator_status_pub = orb_advertise(ORB_ID(estimator_status), &status);

		} else {
			orb_publish(ORB_ID(estimator_status), _estimator_status_pub, &status);
		}

		if (updated) {
			{
				/* Check and save learned magnetometer bias estimates */

				// Check if conditions are OK to for learning of magnetometer bias values
				if (!vehicle_land_detected.landed && // not on ground
				    (vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED) && // vehicle is armed
				    (status.filter_fault_flags == 0) && // there are no filter faults
				    (status.control_mode_flags & (1 << 5))) { // the EKF is operating in the correct mode

					if (_last_magcal_us == 0) {
						_last_magcal_us = now;

					} else {
						_total_cal_time_us += now - _last_magcal_us;
						_last_magcal_us = now;
					}

				} else if (status.filter_fault_flags != 0) {
					// if a filter fault has occurred, assume previous learning was invalid and do not
					// count it towards total learning time.
					_total_cal_time_us = 0;

					for (bool &cal_available : _valid_cal_available) {
						cal_available = false;
					}
				}

				// Start checking mag bias estimates when we have accumulated sufficient calibration time
				if (_total_cal_time_us > 120 * 1000 * 1000ULL) {
					// we have sufficient accumulated valid flight time to form a reliable bias estimate
					// check that the state variance for each axis is within a range indicating filter convergence
					const float max_var_allowed = 100.0f * _mag_bias_saved_variance.get();
					const float min_var_allowed = 0.01f * _mag_bias_saved_variance.get();

					// Declare all bias estimates invalid if any variances are out of range
					bool all_estimates_invalid = false;

					for (uint8_t axis_index = 0; axis_index <= 2; axis_index++) {
						if (status.covariances[axis_index + 19] < min_var_allowed
						    || status.covariances[axis_index + 19] > max_var_allowed) {
							all_estimates_invalid = true;
						}
					}

					// Store valid estimates and their associated variances
					if (!all_estimates_invalid) {
						for (uint8_t axis_index = 0; axis_index <= 2; axis_index++) {
							_last_valid_mag_cal[axis_index] = status.states[axis_index + 19];
							_valid_cal_available[axis_index] = true;
							_last_valid_variance[axis_index] = status.covariances[axis_index + 19];
						}
					}
				}

				// Check and save the last valid calibration when we are disarmed
				if ((vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_STANDBY)
				    && (status.filter_fault_flags == 0)
				    && (sensor_selection.mag_device_id == _mag_bias_id.get())) {

					update_mag_bias(_mag_bias_x, 0);
					update_mag_bias(_mag_bias_y, 1);
					update_mag_bias(_mag_bias_z, 2);

					// reset to prevent data being saved too frequently
					_total_cal_time_us = 0;
				}

				publish_wind_estimate(now);
			}

			{
				// publish estimator innovation data
				ekf2_innovations_s innovations;
				innovations.timestamp = now;
				_ekf.get_vel_pos_innov(&innovations.vel_pos_innov[0]);
				_ekf.get_aux_vel_innov(&innovations.aux_vel_innov[0]);
				_ekf.get_mag_innov(&innovations.mag_innov[0]);
				_ekf.get_heading_innov(&innovations.heading_innov);
				_ekf.get_airspeed_innov(&innovations.airspeed_innov);
				_ekf.get_beta_innov(&innovations.beta_innov);
				_ekf.get_flow_innov(&innovations.flow_innov[0]);
				_ekf.get_hagl_innov(&innovations.hagl_innov);
				_ekf.get_drag_innov(&innovations.drag_innov[0]);

				_ekf.get_vel_pos_innov_var(&innovations.vel_pos_innov_var[0]);
				_ekf.get_mag_innov_var(&innovations.mag_innov_var[0]);
				_ekf.get_heading_innov_var(&innovations.heading_innov_var);
				_ekf.get_airspeed_innov_var(&innovations.airspeed_innov_var);
				_ekf.get_beta_innov_var(&innovations.beta_innov_var);
				_ekf.get_flow_innov_var(&innovations.flow_innov_var[0]);
				_ekf.get_hagl_innov_var(&innovations.hagl_innov_var);
				_ekf.get_drag_innov_var(&innovations.drag_innov_var[0]);

				_ekf.get_output_tracking_error(&innovations.output_tracking_error[0]);

				// calculate noise filtered velocity innovations which are used for pre-flight checking
				if (vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_STANDBY) {
					// calculate coefficients for LPF applied to innovation sequences
					float alpha = constrain(sensors.accelerometer_integral_dt / 1.e6f * _innov_lpf_tau_inv, 0.0f, 1.0f);
					float beta = 1.0f - alpha;

					// filter the velocity and innvovations
					_vel_ne_innov_lpf(0) = beta * _vel_ne_innov_lpf(0) + alpha * constrain(innovations.vel_pos_innov[0],
							       -_vel_innov_spike_lim, _vel_innov_spike_lim);
					_vel_ne_innov_lpf(1) = beta * _vel_ne_innov_lpf(1) + alpha * constrain(innovations.vel_pos_innov[1],
							       -_vel_innov_spike_lim, _vel_innov_spike_lim);
					_vel_d_innov_lpf = beta * _vel_d_innov_lpf + alpha * constrain(innovations.vel_pos_innov[2],
							   -_vel_innov_spike_lim, _vel_innov_spike_lim);

					// set the max allowed yaw innovaton depending on whether we are not aiding navigation using
					// observations in the NE reference frame.
					filter_control_status_u _ekf_control_mask;
					_ekf.get_control_mode(&_ekf_control_mask.value);
					bool doing_ne_aiding = _ekf_control_mask.flags.gps ||  _ekf_control_mask.flags.ev_pos;

					float yaw_test_limit;

					if (doing_ne_aiding) {
						// use a smaller tolerance when doing NE inertial frame aiding
						yaw_test_limit = _nav_yaw_innov_test_lim;

					} else {
						// use a larger tolerance when not doing NE inertial frame aiding
						yaw_test_limit = _yaw_innov_test_lim;
					}

					// filter the yaw innovations using a decaying envelope filter to prevent innovation sign changes due to angle wrapping allowinging large innvoations to pass checks after filtering.
					_yaw_innov_magnitude_lpf = fmaxf(beta * _yaw_innov_magnitude_lpf,
									 fminf(fabsf(innovations.heading_innov), 2.0f * yaw_test_limit));

					_hgt_innov_lpf = beta * _hgt_innov_lpf + alpha * constrain(innovations.vel_pos_innov[5], -_hgt_innov_spike_lim,
							 _hgt_innov_spike_lim);

					// check the yaw and horizontal velocity innovations
					float vel_ne_innov_length = sqrtf(innovations.vel_pos_innov[0] * innovations.vel_pos_innov[0] +
									  innovations.vel_pos_innov[1] * innovations.vel_pos_innov[1]);
					_preflt_horiz_fail = (_vel_ne_innov_lpf.norm() > _vel_innov_test_lim)
							     || (vel_ne_innov_length > 2.0f * _vel_innov_test_lim)
							     || (_yaw_innov_magnitude_lpf > yaw_test_limit);

					// check the vertical velocity and position innovations
					_preflt_vert_fail = (fabsf(_vel_d_innov_lpf) > _vel_innov_test_lim)
							    || (fabsf(innovations.vel_pos_innov[2]) > 2.0f * _vel_innov_test_lim)
							    || (fabsf(_hgt_innov_lpf) > _hgt_innov_test_lim);

					// master pass-fail status
					_preflt_fail = _preflt_horiz_fail || _preflt_vert_fail;

				} else {
					_vel_ne_innov_lpf.zero();
					_vel_d_innov_lpf = 0.0f;
					_hgt_innov_lpf = 0.0f;
					_preflt_horiz_fail = false;
					_preflt_vert_fail = false;
					_preflt_fail = false;
				}

				if (_estimator_innovations_pub == nullptr) {
					_estimator_innovations_pub = orb_advertise(ORB_ID(ekf2_innovations), &innovations);

				} else {
					orb_publish(ORB_ID(ekf2_innovations), _estimator_innovations_pub, &innovations);
				}
			}

		} else if (_replay_mode) {
			// in replay mode we have to tell the replay module not to wait for an update
			// we do this by publishing an attitude with zero timestamp
			vehicle_attitude_s att;
			att.timestamp = now;

			if (_att_pub == nullptr) {
				_att_pub = orb_advertise(ORB_ID(vehicle_attitude), &att);

			} else {
				orb_publish(ORB_ID(vehicle_attitude), _att_pub, &att);
			}
		}

		// publish ekf2_timestamps
		if (_ekf2_timestamps_pub == nullptr) {
			_ekf2_timestamps_pub = orb_advertise(ORB_ID(ekf2_timestamps), &ekf2_timestamps);

		} else {
			orb_publish(ORB_ID(ekf2_timestamps), _ekf2_timestamps_pub, &ekf2_timestamps);
		}
	}
}

int Ekf2::getRangeSubIndex(const int *subs)
{
	for (unsigned i = 0; i < ORB_MULTI_MAX_INSTANCES; i++) {
		bool updated = false;
		orb_check(subs[i], &updated);

		if (updated) {
			distance_sensor_s report;
			orb_copy(ORB_ID(distance_sensor), subs[i], &report);

			// only use the first instace which has the correct orientation
			if (report.orientation == distance_sensor_s::ROTATION_DOWNWARD_FACING) {
				PX4_INFO("Found range finder with instance %d", i);
				return i;
			}
		}
	}

	return -1;
}

bool Ekf2::publish_wind_estimate(const hrt_abstime &timestamp)
{
	bool published = false;

	if (_ekf.get_wind_status()) {
		float velNE_wind[2];
		_ekf.get_wind_velocity(velNE_wind);

		float wind_var[2];
		_ekf.get_wind_velocity_var(wind_var);

		// Publish wind estimate
		wind_estimate_s wind_estimate;
		wind_estimate.timestamp = timestamp;
		wind_estimate.windspeed_north = velNE_wind[0];
		wind_estimate.windspeed_east = velNE_wind[1];
		wind_estimate.variance_north = wind_var[0];
		wind_estimate.variance_east = wind_var[1];

		if (_wind_pub == nullptr) {
			_wind_pub = orb_advertise(ORB_ID(wind_estimate), &wind_estimate);

		} else {
			orb_publish(ORB_ID(wind_estimate), _wind_pub, &wind_estimate);
		}

		published = true;
	}

	return published;
}

const Vector3f Ekf2::get_vel_body_wind()
{
	// Used to correct baro data for positional errors

	matrix::Quatf q;
	_ekf.copy_quaternion(q.data());
	matrix::Dcmf R_to_body(q.inversed());

	// Calculate wind-compensated velocity in body frame
	// Velocity of body origin in local NED frame (m/s)
	float velocity[3];
	_ekf.get_velocity(velocity);

	float velNE_wind[2];
	_ekf.get_wind_velocity(velNE_wind);

	Vector3f v_wind_comp = {velocity[0] - velNE_wind[0], velocity[1] - velNE_wind[1], velocity[2]};

	return R_to_body * v_wind_comp;
}

Ekf2 *Ekf2::instantiate(int argc, char *argv[])
{
	Ekf2 *instance = new Ekf2();

	if (instance) {
		if (argc >= 2 && !strcmp(argv[1], "-r")) {
			instance->set_replay_mode(true);
		}
	}

	return instance;
}

int Ekf2::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int Ekf2::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Attitude and position estimator using an Extended Kalman Filter. It is used for Multirotors and Fixed-Wing.

The documentation can be found on the [tuning_the_ecl_ekf](https://dev.px4.io/en/tutorials/tuning_the_ecl_ekf.html) page.

ekf2 can be started in replay mode (`-r`): in this mode it does not access the system time, but only uses the
timestamps from the sensor topics.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("ekf2", "estimator");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_FLAG('r', "Enable replay mode", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int Ekf2::task_spawn(int argc, char *argv[])
{
	_task_id = px4_task_spawn_cmd("ekf2",
				      SCHED_DEFAULT,
				      SCHED_PRIORITY_ESTIMATOR,
				      6600,
				      (px4_main_t)&run_trampoline,
				      (char *const *)argv);

	if (_task_id < 0) {
		_task_id = -1;
		return -errno;
	}

	return 0;
}

int ekf2_main(int argc, char *argv[])
{
	return Ekf2::main(argc, argv);
}
