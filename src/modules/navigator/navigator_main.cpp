/****************************************************************************
 *
 *   Copyright (c) 2013-2017 PX4 Development Team. All rights reserved.
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
 * @file navigator_main.cpp
 *
 * Handles mission items, geo fencing and failsafe navigation behavior.
 * Published the position setpoint triplet for the position controller.
 *
 * @author Lorenz Meier <lorenz@px4.io>
 * @author Jean Cyr <jean.m.cyr@gmail.com>
 * @author Julian Oes <julian@oes.ch>
 * @author Anton Babushkin <anton.babushkin@me.com>
 * @author Thomas Gubler <thomasgubler@gmail.com>
 */

#include "navigator.h"

#include <cfloat>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <dataman/dataman.h>
#include <drivers/drv_hrt.h>
#include <lib/ecl/geo/geo.h>
#include <mathlib/mathlib.h>
#include <px4_config.h>
#include <px4_defines.h>
#include <px4_posix.h>
#include <px4_tasks.h>
#include <systemlib/mavlink_log.h>
#include <uORB/topics/fw_pos_ctrl_status.h>
#include <uORB/topics/home_position.h>
#include <uORB/topics/mission.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_command_ack.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/transponder_report.h>
#include <uORB/uORB.h>

/**
 * navigator app start / stop handling function
 *
 * @ingroup apps
 */
extern "C" __EXPORT int navigator_main(int argc, char *argv[]);

#define GEOFENCE_CHECK_INTERVAL 200000

namespace navigator
{
Navigator	*g_navigator;
}

Navigator::Navigator() :
	ModuleParams(nullptr),
	_loop_perf(perf_alloc(PC_ELAPSED, "navigator")),
	_geofence(this),
	_mission(this),
	_loiter(this),
	_takeoff(this),
	_land(this),
	_precland(this),
	_rtl(this),
	_rcLoss(this),
	_dataLinkLoss(this),
	_engineFailure(this),
	_gpsFailure(this),
	_follow_target(this)
{
	/* Create a list of our possible navigation types */
	_navigation_mode_array[0] = &_mission;
	_navigation_mode_array[1] = &_loiter;
	_navigation_mode_array[2] = &_rtl;
	_navigation_mode_array[3] = &_dataLinkLoss;
	_navigation_mode_array[4] = &_engineFailure;
	_navigation_mode_array[5] = &_gpsFailure;
	_navigation_mode_array[6] = &_rcLoss;
	_navigation_mode_array[7] = &_takeoff;
	_navigation_mode_array[8] = &_land;
	_navigation_mode_array[9] = &_precland;
	_navigation_mode_array[10] = &_follow_target;
}

void
Navigator::global_position_update()
{
	orb_copy(ORB_ID(vehicle_global_position), _global_pos_sub, &_global_pos);
}

void
Navigator::local_position_update()
{
	orb_copy(ORB_ID(vehicle_local_position), _local_pos_sub, &_local_pos);
}

void
Navigator::gps_position_update()
{
	orb_copy(ORB_ID(vehicle_gps_position), _gps_pos_sub, &_gps_pos);
}

void
Navigator::home_position_update(bool force)
{
	bool updated = false;
	orb_check(_home_pos_sub, &updated);

	if (updated || force) {
		orb_copy(ORB_ID(home_position), _home_pos_sub, &_home_pos);
	}
}

void
Navigator::fw_pos_ctrl_status_update(bool force)
{
	bool updated = false;
	orb_check(_fw_pos_ctrl_status_sub, &updated);

	if (updated || force) {
		orb_copy(ORB_ID(fw_pos_ctrl_status), _fw_pos_ctrl_status_sub, &_fw_pos_ctrl_status);
	}
}

void
Navigator::vehicle_status_update()
{
	if (orb_copy(ORB_ID(vehicle_status), _vstatus_sub, &_vstatus) != OK) {
		/* in case the commander is not be running */
		_vstatus.arming_state = vehicle_status_s::ARMING_STATE_STANDBY;
	}
}

void
Navigator::vehicle_land_detected_update()
{
	orb_copy(ORB_ID(vehicle_land_detected), _land_detected_sub, &_land_detected);
}

void
Navigator::params_update()
{
	parameter_update_s param_update;
	orb_copy(ORB_ID(parameter_update), _param_update_sub, &param_update);
	updateParams();
}

void
Navigator::run()
{
	bool have_geofence_position_data = false;

	/* Try to load the geofence:
	 * if /fs/microsd/etc/geofence.txt load from this file */
	struct stat buffer;

	if (stat(GEOFENCE_FILENAME, &buffer) == 0) {
		PX4_INFO("Loading geofence from %s", GEOFENCE_FILENAME);
		_geofence.loadFromFile(GEOFENCE_FILENAME);
	}

	/* do subscriptions */
	_global_pos_sub = orb_subscribe(ORB_ID(vehicle_global_position));
	_local_pos_sub = orb_subscribe(ORB_ID(vehicle_local_position));
	_gps_pos_sub = orb_subscribe(ORB_ID(vehicle_gps_position));
	_fw_pos_ctrl_status_sub = orb_subscribe(ORB_ID(fw_pos_ctrl_status));
	_vstatus_sub = orb_subscribe(ORB_ID(vehicle_status));
	_land_detected_sub = orb_subscribe(ORB_ID(vehicle_land_detected));
	_home_pos_sub = orb_subscribe(ORB_ID(home_position));
	_offboard_mission_sub = orb_subscribe(ORB_ID(mission));
	_param_update_sub = orb_subscribe(ORB_ID(parameter_update));
	_vehicle_command_sub = orb_subscribe(ORB_ID(vehicle_command));
	_traffic_sub = orb_subscribe(ORB_ID(transponder_report));

	/* copy all topics first time */
	vehicle_status_update();
	vehicle_land_detected_update();
	global_position_update();
	local_position_update();
	gps_position_update();
	home_position_update(true);
	fw_pos_ctrl_status_update(true);
	params_update();

	/* wakeup source(s) */
	px4_pollfd_struct_t fds[1] = {};

	/* Setup of loop */
	fds[0].fd = _local_pos_sub;
	fds[0].events = POLLIN;

	/* rate-limit position subscription to 20 Hz / 50 ms */
	orb_set_interval(_local_pos_sub, 50);

	hrt_abstime last_geofence_check = 0;

	while (!should_exit()) {

		/* wait for up to 1000ms for data */
		int pret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 1000);

		if (pret == 0) {
			/* Let the loop run anyway, don't do `continue` here. */

		} else if (pret < 0) {
			/* this is undesirable but not much we can do - might want to flag unhappy status */
			PX4_ERR("poll error %d, %d", pret, errno);
			usleep(10000);
			continue;

		} else {
			if (fds[0].revents & POLLIN) {
				/* success, local pos is available */
				local_position_update();
			}
		}

		perf_begin(_loop_perf);

		bool updated;

		/* gps updated */
		orb_check(_gps_pos_sub, &updated);

		if (updated) {
			gps_position_update();

			if (_geofence.getSource() == Geofence::GF_SOURCE_GPS) {
				have_geofence_position_data = true;
			}
		}

		/* global position updated */
		orb_check(_global_pos_sub, &updated);

		if (updated) {
			global_position_update();

			if (_geofence.getSource() == Geofence::GF_SOURCE_GLOBALPOS) {
				have_geofence_position_data = true;
			}
		}

		/* parameters updated */
		orb_check(_param_update_sub, &updated);

		if (updated) {
			params_update();
		}

		/* vehicle status updated */
		orb_check(_vstatus_sub, &updated);

		if (updated) {
			vehicle_status_update();
		}

		/* vehicle land detected updated */
		orb_check(_land_detected_sub, &updated);

		if (updated) {
			vehicle_land_detected_update();
		}

		/* navigation capabilities updated */
		orb_check(_fw_pos_ctrl_status_sub, &updated);

		if (updated) {
			fw_pos_ctrl_status_update();
		}

		/* home position updated */
		orb_check(_home_pos_sub, &updated);

		if (updated) {
			home_position_update();
		}

		/* vehicle_command updated */
		orb_check(_vehicle_command_sub, &updated);

		if (updated) {
			vehicle_command_s cmd;
			orb_copy(ORB_ID(vehicle_command), _vehicle_command_sub, &cmd);

			if (cmd.command == vehicle_command_s::VEHICLE_CMD_DO_GO_AROUND) {

				// DO_GO_AROUND is currently handled by the position controller (unacknowledged)
				// TODO: move DO_GO_AROUND handling to navigator
				publish_vehicle_command_ack(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED);

			} else if (cmd.command == vehicle_command_s::VEHICLE_CMD_DO_REPOSITION) {

				position_setpoint_triplet_s *rep = get_reposition_triplet();
				position_setpoint_triplet_s *curr = get_position_setpoint_triplet();

				// store current position as previous position and goal as next
				rep->previous.yaw = get_global_position()->yaw;
				rep->previous.lat = get_global_position()->lat;
				rep->previous.lon = get_global_position()->lon;
				rep->previous.alt = get_global_position()->alt;

				rep->current.loiter_radius = get_loiter_radius();
				rep->current.loiter_direction = 1;
				rep->current.type = position_setpoint_s::SETPOINT_TYPE_LOITER;
				rep->current.cruising_speed = get_cruising_speed();
				rep->current.cruising_throttle = get_cruising_throttle();

				// Go on and check which changes had been requested
				if (PX4_ISFINITE(cmd.param4)) {
					rep->current.yaw = cmd.param4;

				} else {
					rep->current.yaw = NAN;
				}

				if (PX4_ISFINITE(cmd.param5) && PX4_ISFINITE(cmd.param6)) {

					// Position change with optional altitude change
					rep->current.lat = (cmd.param5 < 1000) ? cmd.param5 : cmd.param5 / (double)1e7;
					rep->current.lon = (cmd.param6 < 1000) ? cmd.param6 : cmd.param6 / (double)1e7;

					if (PX4_ISFINITE(cmd.param7)) {
						rep->current.alt = cmd.param7;

					} else {
						rep->current.alt = get_global_position()->alt;
					}

				} else if (PX4_ISFINITE(cmd.param7) && curr->current.valid
					   && PX4_ISFINITE(curr->current.lat)
					   && PX4_ISFINITE(curr->current.lon)) {

					// Altitude without position change
					rep->current.lat = curr->current.lat;
					rep->current.lon = curr->current.lon;
					rep->current.alt = cmd.param7;

				} else {
					// All three set to NaN - hold in current position
					rep->current.lat = get_global_position()->lat;
					rep->current.lon = get_global_position()->lon;
					rep->current.alt = get_global_position()->alt;
				}

				rep->previous.valid = true;
				rep->current.valid = true;
				rep->next.valid = false;

				// CMD_DO_REPOSITION is acknowledged by commander

			} else if (cmd.command == vehicle_command_s::VEHICLE_CMD_NAV_TAKEOFF) {
				position_setpoint_triplet_s *rep = get_takeoff_triplet();

				// store current position as previous position and goal as next
				rep->previous.yaw = get_global_position()->yaw;
				rep->previous.lat = get_global_position()->lat;
				rep->previous.lon = get_global_position()->lon;
				rep->previous.alt = get_global_position()->alt;

				rep->current.loiter_radius = get_loiter_radius();
				rep->current.loiter_direction = 1;
				rep->current.type = position_setpoint_s::SETPOINT_TYPE_TAKEOFF;

				if (home_position_valid()) {
					rep->current.yaw = cmd.param4;
					rep->previous.valid = true;

				} else {
					rep->current.yaw = get_local_position()->yaw;
					rep->previous.valid = false;
				}

				if (PX4_ISFINITE(cmd.param5) && PX4_ISFINITE(cmd.param6)) {
					rep->current.lat = (cmd.param5 < 1000) ? cmd.param5 : cmd.param5 / (double)1e7;
					rep->current.lon = (cmd.param6 < 1000) ? cmd.param6 : cmd.param6 / (double)1e7;

				} else {
					// If one of them is non-finite, reset both
					rep->current.lat = NAN;
					rep->current.lon = NAN;
				}

				rep->current.alt = cmd.param7;

				rep->current.valid = true;
				rep->next.valid = false;

				// CMD_NAV_TAKEOFF is acknowledged by commander

			} else if (cmd.command == vehicle_command_s::VEHICLE_CMD_DO_LAND_START) {

				/* find NAV_CMD_DO_LAND_START in the mission and
				 * use MAV_CMD_MISSION_START to start the mission there
				 */
				if (_mission.land_start()) {
					vehicle_command_s vcmd = {};
					vcmd.command = vehicle_command_s::VEHICLE_CMD_MISSION_START;
					vcmd.param1 = _mission.get_land_start_index();
					publish_vehicle_cmd(&vcmd);

				} else {
					PX4_WARN("planned mission landing not available");
				}

				publish_vehicle_command_ack(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED);

			} else if (cmd.command == vehicle_command_s::VEHICLE_CMD_MISSION_START) {
				if (_mission_result.valid && PX4_ISFINITE(cmd.param1) && (cmd.param1 >= 0)) {
					if (!_mission.set_current_offboard_mission_index(cmd.param1)) {
						PX4_WARN("CMD_MISSION_START failed");
					}
				}

				// CMD_MISSION_START is acknowledged by commander

			} else if (cmd.command == vehicle_command_s::VEHICLE_CMD_DO_CHANGE_SPEED) {
				if (cmd.param2 > FLT_EPSILON) {
					// XXX not differentiating ground and airspeed yet
					set_cruising_speed(cmd.param2);

				} else {
					set_cruising_speed();

					/* if no speed target was given try to set throttle */
					if (cmd.param3 > FLT_EPSILON) {
						set_cruising_throttle(cmd.param3 / 100);

					} else {
						set_cruising_throttle();
					}
				}

				// TODO: handle responses for supported DO_CHANGE_SPEED options?
				publish_vehicle_command_ack(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED);

			} else if (cmd.command == vehicle_command_s::VEHICLE_CMD_DO_SET_ROI
				   || cmd.command == vehicle_command_s::VEHICLE_CMD_NAV_ROI
				   || cmd.command == vehicle_command_s::VEHICLE_CMD_DO_SET_ROI_LOCATION
				   || cmd.command == vehicle_command_s::VEHICLE_CMD_DO_SET_ROI_WPNEXT_OFFSET
				   || cmd.command == vehicle_command_s::VEHICLE_CMD_DO_SET_ROI_NONE) {
				_vroi = {};

				switch (cmd.command) {
				case vehicle_command_s::VEHICLE_CMD_DO_SET_ROI:
				case vehicle_command_s::VEHICLE_CMD_NAV_ROI:
					_vroi.mode = cmd.param1;
					break;

				case vehicle_command_s::VEHICLE_CMD_DO_SET_ROI_LOCATION:
					_vroi.mode = vehicle_command_s::VEHICLE_ROI_LOCATION;
					_vroi.lat = cmd.param5;
					_vroi.lon = cmd.param6;
					_vroi.alt = cmd.param7;
					break;

				case vehicle_command_s::VEHICLE_CMD_DO_SET_ROI_WPNEXT_OFFSET:
					_vroi.mode = vehicle_command_s::VEHICLE_ROI_WPNEXT;
					_vroi.pitch_offset = (float)cmd.param5 * M_DEG_TO_RAD_F;
					_vroi.roll_offset = (float)cmd.param6 * M_DEG_TO_RAD_F;
					_vroi.yaw_offset = (float)cmd.param7 * M_DEG_TO_RAD_F;
					break;

				case vehicle_command_s::VEHICLE_CMD_DO_SET_ROI_NONE:
					_vroi.mode = vehicle_command_s::VEHICLE_ROI_NONE;
					break;

				default:
					_vroi.mode = vehicle_command_s::VEHICLE_ROI_NONE;
					break;
				}

				_vroi.timestamp = hrt_absolute_time();

				if (_vehicle_roi_pub != nullptr) {
					orb_publish(ORB_ID(vehicle_roi), _vehicle_roi_pub, &_vroi);

				} else {
					_vehicle_roi_pub = orb_advertise(ORB_ID(vehicle_roi), &_vroi);
				}

				publish_vehicle_command_ack(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED);
			}
		}

		/* Check for traffic */
		check_traffic();

		/* Check geofence violation */
		if (have_geofence_position_data &&
		    (_geofence.getGeofenceAction() != geofence_result_s::GF_ACTION_NONE) &&
		    (hrt_elapsed_time(&last_geofence_check) > GEOFENCE_CHECK_INTERVAL)) {

			bool inside = _geofence.check(_global_pos, _gps_pos, _home_pos,
						      home_position_valid());
			last_geofence_check = hrt_absolute_time();
			have_geofence_position_data = false;

			_geofence_result.timestamp = hrt_absolute_time();
			_geofence_result.geofence_action = _geofence.getGeofenceAction();
			_geofence_result.home_required = _geofence.isHomeRequired();

			if (!inside) {
				/* inform other apps via the mission result */
				_geofence_result.geofence_violated = true;

				/* Issue a warning about the geofence violation once */
				if (!_geofence_violation_warning_sent) {
					mavlink_log_critical(&_mavlink_log_pub, "Geofence violation");
					_geofence_violation_warning_sent = true;
				}

			} else {
				/* inform other apps via the mission result */
				_geofence_result.geofence_violated = false;

				/* Reset the _geofence_violation_warning_sent field */
				_geofence_violation_warning_sent = false;
			}

			publish_geofence_result();
		}

		/* Do stuff according to navigation state set by commander */
		NavigatorMode *navigation_mode_new{nullptr};

		switch (_vstatus.nav_state) {
		case vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION:
			_pos_sp_triplet_published_invalid_once = false;

			_mission.set_execution_mode(mission_result_s::MISSION_EXECUTION_MODE_NORMAL);
			navigation_mode_new = &_mission;

			break;

		case vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER:
			_pos_sp_triplet_published_invalid_once = false;
			navigation_mode_new = &_loiter;
			break;

		case vehicle_status_s::NAVIGATION_STATE_AUTO_RCRECOVER:
			_pos_sp_triplet_published_invalid_once = false;
			navigation_mode_new = &_rcLoss;
			break;

		case vehicle_status_s::NAVIGATION_STATE_AUTO_RTL: {
				_pos_sp_triplet_published_invalid_once = false;

				const bool rtl_activated = _previous_nav_state != vehicle_status_s::NAVIGATION_STATE_AUTO_RTL;

				switch (rtl_type()) {
				case RTL::RTL_LAND:
					if (rtl_activated) {
						mavlink_and_console_log_info(get_mavlink_log_pub(), "RTL LAND activated");
					}

					// if RTL is set to use a mission landing and mission has a planned landing, then use MISSION to fly there directly
					if (on_mission_landing() && !get_land_detected()->landed) {
						_mission.set_execution_mode(mission_result_s::MISSION_EXECUTION_MODE_FAST_FORWARD);
						navigation_mode_new = &_mission;

					} else {
						navigation_mode_new = &_rtl;
					}

					break;

				case RTL::RTL_MISSION:
					if (_mission.get_land_start_available() && !get_land_detected()->landed) {
						// the mission contains a landing spot
						_mission.set_execution_mode(mission_result_s::MISSION_EXECUTION_MODE_FAST_FORWARD);

						if (_navigation_mode != &_mission) {
							if (_navigation_mode == nullptr) {
								// switching from an manual mode, go to landing if not already landing
								if (!on_mission_landing()) {
									start_mission_landing();
								}

							} else {
								// switching from an auto mode, continue the mission from the closest item
								_mission.set_closest_item_as_current();
							}
						}

						if (rtl_activated) {
							mavlink_and_console_log_info(get_mavlink_log_pub(), "RTL Mission activated, continue mission");
						}

						navigation_mode_new = &_mission;

					} else {
						// fly the mission in reverse if switching from a non-manual mode
						_mission.set_execution_mode(mission_result_s::MISSION_EXECUTION_MODE_REVERSE);

						if ((_navigation_mode != nullptr && (_navigation_mode != &_rtl || _mission.get_mission_changed())) &&
						    (! _mission.get_mission_finished()) &&
						    (!get_land_detected()->landed)) {
							// determine the closest mission item if switching from a non-mission mode, and we are either not already
							// mission mode or the mission waypoints changed.
							// The seconds condition is required so that when no mission was uploaded and one is available the closest
							// mission item is determined and also that if the user changes the active mission index while rtl is active
							// always that waypoint is tracked first.
							if ((_navigation_mode != &_mission) && (rtl_activated || _mission.get_mission_waypoints_changed())) {
								_mission.set_closest_item_as_current();
							}

							if (rtl_activated) {
								mavlink_and_console_log_info(get_mavlink_log_pub(), "RTL Mission activated, fly mission in reverse");
							}

							navigation_mode_new = &_mission;

						} else {
							if (rtl_activated) {
								mavlink_and_console_log_info(get_mavlink_log_pub(), "RTL Mission activated, fly to home");
							}

							navigation_mode_new = &_rtl;
						}
					}

					break;

				default:
					if (rtl_activated) {
						mavlink_and_console_log_info(get_mavlink_log_pub(), "RTL HOME activated");
					}

					navigation_mode_new = &_rtl;
					break;
				}

				break;
			}

		case vehicle_status_s::NAVIGATION_STATE_AUTO_TAKEOFF:
			_pos_sp_triplet_published_invalid_once = false;
			navigation_mode_new = &_takeoff;
			break;

		case vehicle_status_s::NAVIGATION_STATE_AUTO_LAND:
			_pos_sp_triplet_published_invalid_once = false;
			navigation_mode_new = &_land;
			break;

		case vehicle_status_s::NAVIGATION_STATE_AUTO_PRECLAND:
			_pos_sp_triplet_published_invalid_once = false;
			navigation_mode_new = &_precland;
			_precland.set_mode(PrecLandMode::Required);
			break;

		case vehicle_status_s::NAVIGATION_STATE_DESCEND:
			_pos_sp_triplet_published_invalid_once = false;
			navigation_mode_new = &_land;
			break;

		case vehicle_status_s::NAVIGATION_STATE_AUTO_RTGS:
			_pos_sp_triplet_published_invalid_once = false;
			navigation_mode_new = &_dataLinkLoss;
			break;

		case vehicle_status_s::NAVIGATION_STATE_AUTO_LANDENGFAIL:
			_pos_sp_triplet_published_invalid_once = false;
			navigation_mode_new = &_engineFailure;
			break;

		case vehicle_status_s::NAVIGATION_STATE_AUTO_LANDGPSFAIL:
			_pos_sp_triplet_published_invalid_once = false;
			navigation_mode_new = &_gpsFailure;
			break;

		case vehicle_status_s::NAVIGATION_STATE_AUTO_FOLLOW_TARGET:
			_pos_sp_triplet_published_invalid_once = false;
			navigation_mode_new = &_follow_target;
			break;

		case vehicle_status_s::NAVIGATION_STATE_MANUAL:
		case vehicle_status_s::NAVIGATION_STATE_ACRO:
		case vehicle_status_s::NAVIGATION_STATE_ALTCTL:
		case vehicle_status_s::NAVIGATION_STATE_POSCTL:
		case vehicle_status_s::NAVIGATION_STATE_TERMINATION:
		case vehicle_status_s::NAVIGATION_STATE_OFFBOARD:
		case vehicle_status_s::NAVIGATION_STATE_STAB:
		default:
			_pos_sp_triplet_published_invalid_once = false;
			navigation_mode_new = nullptr;
			_can_loiter_at_sp = false;
			break;
		}

		// update the vehicle status
		_previous_nav_state = _vstatus.nav_state;

		/* we have a new navigation mode: reset triplet */
		if (_navigation_mode != navigation_mode_new) {
			// We don't reset the triplet if we just did an auto-takeoff and are now
			// going to loiter. Otherwise, we lose the takeoff altitude and end up lower
			// than where we wanted to go.
			//
			// FIXME: a better solution would be to add reset where they are needed and remove
			//        this general reset here.
			if (!(_navigation_mode == &_takeoff &&
			      navigation_mode_new == &_loiter)) {
				reset_triplets();
			}
		}

		_navigation_mode = navigation_mode_new;

		/* iterate through navigation modes and set active/inactive for each */
		for (unsigned int i = 0; i < NAVIGATOR_MODE_ARRAY_SIZE; i++) {
			_navigation_mode_array[i]->run(_navigation_mode == _navigation_mode_array[i]);
		}

		/* if we landed and have not received takeoff setpoint then stay in idle */
		if (_land_detected.landed &&
		    !((_vstatus.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_TAKEOFF)
		      || (_vstatus.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION))) {

			_pos_sp_triplet.current.type = position_setpoint_s::SETPOINT_TYPE_IDLE;
			_pos_sp_triplet.current.valid = true;
			_pos_sp_triplet.previous.valid = false;
			_pos_sp_triplet.next.valid = false;

		}

		/* if nothing is running, set position setpoint triplet invalid once */
		if (_navigation_mode == nullptr && !_pos_sp_triplet_published_invalid_once) {
			_pos_sp_triplet_published_invalid_once = true;
			reset_triplets();
		}

		if (_pos_sp_triplet_updated) {
			_pos_sp_triplet.timestamp = hrt_absolute_time();
			publish_position_setpoint_triplet();
			_pos_sp_triplet_updated = false;
		}

		if (_mission_result_updated) {
			publish_mission_result();
			_mission_result_updated = false;
		}

		perf_end(_loop_perf);
	}

	orb_unsubscribe(_global_pos_sub);
	orb_unsubscribe(_local_pos_sub);
	orb_unsubscribe(_gps_pos_sub);
	orb_unsubscribe(_fw_pos_ctrl_status_sub);
	orb_unsubscribe(_vstatus_sub);
	orb_unsubscribe(_land_detected_sub);
	orb_unsubscribe(_home_pos_sub);
	orb_unsubscribe(_offboard_mission_sub);
	orb_unsubscribe(_param_update_sub);
	orb_unsubscribe(_vehicle_command_sub);
}

int Navigator::task_spawn(int argc, char *argv[])
{
	_task_id = px4_task_spawn_cmd("navigator",
				      SCHED_DEFAULT,
				      SCHED_PRIORITY_NAVIGATION,
				      1800,
				      (px4_main_t)&run_trampoline,
				      (char *const *)argv);

	if (_task_id < 0) {
		_task_id = -1;
		return -errno;
	}

	return 0;
}

Navigator *Navigator::instantiate(int argc, char *argv[])
{
	Navigator *instance = new Navigator();

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
	}

	return instance;
}

int
Navigator::print_status()
{
	PX4_INFO("Running");

	_geofence.printStatus();
	return 0;
}

void
Navigator::publish_position_setpoint_triplet()
{
	/* do not publish an empty triplet */
	if (!_pos_sp_triplet.current.valid) {
		return;
	}

	/* lazily publish the position setpoint triplet only once available */
	if (_pos_sp_triplet_pub != nullptr) {
		orb_publish(ORB_ID(position_setpoint_triplet), _pos_sp_triplet_pub, &_pos_sp_triplet);

	} else {
		_pos_sp_triplet_pub = orb_advertise(ORB_ID(position_setpoint_triplet), &_pos_sp_triplet);
	}
}

float
Navigator::get_default_acceptance_radius()
{
	return _param_acceptance_radius.get();
}

float
Navigator::get_acceptance_radius()
{
	return get_acceptance_radius(_param_acceptance_radius.get());
}

float
Navigator::get_altitude_acceptance_radius()
{
	if (!get_vstatus()->is_rotary_wing) {
		return _param_fw_alt_acceptance_radius.get();

	} else {
		return _param_mc_alt_acceptance_radius.get();
	}
}

float
Navigator::get_cruising_speed()
{
	/* there are three options: The mission-requested cruise speed, or the current hover / plane speed */
	if (_vstatus.is_rotary_wing) {
		if (is_planned_mission() && _mission_cruising_speed_mc > 0.0f) {
			return _mission_cruising_speed_mc;

		} else {
			return -1.0f;
		}

	} else {
		if (is_planned_mission() && _mission_cruising_speed_fw > 0.0f) {
			return _mission_cruising_speed_fw;

		} else {
			return -1.0f;
		}
	}
}

void
Navigator::set_cruising_speed(float speed)
{
	if (_vstatus.is_rotary_wing) {
		_mission_cruising_speed_mc = speed;

	} else {
		_mission_cruising_speed_fw = speed;
	}
}

void
Navigator::reset_cruising_speed()
{
	_mission_cruising_speed_mc = -1.0f;
	_mission_cruising_speed_fw = -1.0f;
}

void
Navigator::reset_triplets()
{
	_pos_sp_triplet.current.valid = false;
	_pos_sp_triplet.previous.valid = false;
	_pos_sp_triplet.next.valid = false;
	_pos_sp_triplet_updated = true;
}

float
Navigator::get_cruising_throttle()
{
	/* Return the mission-requested cruise speed, or default FW_THR_CRUISE value */
	if (_mission_throttle > FLT_EPSILON) {
		return _mission_throttle;

	} else {
		return -1.0f;
	}
}

float
Navigator::get_acceptance_radius(float mission_item_radius)
{
	float radius = mission_item_radius;

	// XXX only use navigation capabilities for now
	// when in fixed wing mode
	// this might need locking against a commanded transition
	// so that a stale _vstatus doesn't trigger an accepted mission item.
//	if (!_vstatus.is_rotary_wing && !_vstatus.in_transition_mode) {
//		if ((hrt_elapsed_time(&_fw_pos_ctrl_status.timestamp) < 5000000) && (_fw_pos_ctrl_status.turn_distance > radius)) {
//			radius = _fw_pos_ctrl_status.turn_distance;
//		}
//	}

	return radius;
}

void
Navigator::load_fence_from_file(const char *filename)
{
	_geofence.loadFromFile(filename);
}

/**
 * Creates a fake traffic measurement with supplied parameters.
 *
 */
void Navigator::fake_traffic(const char *callsign, float distance, float direction, float traffic_heading,
			     float altitude_diff, float hor_velocity, float ver_velocity)
{
	double lat, lon;
	waypoint_from_heading_and_distance(get_global_position()->lat, get_global_position()->lon, direction, distance, &lat,
					   &lon);
	float alt = get_global_position()->alt + altitude_diff;

	// float vel_n = get_global_position()->vel_n;
	// float vel_e = get_global_position()->vel_e;
	// float vel_d = get_global_position()->vel_d;

	transponder_report_s tr = {};
	tr.timestamp = hrt_absolute_time();
	tr.ICAO_address = 1234;
	tr.lat = lat; // Latitude, expressed as degrees
	tr.lon = lon; // Longitude, expressed as degrees
	tr.altitude_type = 0;
	tr.altitude = alt;
	tr.heading = traffic_heading; //-atan2(vel_e, vel_n); // Course over ground in radians
	tr.hor_velocity	= hor_velocity; //sqrtf(vel_e * vel_e + vel_n * vel_n); // The horizontal velocity in m/s
	tr.ver_velocity = ver_velocity; //-vel_d; // The vertical velocity in m/s, positive is up
	strncpy(&tr.callsign[0], callsign, sizeof(tr.callsign) - 1);
	tr.callsign[sizeof(tr.callsign) - 1] = 0;
	tr.emitter_type = 0; // Type from ADSB_EMITTER_TYPE enum
	tr.tslc = 2; // Time since last communication in seconds
	tr.flags = transponder_report_s::PX4_ADSB_FLAGS_VALID_COORDS | transponder_report_s::PX4_ADSB_FLAGS_VALID_HEADING |
		   transponder_report_s::PX4_ADSB_FLAGS_VALID_VELOCITY |
		   transponder_report_s::PX4_ADSB_FLAGS_VALID_ALTITUDE |
		   transponder_report_s::PX4_ADSB_FLAGS_VALID_CALLSIGN; // Flags to indicate various statuses including valid data fields
	tr.squawk = 6667;

	orb_advert_t h = orb_advertise_queue(ORB_ID(transponder_report), &tr, transponder_report_s::ORB_QUEUE_LENGTH);
	(void)orb_unadvertise(h);
}

void Navigator::check_traffic()
{
	double lat = get_global_position()->lat;
	double lon = get_global_position()->lon;
	float alt = get_global_position()->alt;

	// TODO for non-multirotors predicting the future
	// position as accurately as possible will become relevant
	// float vel_n = get_global_position()->vel_n;
	// float vel_e = get_global_position()->vel_e;
	// float vel_d = get_global_position()->vel_d;

	bool changed;
	orb_check(_traffic_sub, &changed);

	float horizontal_separation = 500;
	float vertical_separation = 500;

	while (changed) {

		transponder_report_s tr;
		orb_copy(ORB_ID(transponder_report), _traffic_sub, &tr);

		uint16_t required_flags = transponder_report_s::PX4_ADSB_FLAGS_VALID_COORDS |
					  transponder_report_s::PX4_ADSB_FLAGS_VALID_HEADING |
					  transponder_report_s::PX4_ADSB_FLAGS_VALID_VELOCITY | transponder_report_s::PX4_ADSB_FLAGS_VALID_ALTITUDE;

		if ((tr.flags & required_flags) != required_flags) {
			orb_check(_traffic_sub, &changed);
			continue;
		}

		float d_hor, d_vert;
		get_distance_to_point_global_wgs84(lat, lon, alt,
						   tr.lat, tr.lon, tr.altitude, &d_hor, &d_vert);


		// predict final altitude (positive is up) in prediction time frame
		float end_alt = tr.altitude + (d_vert / tr.hor_velocity) * tr.ver_velocity;

		// Predict until the vehicle would have passed this system at its current speed
		float prediction_distance = d_hor + 1000.0f;

		// If the altitude is not getting close to us, do not calculate
		// the horizontal separation.
		// Since commercial flights do most of the time keep flight levels
		// check for the current and for the predicted flight level.
		// we also make the implicit assumption that this system is on the lowest
		// flight level close to ground in the
		// (end_alt - horizontal_separation < alt) condition. If this system should
		// ever be used in normal airspace this implementation would anyway be
		// inappropriate as it should be replaced with a TCAS compliant solution.

		if ((fabsf(alt - tr.altitude) < vertical_separation) || ((end_alt - horizontal_separation) < alt)) {

			double end_lat, end_lon;
			waypoint_from_heading_and_distance(tr.lat, tr.lon, tr.heading, prediction_distance, &end_lat, &end_lon);

			struct crosstrack_error_s cr;

			if (!get_distance_to_line(&cr, lat, lon, tr.lat, tr.lon, end_lat, end_lon)) {

				if (!cr.past_end && (fabsf(cr.distance) < horizontal_separation)) {

					// direction of traffic in human-readable 0..360 degree in earth frame
					int traffic_direction = math::degrees(tr.heading) + 180;

					switch (_param_traffic_avoidance_mode.get()) {

					case 0: {
							/* ignore */
							PX4_WARN("TRAFFIC %s, hdg: %d", tr.flags & transponder_report_s::PX4_ADSB_FLAGS_VALID_CALLSIGN ? tr.callsign :
								 "unknown",
								 traffic_direction);
							break;
						}

					case 1: {
							mavlink_log_critical(&_mavlink_log_pub, "WARNING TRAFFIC %s at heading %d, land immediately",
									     tr.flags & transponder_report_s::PX4_ADSB_FLAGS_VALID_CALLSIGN ? tr.callsign : "unknown",
									     traffic_direction);
							break;
						}

					case 2: {
							mavlink_log_critical(&_mavlink_log_pub, "AVOIDING TRAFFIC %s heading %d, returning home",
									     tr.flags & transponder_report_s::PX4_ADSB_FLAGS_VALID_CALLSIGN ? tr.callsign : "unknown",
									     traffic_direction);

							// set the return altitude to minimum
							_rtl.set_return_alt_min(true);

							// ask the commander to execute an RTL
							vehicle_command_s vcmd = {};
							vcmd.command = vehicle_command_s::VEHICLE_CMD_NAV_RETURN_TO_LAUNCH;
							publish_vehicle_cmd(&vcmd);
							break;
						}
					}
				}
			}
		}

		orb_check(_traffic_sub, &changed);
	}
}

bool
Navigator::abort_landing()
{
	bool should_abort = false;

	if (!_vstatus.is_rotary_wing && !_vstatus.in_transition_mode) {
		if (hrt_elapsed_time(&_fw_pos_ctrl_status.timestamp) < 1000000) {

			if (get_position_setpoint_triplet()->current.valid
			    && get_position_setpoint_triplet()->current.type == position_setpoint_s::SETPOINT_TYPE_LAND) {

				should_abort = _fw_pos_ctrl_status.abort_landing;
			}
		}
	}

	return should_abort;
}

bool
Navigator::force_vtol()
{
	return _vstatus.is_vtol &&
	       (!_vstatus.is_rotary_wing || _vstatus.in_transition_to_fw)
	       && _param_force_vtol.get();
}

int Navigator::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Module that is responsible for autonomous flight modes. This includes missions (read from dataman),
takeoff and RTL.
It is also responsible for geofence violation checking.

### Implementation
The different internal modes are implemented as separate classes that inherit from a common base class `NavigatorMode`.
The member `_navigation_mode` contains the current active mode.

Navigator publishes position setpoint triplets (`position_setpoint_triplet_s`), which are then used by the position
controller.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("navigator", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_COMMAND_DESCR("fencefile", "load a geofence file from SD card, stored at etc/geofence.txt");
	PRINT_MODULE_USAGE_COMMAND_DESCR("fake_traffic", "publishes 3 fake transponder_report_s uORB messages");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int Navigator::custom_command(int argc, char *argv[])
{
	if (!is_running()) {
		print_usage("not running");
		return 1;
	}

	if (!strcmp(argv[0], "fencefile")) {
		get_instance()->load_fence_from_file(GEOFENCE_FILENAME);
		return 0;

	} else if (!strcmp(argv[0], "fake_traffic")) {
		get_instance()->fake_traffic("LX007", 500, 1.0f, -1.0f, 100.0f, 90.0f, 0.001f);
		get_instance()->fake_traffic("LX55", 1000, 0, 0, 100.0f, 90.0f, 0.001f);
		get_instance()->fake_traffic("LX20", 15000, 1.0f, -1.0f, 280.0f, 90.0f, 0.001f);
		return 0;
	}

	return print_usage("unknown command");
}

int navigator_main(int argc, char *argv[])
{
	return Navigator::main(argc, argv);
}

void
Navigator::publish_mission_result()
{
	_mission_result.timestamp = hrt_absolute_time();

	/* lazily publish the mission result only once available */
	if (_mission_result_pub != nullptr) {
		/* publish mission result */
		orb_publish(ORB_ID(mission_result), _mission_result_pub, &_mission_result);

	} else {
		/* advertise and publish */
		_mission_result_pub = orb_advertise(ORB_ID(mission_result), &_mission_result);
	}

	/* reset some of the flags */
	_mission_result.item_do_jump_changed = false;
	_mission_result.item_changed_index = 0;
	_mission_result.item_do_jump_remaining = 0;
}

void
Navigator::publish_geofence_result()
{
	/* lazily publish the geofence result only once available */
	if (_geofence_result_pub != nullptr) {
		/* publish mission result */
		orb_publish(ORB_ID(geofence_result), _geofence_result_pub, &_geofence_result);

	} else {
		/* advertise and publish */
		_geofence_result_pub = orb_advertise(ORB_ID(geofence_result), &_geofence_result);
	}
}

void
Navigator::set_mission_failure(const char *reason)
{
	if (!_mission_result.failure) {
		_mission_result.failure = true;
		set_mission_result_updated();
		mavlink_log_critical(&_mavlink_log_pub, "%s", reason);
	}
}

void
Navigator::publish_vehicle_cmd(vehicle_command_s *vcmd)
{
	vcmd->timestamp = hrt_absolute_time();
	vcmd->source_system = _vstatus.system_id;
	vcmd->source_component = _vstatus.component_id;
	vcmd->target_system = _vstatus.system_id;
	vcmd->confirmation = false;
	vcmd->from_external = false;

	// The camera commands are not processed on the autopilot but will be
	// sent to the mavlink links to other components.
	switch (vcmd->command) {
	case NAV_CMD_IMAGE_START_CAPTURE:
	case NAV_CMD_IMAGE_STOP_CAPTURE:
	case NAV_CMD_VIDEO_START_CAPTURE:
	case NAV_CMD_VIDEO_STOP_CAPTURE:
		vcmd->target_component = 100; // MAV_COMP_ID_CAMERA
		break;

	default:
		vcmd->target_component = _vstatus.component_id;
		break;
	}

	if (_vehicle_cmd_pub != nullptr) {
		orb_publish(ORB_ID(vehicle_command), _vehicle_cmd_pub, vcmd);

	} else {
		_vehicle_cmd_pub = orb_advertise_queue(ORB_ID(vehicle_command), vcmd, vehicle_command_s::ORB_QUEUE_LENGTH);
	}
}

void
Navigator::publish_vehicle_command_ack(const vehicle_command_s &cmd, uint8_t result)
{
	vehicle_command_ack_s command_ack = {};

	command_ack.timestamp = hrt_absolute_time();
	command_ack.command = cmd.command;
	command_ack.target_system = cmd.source_system;
	command_ack.target_component = cmd.source_component;
	command_ack.from_external = false;

	command_ack.result = result;
	command_ack.result_param1 = 0;
	command_ack.result_param2 = 0;

	if (_vehicle_cmd_ack_pub != nullptr) {
		orb_publish(ORB_ID(vehicle_command_ack), _vehicle_cmd_ack_pub, &command_ack);

	} else {
		_vehicle_cmd_ack_pub = orb_advertise_queue(ORB_ID(vehicle_command_ack), &command_ack,
				       vehicle_command_ack_s::ORB_QUEUE_LENGTH);
	}
}
