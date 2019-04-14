/****************************************************************************
 *
 *   Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
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
 * @file px4_daemon_app.c
 * daemon application example for PX4 autopilot
 *
 * @author Example User <mail@example.com>
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <px4_posix.h>
#include <px4_config.h>
#include <nuttx/sched.h>
#include <drivers/drv_hrt.h>
#include <mathlib/mathlib.h>
#include <matrix/math.hpp>
#include <px4_tasks.h>
#include <systemlib/err.h>
#include <uORB/uORB.h>
#include <uORB/topics/debug_key_value.h>
#include <uORB/topics/vehicle_filter_attitude.h>
#include <uORB/topics/vehicle_attitude.h>
#include <queue>
static bool thread_should_exit = false;		/**< daemon exit flag */
static bool thread_running = false;		/**< daemon status flag */
static int daemon_task;				/**< Handle of daemon task / thread */

/**
 * daemon management function.
 */
extern "C" __EXPORT int filter_angle_main(int argc, char *argv[]);

/**
 * Mainloop of daemon.
 */
int filter_angle_thread_main(int argc, char *argv[]);

/**
 * Print the correct usage.
 */
static void usage(const char *reason);

static void
usage(const char *reason)
{
	if (reason) {
		warnx("%s\n", reason);
	}

	warnx("usage: daemon {start|stop|status} [-p <additional params>]\n\n");
}

/**
 * The daemon app only briefly exists to start
 * the background job. The stack size assigned in the
 * Makefile does only apply to this management task.
 *
 * The actual stack size should be set in the call
 * to task_create().
 */
int filter_angle_main(int argc, char *argv[])
{
	if (argc < 2) {
		usage("missing command");
		return 1;
	}

	if (!strcmp(argv[1], "start")) {

		if (thread_running) {
			warnx("daemon already running\n");
			/* this is not an error */
			return 0;
		}

		thread_should_exit = false;
		daemon_task = px4_task_spawn_cmd("filter_angle",
						 SCHED_DEFAULT,
						 SCHED_PRIORITY_DEFAULT,
						 2000,
                         filter_angle_thread_main,
						 (argv) ? (char *const *)&argv[2] : (char *const *)NULL);
		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		thread_should_exit = true;
		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (thread_running) {
			warnx("\trunning\n");

		} else {
			warnx("\tnot started\n");
		}

		return 0;
	}

	usage("unrecognized command");
	return 1;
}

int filter_angle_thread_main(int argc, char *argv[])
{

    //订阅角度
    int atti_sub_fd = orb_subscribe(ORB_ID(vehicle_attitude));
    int error_counter = 0;

    px4_pollfd_struct_t fds[1];
    fds[0].fd = atti_sub_fd;
    fds[0].events = POLLIN;
    //公告滤波后的角度
    //struct vehicle_filter_attitude_s fa_dgb ={.key = "filter", .value=0.0f, .rollspeed=0.0f};
    struct debug_key_value_s fa_dgb ={0,0,0,"filter"};

    orb_advert_t pub_dbg = orb_advertise(ORB_ID(debug_key_value), &fa_dgb);
	warnx("[daemon] starting\n");

	thread_running = true;

	while (!thread_should_exit) {
        int poll_ret = px4_poll(fds, 1, 1000);

        if (poll_ret == 0) {
            PX4_ERR("Got no data within a second");

        } else if (poll_ret < 0) {
            if (error_counter < 10 || error_counter % 50 == 0) {
                PX4_ERR("ERROR return value from poll(): %d", poll_ret);
            }
            error_counter++;
        } else {
            if (fds[0].revents & POLLIN) {
                uint64_t timestamp_us = hrt_absolute_time();
                uint32_t timestamp_ms = timestamp_us / 1000;
                struct vehicle_attitude_s raw;
                /* copy sensors raw data into local buffer */
                orb_copy(ORB_ID(vehicle_attitude), atti_sub_fd, &raw);
                matrix::Eulerf euler = matrix::Quatf(raw.q);

                fa_dgb.timestamp_ms = timestamp_ms;
                fa_dgb.value = (float)0.92*fa_dgb.value + (float)0.08*euler.phi();
                //fa_dgb.value = (float)0.92*fa_dgb.value + (float)0.08*euler.phi();
                orb_publish(ORB_ID(debug_key_value), pub_dbg, &fa_dgb);
            }
        }
	}

	warnx("[daemon] exiting.\n");

	thread_running = false;

	return 0;
}
