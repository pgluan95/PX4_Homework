/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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

#include "monitoring.h"

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>

#include <uORB/topics/parameter_update.h>



int Monitoring::print_status()
{
	PX4_INFO("Running");
	// float* accel = _sensor_combined.accelerometer_m_s2;
	//double*lat = _vehicle_global_position.lat;
	//PX4_INFO("lat:%f",_vehicle_global_position.lat);
	// // TODO: print additional runtime information about the state of the module
	// PX4_INFO("accel:%f, %f, %f", (double)accel[0],(double)accel[1],(double)accel[2]);

	return 0;
}

int Monitoring::custom_command(int argc, char *argv[])
{
	/*
	if (!is_running()) {
		print_usage("not running");
		return 1;
	}

	// additional custom commands can be handled like this:
	if (!strcmp(argv[0], "do-something")) {
		get_instance()->do_something();
		return 0;
	}
	 */

	return print_usage("unknown command");
}


int Monitoring::task_spawn(int argc, char *argv[])
{
	_task_id = px4_task_spawn_cmd("monitoring",
				      SCHED_DEFAULT,
				      SCHED_PRIORITY_DEFAULT,
				      1024,
				      (px4_main_t)&run_trampoline,
				      (char *const *)argv);

	if (_task_id < 0) {
		_task_id = -1;
		return -errno;
	}

	return 0;
}

Monitoring *Monitoring::instantiate(int argc, char *argv[])
{
	int example_param = 0;
	bool example_flag = false;
	bool error_flag = false;

	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;

	// parse CLI arguments
	while ((ch = px4_getopt(argc, argv, "p:f", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'p':
			example_param = (int)strtol(myoptarg, nullptr, 10);
			break;

		case 'f':
			example_flag = true;
			break;

		case '?':
			error_flag = true;
			break;

		default:
			PX4_WARN("unrecognized flag");
			error_flag = true;
			break;
		}
	}

	if (error_flag) {
		return nullptr;
	}

	Monitoring *instance = new Monitoring(example_param, example_flag);

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
	}

	return instance;
}

Monitoring::Monitoring(int example_param, bool example_flag)
	: ModuleParams(nullptr)
{
}

void Monitoring::run()
{
	// Example: run the loop synchronized to the sensor_combined topic publication
	//int sensor_combined_sub = orb_subscribe(ORB_ID(sensor_combined));
	int vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));
	int vehicle_global_position_sub = orb_subscribe(ORB_ID(vehicle_global_position));
	int battery_status_sub = orb_subscribe(ORB_ID(battery_status));


	px4_pollfd_struct_t fds[3];
	//fds[0].fd = sensor_combined_sub;
	fds[0].fd = vehicle_status_sub;
	fds[1].fd = battery_status_sub;
	fds[2].fd = vehicle_global_position_sub;
	fds[0].events = POLLIN;

	// initialize parameters
	parameters_update(true);

	while (!should_exit()) {

		// wait for up to 1000ms for data
		int pret = px4_poll(fds, (sizeof(fds) / sizeof(fds[0])), 1000);

		if (pret == 0) {
			// Timeout: let the loop run anyway, don't do `continue` here

		} else if (pret < 0) {
			// this is undesirable but not much we can do
			PX4_ERR("poll error %d, %d", pret, errno);
			px4_usleep(50000);
			continue;

		} else if (fds[0].revents & POLLIN) {

			//struct sensor_combined_s sensor_combined;
			//orb_copy(ORB_ID(sensor_combined), sensor_combined_sub, &_sensor_combined);
			orb_copy(ORB_ID(vehicle_status), vehicle_status_sub, &_vehicle_status);
			orb_copy(ORB_ID(battery_status), battery_status_sub, &_battery_status);
			orb_copy(ORB_ID(vehicle_global_position), vehicle_global_position_sub, & _vehicle_global_position);
			// TODO: do something with the data...
			struct monitoring_s _monitoring;
			_monitoring.timestamp = hrt_absolute_time();
			_monitoring.nav_state = _vehicle_status.nav_state;


			_monitoring.lat = _vehicle_global_position.lat;
			_monitoring.lon = _vehicle_global_position.lon;
			_monitoring.remaining = _battery_status.remaining;

			// _monitoring.accel_x = _sensor_combined.accelerometer_m_s2[0];
			// _monitoring.accel_y = _sensor_combined.accelerometer_m_s2[1];
			// _monitoring.accel_z = _sensor_combined.accelerometer_m_s2[2];

			_monitoring_pub.publish(_monitoring);

		}

		parameters_update();
	}

	//orb_unsubscribe(sensor_combined_sub);

	orb_unsubscribe(vehicle_status_sub);
	orb_unsubscribe(battery_status_sub);
	orb_unsubscribe(vehicle_global_position_sub);
}

void Monitoring::parameters_update(bool force)
{
	// check for parameter updates
	if (_parameter_update_sub.updated() || force) {
		// clear update
		parameter_update_s update;
		_parameter_update_sub.copy(&update);

		// update parameters from storage
		updateParams();
	}
}

int Monitoring::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Section that describes the provided module functionality.

This is a template for a module running as a task in the background with start/stop/status functionality.

### Implementation
Section describing the high-level implementation of this module.

### Examples
CLI usage example:
$ module start -f -p 42

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("module", "template");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_FLAG('f', "Optional example flag", true);
	PRINT_MODULE_USAGE_PARAM_INT('p', 0, 0, 1000, "Optional example parameter", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int monitoring_main(int argc, char *argv[])
{
	return Monitoring::main(argc, argv);
}
