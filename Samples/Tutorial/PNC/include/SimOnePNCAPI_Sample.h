#ifndef SIMONEPNCAPI_SAMPLE_H
#define SIMONEPNCAPI_SAMPLE_H

#include <vector>
#include <thread>
#include <string>
// #include <fstream>
#include <math.h>

#include "Service/SimOneIOStruct.h"
#include "SimOneServiceAPI.h"
#include "SimOnePNCAPI.h"
#include "SimOneSensorAPI.h"
#include "SimOneV2XAPI.h"
#include "timer.hpp"

#include "logger/logger.h"

class pncapi_sample
{
public:
	pncapi_sample();
	~pncapi_sample();

	int64_t getCurrentTime();

	void simone_ini();
	void set_pose_ctl();
	void set_drive_ctl();
	void set_drive_trajectory();
	void get_sensor_detection();
	static void set_v2x_info(const char* mainVehicleId, const char* sensorId, SimOne_Data_V2XNFS *pDetections) ;
	static void set_scenario_event(const char *mainVehicleId, const char *event, const char *data);

	void pub();

protected:
	Logging::Logger log_simone_ini;
	Logging::Logger log_set_pose_ctl;
	Logging::Logger log_set_drive_ctl;
	Logging::Logger log_set_drive_trajectory;
	Logging::Logger log_get_sensor_detection;
	static Logging::Logger log_scenario_event;
	static Logging::Logger log_v2x_info;
};

#endif