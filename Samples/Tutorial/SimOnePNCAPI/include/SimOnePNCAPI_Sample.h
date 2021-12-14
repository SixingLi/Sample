#ifndef SIMONEPNCAPI_SAMPLE_H
#define SIMONEPNCAPI_SAMPLE_H

#include <vector>
#include <thread>
#include <string>
// #include <fstream>

#include "SimOneServiceAPI.h"
#include "SimOnePNCAPI.h"
#include "SimOneSensorAPI.h"
#include "timer.hpp"

#include "logger/logger.h"

class pncapi_sample
{
public:
	pncapi_sample();
	~pncapi_sample();

	void simone_ini();
	void set_pose_ctl();

	void pub();

protected:
	Logging::Logger log_set_pose_ctl;
	Logging::Logger log_simone_ini;
};

#endif