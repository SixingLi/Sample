#include "UtilEvaluation.h"
#include <iostream>
#include <thread>
#include <math.h>
#include <stdarg.h>

std::string UtilEvaluation::CurrentTimestamp()
{
	std::uint64_t timestamp = std::uint64_t(std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count() / 1000.0);
	return std::to_string(timestamp);
}

cybertron::json UtilEvaluation::ConvertToJson(const SimOne_Data_Gps& gps)
{
	cybertron::json jsonData;
	jsonData["category"] = "gps";
	jsonData["timestamp"] = UtilEvaluation::CurrentTimestamp();
	jsonData["frame"] = gps.frame;
	jsonData["pos"] = {gps.posX, gps.posY, gps.posZ};
	jsonData["rot"] = { gps.oriX, gps.oriY, gps.oriZ };
	jsonData["vel"] = { gps.velX, gps.velY, gps.velZ };
	jsonData["acc"] = { gps.accelX, gps.accelY, gps.accelZ };
	jsonData["angVel"] = { gps.angVelX, gps.angVelY, gps.angVelZ };
	jsonData["throttle"] = gps.throttle;
	jsonData["brake"] = gps.brake;
	jsonData["steering"] = gps.steering;
	jsonData["gear"] = gps.gear;
	jsonData["engineRpm"] = gps.engineRpm;
	jsonData["wheelSpeed"] = { gps.wheelSpeedFL, gps.wheelSpeedFR, gps.wheelSpeedRL, gps.wheelSpeedRR };
	return jsonData;
}
