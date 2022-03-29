#pragma once

#include <string>
#include <chrono>
#include <fstream>
#include "Service/SimOneIOStruct.h"
#include "cybertron/DefinesCore.hpp"
#include "cybertron/core/JsonReader.hpp"

class UtilEvaluation {
public:
	static std::string CurrentTimestamp();
	static cybertron::json ConvertToJson(const SimOne_Data_Gps& gps);
};
