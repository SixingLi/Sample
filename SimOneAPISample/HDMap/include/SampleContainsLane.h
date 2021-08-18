#pragma once

#include "SimOneServiceAPI.h"
#include "SimOneHDMapAPI.h"
#include <iostream>

using SSD::SimString;
using SSD::SimPoint3D;

void SampleContainsLane(const SimString& laneId)
{
	if (!SimOneAPI::ContainsLane(laneId))
	{
		SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelWarning, "Error: lane does not eixst in the map.");
		return;
	}		
	SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelInformation, "lane exists in the map.");
}