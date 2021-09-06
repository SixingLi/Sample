#pragma once
#include "SimOneServiceAPI.h"
#include <iostream>

using SSD::SimString;
using SSD::SimPoint3D;

void SampleGetStoplineList(const SimString& laneId, SSD::SimVector<HDMapStandalone::MSignal> TrafficLightList)
{
	if (TrafficLightList.size() < 1)
	{
		SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ELogLevelWarning, "No traffic light exists!");
		return;
	}
	SSD::SimVector<HDMapStandalone::MObject> stoplineList;
	for (auto&item : TrafficLightList)
	{
		SimOneAPI::GetStoplineList(item, laneId, stoplineList);
		for (size_t i = 0; i < stoplineList.size(); i++)
		{
			auto& objectItem = stoplineList[i];
			SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ELogLevelDebug, "objectItem.id: %ld", objectItem.id);
			SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ELogLevelDebug, "objectItem.type: %s", objectItem.type.GetString());
			SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ELogLevelDebug, "[pt.x: %f, pt.y: %f, pt.y: %f]", objectItem.pt.x, objectItem.pt.y, objectItem.pt.z);
			SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ELogLevelDebug, "objectItem.boundaryKnots: %d", objectItem.boundaryKnots.size());
		}
	}
}