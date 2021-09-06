#pragma once
#include "SimOneServiceAPI.h"
#include <iostream>

using SSD::SimString;

SSD::SimVector<HDMapStandalone::MSignal> SampleGetTrafficLightList()
{
	SSD::SimVector<HDMapStandalone::MSignal> TrafficLightList;
	SimOneAPI::GetTrafficLightList(TrafficLightList);
	if (TrafficLightList.size() < 1)
	{
		SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ELogLevelWarning, "No traffic light exists!");
		return TrafficLightList;
	}
	
	for (auto& item : TrafficLightList)
	{
		SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ELogLevelDebug, "sign id: %ld", item.id);
		SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ELogLevelDebug, "type: %s", item.type.GetString());
		SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ELogLevelDebug, "subType: %s", item.subType.GetString());
		SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ELogLevelDebug, "item.pt.x: %f, item.pt.y: %f, item.pt.z: %f", item.pt.x, item.pt.y, item.pt.z);
		SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ELogLevelDebug, "heading.pt.x: %f, heading.pt.y: %f, heading.pt.z: %f,", item.heading.x, item.heading.y, item.heading.z);
		SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ELogLevelDebug, "value: %s", item.value.GetString());
		SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ELogLevelDebug, "unit: %s", item.unit.GetString());
		SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ELogLevelDebug, "isDynamic: %d", item.isDynamic);
	}
	return TrafficLightList;
}