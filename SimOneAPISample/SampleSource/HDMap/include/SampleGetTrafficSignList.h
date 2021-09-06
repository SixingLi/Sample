#pragma once
#include "SimOneServiceAPI.h"
#include <iostream>

using SSD::SimString;

void SampleGetTrafficSignList()
{
	SSD::SimVector<HDMapStandalone::MSignal> signList;
	SimOneAPI::GetTrafficSignList(signList);
	if (signList.size() < 1)
	{
		SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ELogLevelWarning, "no trafficSign");
		return;
	}
	for (size_t i = 0; i < signList.size(); i++)
	{
		HDMapStandalone::MSignal sign = signList[i];

		SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ELogLevelDebug, "sign id: %ld", sign.id);
		SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ELogLevelDebug, "sign.type: %s", sign.type.GetString());
		SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ELogLevelDebug, "sign.subType: %s", sign.subType.GetString());
		SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ELogLevelDebug, "[sign.pt.x: %f, sign.pt.y: %f, sign.pt.z: %f]", sign.pt.x, sign.pt.y, sign.pt.z);
		SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ELogLevelDebug, "[sign.heading.x: %f, sign.heading.y: %f, sign.heading.y: %f]", sign.heading.x, sign.heading.y, sign.heading.z);
		SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ELogLevelDebug, "sign.value: %s", sign.value.GetString());
		SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ELogLevelDebug, "sign.unit: %s", sign.unit.GetString());
		SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ELogLevelDebug, "sign.isDynamic: %d", sign.isDynamic);
	}

}
