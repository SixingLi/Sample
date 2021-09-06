#pragma once
#include "SimOneServiceAPI.h"
#include <iostream>

using SSD::SimString;

void SampleGetCrossHatchList(const SimString& laneId)
{
	SSD::SimVector<HDMapStandalone::MObject> crossHatchList;
	SimOneAPI::GetCrossHatchList(laneId, crossHatchList);
	if (crossHatchList.size() < 1)
	{
		SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ELogLevelWarning, "not crossHatchList!");
		return;
	}
	for (size_t i = 0; i < crossHatchList.size(); i++)
	{
		HDMapStandalone::MObject item = crossHatchList[i];
		SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ELogLevelDebug, "item.id: %ld", item.id);
		SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ELogLevelDebug, "item.type: %s", item.type.GetString());
		SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ELogLevelDebug, "item.pt.x: %f, item.pt.y: %f, item.pt.z: %f", item.pt.x, item.pt.y, item.pt.z);
		SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ELogLevelDebug, "item.boundaryKnots: %d", item.boundaryKnots.size());
	}
}
