#pragma once
#include "SimOneServiceAPI.h"
#include <iostream>

using SSD::SimString;

void SampleGetHeights(const SSD::SimPoint3D& inputPt, double radius)
{
	SSD::SimVector<double> heights;
	SSD::SimVector<long> roadIds;
	SSD::SimVector<bool> insideRoadStates;
	bool result = SimOneAPI::GetHeights(inputPt, radius, heights, roadIds, insideRoadStates);
	if (!result)
	{
		SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ELogLevelWarning, "not exist");
		return;
	}
	for (auto&item : heights)
	{
		SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ELogLevelDebug, "height: %f", item);
	}
	for (auto&item : roadIds)
	{
		SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ELogLevelDebug, "roadId: %ld", item);
	}
	for (auto&item : insideRoadStates)
	{
		SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ELogLevelDebug, "insideRoadState: %d", item);
	}
}
