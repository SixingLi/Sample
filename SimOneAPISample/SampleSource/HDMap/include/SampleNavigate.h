#pragma once
#include "SimOneServiceAPI.h"
#include <iostream>

using SSD::SimString;

void SampleNavigate(const SSD::SimPoint3DVector& inputPoints)
{
	SSD::SimVector<int> indexOfValidPoints;
	SSD::SimVector<long> roadIdList;
	bool result = SimOneAPI::Navigate(inputPoints, indexOfValidPoints, roadIdList);
	if (!result)
	{
		SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ELogLevelWarning, "Not exists!");
	}
	for (auto&item: indexOfValidPoints)
	{
		SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ELogLevelDebug, "indexOfValidPoint: %d", item);
	}
	for (auto&item : roadIdList)
	{
		SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ELogLevelDebug, "roadId: %ld", item);
	}
}
