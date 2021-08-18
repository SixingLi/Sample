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
		SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelWarning, "Not exists!");
	}
	for (auto&item: indexOfValidPoints)
	{
		SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelDebug, "indexOfValidPoint: %d", item);
	}
	for (auto&item : roadIdList)
	{
		SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelDebug, "roadId: %ld", item);
	}
}
