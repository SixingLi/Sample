#pragma once
#include "SimOneServiceAPI.h"
#include <iostream>

using SSD::SimString;
using SSD::SimPoint3D;

void SampleGetLaneMiddlePoint(const SimPoint3D& pos, const SimString& laneId)
{
	SSD::SimPoint3D targetPoint;
	SSD::SimPoint3D dir;
	bool result = SimOneAPI::GetLaneMiddlePoint(pos, laneId, targetPoint, dir);
	if (!result)
	{
		SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Warning, "Not exists!");
		return;
	}

	SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Debug, "[targetPoint.x: %f,targetPoint.y: %f,targetPoint.z: %f]", targetPoint.x, targetPoint.y, targetPoint.z);
	SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Debug, "[dir.x: %f, dir.y: %f, dir.z: %f]", dir.x, dir.y, dir.z);
}