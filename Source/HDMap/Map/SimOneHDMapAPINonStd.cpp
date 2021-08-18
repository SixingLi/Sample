
#include "Service/SimOneNetService.hpp"
#include "Map/SimOneHDMapAPINonStd.h"

#ifdef __cplusplus
extern "C"
{
#endif
#ifndef  WITHOUT_HDMAP
SIMONE_SM_API SimOneAPI::LaneInfo_ GetLaneInfo(const SSD::SimPoint3D& pos, const SimOneAPI::TyrePosInfo_& tyrePosInfo, const double& forward)
{
	return SimOneAPIService::GetLaneInfo(pos, tyrePosInfo, forward);
}

SIMONE_SM_API SSD::SimVector<int> GetLaneIndexList(const SSD::SimPoint3D& pos, int& currentLaneIndex, SSD::SimStringVector& laneIdList)
{
	return SimOneAPIService::GetLaneIndexList(pos, currentLaneIndex, laneIdList);
}

SIMONE_SM_API SimOneAPI::EDirectionType_ GetIconType(const SSD::SimPoint3D& pos)
{
	return SimOneAPIService::GetIconType(pos);
}

SIMONE_SM_API bool SimOneAPI::GetLaneSampleByLocation(const SSD::SimPoint3D& pos, HDMapStandalone::MLaneInfo& info)
{
	return SimOneAPIService::GetLaneSampleByLocation(pos, info);
}
#endif
#ifdef __cplusplus
}
#endif