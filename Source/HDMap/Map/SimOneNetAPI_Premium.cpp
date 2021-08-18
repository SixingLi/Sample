
#include "Service/SimOneNetService.hpp"
#include "Map/SimOneNetAPI_Premium.h"

#ifdef __cplusplus
extern "C"
{
#endif
#ifndef WITHOUT_HDMAP
	SIMONE_NET_API void SimOneAPI::GetLaneData(SSD::SimVector<HDMapStandalone::MLaneInfo>& data)
	{
		return SimOneAPIService::GetLaneData(data);
	}

	SIMONE_NET_API SSD::SimVector<long> SimOneAPI::GetJunctionList()
	{
		return SimOneAPIService::GetJunctionList();
	}

	SIMONE_NET_API double SimOneAPI::GetRoadLength(const long& roadId)
	{
		return SimOneAPIService::GetRoadLength(roadId);
	}

	SIMONE_NET_API bool SimOneAPI::GetSectionLaneList(const SSD::SimString& laneId, SSD::SimStringVector& sectionLaneList)
	{
		return SimOneAPIService::GetSectionLaneList(laneId, sectionLaneList);
	}

	SIMONE_NET_API bool SimOneAPI::IsTwoSideRoad(const long& roadId)
	{
		return SimOneAPIService::IsTwoSideRoad(roadId);
	}

	SIMONE_NET_API SimOneAPI::LaneInfo SimOneAPI::GetForwardLaneInfo(const SSD::SimPoint3D& pos, const SimOneAPI::TyrePosInfo& tyrePosInfo, const double& forward)
	{
		return SimOneAPIService::GetForwardLaneInfo(pos, tyrePosInfo, forward);
	}

	SIMONE_NET_API bool SimOneAPI::GetTopoGraph(HDMapStandalone::MTopoGraph& topoGraph)
	{
		return SimOneAPIService::GetTopoGraph(topoGraph);
	}

	SIMONE_NET_API double SimOneAPI::GetLaneLength(const SSD::SimString& id)
	{
		return SimOneAPIService::GetLaneLength(id);
	}

	SIMONE_NET_API bool SimOneAPI::IsDriving(const SSD::SimString& id)
	{
		return SimOneAPIService::IsDriving(id);
	}

	SIMONE_NET_API bool SimOneAPI::IsInJunction(const SSD::SimString& id, long& juncId)
	{
		return SimOneAPIService::IsInJunction(id, juncId);
	}

	SIMONE_NET_API bool SimOneAPI::IsInsideLane(const SSD::SimPoint3D& inputPt, const SSD::SimString& laneName, HDMapStandalone::MSideState& sideState)
	{
		return SimOneAPIService::IsInsideLane(inputPt, laneName, sideState);
	}

	SIMONE_NET_API bool SimOneAPI::GetNearMostLaneWithHeight(const SSD::SimPoint3D& pos, bool drivingOnly, SSD::SimString& id, double& s, double& t,
		double& s_toCenterLine, double& t_toCenterLine, bool& insideLane)
	{
		return SimOneAPIService::GetNearMostLaneWithHeight(pos, drivingOnly, id, s, t, s_toCenterLine, t_toCenterLine, insideLane);
	}

	SIMONE_NET_API bool SimOneAPI::GetForwardLaneSample(const SSD::SimPoint3D& inputPt, const SSD::SimString& laneName, const double& forward, SSD::SimVector<HDMapStandalone::MLaneInfo>& laneInfoList)
	{
		return SimOneAPIService::GetForwardLaneSample(inputPt, laneName, forward, laneInfoList);
	}

	SIMONE_NET_API void SimOneAPI::GetLaneLineInfo(SSD::SimVector<HDMapStandalone::MLaneLineInfo>& laneLineInfo)
	{
		return SimOneAPIService::GetLaneLineInfo(laneLineInfo);
	}

	SIMONE_NET_API bool SimOneAPI::GenerateRoute_v2(const SSD::SimPoint3DVector& sampleNodes, SSD::SimVector<int>& indexOfValidPoints,
		HDMapStandalone::MRoutePath& path, SSD::SimVector<HDMapStandalone::MRoutePoint>& routePtList)
	{
		return SimOneAPIService::GenerateRoute(sampleNodes, indexOfValidPoints, path, routePtList);
	}

	SIMONE_NET_API void SimOneAPI::GetSectionList(const long& roadId, SSD::SimStringVector& rightList, SSD::SimStringVector& leftList)
	{
		SimOneAPIService::GetSectionList(roadId, rightList, leftList);
	}
#endif
#ifdef __cplusplus
}
#endif