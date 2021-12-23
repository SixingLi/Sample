#ifndef WITHOUT_HDMAP
#include "Service/SimOneNetService.hpp"
#include "Service/SimOneIOStruct.h"
#include "SimOneHDMapAPI.h"
//#include "UtilSharedMemory.hpp"
//#include "map/proto/map_lane.pb.h"
#include <iostream>
#include <thread>
#include <math.h>
#include <stdarg.h>
#pragma warning(disable:4244)
#ifdef __cplusplus
extern "C"
{
#endif
	SIMONE_API bool SimOneAPI::LoadHDMap(int timeOutSeconds)
	{
		return SimOneAPIService::LoadHDMap(timeOutSeconds);
	}

	SIMONE_API bool SimOneAPI::GetNearMostLane(const SSD::SimPoint3D& pos, SSD::SimString& id, double& s, double& t, double& s_toCenterLine, double& t_toCenterLine)
	{
		return SimOneAPIService::GetNearMostLane(pos, id, s, t, s_toCenterLine, t_toCenterLine);
	}

	SIMONE_API bool SimOneAPI::GetNearLanes(const SSD::SimPoint3D& pos, const double& distance, SSD::SimStringVector& nearLanes)
	{
		return SimOneAPIService::GetNearLanes(pos, distance, nearLanes);
	}

	SIMONE_API bool SimOneAPI::GetNearLanesWithAngle(const SSD::SimPoint3D& pos, const double& distance,
		const double& headingAngle, const double& angleShift, SSD::SimStringVector& nearLanes)
	{
		return SimOneAPIService::GetNearLanesWithAngle(pos, distance, headingAngle, angleShift, nearLanes);
	}

	SIMONE_API bool SimOneAPI::GetDistanceToLaneBoundary(const SSD::SimPoint3D& pos, SSD::SimString& id, double& distToLeft, double& distToRight, double& distToLeft2D, double& distToRight2D)
	{
		return SimOneAPIService::GetDistanceToLaneBoundary(pos, id, distToLeft, distToRight, distToLeft2D, distToRight2D);
	}

	SIMONE_API bool SimOneAPI::GetLaneSample(const SSD::SimString &id, HDMapStandalone::MLaneInfo& info)
	{
		return SimOneAPIService::GetLaneSample(id, info);
	}

	SIMONE_API bool SimOneAPI::GetLaneLink(const SSD::SimString& id, HDMapStandalone::MLaneLink& laneLink)
	{
		return SimOneAPIService::GetLaneLink(id, laneLink);
	}

	SIMONE_API bool SimOneAPI::GetLaneType(const SSD::SimString& id, HDMapStandalone::MLaneType& laneType)
	{
		return SimOneAPIService::GetLaneType(id, laneType);
	}

	SIMONE_API bool SimOneAPI::GetLaneWidth(const SSD::SimString& id, const SSD::SimPoint3D& pos, double& width)
	{
		return SimOneAPIService::GetLaneWidth(id, pos, width);
	}

	SIMONE_API bool SimOneAPI::GetLaneST(const SSD::SimString& id, const SSD::SimPoint3D& pos, double& s, double& t)
	{
		return SimOneAPIService::GetLaneST(id, pos, s, t);
	}

	SIMONE_API bool SimOneAPI::GetRoadST(const SSD::SimString& id, const SSD::SimPoint3D& pos, double& s, double& t, double& z)
	{
		return SimOneAPIService::GetRoadST(id, pos, s, t, z);
	}

	SIMONE_API bool SimOneAPI::GetInertialFromLaneST(const SSD::SimString& id, const double& s, const double& t, SSD::SimPoint3D& inertial, SSD::SimPoint3D& dir)
	{
		return SimOneAPIService::GetInertialFromLaneST(id, s, t, inertial, dir);
	}

	SIMONE_API bool SimOneAPI::ContainsLane(const SSD::SimString& id)
	{
		return SimOneAPIService::ContainsLane(id);
	}

	SIMONE_API void SimOneAPI::GetParkingSpaceList(SSD::SimVector<HDMapStandalone::MParkingSpace>& parkingSpaceList)
	{
		return SimOneAPIService::GetParkingSpaceList(parkingSpaceList);
	}

	SIMONE_API bool SimOneAPI::GenerateRoute(const SSD::SimPoint3DVector& inputPoints, SSD::SimVector<int>& indexOfValidPoints, SSD::SimPoint3DVector& route)
	{
		return SimOneAPIService::GenerateRoute(inputPoints, indexOfValidPoints, route);
	}

	SIMONE_API bool SimOneAPI::Navigate(const SSD::SimPoint3DVector& inputPoints, SSD::SimVector<int>& indexOfValidPoints, SSD::SimVector<long>& roadIdList)
	{
		return SimOneAPIService::Navigate(inputPoints, indexOfValidPoints, roadIdList);
	}

	SIMONE_API bool SimOneAPI::GetRoadMark(const SSD::SimPoint3D& pos, const SSD::SimString& id, HDMapStandalone::MRoadMark& left, HDMapStandalone::MRoadMark& right)
	{
		return SimOneAPIService::GetRoadMark(pos, id, left, right);
	}

	SIMONE_API void SimOneAPI::GetTrafficLightList(SSD::SimVector<HDMapStandalone::MSignal>& list)
	{
		//use std::move is to avoid killing performance, that extern "C" has limit to expose C++ object as return value
		list = std::move(SimOneAPIService::GetTrafficLightList());
	}

	SIMONE_API void SimOneAPI::GetTrafficSignList(SSD::SimVector<HDMapStandalone::MSignal>& list)
	{
		//use std::move is to avoid killing performance, that extern "C" has limit to expose C++ object as return value
		list = std::move(SimOneAPIService::GetTrafficSignList());
	}

	SIMONE_API void SimOneAPI::GetStoplineList(const HDMapStandalone::MSignal& light, const SSD::SimString& id, SSD::SimVector<HDMapStandalone::MObject>& stoplineList)
	{
		//use std::move is to avoid killing performance, that extern "C" has limit to expose C++ object as return value
		stoplineList = std::move(SimOneAPIService::GetStoplineList(light, id));
	}

	SIMONE_API void SimOneAPI::GetCrosswalkList(const HDMapStandalone::MSignal& light, const SSD::SimString& id, SSD::SimVector<HDMapStandalone::MObject>& crosswalkList)
	{
		//use std::move is to avoid killing performance, that extern "C" has limit to expose C++ object as return value
		crosswalkList = std::move(SimOneAPIService::GetCrosswalkList(light, id));
	}

	SIMONE_API void SimOneAPI::GetCrossHatchList(const SSD::SimString& id, SSD::SimVector<HDMapStandalone::MObject>& crossHatchList)
	{
		//use std::move is to avoid killing performance, that extern "C" has limit to expose C++ object as return value
		crossHatchList = std::move(SimOneAPIService::GetCrossHatchList(id));
	}

	SIMONE_API bool SimOneAPI::GetLaneMiddlePoint(const SSD::SimPoint3D& inputPt, const SSD::SimString& id, SSD::SimPoint3D& targetPoint, SSD::SimPoint3D& dir)
	{
		return SimOneAPIService::GetLaneMiddlePoint(inputPt, id, targetPoint, dir);
	}

	SIMONE_API bool SimOneAPI::GetHeights(const SSD::SimPoint3D& inputPt, const double& radius, SSD::SimVector<double>& heights,
		SSD::SimVector<long>& roadIds, SSD::SimVector<bool>& insideRoadStates)
	{
		return SimOneAPIService::GetHeights(inputPt, radius, heights, roadIds, insideRoadStates);
	}

	SIMONE_API void SimOneAPI::GetLaneData(SSD::SimVector<HDMapStandalone::MLaneInfo>& data)
	{
		return SimOneAPIService::GetLaneData(data);
	}

	SIMONE_API SSD::SimVector<long> SimOneAPI::GetJunctionList()
	{
		return SimOneAPIService::GetJunctionList();
	}

	SIMONE_API double SimOneAPI::GetRoadLength(const long& roadId)
	{
		return SimOneAPIService::GetRoadLength(roadId);
	}

	SIMONE_API bool SimOneAPI::GetSectionLaneList(const SSD::SimString& laneId, SSD::SimStringVector& sectionLaneList)
	{
		return SimOneAPIService::GetSectionLaneList(laneId, sectionLaneList);
	}

	SIMONE_API bool SimOneAPI::IsTwoSideRoad(const long& roadId)
	{
		return SimOneAPIService::IsTwoSideRoad(roadId);
	}

	SIMONE_API double SimOneAPI::GetLaneLength(const SSD::SimString& id)
	{
		return SimOneAPIService::GetLaneLength(id);
	}

	SIMONE_API bool SimOneAPI::IsDriving(const SSD::SimString& id)
	{
		return SimOneAPIService::IsDriving(id);
	}

	SIMONE_API bool SimOneAPI::IsInJunction(const SSD::SimString& id, long& juncId)
	{
		return SimOneAPIService::IsInJunction(id, juncId);
	}

	SIMONE_API bool SimOneAPI::IsInsideLane(const SSD::SimPoint3D& inputPt, const SSD::SimString& laneName, HDMapStandalone::MSideState& sideState)
	{
		return SimOneAPIService::IsInsideLane(inputPt, laneName, sideState);
	}

	//add v3

	SIMONE_API bool SimOneAPI::GetLaneSampleByLocation(const SSD::SimPoint3D& pos, HDMapStandalone::MLaneInfo& info)
	{
		return SimOneAPIService::GetLaneSampleByLocation(pos, info);
	}

#ifdef __cplusplus
}
#endif
#endif
