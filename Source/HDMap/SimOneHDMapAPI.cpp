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
#define MAX_DRIVER_NAME_LEN 10
    static char gDriverNameArray[MAX_MAINVEHICLE_NUM][MAX_DRIVER_NAME_LEN];

	SIMONE_NET_API bool SimOneAPI::GetHDMapData(SimOne_Data_Map& hdMap)
	{
		return SimOneAPIService::GetHDMapData(hdMap);
	}

	SIMONE_NET_API bool SimOneAPI::LoadHDMap(const int& timeOutSeconds)
	{
		return SimOneAPIService::LoadHDMap(timeOutSeconds);
	}

	SIMONE_NET_API bool SimOneAPI::GetNearMostLane(const SSD::SimPoint3D& pos, SSD::SimString& id, double& s, double& t, double& s_toCenterLine, double& t_toCenterLine)
	{
		return SimOneAPIService::GetNearMostLane(pos, id, s, t, s_toCenterLine, t_toCenterLine);
	}

	SIMONE_NET_API bool SimOneAPI::GetNearLanes(const SSD::SimPoint3D& pos, const double& distance, SSD::SimStringVector& nearLanes)
	{
		return SimOneAPIService::GetNearLanes(pos, distance, nearLanes);
	}

	SIMONE_NET_API bool SimOneAPI::GetNearLanesWithAngle(const SSD::SimPoint3D& pos, const double& distance,
		const double& headingAngle, const double& angleShift, SSD::SimStringVector& nearLanes)
	{
		return SimOneAPIService::GetNearLanesWithAngle(pos, distance, headingAngle, angleShift, nearLanes);
	}

	SIMONE_NET_API bool SimOneAPI::GetDistanceToLaneBoundary(const SSD::SimPoint3D& pos, SSD::SimString& id, double& distToLeft, double& distToRight, double& distToLeft2D, double& distToRight2D)
	{
		return SimOneAPIService::GetDistanceToLaneBoundary(pos, id, distToLeft, distToRight, distToLeft2D, distToRight2D);
	}

	SIMONE_NET_API bool SimOneAPI::GetLaneSample(const SSD::SimString &id, HDMapStandalone::MLaneInfo& info)
	{
		return SimOneAPIService::GetLaneSample(id, info);
	}

	SIMONE_NET_API bool SimOneAPI::GetLaneLink(const SSD::SimString& id, HDMapStandalone::MLaneLink& laneLink)
	{
		return SimOneAPIService::GetLaneLink(id, laneLink);
	}

	SIMONE_NET_API bool SimOneAPI::GetLaneType(const SSD::SimString& id, HDMapStandalone::MLaneType& laneType)
	{
		return SimOneAPIService::GetLaneType(id, laneType);
	}

	SIMONE_NET_API bool SimOneAPI::GetLaneWidth(const SSD::SimString& id, const SSD::SimPoint3D& pos, double& width)
	{
		return SimOneAPIService::GetLaneWidth(id, pos, width);
	}

	SIMONE_NET_API bool SimOneAPI::GetLaneST(const SSD::SimString& id, const SSD::SimPoint3D& pos, double& s, double& t)
	{
		return SimOneAPIService::GetLaneST(id, pos, s, t);
	}

	SIMONE_NET_API bool SimOneAPI::GetRoadST(const SSD::SimString& id, const SSD::SimPoint3D& pos, double& s, double& t, double& z)
	{
		return SimOneAPIService::GetRoadST(id, pos, s, t, z);
	}

	SIMONE_NET_API bool SimOneAPI::GetInertialFromLaneST(const SSD::SimString& id, const double& s, const double& t, SSD::SimPoint3D& inertial, SSD::SimPoint3D& dir)
	{
		return SimOneAPIService::GetInertialFromLaneST(id, s, t, inertial, dir);
	}

	SIMONE_NET_API bool SimOneAPI::ContainsLane(const SSD::SimString& id)
	{
		return SimOneAPIService::ContainsLane(id);
	}

	SIMONE_NET_API void SimOneAPI::GetParkingSpaceList(SSD::SimVector<HDMapStandalone::MParkingSpace>& parkingSpaceList)
	{
		return SimOneAPIService::GetParkingSpaceList(parkingSpaceList);
	}

	SIMONE_NET_API bool SimOneAPI::GenerateRoute(const SSD::SimPoint3DVector& inputPoints, SSD::SimVector<int>& indexOfValidPoints, SSD::SimPoint3DVector& route)
	{
		return SimOneAPIService::GenerateRoute(inputPoints, indexOfValidPoints, route);
	}

	SIMONE_NET_API bool SimOneAPI::Navigate(const SSD::SimPoint3DVector& inputPoints, SSD::SimVector<int>& indexOfValidPoints, SSD::SimVector<long>& roadIdList)
	{
		return SimOneAPIService::Navigate(inputPoints, indexOfValidPoints, roadIdList);
	}

	SIMONE_NET_API bool SimOneAPI::GetRoadMark(const SSD::SimPoint3D& pos, const SSD::SimString& id, HDMapStandalone::MRoadMark& left, HDMapStandalone::MRoadMark& right)
	{
		return SimOneAPIService::GetRoadMark(pos, id, left, right);
	}

	SIMONE_NET_API void SimOneAPI::GetTrafficLightList(SSD::SimVector<HDMapStandalone::MSignal>& list)
	{
		//use std::move is to avoid killing performance, that extern "C" has limit to expose C++ object as return value
		list = std::move(SimOneAPIService::GetTrafficLightList());
	}

	SIMONE_NET_API void SimOneAPI::GetTrafficSignList(SSD::SimVector<HDMapStandalone::MSignal>& list)
	{
		//use std::move is to avoid killing performance, that extern "C" has limit to expose C++ object as return value
		list = std::move(SimOneAPIService::GetTrafficSignList());
	}

	SIMONE_NET_API void SimOneAPI::GetStoplineList(const HDMapStandalone::MSignal& light, const SSD::SimString& id, SSD::SimVector<HDMapStandalone::MObject>& stoplineList)
	{
		//use std::move is to avoid killing performance, that extern "C" has limit to expose C++ object as return value
		stoplineList = std::move(SimOneAPIService::GetStoplineList(light, id));
	}

	SIMONE_NET_API void SimOneAPI::GetCrosswalkList(const HDMapStandalone::MSignal& light, const SSD::SimString& id, SSD::SimVector<HDMapStandalone::MObject>& crosswalkList)
	{
		//use std::move is to avoid killing performance, that extern "C" has limit to expose C++ object as return value
		crosswalkList = std::move(SimOneAPIService::GetCrosswalkList(light, id));
	}

	SIMONE_NET_API void SimOneAPI::GetCrossHatchList(const SSD::SimString& id, SSD::SimVector<HDMapStandalone::MObject>& crossHatchList)
	{
		//use std::move is to avoid killing performance, that extern "C" has limit to expose C++ object as return value
		crossHatchList = std::move(SimOneAPIService::GetCrossHatchList(id));
	}

	SIMONE_NET_API bool SimOneAPI::GetLaneMiddlePoint(const SSD::SimPoint3D& inputPt, const SSD::SimString& id, SSD::SimPoint3D& targetPoint, SSD::SimPoint3D& dir)
	{
		return SimOneAPIService::GetLaneMiddlePoint(inputPt, id, targetPoint, dir);
	}

	SIMONE_NET_API bool SimOneAPI::GetHeights(const SSD::SimPoint3D& inputPt, const double& radius, SSD::SimVector<double>& heights,
		SSD::SimVector<long>& roadIds, SSD::SimVector<bool>& insideRoadStates)
	{
		return SimOneAPIService::GetHeights(inputPt, radius, heights, roadIds, insideRoadStates);
	}

#ifdef __cplusplus
}
#endif
#endif
