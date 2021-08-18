﻿// ==========================================================================
// Copyright (C) 2018 - 2021 Beijing 51WORLD Digital Twin Technology Co., Ltd. 
// , and/or its licensors.  All rights reserved.
//
// The coded instructions, statements, computer programs, and/or related 
// material (collectively the "Data") in these files contain unpublished
// information proprietary to Beijing 51WORLD Digital Twin Technology Co., Ltd. 
// ("51WORLD") and/or its licensors,  which is protected by the People's 
// Republic of China and/or other countries copyright law and by 
// international treaties.
//
// The Data may not be disclosed or distributed to third parties or be
// copied or duplicated, in whole or in part, without the prior written
// consent of 51WORLD.
//
// The copyright notices in the Software and this entire statement,
// including the above license grant, this restriction and the following
// disclaimer, must be included in all copies of the Software, in whole
// or in part, and all derivative works of the Software, unless such copies
// or derivative works are solely in the form of machine-executable object
// code generated by a source language processor.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND.
// 51WORLD DOES NOT MAKE AND HEREBY DISCLAIMS ANY EXPRESS OR IMPLIED
// WARRANTIES INCLUDING, BUT NOT LIMITED TO, THE WARRANTIES OF
// NON-INFRINGEMENT, MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE,
// OR ARISING FROM A COURSE OF DEALING, USAGE, OR TRADE PRACTICE. IN NO
// EVENT WILL 51WORLD AND/OR ITS LICENSORS BE LIABLE FOR ANY LOST
// REVENUES, DATA, OR PROFITS, OR SPECIAL, DIRECT, INDIRECT, OR
// CONSEQUENTIAL DAMAGES, EVEN IF 51WORLD AND/OR ITS LICENSORS HAS
// BEEN ADVISED OF THE POSSIBILITY OR PROBABILITY OF SUCH DAMAGES.
// ==========================================================================
#pragma once

#pragma warning(disable:4819)
#pragma warning(disable:4190)

#ifndef WITHOUT_HDMAP
#include "SimOneHDMapAPI.h"
#include "SSD/SimString.h"
#include "SSD/SimVector.h"
#include "SSD/SimPoint3D.h"
#include "public/common/MTopoGraph.h"
#include "public/common/MRoadMark.h"
#include "public/common/MEnum.h"
#include "public/common/MLaneInfo.h"
#include "public/common/MRoutePath.h"
#endif

#ifdef __cplusplus
extern "C"
{
#endif
namespace SimOneAPI {
#ifndef WITHOUT_HDMAP
	struct LaneSample
	{
		int laneCode;  //1, 2, ...
		bool inJunction = false;
		SSD::SimPoint3DVector leftBoundary;    //Left boundary sample data£ºlane_line_left_data
		SSD::SimPoint3DVector rightBoundary;  //Right boundary sample data£ºlane_line_right_data
	};

	struct LaneData
	{
		SSD::SimVector<LaneSample> laneSampleList;
		SSD::SimStringVector laneNameList;
		HDMapStandalone::MRoadMark leftRoadMark;
		HDMapStandalone::MRoadMark rightRoadMark;
	};

	struct LaneInfo
	{
		SSD::SimString currentLane;
		SSD::SimVector<LaneData> dataList;
	};

	struct TyrePosInfo
	{
		SSD::SimPoint3D frontLeft;
		SSD::SimPoint3D frontRight;
		SSD::SimPoint3D rearLeft;
		SSD::SimPoint3D rearRight;
	};


	/*!
	获取所有车道线信息列表。
	\li function:
	*	GetLaneData
	\li brief:
	*	 Get all lane's info in the map.
	@param[out]
	*   data: All lane's MLaneInfo object as a list.
	*/
	SIMONE_NET_API void GetLaneData(SSD::SimVector<HDMapStandalone::MLaneInfo>& data);

	/*!
	获取所有Junction ID列表。
	\li function:
	*	GetJunctionList
	\li brief:
	*	 Get all junction id list in the map.
	@return
	*	Junction Id list.
	*/
	SIMONE_NET_API SSD::SimVector<long> GetJunctionList();

	/*!
	获取道路长度
	\li function:
	*	GetRoadLength
	\li brief:
	*	 Get road's length.
	@param
	*   id: Input road ID.
	@return
	*	Length of road.
	*/
	SIMONE_NET_API double GetRoadLength(const long& roadId);

	/*!
	获取指定车道线所在Section的所有车道线ID列表
	\li function:
	*	GetSectionLaneList
	\li brief:
	*	Get lane id list in the same section for specified lane id.
		Note that roadId_sectionIndex_laneId's laneId should not be set as 0, as it does not make sense to use 0.
	@param
	*   laneId: Input lane ID. ID with this format roadId_sectionIndex_laneId.
	@param[out]
	*   sectionLaneList: Lane id list in the same section for specified lane.
	@return
	*	True when any lane(lanes) is(are) found, else returns false.
	*/
	SIMONE_NET_API bool GetSectionLaneList(const SSD::SimString& laneId, SSD::SimStringVector& sectionLaneList);

	/*!
	判断指定道路是否是双向道路
	\li function:
	*	IsTwoSideRoad
	\li brief:
	*	Check whether specified road is two-side road or not.
	@param
	*   roadId: Input road ID.
	@return
	*	True if is two-side road, else returns false.
	*/
	SIMONE_NET_API bool IsTwoSideRoad(const long& roadId);
	
	/*!
	获取输入点所在路段的车道和道路相关信息，包括车道编号，地面指示行车箭头，前方100米车道线采样点，是否压线，车道线类型
	\li function:
	*	GetLaneIndexList
	\li brief:
	*	Get current location belonging road section by returning all lanes' code, like 0, 1, 2, ... .
	The reported lanes are on the same side of road section regarding current location. The reversed side lanes should be omitted.
	@param
	*   pos: Input 3d location.
	@param
	*   tyrePosInfo_: Input tyre postion info.
	@param
	*   forward: Input forward distance to detect lane lines.
	@param[out]
	*   currentLaneIndex: Lane code that current location belongs to. Code as this format, 0, 1, 2, ... .
	@param[out]
	*   laneIdList: Belong road section's all lanes' ID list. Each ID is global unique in this map.
	@return
	*	Belong road section's all lanes' code list. Code as this format, 0, 1, 2, ... . Can be empty.
	*/
	SIMONE_NET_API LaneInfo GetForwardLaneInfo(const SSD::SimPoint3D& pos, const TyrePosInfo& tyrePosInfo, const double& forward);

	/*!
	获取主车位置所在车道信息(包含车道ID，左右边缘线，虚拟中心线)
	\li function:
	*	GetLaneSampleByLocation
	\li brief:
	*	 Get all lane data in the loaded map
	@param
	*   pos: Input 3d location.
	@param[out]
	*   info: Lane information(HDMapStandalone::MLaneInfo) of specified lane.
	@return
	*	True if specified lane exists in the map, else returns false.
	*/
	SIMONE_NET_API bool GetTopoGraph(HDMapStandalone::MTopoGraph& topoGraph);

	/*!
	获取车道长度
	\li function:
	*	GetLaneLength
	\li brief:
	*	 Get lane's length.
	@param
	*   id: Input lane ID. ID with this format roadId_sectionIndex_laneId.
	@return
	*	Length of lane.
	*/
	SIMONE_NET_API double GetLaneLength(const SSD::SimString& id);

	/*!
	判断车道是否为机动车道
	\li function:
	*	IsDriving
	\li brief:
	*	Check whether current lane is driving.
	@param
	*   id: Input lane ID. ID with this format roadId_sectionIndex_laneId.
	@return
	*	True if specified lane is driving type, else returns false.
	*/
	SIMONE_NET_API bool IsDriving(const SSD::SimString& id);

	/*!
	判断车道是否在交叉路口内
	\li function:
	*	IsInJunction
	\li brief:
	*	Check whether current lane belongs to a junction.
	@param
	*   id: Input lane ID. ID with this format roadId_sectionIndex_laneId.
	@param[out]
	*   juncId: owner junction id.
	@return
	*	True if specified lane is in a junction, else returns false.
	*/
	SIMONE_NET_API bool IsInJunction(const SSD::SimString& id, long& juncId);

	/*!
	判断坐标点是否在车道内
	\li function:
	*	IsInsideLane
	\li brief:
	*	Check whether current lane belongs to a junction.
	@param
	*   inputPt: Input 3d location.
	@param
	*   laneName: Input lane ID. ID with this format roadId_sectionIndex_laneId.
	@param[out]
	*   sideState: MSideState that specifies whether it is outside as left side or right side, or is inside.
	@return
	*	True if specified point is inside the specified lane, else returns false.
	*/
	SIMONE_NET_API bool IsInsideLane(const SSD::SimPoint3D& inputPt, const SSD::SimString& laneName, HDMapStandalone::MSideState& sideState);

	/*!
	考虑高程下获取最接近输入点的车道
	\li function:
	*	GetNearMostLaneWithHeight
	\li brief:
	*	Respecting height of input point, get the lane which is near most to or geometry overlapping the input point.
		When there are more than one lane's geometry overlaps the input point, will pick the distance near most one.
		If input point's height is 2 meters gap with lane's height, such lane will be ignored.
	@param
	*   pos: Input 3d location.
	@param
	*   drivingOnly: A flag whether to find dirving only lanes or not.
	@param[out]
	*   id: Lane ID of founded lane. ID with this format roadId_sectionIndex_laneId.
	@param[out]
	*   s, t: The input point's value pair in s-t coordinate system, relative to the found lane.
	@param[out]
	*   s_toCenterLine, t_toCenterLine: is the input point's value pair in s-t coordinate system, relative to the found lane's owner road's center line.
	*   Values are fuzzy accurate, please use API GetRoadST for highly accurate values for [s_toCenterLine, t_toCenterLine].
	@param[out]
	*   insideLane: Returns whether input point is inside the near most lane or not.
	@return
	*	True when any lane is found, else returns false.
	*/
	SIMONE_NET_API bool GetNearMostLaneWithHeight(const SSD::SimPoint3D& pos, bool drivingOnly, SSD::SimString& id, double& s, double& t,
		double& s_toCenterLine, double& t_toCenterLine, bool& insideLane);

	/*!
	获取前方指定距离所途径的所有车道信息(包含车道ID，左右边缘线，虚拟中心线)
	\li function:
	*	GetForwardLaneSample
	\li brief:
	*	 Get all lane data by specified distance in forward direction of current position.
	@param
	*   inputPt: Input 3d location.
	@param
	*   laneName: Input lane ID. ID with this format roadId_sectionIndex_laneId.
	@param
	*   forward: The distance in forward direction. Its maximum limit is 2000 meters, and it minimum limit is greater than 0 meter.
	@param[out]
	*   laneInfoList: MLaneInfo object list.
	@return
	*	True if data is found, else return false.
	*/
	SIMONE_NET_API bool GetForwardLaneSample(const SSD::SimPoint3D& inputPt, const SSD::SimString& laneName, const double& forward, SSD::SimVector<HDMapStandalone::MLaneInfo>& laneInfoList);

	/*!
	获取地图中所有车道线信息
	\li function:
	*	GetLaneLineInfo
	\li brief:
	*	 Get all lanes' lane line info in the loaded map.
	@param
	*   forward: The distance in forward direction. Its maximum limit is 2000 meters, and it minimum limit is greater than 0 meter.
	@param[out]
	*   laneInfoList: MLaneLineInfo list for all lanes.
	*/
	SIMONE_NET_API void GetLaneLineInfo(SSD::SimVector<HDMapStandalone::MLaneLineInfo>& laneLineInfo);

	/*!
	获取路网路径规划
	\li function:
	*	GenerateRoute_v2
	\li brief:
	*	Generate route for specified input points
	@param
	*   inputPoints: Input points that to guide generated route should pass over
	@param[out]
	*   indexOfValidPoints: Pick valid ones from input points. Valid ones will be used for generating route.
	@param[out]
	*   path: Generated route path.
	@param[out]
	*   routePtList: MRoutePoint object list.
	@return
	*	True if any route has been generated, else returns false.
	*/
	SIMONE_NET_API bool GenerateRoute_v2(const SSD::SimPoint3DVector& sampleNodes, SSD::SimVector<int>& indexOfValidPoints,
		HDMapStandalone::MRoutePath& path, SSD::SimVector<HDMapStandalone::MRoutePoint>& routePtList);

	/*!
	获取指定道路的左右Section名字列表
	\li function:
	*	GetSectionList
	\li brief:
	*	Get right and left section name list for specified road id.
	@param
	*   roadId: Specified road id.
	@param[out]
	*   rightList: Returns right side section name list.
	@param[out]
	*   leftList: Returns left side section name list.
	*/
	SIMONE_NET_API void GetSectionList(const long& roadId, SSD::SimStringVector& rightList, SSD::SimStringVector& leftList);
#endif
}
#ifdef __cplusplus
}
#endif
