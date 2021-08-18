#include <iostream>
#include <vector>
#include <cmath>
#include "SSD/SimPoint3D.h"
#include "SSD/SimString.h"
#include "public/common/MLaneInfo.h"
#include "public/common/MLaneId.h"
#include "UtilDriver.h"
#include "assert.h"
#include "SimOneSensorAPI.h"
#include "SimOneV2XAPI.h"
#include "SimOneServiceAPI.h"
#include "SimOneHDMapAPI.h"

using namespace HDMapStandalone;

SimOne_Data_Gps Gps = SimOne_Data_Gps();
SimOne_Data_CaseInfo pCaseInfoTest = SimOne_Data_CaseInfo();
SimOne_Data_WayPoints WayPoints;
SSD::SimPoint3DVector targetPath;


SSD::SimVector<long> GetNavigateRoadIdList(const SSD::SimPoint3D& startPt, const SSD::SimPoint3D& endPt)
{
	SSD::SimVector<long> naviRoadIdList;
	SSD::SimPoint3DVector ptList;
	ptList.push_back(startPt);
	ptList.push_back(endPt);
	SSD::SimVector<int> indexOfValidPoints;
	SimOneAPI::Navigate(ptList, indexOfValidPoints, naviRoadIdList);
	return std::move(naviRoadIdList);
}

bool GetValidSuccessor(const MLaneId& laneId, const long& currentRoadId, const long& nextRoadId, MLaneId& successor)
{
	if (nextRoadId == -1)
	{
		return false;
	}
	MLaneLink laneLink;

	//Get connected road information
	bool valid = SimOneAPI::GetLaneLink(laneId.ToString(), laneLink);
	assert(valid);

	//Check whether the current road has a successor road
	if (laneLink.successorLaneNameList.size() == 0)
	{
		return false;
	}

	//Traverse all the successor roads, find the RoadId of the successor road is the target road of the current road RoadId
	for (auto& successorLane : laneLink.successorLaneNameList)
	{
		MLaneId successorId(successorLane);
		if (successorId.roadId != currentRoadId)
		{
			if (successorId.roadId == nextRoadId)
			{
				successor = successorLane;
				return true;
			}
		}
		else
		{
			//Recursive
			MLaneId successorOfSuccessor;
			if (GetValidSuccessor(successorId, currentRoadId, nextRoadId, successorOfSuccessor))
			{
				if (successorOfSuccessor.roadId == nextRoadId)
				{
					successor = successorId;
					return true;
				}
			}
		}
	}
	return false;
}

void AddSamples(const MLaneId& laneId, SSD::SimPoint3DVector& path)
{
	MLaneInfo laneInfo;
	//Get the centerline of the lane according to the road Id
	if (SimOneAPI::GetLaneSample(laneId.ToString(), laneInfo))
	{
		path.reserve(path.size() + laneInfo.centerLine.size());
		for (auto& pt : laneInfo.centerLine)
		{
			//Put the center point in the target path
			path.push_back(pt);
		}
	}
}

SSD::SimPoint3DVector GetReferencePath(const SSD::SimPoint3D& startPt, const SSD::SimVector<long>& naviRoadIdList)
{
	SSD::SimPoint3DVector path;
	double s, t, s_toCenterLine, t_toCenterLine;
	HDMapStandalone::MLaneInfo info;

	SSD::SimString laneName;

	//Get the lane information of the current main vehicle
	bool ret = SimOneAPI::GetNearMostLane(startPt, laneName, s, t, s_toCenterLine, t_toCenterLine);
	assert(ret);
	MLaneId id(laneName);
	if (id.roadId != naviRoadIdList[0])
	{
		return std::move(path);
	}

	AddSamples(id, path);

	int index = 0;

	//Get the centerline of all roads
	while (index < (int)naviRoadIdList.size())
	{
		long roadId = naviRoadIdList[index];
		long nextRoadId = -1;
		if (index + 1 < (int)naviRoadIdList.size())
		{
			nextRoadId = naviRoadIdList[index + 1];
		}
		MLaneId successorId;

		//查找正确的前驱道路Id
		if (GetValidSuccessor(id, roadId, nextRoadId, successorId))
		{
			AddSamples(successorId, path);
			id = successorId;
			if (successorId.roadId != roadId)
			{
				index++;
			}
		}
		else
		{
			break;
		}
	}

	return std::move(path);
}


//Main function
//
int main()
{

	bool isJoinTimeLoop = false;
	int MainVehicleId = 0;
	SimOneAPI::SimOneAPIInitialized(MainVehicleId, isJoinTimeLoop);
	//Wait for the Sim-One case to run
	while (1) {
		int frame = SimOneAPI::Wait();
		SimOneAPI::GetGps(MainVehicleId,&Gps);
		if (SimOneAPI::GetCaseRunStatus() == SimOne_Case_Status::SimOne_Case_Status_Running && (Gps.timestamp > 0)) {
			printf("SimOne Initialized\n");
			SimOneAPI::NextFrame(frame);
			break;
		}
		printf("SimOne Initializing...\n");
	}

	//Load map data
	int timeout = 20;
	bool slowDown = false;
	int obstacltFlag = 0;
	if (!SimOneAPI::LoadHDMap(timeout)) {
		std::cout << "Failed to load map!" << std::endl;
		return 0;
	}

	SSD::SimPoint3D startPt, endPt;
	if (SimOneAPI::GetGps(MainVehicleId,&Gps))
	{
		startPt.x = Gps.posX;
		startPt.y = Gps.posY;
		startPt.z = Gps.posZ;

		//Get the end point
		if (SimOneAPI::GetWayPoints(&WayPoints))
		{
			int waySize = WayPoints.wayPointsSize;
			endPt.x = WayPoints.wayPoints[waySize - 1].posX;
			endPt.y = WayPoints.wayPoints[waySize - 1].posY;
			endPt.z = 0;
		}

		//according to the start and end points，Get the road id information of the route
		SSD::SimVector<long> naviRoadIdList = GetNavigateRoadIdList(startPt, endPt);

		if (SimOneAPI::GetGps(MainVehicleId,&Gps))
		{
			float MainVehiclePosX = Gps.posX;
			float MainVehiclePosY = Gps.posY;
			float MainVehiclePosZ = Gps.posZ;
			std::cout << MainVehiclePosX << std::endl;
			SSD::SimPoint3D MainVehicle(MainVehiclePosX, MainVehiclePosY, MainVehiclePosZ);
			targetPath = std::move(GetReferencePath(MainVehicle, naviRoadIdList));
		}
	}

	while (1)
	{
		int frame = SimOneAPI::Wait();
		//exit
		if (SimOneAPI::GetCaseRunStatus() == SimOne_Case_Status::SimOne_Case_Status_Stop) {
			break;
		}

		//According to the path planning point, control the main vehicle
		SimOneAPI::GetGps(MainVehicleId,&Gps);
		std::cout << "targetpath size :" << targetPath.size() << std::endl;
		std::cout << "current mainVehicle posX, posY:" << Gps.posX << " " << Gps.posY << std::endl;

		float steering = (float)UtilDriver::calculateSteering(targetPath, &Gps);
		std::cout << "steering:" << steering << std::endl;

		if ((int(sqrtf(pow(Gps.velX, 2) + pow(Gps.velY, 2))) * 3.6f) > 30) {
			UtilDriver::setDriver(Gps.timestamp, 0.f, 0.1f, steering);
		}
		else {
			UtilDriver::setDriver(Gps.timestamp, 0.3f, 0.f, steering);
		}
		SimOneAPI::NextFrame(frame);
	}
}
