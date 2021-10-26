#include <iostream>
#include <vector>
#include <cmath>
#include <sstream>

#include "UtilDriver.h"
#include "UtilMath.h"
#include "../include/utilTargetObstacle.h"

#include "SimOneSensorAPI.h"
#include "SimOneV2XAPI.h"
#include "SimOneServiceAPI.h"
#include "SimOneHDMapAPI.h"

SimOne_Data_Gps Gps = SimOne_Data_Gps();
SimOne_Data_CaseInfo pCaseInfoTest = SimOne_Data_CaseInfo();


//Main function
//
int main()
{
	//Wait for the Sim-One case to run
	const char* MainVehicleId ="0";
	bool isJoinTimeLoop = 0;
	SimOneAPI::InitSimOneAPI(MainVehicleId, isJoinTimeLoop);
	while (1) {
		int frame = SimOneAPI::Wait();
		SimOneAPI::GetGps(MainVehicleId,&Gps);
		if (SimOneAPI::GetCaseRunStatus() == ESimOne_Case_Status::ESimOne_Case_Status_Running && (Gps.timestamp > 0)) {
			printf("SimOne Initialized\n");
			SimOneAPI::NextFrame(frame);
			break;
		}
		printf("SimOne Initializing...\n");
		
	}

	//Load Map Data
	int timeout = 20;
	bool slowDown = false;
	int obstacltFlag = 0;
	if (!SimOneAPI::LoadHDMap(timeout)) {
		std::cout << "Failed to load map!" << std::endl;
		return 0;
	}

	
	SSD::SimPoint3D MainVehiclePos(Gps.posX, Gps.posY, Gps.posZ);
	SSD::SimString laneId = utilTargetLane::GetNearMostLane(MainVehiclePos);
	SSD::SimPoint3DVector targetPath = utilTargetLane::GetLaneSample(laneId);

	while (1) {
		int frame = SimOneAPI::Wait();
		
		//exit
		if (SimOneAPI::GetCaseRunStatus() == ESimOne_Case_Status::ESimOne_Case_Status_Stop) {
			break;
		}

		static int warningSignIndex = -1;

		if (SimOneAPI::GetGps(MainVehicleId,&Gps))
		{
			slowDown = utilTargetObstacle::DetectSpeedLimitSign(Gps, warningSignIndex);
			float steering = (float)UtilDriver::calculateSteering(targetPath, &Gps);

			if (slowDown)
			{   //start limit speed
				std::cout << "slow down" << std::endl;
				UtilDriver::setDriver(Gps.timestamp, 0.f, 1.f, steering);
			}
			else
			{
				std::cout << "normal speed" << std::endl;
				UtilDriver::setDriver(Gps.timestamp, 0.3f, 0.f, steering);
			}

			std::vector<utilTargetObstacle::ObstacleStruct> allObstacles = utilTargetObstacle::GetObstacleList();
		
			SSD::SimString leftNeighborLaneName = utilTargetLane::GetLeftNeighborLane(laneId);
			HDMapStandalone::MLaneType type;
			SimOneAPI::GetLaneType(leftNeighborLaneName, type);

			static int warningObstacleIndex = -1;
			if ((int)allObstacles.size() == 0)
			{
				warningObstacleIndex = -1;
			}
			SSD::SimPoint3D vehiclePos(Gps.posX, Gps.posY, Gps.posZ);

			if (warningObstacleIndex == -1)
			{
				slowDown = utilTargetObstacle::DetectObstacle(vehiclePos, allObstacles, type, laneId, leftNeighborLaneName, targetPath, warningObstacleIndex);
			}
			else
			{
				if (utilTargetObstacle::PassedObstacle(vehiclePos, allObstacles[warningObstacleIndex], laneId))
				{
					slowDown = false;
				}
			}
			//std::cout << "targetPath point size:" << targetPath.size() << std::endl;
		}
		SimOneAPI::NextFrame(frame);
	}
}


