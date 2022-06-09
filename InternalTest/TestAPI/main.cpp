﻿#include "horizon_map_env_ndm.h"
#include "SimOneServiceAPI.h"
#include "SimOneSensorAPI.h"
#include "SimOnePNCAPI.h"

#include <thread>

void Test_NDM_MSG(const SSD::SimPoint3D &input, double forward, const SSD::SimPoint3DVector &inputPts)
{
	HorizonMapEnv::MapEnvMsg_Creator mMapEnv(input, forward, inputPts);
	mMapEnv.CreateMapEnvMsg();
}

int main(int argc, char* argv[])
{
	const char* mv_id = "0";
	double forward = 2000;

	bool isJoinTimeLoop = false;
	const char* serverIP = "10.66.9.111";
	SimOneAPI::InitSimOneAPI(mv_id, isJoinTimeLoop, serverIP);
	if (!SimOneAPI::LoadHDMap(50)) {
		std::cout << "load map failed!!!" << std::endl;
		return -1;
	}
 
	std::unique_ptr<SimOne_Data_WayPoints> pWayPoints = std::make_unique<SimOne_Data_WayPoints>();
	SSD::SimPoint3DVector inputPts;
	if (SimOneAPI::GetWayPoints(mv_id, pWayPoints.get())) {
		for (int index = 0; index < pWayPoints->wayPointsSize; index++) {
			inputPts.push_back({ pWayPoints->wayPoints[index].posX, pWayPoints->wayPoints[index].posY,0 });
		}
	}

	std::unique_ptr<SimOne_Data_Gps> pGps = std::make_unique<SimOne_Data_Gps>();
	int lastFrame = 0;
	while (1)
	{
		bool flag = SimOneAPI::GetGps(mv_id, pGps.get());
		float posX; // Position X on Opendrive (by meter)
		float posY; // Position Y on Opendrive (by meter)
		float posZ; // Position Z on Opendrive (by meter)

		if (flag && pGps->frame != lastFrame)
		{
			SSD::SimPoint3D input{ pGps->posX,pGps->posY,pGps->posZ };

			Test_NDM_MSG(input, forward, inputPts);
			lastFrame = pGps->frame;
		}
		if (!flag)
		{
			std::cout << "Get GPS Fail" << std::endl;
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(3000));
	}


	//tester t(mv_id);

	//t.Test_InitSimOneAPI(isJoinTimeLoop, serverIP);

	// std::vector<std::string> apiNames = {"GetTrafficLightList"}; // {"GetTrafficSignList","GetTrafficLightList","GetCrossHatchList","GetLaneLink"};
	// t.Test_HDMap_ALL(apiNames);
	// t.Test_GetHdMapData();
	// t.Test_GetSensorConfigurations();
	// t.Test_V2X(true);
	// t.Test_UltrasonicRadars(true);
	// t.Test_UltrasonicRadar();
	//t.Test_SensorLaneInfo(true);

	//t.Test_GPS(false);
	//t.Test_SensorDetections(true);
	// t.Test_GetMainVehicleStatus(true);
	//t.Test_RadarDetection(false);
	//t.Test_GetGroundTruth(false);
	// system("pause");

	// t.Test_SetEnvironment();
	// std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	// t.Test_GetEnvironment();

	// t.Test_SetVehicleEvent();

	while (1)
	{
		// t.Test_TerminateSimOneAPI();
		std::this_thread::sleep_for(std::chrono::milliseconds(30));
	}
	return 0;
}
