#include "SimOneSensorAPI.h"
#include "SimOneV2XAPI.h"
#include "SimOnePNCAPI.h"
#include "SimOneServiceAPI.h"
#include <thread> 
#include <chrono>
#include <iostream>
#include <windows.h>

void Test_V2X(bool IsCallBackMode);
void Test_UltrasonicRadar(bool IsCallBackMode);
void Test_SensorLaneInfo(bool IsCallBackMode);

using namespace std;
int main(int argc, char* argv[])
{
	bool isJoinTimeLoop = false;
	const char* MainVehicleId = "0";
	SimOneAPI::InitSimOneAPI(MainVehicleId, isJoinTimeLoop);
	
	std::unique_ptr<SimOne_Data_Gps> pGps = std::make_unique<SimOne_Data_Gps>();
	std::unique_ptr<SimOne_Data_RadarDetection> pRadarDetection = std::make_unique<SimOne_Data_RadarDetection>();
	std::unique_ptr<SimOne_Data_UltrasonicRadars> pUltrasonics = std::make_unique<SimOne_Data_UltrasonicRadars>();
	std::unique_ptr<SimOne_Data_Obstacle>  pObstacle = std::make_unique<SimOne_Data_Obstacle>();
	std::map<int, SimOne_Data_RadarDetection> RadarDetectionMap;
	std::unique_ptr<SimOne_Data_SensorConfigurations> SensorConfigurations = std::make_unique<SimOne_Data_SensorConfigurations>();
	
	//Test_V2X(false);
	//Test_UltrasonicRadar(false);
	Test_SensorLaneInfo(false);
	system("pause");
	return 0;
}

void Test_SensorLaneInfo(bool IsCallBackMode) {
	if (IsCallBackMode) {
		auto function = [](const char* mainVehicleId, const char* sensorId, SimOne_Data_LaneInfo *pDetections) {
			std::cout << pDetections->frame << "," << pDetections->laneType << "," << pDetections->laneLeftID << "," << pDetections->laneRightID << "," << pDetections->laneRightID<<endl;//The Lane's leftLane ID 
		};
		SimOneAPI::SetSensorLaneInfoCB(function);
	}
	else {
		std::unique_ptr<SimOne_Data_LaneInfo> pDetections = std::make_unique<SimOne_Data_LaneInfo>();
		while (SimOneAPI::GetSensorLaneInfo("0", "objectBasedCamera1", pDetections.get())) {
			std::cout << pDetections->frame <<","<< pDetections->laneType<<","<< pDetections->laneLeftID<<","<< pDetections->laneRightID<<"," << pDetections->laneRightID<<endl;//The Lane's leftLane ID  
		}
	}
}

void Test_UltrasonicRadar(bool IsCallBackMode){
	if (IsCallBackMode) {
		auto function = [](const char * mainVehicleId, SimOne_Data_UltrasonicRadars *pUltrasonics) {
				std::cout << pUltrasonics->frame << "," << pUltrasonics->ultrasonicRadarNum << "," << pUltrasonics->timestamp << "," << pUltrasonics->ultrasonicRadars[0].obstacleDetections[0].obstacleRanges << "," << pUltrasonics->ultrasonicRadars[0].obstacleDetections[0].x << "," << pUltrasonics->ultrasonicRadars[0].obstacleDetections[0].y << endl;
		};
		SimOneAPI::SetUltrasonicRadarsCB(function);
	}
	else {
		std::unique_ptr<SimOne_Data_UltrasonicRadar> pDetections = std::make_unique<SimOne_Data_UltrasonicRadar>();
		while (SimOneAPI::GetUltrasonicRadar("0", "ultrasonic1", pDetections.get())) {
			for (int i = 0; i < pDetections->obstacleNum; i++) {
				std::cout << pDetections->frame << "," << pDetections->obstacleNum << "," << pDetections->obstacleDetections[i].obstacleRanges << "," << pDetections->obstacleDetections[i].x << "," << pDetections->obstacleDetections[i].y << endl;
			}
		}
	}
}

void Test_V2X(bool IsCallBackMode) {
	if (IsCallBackMode) {
		auto function = [](const char* mainVehicleId, const char* sensorId, SimOne_Data_V2XNFS *pDetections) {
			std::cout << "########### SetV2XInfoUpdateCB strlen= "<<pDetections->V2XMsgFrameSize <<"  "<<pDetections->MsgFrameData << std::endl;
		};
		SimOneAPI::SetV2XInfoUpdateCB(function);
	}
	else {
		std::unique_ptr<SimOne_Data_V2XNFS> pDetections = std::make_unique<SimOne_Data_V2XNFS>();
		while (1) {
			if (SimOneAPI::GetV2XInfo("0", "v2x", ESimOne_V2X_MessageFrame_PR::ESimOne_V2X_MessageFrame_PR_bsmFrame, pDetections.get())) {
				std::cout << "########### GetV2XInfo strlen = " << strlen(pDetections->MsgFrameData) << "  " << pDetections->MsgFrameData << std::endl;
				std::this_thread::sleep_for(std::chrono::milliseconds(20));
			}
			else {
				std::cout << "########### GetV2XInfo Fail" << std::endl;
			}
		}
	}
}

