#include "SimOneSensorAPI.h"
#include "SimOneV2XAPI.h"
#include "SimOnePNCAPI.h"
#include "SimOneServiceAPI.h"
#include <thread> 
#include <chrono>
#include <iostream>
#include <string.h>
// #include <windows.h>

void Test_V2X(const char * MainVehicleID, bool IsCallBackMode);
void Test_GPS(const char * MainVehicleID, bool IsCallBackMode);
void Test_UltrasonicRadar(const char * MainVehicleID,bool IsCallBackMode);
void Test_SensorLaneInfo(const char * MainVehicleID,bool IsCallBackMode);
void Test_SensorSensorDetection(const char * MainVehicleID, bool isCallBackMode);
using namespace std;
int main(int argc, char* argv[])
{
	bool isJoinTimeLoop = false;
	string MainVehicleId = "1";
	SimOneAPI::InitSimOneAPI(MainVehicleId.c_str(), isJoinTimeLoop);
	
	//Test_V2X(false);
	//Test_UltrasonicRadar(false);
	//Test_SensorLaneInfo(false);
	//Test_GPS("0",false);
	Test_SensorSensorDetection(MainVehicleId.c_str(),true);
	system("pause");
	return 0;
}

void Test_SensorSensorDetection(const char * MainVehicleID, bool isCallBackMode) {
	if (isCallBackMode) {
		auto function = [](const char* mainVehicleId, const char* sensorId, SimOne_Data_SensorDetections *pGroundtruth) {
			std::cout << pGroundtruth->frame << "," << pGroundtruth->objectSize << endl;//The Lane's leftLane ID 
		};
		SimOneAPI::SetSensorDetectionsUpdateCB(function);
	}
	else {
		std::unique_ptr<SimOne_Data_SensorDetections> pGroundtruth = std::make_unique<SimOne_Data_SensorDetections>();
		while (SimOneAPI::GetSensorDetections(MainVehicleID, "objectBasedCamera1", pGroundtruth.get())) {
			std::cout << pGroundtruth->frame << "," << pGroundtruth->objectSize << endl;//The Lane's leftLane ID 
		}
	}
}

void Test_SensorLaneInfo(const char * MainVehicleID, bool IsCallBackMode) {
	if (IsCallBackMode) {
		auto function = [](const char* mainVehicleId, const char* sensorId, SimOne_Data_LaneInfo *pDetections) {
			std::cout << pDetections->frame << "," << pDetections->laneType << "," << pDetections->laneLeftID << "," << pDetections->laneRightID << "," << pDetections->laneRightID<<endl;//The Lane's leftLane ID 
		};
		SimOneAPI::SetSensorLaneInfoCB(function);
	}
	else {
		std::unique_ptr<SimOne_Data_LaneInfo> pDetections = std::make_unique<SimOne_Data_LaneInfo>();
		while (SimOneAPI::GetSensorLaneInfo(MainVehicleID, "objectBasedCamera1", pDetections.get())) {
			std::cout << pDetections->frame <<","<< pDetections->laneType<<","<< pDetections->laneLeftID<<","<< pDetections->laneRightID<<"," << pDetections->laneRightID<<endl;//The Lane's leftLane ID  
		}
	}
}

void Test_UltrasonicRadar(const char * MainVehicleID, bool IsCallBackMode){
	if (IsCallBackMode) {
		auto function = [](const char * mainVehicleId, SimOne_Data_UltrasonicRadars *pUltrasonics) {
				std::cout << pUltrasonics->frame << "," << pUltrasonics->ultrasonicRadarNum << "," << pUltrasonics->timestamp << "," << pUltrasonics->ultrasonicRadars[0].obstacleDetections[0].obstacleRanges << "," << pUltrasonics->ultrasonicRadars[0].obstacleDetections[0].x << "," << pUltrasonics->ultrasonicRadars[0].obstacleDetections[0].y << endl;
		};
		SimOneAPI::SetUltrasonicRadarsCB(function);
	}
	else {
		std::unique_ptr<SimOne_Data_UltrasonicRadar> pDetections = std::make_unique<SimOne_Data_UltrasonicRadar>();
		while (SimOneAPI::GetUltrasonicRadar(MainVehicleID, "ultrasonic1", pDetections.get())) {
			for (int i = 0; i < pDetections->obstacleNum; i++) {
				std::cout << pDetections->frame << "," << pDetections->obstacleNum << "," << pDetections->obstacleDetections[i].obstacleRanges << "," << pDetections->obstacleDetections[i].x << "," << pDetections->obstacleDetections[i].y << endl;
			}
		}
	}
}

void Test_GPS(const char * MainVehicleID, bool IsCallBackMode) {
	if (IsCallBackMode) {
		auto function = []( const char* mainVehicleId, SimOne_Data_Gps *pGps){
				std::cout<<"pGps->posX:"<<pGps->posX<<",pGps->posY"<<pGps->posY<<endl;	
		};
		 SimOneAPI::SetGpsUpdateCB(function);
	}
	else {
		std::unique_ptr<SimOne_Data_Gps> pGps = std::make_unique<SimOne_Data_Gps>();
		while(1)
		{
			if(SimOneAPI::GetGps(MainVehicleID, pGps.get())){
				std::cout<<"pGps->posX:"<<pGps->posX<<",pGps->posY"<<pGps->posY<<endl;					
			}else{

				std::cout<<"Get GPS Fail"<<endl;
			}
		}
	}
}

void Test_V2X(const char * MainVehicleID, bool IsCallBackMode) {

	if (IsCallBackMode) {
		auto function = [](const char* mainVehicleId, const char* sensorId, SimOne_Data_V2XNFS *pDetections) {
			std::cout << "########### SetV2XInfoUpdateCB strlen= "<<pDetections->V2XMsgFrameSize <<"  "<<pDetections->MsgFrameData << std::endl;
		};
		SimOneAPI::SetV2XInfoUpdateCB(function);
	}
	else {
		std::unique_ptr<SimOne_Data_V2XNFS> pDetections = std::make_unique<SimOne_Data_V2XNFS>();
		while (1) {
			if (SimOneAPI::GetV2XInfo(MainVehicleID, "v2x", ESimOne_V2X_MessageFrame_PR::ESimOne_V2X_MessageFrame_PR_bsmFrame, pDetections.get())) {
				std::cout << "########### GetV2XInfo strlen = " << strlen(pDetections->MsgFrameData) << "  " << pDetections->MsgFrameData << std::endl;
				std::this_thread::sleep_for(std::chrono::milliseconds(20));
			}
			else {
				std::cout << "########### GetV2XInfo Fail" << std::endl;
			}
		}
	}
}

