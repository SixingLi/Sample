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
void Test_UltrasonicRadar(const char * MainVehicleID);
void Test_UltrasonicRadars(const char * MainVehicleID, bool IsCallBackMode);
void Test_SensorLaneInfo(const char * MainVehicleID,bool IsCallBackMode);
void Test_SensorSensorDetection(const char * MainVehicleID, bool isCallBackMode);
void Test_RadarDetection(const char * MainVehicleID, bool isCallBackMode);
using namespace std;
int main(int argc, char* argv[])
{
	bool isJoinTimeLoop = false;
	string MainVehicleId = "0";
	SimOneAPI::InitSimOneAPI(MainVehicleId.c_str(), isJoinTimeLoop);
	
	//Test_V2X(false);
	//Test_UltrasonicRadars(MainVehicleId.c_str(),false);
	//Test_UltrasonicRadar(MainVehicleId.c_str());
	//Test_SensorLaneInfo(MainVehicleId.c_str(),true);
	//Test_GPS(MainVehicleId.c_str(),true);
	Test_SensorSensorDetection(MainVehicleId.c_str(),false);
	//Test_RadarDetection(MainVehicleId.c_str(), false);
	system("pause");
	return 0;
}

void Test_RadarDetection(const char * MainVehicleID, bool IsCallBackMode) {
	if (IsCallBackMode) {
		auto function = [](const char* mainVehicleId, const char* sensorId, SimOne_Data_RadarDetection *pDetections) {
			std::cout <<"mainVehicleId:"<< mainVehicleId<<", pDetections->frame:"<<pDetections->frame << ", pDetections->detectNum:"<<pDetections->detectNum <<endl;//The Lane's leftLane ID 
			for (auto i = 0; i < pDetections->detectNum;i++) {
				std::cout <<" test.rcsdb:" << pDetections->detections[i].rcsdb << "," <<" test.rcsdb:"<< pDetections->detections[i].probability << endl;//The Lane's leftLane ID 
			}
		};
		SimOneAPI::SetRadarDetectionsUpdateCB(function);
	}
	else {
		std::unique_ptr<SimOne_Data_RadarDetection> pDetections = std::make_unique<SimOne_Data_RadarDetection>();
		while (true) {
			SimOneAPI::GetRadarDetections(MainVehicleID, "objectBasedRadar1", pDetections.get());
			std::cout <<"mainVehicleId:"<< MainVehicleID <<", pDetections->frame:"<<pDetections->frame << ", pDetections->detectNum:"<<pDetections->detectNum <<endl;//The Lane's leftLane ID 
			std::this_thread::sleep_for(std::chrono::milliseconds(10));
		}
	}
}
void Test_SensorSensorDetection(const char * MainVehicleID, bool isCallBackMode) {
	if (isCallBackMode) {
		auto function = [](const char* MainVehicleID, const char* sensorId, SimOne_Data_SensorDetections *pGroundtruth) {
			printf("hostVehicle:%s frame:%d objectSize:%d\n", MainVehicleID, pGroundtruth->frame, pGroundtruth->objectSize);
			//std::cout <<"hostVehicle:"<<*mainVehicleId<<"frame:"<< pGroundtruth->frame << "," << pGroundtruth->objectSize << endl;//The Lane's leftLane ID 
		};
		SimOneAPI::SetSensorDetectionsUpdateCB(function);
	}
	else {
		std::unique_ptr<SimOne_Data_SensorDetections> pGroundtruth = std::make_unique<SimOne_Data_SensorDetections>();
		while (true) {
			SimOneAPI::GetSensorDetections(MainVehicleID, "sensorFusion1", pGroundtruth.get());
			std::cout << pGroundtruth->frame << "," << pGroundtruth->objectSize << endl;//The Lane's leftLane ID 
			std::this_thread::sleep_for(std::chrono::milliseconds(10));
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
		while (true) {
			SimOneAPI::GetSensorLaneInfo(MainVehicleID, "sensorFusion1", pDetections.get());
			std::cout << pDetections->frame <<","<< pDetections->laneType<<","<< pDetections->laneLeftID<<","<< pDetections->laneLeftID <<"," << pDetections->laneRightID<<endl;//The Lane's leftLane ID  
			std::this_thread::sleep_for(std::chrono::milliseconds(10));
		}
	}
}

void Test_UltrasonicRadars(const char * MainVehicleID, bool IsCallBackMode){
	if (IsCallBackMode) {
		auto function = [](const char * mainVehicleId, SimOne_Data_UltrasonicRadars *pUltrasonics) {
			if(pUltrasonics->ultrasonicRadars[0].obstacleNum>0)
				std::cout << pUltrasonics->frame << "," << pUltrasonics->ultrasonicRadarNum << "," << pUltrasonics->timestamp << "," << pUltrasonics->ultrasonicRadars[0].obstacleDetections[0].obstacleRanges << "," << pUltrasonics->ultrasonicRadars[0].obstacleDetections[0].x << "," << pUltrasonics->ultrasonicRadars[0].obstacleDetections[0].y << endl;
		};
		SimOneAPI::SetUltrasonicRadarsCB(function);
	}
	else {
		std::unique_ptr<SimOne_Data_UltrasonicRadars> pDetections = std::make_unique<SimOne_Data_UltrasonicRadars>();
		while (true) {
			SimOneAPI::GetUltrasonicRadars(MainVehicleID, pDetections.get());
			for (int i = 0; i < pDetections->ultrasonicRadarNum; i++) {
				std::cout << pDetections->frame << ", " << pDetections->ultrasonicRadars[i].obstacleNum << ", " << pDetections->ultrasonicRadars[i].obstacleDetections[i].obstacleRanges << ", " << pDetections-> ultrasonicRadars[i].obstacleDetections[i].x << ", " << pDetections->ultrasonicRadars[i].obstacleDetections[i].y << endl;
			}
			std::this_thread::sleep_for(std::chrono::milliseconds(10));
		}
	}
}

void Test_UltrasonicRadar(const char * MainVehicleID) {
	std::unique_ptr<SimOne_Data_UltrasonicRadar> pDetections = std::make_unique<SimOne_Data_UltrasonicRadar>();
	while (true) {
		SimOneAPI::GetUltrasonicRadar(MainVehicleID, "ultrasonic1", pDetections.get());
		for (int i = 0; i < pDetections->obstacleNum; i++) {
			std::cout << pDetections->frame << ", " << pDetections->obstacleDetections[i].x << ", " << pDetections->obstacleDetections[i].obstacleRanges << ", " << pDetections->obstacleDetections[i].x << ", " << pDetections->obstacleDetections[i].y << endl;
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
	}
}

void Test_GPS(const char * MainVehicleID, bool IsCallBackMode) {
	if (IsCallBackMode) {
		auto function = []( const char* mainVehicleId, SimOne_Data_Gps *pGps){
				std::cout<<"pGps->frame:"<< pGps->frame<<", pGps->posX:"<<pGps->posX<<",pGps->posY"<<pGps->posY<<endl;
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
			std::this_thread::sleep_for(std::chrono::milliseconds(10));
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
			std::this_thread::sleep_for(std::chrono::milliseconds(10));
		}
	}
}

