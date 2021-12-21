#include "SimOneSensorAPI.h"
#include "SimOneV2XAPI.h"
#include "SimOnePNCAPI.h"
#include "SimOneServiceAPI.h"
#include "SimOneHDMapAPI.h"

#include <thread> 
#include <chrono>
#include <iostream>
#include <string.h>
// #include <windows.h>
using namespace std;

// SimOneServiceAPI
void Test_InitSimOneAPI(const char* mainVehicleId = "0", bool isFrameSync =false, const char *serverIP = "127.0.0.1", int port = 23789,void(*startCase)()=0, void(*endCase)()=0, int registerNodeId=0);
// void Test_GetVersion();
// void Test_SendRouteMessage(int length, void* pBuffer, int msgId, int toNodeId, ESimOne_Client_Type toNodeType); ------
// void Test_ReceiveRouteMessageCB(void(*cb)(int fromId, ESimOne_Client_Type fromType, int length, const void* pBuffer, int commandId)); ------
// void Test_SetLogOut(ESimOne_LogLevel_Type level, const char *format, ...); ------

// SimOneV2X
void Test_V2X(const char * MainVehicleID, bool IsCallBackMode);

// SimOneSensor
void Test_SetEnvironment();
void Test_GetEnvironment();

// void Test_GPS(const char * MainVehicleID, bool IsCallBackMode);
// void Test_UltrasonicRadar(const char * MainVehicleID);
// void Test_UltrasonicRadars(const char * MainVehicleID, bool IsCallBackMode);
// void Test_SensorLaneInfo(const char * MainVehicleID,bool IsCallBackMode);
// void Test_SensorSensorDetection(const char * MainVehicleID, bool IsCallBackMode);
// void Test_RadarDetection(const char * MainVehicleID, bool IsCallBackMode);
// void Test_GetGroundTruth(const char * MainVehicleID, bool IsCallBackMode);
// bool Test_HDMap_ALL(const vector<string> &apiNames);

string MainVehicleId = "0";
// SimOnePNCAPI

int main(int argc, char* argv[])
{
	Test_InitSimOneAPI("0", true, "10.66.9.111");
	// Test_GetVersion();
	
	Test_V2X("0", false);
	//Test_UltrasonicRadars(MainVehicleId.c_str(),false);
	//Test_UltrasonicRadar(MainVehicleId.c_str());
	//Test_SensorLaneInfo(MainVehicleId.c_str(),false);
	//Test_GPS(MainVehicleId.c_str(),true);
	//Test_SensorSensorDetection(MainVehicleId.c_str(),true);
	//Test_RadarDetection(MainVehicleId.c_str(), false);
	//Test_GetGroundTruth(MainVehicleId.c_str(), false);
	// system("pause");

	Test_SetEnvironment();
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	Test_GetEnvironment();

	while (1)
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(30));
	}
	return 0;
}

void Test_InitSimOneAPI(const char* mainVehicleId, bool isFrameSync, const char *serverIP, int port,void(*startCase)(), void(*endCase)(), int registerNodeId)
{
	SimOneAPI::InitSimOneAPI(mainVehicleId, isFrameSync, serverIP);
}

void Test_GetVersion()
{
	std::cout << "Version:" << SimOneAPI::GetVersion() << endl;;
}

void Test_SetEnvironment()
{
	// std::unique_ptr<SimOne_Data_Environment> Environment = std::make_unique<SimOne_Data_Environment>();
	SimOne_Data_Environment pEnvironment;

	pEnvironment.timeOfDay = 1000;
        pEnvironment.heightAngle = 90;
        pEnvironment.directionalLight = 0.5f;
        pEnvironment.ambientLight = 0.5f;
        pEnvironment.artificialLight = 0.5f;
        pEnvironment.cloudDensity = 0.5f;
        pEnvironment.fogDensity = 0.5f;
        pEnvironment.rainDensity = 0.5f;
        pEnvironment.snowDensity = 0.5f;
        pEnvironment.groundHumidityLevel = 0.5f;
        pEnvironment.groundDirtyLevel = 0.5f;
        std::cout << "SetEnvironment timeOfDay ([0, 2400]): " << pEnvironment.timeOfDay << std::endl;
        std::cout << "SetEnvironment heightAngle ([0, 90]): " << pEnvironment.heightAngle << std::endl;
        std::cout << "SetEnvironment directionalLight ([0, 1]): " << pEnvironment.directionalLight << std::endl;
        std::cout << "SetEnvironment ambientLight ([0, 1]): " << pEnvironment.ambientLight << std::endl;
        std::cout << "SetEnvironment artificialLight ([0, 1]): " << pEnvironment.artificialLight << std::endl;
        std::cout << "SetEnvironment cloudDensity ([0, 1]): " << pEnvironment.cloudDensity << std::endl;
        std::cout << "SetEnvironment fogDensity ([0, 1]): " << pEnvironment.fogDensity << std::endl;
        std::cout << "SetEnvironment rainDensity ([0, 1]): " << pEnvironment.rainDensity << std::endl;
        std::cout << "SetEnvironment snowDensity ([0, 1]): " << pEnvironment.snowDensity << std::endl;
        std::cout << "SetEnvironment groundHumidityLevel ([0, 1]): " << pEnvironment.groundHumidityLevel << std::endl;
        std::cout << "SetEnvironment groundDirtyLevel ([0, 1]): " << pEnvironment.groundDirtyLevel << std::endl;
        if(!SimOneAPI::SetEnvironment(&pEnvironment))
	{
        	std::cout << "SetEnvironment Failed!" << std::endl; 
	}

	std::cout << "------------ SetEnvironment Done ------------" << std::endl; 
}

void Test_GetEnvironment()
{
	std::unique_ptr<SimOne_Data_Environment> pEnvironment = std::make_unique<SimOne_Data_Environment>();

	if (!SimOneAPI::GetEnvironment(pEnvironment.get()))
	{
		std::cout << "------------ GetEnvironment Failed ! ------------" << std::endl;
		return;
	}
	std::cout << "GetEnvironment timeOfDay ([0, 2400]): " << pEnvironment->timeOfDay << std::endl;
	std::cout << "GetEnvironment heightAngle ([0, 90]): " << pEnvironment->heightAngle << std::endl;
	std::cout << "GetEnvironment directionalLight ([0, 1]): " << pEnvironment->directionalLight << std::endl;
	std::cout << "GetEnvironment ambientLight ([0, 1]): " << pEnvironment->ambientLight << std::endl;
	std::cout << "GetEnvironment artificialLight ([0, 1]): " << pEnvironment->artificialLight << std::endl;
	std::cout << "GetEnvironment cloudDensity ([0, 1]): " << pEnvironment->cloudDensity << std::endl;
	std::cout << "GetEnvironment fogDensity ([0, 1]): " << pEnvironment->fogDensity << std::endl;
	std::cout << "GetEnvironment rainDensity ([0, 1]): " << pEnvironment->rainDensity << std::endl;
	std::cout << "GetEnvironment snowDensity ([0, 1]): " << pEnvironment->snowDensity << std::endl;
	std::cout << "GetEnvironment groundHumidityLevel ([0, 1]): " << pEnvironment->groundHumidityLevel << std::endl;
	std::cout << "GetEnvironment groundDirtyLevel ([0, 1]): " << pEnvironment->groundDirtyLevel << std::endl;
	std::cout << "------------ GetEnvironment Done ------------" << std::endl;
}

void Test_GetGroundTruth(const char * MainVehicleID, bool IsCallBackMode) {
	if (IsCallBackMode) {
		auto function = [](const char* mainVehicleId, SimOne_Data_Obstacle *pObstacle) {
			std::cout << "mainVehicleId:" << mainVehicleId << ", pDetections->frame:" << pObstacle->frame << ", pDetections->detectNum:" << pObstacle->obstacleSize << endl;//The Lane's leftLane ID 
			for (int i = 0; i < pObstacle->obstacleSize; i++) {
				std::cout << "obstacle.type:" << pObstacle->obstacle[i].type << endl;;
			}
		};
		SimOneAPI::SetGroundTruthUpdateCB(function);
	}
	else {
		std::unique_ptr<SimOne_Data_Obstacle> pDetections = std::make_unique<SimOne_Data_Obstacle>();
		while (true) {
			bool flag = SimOneAPI::GetGroundTruth(MainVehicleID,  pDetections.get());
			if (flag) {
				std::cout << "mainVehicleId:" << MainVehicleID << ", pDetections->frame:" << pDetections->frame << ", pDetections->detectNum:" << pDetections->obstacleSize << endl;//The Lane's leftLane ID 
				for (int i = 0; i < pDetections->obstacleSize; i++) {
					std::cout << "obstacle.type:" << pDetections->obstacle[i].type << endl;;
				}
			}
			std::this_thread::sleep_for(std::chrono::milliseconds(30));
		}
	}
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
			bool flag = SimOneAPI::GetRadarDetections(MainVehicleID, "objectBasedRadar1", pDetections.get());
			if (flag) {
				std::cout <<"mainVehicleId:"<< MainVehicleID <<", pDetections->frame:"<<pDetections->frame << ", pDetections->detectNum:"<<pDetections->detectNum <<endl;//The Lane's leftLane ID 
				for (int i = 0; i < pDetections->detectNum; i++)
				{
					std::cout << "detections.range:" << pDetections->detections[i].range <<",pDetections->detections[i].azimuth:"<< pDetections->detections[i].azimuth <<endl;
				}
			}
			std::this_thread::sleep_for(std::chrono::milliseconds(30));
		}
	}
}
void Test_SensorSensorDetection(const char * MainVehicleID, bool IsCallBackMode) {
	if (IsCallBackMode) {
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
			std::cout << "########### sensorId:" << sensorId<<" SetV2XInfoUpdateCB strlen= "<<pDetections->V2XMsgFrameSize <<"  "<<pDetections->MsgFrameData << std::endl;
		};
		SimOneAPI::SetV2XInfoUpdateCB(function);
	}
	else {
		std::unique_ptr<SimOne_Data_V2XNFS> pDetections = std::make_unique<SimOne_Data_V2XNFS>();
		while (1) {
			if (SimOneAPI::GetV2XInfo(MainVehicleID, "obu1", ESimOne_V2X_MessageFrame_PR::ESimOne_V2X_MessageFrame_PR_bsmFrame, pDetections.get())) {
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

bool Test_HDMap_ALL(const vector<string> &apiNames) {
	int success_count = 0;
	if (SimOneAPI::LoadHDMap(3)) {
		for (auto apiName : apiNames) {
			std::cout << "get hdmap data success" << endl;
			if (apiName == "GetTrafficSignList") {
				SSD::SimVector<HDMapStandalone::MSignal> list;
				SimOneAPI::GetTrafficSignList(list);
				int listSize = list.size();
				if (listSize > 0) {
					std::cout << ">>>>>>>>>>>>>>>>>>>>>  GetTrafficSignList Size = " << listSize << endl;
					success_count++;
				}
			}
			else if (apiName == "GetTrafficLightList") {
				SSD::SimVector<HDMapStandalone::MSignal> list;
				SimOneAPI::GetTrafficLightList(list);
				int listSize = list.size();
				if (listSize > 0) {
					std::cout << ">>>>>>>>>>>>>>>>>>>>>  GetTrafficLightList Size = " << listSize << endl;
					success_count++;
				}

				for (auto signal : list){
					std::unique_ptr<SimOne_Data_TrafficLight> pDetections = std::make_unique<SimOne_Data_TrafficLight>();
					if(SimOneAPI::GetTrafficLight(MainVehicleId.c_str(), signal.id, pDetections.get()))
						printf(">>>>>>>>>>>>>>>>>>>>>  SoGetTrafficLights opendriveId = %d, trafficLigtData.countDown = %d, trafficLigtData.status = %d\n", signal.id, pDetections->countDown, pDetections->status);

				}
			}
			else if (apiName == "GetCrossHatchList") {
				SSD::SimString id;
				SSD::SimVector<HDMapStandalone::MObject> crossHatchList;
				double s, t, s_toCenterLine, t_toCenterLine;
				std::unique_ptr<SimOne_Data_Gps> pDetections = std::make_unique<SimOne_Data_Gps>();
				SimOneAPI::GetGps(MainVehicleId.c_str(), pDetections.get());
				SSD::SimPoint3D pos = { pDetections->posX,pDetections->posY ,pDetections->posZ };
				SimOneAPI::GetNearMostLane(pos, id, s, t, s_toCenterLine, t_toCenterLine);
				SimOneAPI::GetCrossHatchList(id, crossHatchList);
				int listSize = crossHatchList.size();
				if (listSize > 0) {
					std::cout << ">>>>>>>>>>>>>>>>>>>>>  getCrossHatchList Size = " << listSize << endl;
					success_count++;
				}
			}
			else if (apiName == "GetLaneLink") {
				SSD::SimString id;
				HDMapStandalone::MLaneLink laneLink;
				double s, t, s_toCenterLine, t_toCenterLine;
				std::unique_ptr<SimOne_Data_Gps> pDetections = std::make_unique<SimOne_Data_Gps>();
				SimOneAPI::GetGps(MainVehicleId.c_str(), pDetections.get());
				SSD::SimPoint3D pos = { pDetections->posX,pDetections->posY ,pDetections->posZ };
				SimOneAPI::GetNearMostLane(pos, id, s, t, s_toCenterLine, t_toCenterLine);
				SimOneAPI::GetLaneLink(id, laneLink);
				string laneID = id.GetString();
				int listSize = laneLink.successorLaneNameList.size();
				SSD::SimString leftLaneName = laneLink.leftNeighborLaneName;
				if (listSize > 0 && !leftLaneName.Empty()) {
					std::cout << ">>>>>>>>>>>>>>>>>>>>>  getLaneLink successorLaneNameList Size = " << listSize << endl;
					success_count++;
				}
			}
		}
	}
	else {
		std::cout << "#####################  LoadHDMap Data fail!!!" << endl;
		std::cout << "#####################  Test_HDMap_ALL fail!!!" << endl;
		return false;
	}
	if (success_count == apiNames.size()) {
		std::cout << "#####################  Test_HDMap_ALL success!!!" << endl;
		return true;
	}
	else {
		std::cout << "#####################  Test_HDMap_ALL fail!!!" << endl;
		return false;
	}
}

