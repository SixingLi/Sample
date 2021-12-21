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


void Test_GetSensorConfigurations(const char * MainVehicleID);
void Test_V2X(const char * MainVehicleID, bool IsCallBackMode);
void Test_GPS(const char * MainVehicleID, bool IsCallBackMode);
void Test_UltrasonicRadar(const char * MainVehicleID);
void Test_UltrasonicRadars(const char * MainVehicleID, bool IsCallBackMode);
void Test_SensorLaneInfo(const char * MainVehicleID,bool IsCallBackMode);
void Test_SensorSensorDetection(const char * MainVehicleID, bool IsCallBackMode);
void Test_RadarDetection(const char * MainVehicleID, bool IsCallBackMode);
void Test_GetGroundTruth(const char * MainVehicleID, bool IsCallBackMode);
bool Test_HDMap_ALL(const vector<string> &apiNames);

string MainVehicleId = "1";

int main(int argc, char* argv[])
{
	bool isJoinTimeLoop = false;	
	SimOneAPI::InitSimOneAPI(MainVehicleId.c_str(), isJoinTimeLoop);
	//std::vector<std::string> apiNames = {"GetTrafficSignList","GetTrafficLightList","GetCrossHatchList","GetLaneLink"};
	//std::vector<std::string> apiNames = {"GetTrafficLightList"};
	//Test_HDMap_ALL(apiNames);
	Test_GetSensorConfigurations(MainVehicleId.c_str());
	//Test_V2X(MainVehicleId.c_str(),true);
	//Test_UltrasonicRadars(MainVehicleId.c_str(),false);
	//Test_UltrasonicRadar(MainVehicleId.c_str());
	//Test_SensorLaneInfo(MainVehicleId.c_str(),false);
	//Test_GPS(MainVehicleId.c_str(),true);
	//Test_SensorSensorDetection(MainVehicleId.c_str(),true);
	//Test_RadarDetection(MainVehicleId.c_str(), false);
	//Test_GetGroundTruth(MainVehicleId.c_str(), false);
	system("pause");
	return 0;
}

void Test_GetSensorConfigurations(const char * MainVehicleID) {
	while (true) {
		std::unique_ptr<SimOne_Data_SensorConfigurations> pConfigs = std::make_unique<SimOne_Data_SensorConfigurations>();
		if (SimOneAPI::GetSensorConfigurations(MainVehicleID, pConfigs.get())) {
			for (int i = 0; i < pConfigs->dataSize; i++) {

				std::cout << "mainVehicleId:" << MainVehicleID << ", pConfigs->data[i].sensorId:" << pConfigs->data[i].sensorId << ", pConfigs->data[i].sensorType:" << pConfigs->data[i].sensorType << endl;//The Lane's leftLane ID 
			}
		}
		else {
			std::cout << "there is no sensor config in current vehicle!!!" << endl;//The Lane's leftLane ID 
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(30));
	}
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

