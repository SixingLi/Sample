#include "SimOneSensorAPI.h"
#include "SimOneV2XAPI.h"
#include "SimOnePNCAPI.h"
#include "SimOneServiceAPI.h"
#include <thread> 
#include <chrono>
#include <iostream>
using namespace std;
int main(int argc, char* argv[])
{
	bool isJoinTimeLoop = false;
	const char* MainVehicleId = "0";
	std::unique_ptr<SimOne_Data_Obstacle> pObstacle = std::make_unique<SimOne_Data_Obstacle>();
	SimOneAPI::InitSimOneAPI(MainVehicleId, isJoinTimeLoop);
	std::unique_ptr<SimOne_Data_V2XNFS> pDetections = std::make_unique<SimOne_Data_V2XNFS>();
	std::unique_ptr<SimOne_Data_Gps> pGps = std::make_unique<SimOne_Data_Gps>();

	const char* mainVehicleId = "0";
	//bool isJoinTimeLoop = false;

	while (1)
	{
		std::unique_ptr<SimOne_Data_RadarDetection> pRadarDetection = std::make_unique<SimOne_Data_RadarDetection>();
		std::unique_ptr<SimOne_Data_UltrasonicRadars> pUltrasonics = std::make_unique<SimOne_Data_UltrasonicRadars>();
		std::map<int, SimOne_Data_RadarDetection> RadarDetectionMap;
		std::unique_ptr<SimOne_Data_SensorConfigurations> SensorConfigurations = std::make_unique<SimOne_Data_SensorConfigurations>();
		//bool resultGetRadarDetections = SimOneAPI::GetRadarDetections(0, "1", pRadarDetection.get());
		//EXPECT_TRUE(resultGetRadarDetections) << "GetRadarDetections Error" << std::endl;
		//char *SensorID;
		//bool resultSensorConfiguration = SimOneAPI::GetSensorConfigurations(SensorConfigurations.get());
		//for (int n = 0; n < SensorConfigurations->dataSize; n++) {
		//        if (SensorConfigurations->data[n].sensorType == "objectBasedRadar") {
		//                SensorID = SensorConfigurations->data[n].sensorId;
		//        }
		//
		//}
		//std::cout << "SensorID---------------------" << SensorID;
		//std::cout<<"sensorid:"
		//char ChsensorID[1];
		//strcpy(ChsensorID, (char *)&SensorID);
		while (1) {
			//auto function = [](int mainVehicleId, SimOne_Data_UltrasonicRadars *pUltrasonics) {
			//	std::cout << pUltrasonics->frame << "," << pUltrasonics->ultrasonicRadarNum << "," << pUltrasonics->timestamp << "," << pUltrasonics->ultrasonicRadars[0].obstacleDetections[0].obstacleRanges << "," << pUltrasonics->ultrasonicRadars[0].obstacleDetections[0].x << "," << pUltrasonics->ultrasonicRadars[0].obstacleDetections[0].y << endl;
			//};
			//SimOneAPI::SetUltrasonicRadarsCB(function);
			SimOneAPI::GetUltrasonicRadars(0, pUltrasonics.get());
			std::cout << pUltrasonics->frame << "," << pUltrasonics->ultrasonicRadarNum  << "," << pUltrasonics->timestamp << "," << pUltrasonics->ultrasonicRadars[0].obstacleDetections[0].obstacleRanges << "," << pUltrasonics->ultrasonicRadars[0].obstacleDetections[0].x << "," << pUltrasonics->ultrasonicRadars[0].obstacleDetections[0].y << endl;

		/*	bool resultGetRadarDetections = SimOneAPI::GetRadarDetections(0, "objectBasedRadar1", pRadarDetection.get());
			std::cout << "pRadarDetection->timestamp: " << pRadarDetection->timestamp << ",pRadarDetection.detectNum:" << pRadarDetection->detectNum << std::endl;
			std::cout << "GetRadarDetections posX: " << pRadarDetection->detections[0].posX << std::endl;
			std::cout << "GetRadarDetections posY: " << pRadarDetection->detections[0].posY << std::endl;
			std::cout << "GetRadarDetections posZ: " << pRadarDetection->detections[0].posZ << std::endl;*/
			//if (pRadarDetection->frame <= 5) {
			//	RadarDetectionMap.insert(map<int, SimOne_Data_RadarDetection>::value_type(pRadarDetection->frame, *pRadarDetection));
			//}
			//if (pRadarDetection->frame > 5) {
			//	std::this_thread::sleep_for(std::chrono::milliseconds(1000));
			//	RadarDetectionMap.insert(map<int, SimOne_Data_RadarDetection>::value_type(pRadarDetection->frame, *pRadarDetection));
			//}
			//if (SimOneAPI::GetCaseRunStatus() == SimOne_Case_Status::SimOne_Case_Status_Stop) {
			//	break;
			//}
		}
		std::cout << RadarDetectionMap.size() << std::endl;
		map<int, SimOne_Data_RadarDetection>::iterator iter;
		for (iter = RadarDetectionMap.begin(); iter != RadarDetectionMap.end(); iter++) {
			std::cout << "frame: " << iter->first << std::endl;
			std::cout << "detectNum: " << iter->second.detectNum << std::endl;
			//EXPECT_GT(iter->second.detectNum, 0);
			for (int i = 0; i < iter->second.detectNum; i++) {
				std::cout << "GetRadarDetections id: " << iter->second.detections[i].id << std::endl;
				//std::cout << "GetRadarDetections subId: " << iter->second.detections[i].subId << std::endl;
				//std::cout << "GetRadarDetections type: " << iter->second.detections[i].type << std::endl;
				//std::cout << "GetRadarDetections posX: " << iter->second.detections[i].posX << std::endl;
				//std::cout << "GetRadarDetections posY: " << iter->second.detections[i].posY << std::endl;
				//std::cout << "GetRadarDetections posZ: " << iter->second.detections[i].posZ << std::endl;
				//std::cout << "GetRadarDetections velX: " << iter->second.detections[i].velX << std::endl;
				//std::cout << "GetRadarDetections velY: " << iter->second.detections[i].velY << std::endl;
				//std::cout << "GetRadarDetections velZ: " << iter->second.detections[i].velZ << std::endl;
				//std::cout << "GetRadarDetections range: " << iter->second.detections[i].range << std::endl;
				//std::cout << "GetRadarDetections rangeRate: " << iter->second.detections[i].rangeRate << std::endl;
				//std::cout << "GetRadarDetections azimuth: " << iter->second.detections[i].azimuth << std::endl;
				//std::cout << "GetRadarDetections vertical: " << iter->second.detections[i].vertical << std::endl;
				//std::cout << "GetRadarDetections snrdb: " << iter->second.detections[i].snrdb << std::endl;
				//std::cout << "GetRadarDetections rcsdb: " << iter->second.detections[i].rcsdb << std::endl;
				//std::cout << "GetRadarDetections probability: " << iter->second.detections[i].probability << std::endl;
			}
		}


		//SimOne_Data_Signal_Lights pSignalLights;
		//pSignalLights.signalLights = ESimOne_Signal_Light_RightBlinker;
		//SimOneAPI::SetSignalLights(0, &pSignalLights);
		////1 GetGps
		//if (SimOneAPI::GetGps(mainVehicleId, pGps.get())) {
		//	cout << "MainVehicle Pos.X: " << pGps.get()->posX << ", MainVehicle Pos.Y: " << pGps.get()->posY << endl;
		//}
		//else {
		//	cout << "GetGps fail " << endl;
		//}


		//////1 GetGroundTruth
		////SimOneAPI::GetGroundTruth(0, pObstacle.get());
		////if (pObstacle.get() != NULL) {
		////	std::cout << "obstacleSize: " << pObstacle->obstacleSize << " height:" << pObstacle->obstacle[0].height << std::endl;
		////}

		//auto function = [](int mainVehicleId, const char* sensorId, SimOne_Data_V2XNFS *pDetections) {
		//	std::cout << pDetections->MsgFrameData << std::endl;
		//};
		//SimOneAPI::SetV2XInfoUpdateCB(function);



		//if (SimOneAPI::GetV2XInfo(0, "v2x", MessageFrame_PR_bsmFrame, pDetections.get())) {
		//	std::cout << "strlen = " << strlen(pDetections->MsgFrameData) << "  " << pDetections->MsgFrameData << std::endl;
		//	std::this_thread::sleep_for(std::chrono::milliseconds(50));
		//}
	}
	return 0;
}
