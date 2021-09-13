//#include <string>
//#include <chrono>
//#include <fstream>
//#include "utilTest.h"
//
//#include "decode.h"
//#include <sstream>

#include <vector>
#include <thread>
#include <string>
#include <fstream>
#include "utilTest.h"

const char* ip = "127.0.0.1";
unsigned short portLidar = 6699;
unsigned short infoPort = 7788;
unsigned short port = 7890;
SimOne_Data_Point_Cloud Point_Cloud;
SimOne_Data_Image Image;
using namespace std;
using SSD::SimString;

//for SimOneSensorAPI.h test
TEST_F(GlobalTestNoSync, SetGpsUpdateCB) {
	static bool flagSetGpsUpdateCB = true;
	SimOneAPI::SetGpsUpdateCB([](int mainVehicleId, SimOne_Data_Gps *pGps) {
		std::cout << "SetGpsUpdateCB timestamp:" << pGps->timestamp << std::endl;
		std::cout << "SetGpsUpdateCB frame:" << pGps->frame << std::endl;
		std::cout << "SetGpsUpdateCB posX:" << pGps->posX << std::endl;
		std::cout << "SetGpsUpdateCB posY:" << pGps->posY << std::endl;
		std::cout << "SetGpsUpdateCB oriX:" << pGps->oriX << std::endl;
		std::cout << "SetGpsUpdateCB oriY:" << pGps->oriY << std::endl;
		std::cout << "SetGpsUpdateCB oriZ:" << pGps->oriZ << std::endl;
		std::cout << "SetGpsUpdateCB velX:" << pGps->velX << std::endl;
		std::cout << "SetGpsUpdateCB velY:" << pGps->velY << std::endl;
		std::cout << "SetGpsUpdateCB velZ:" << pGps->posY << std::endl;
		std::cout << "SetGpsUpdateCB oriY:" << pGps->oriY << std::endl;
		std::cout << "SetGpsUpdateCB oriZ:" << pGps->oriZ << std::endl;
		std::cout << "SetGpsUpdateCB throttle:" << pGps->throttle << std::endl;
		std::cout << "SetGpsUpdateCB brake:" << pGps->steering << std::endl;
		std::cout << "SetGpsUpdateCB gear:" << pGps->gear << std::endl;
		std::cout << "SetGpsUpdateCB engineRpm:" << pGps->engineRpm << std::endl;
		std::cout << "SetGpsUpdateCB odometer:" << pGps->odometer << std::endl;
		flagSetGpsUpdateCB = false;
		EXPECT_GT(0, pGps->posX);
		/*EXPECT_FLOAT_EQ(0.1, pGps->throttle);
		EXPECT_EQ(4, pGps->gear);*/

	});
	while (flagSetGpsUpdateCB) {}
	if (flagSetGpsUpdateCB) {
		GTEST_FATAL_FAILURE_("not call back");
	}
}

TEST_F(GlobalTestNoSync, GetGpsResult) {
	std::unique_ptr<SimOne_Data_Gps> pGps = std::make_unique<SimOne_Data_Gps>();
	std::map<int, SimOne_Data_Gps>GpsMap;
	// assert APIData
	//EXPECT_TRUE(resultGetGps) << "GetGps Error" << std::endl;
	while (1) {
		bool resultGetGps = SimOneAPI::GetGps(0, pGps.get());
		if (pGps->frame <= 5) {
			//printf("==1==HotAreaGPSData frame:%d posX:%f posY:%f\n", pGps->frame, pGps->posX, pGps->posY);
			GpsMap.insert(map<int, SimOne_Data_Gps>::value_type(pGps->frame, *pGps));
		}
		if (pGps->frame > 5) {
			getchar();
			std::this_thread::sleep_for(std::chrono::milliseconds(1000));
			GpsMap.insert(map<int, SimOne_Data_Gps>::value_type(pGps->frame, *pGps));
		}
		if (SimOneAPI::GetCaseRunStatus() == SimOne_Case_Status::SimOne_Case_Status_Stop) {
			break;
		}
	}
	std::cout << GpsMap.size() << std::endl;
	map<int, SimOne_Data_Gps>::iterator iter;
	for (iter = GpsMap.begin(); iter != GpsMap.end(); iter++) {
		std::cout << "----------------------------------------  frame: ----------------------------------------->" << iter->first << std::endl;
		std::cout << "##################  MainVehicleExtraDataIndics ###################" << std::endl;
		std::cout << "extraStateSize: " << iter->second.extraStateSize << std::endl;
		for (int i = 0; i < iter->second.extraStateSize; i++) {
			std::cout << "extraStates: " << iter->second.extraStates[i] << std::endl;
		}
		std::cout << "################## Position ###################" << std::endl;
		std::cout << "posX: " << iter->second.posX << std::endl;
		std::cout << "posY: " << iter->second.posY << std::endl;
		std::cout << "posZ: " << iter->second.posZ << std::endl;
		std::cout << "################## Rotation ###################" << std::endl;
		std::cout << "oriX: " << iter->second.oriX << std::endl;
		std::cout << "oriY: " << iter->second.oriY << std::endl;
		std::cout << "oriZ: " << iter->second.oriZ << std::endl;
		std::cout << "################## MainVehicle Velocity ###################" << std::endl;
		std::cout << "velX: " << iter->second.velX << std::endl;
		std::cout << "velY: " << iter->second.velY << std::endl;
		std::cout << "velZ: " << iter->second.velZ << std::endl;
		std::cout << "################## MainVehicle drive state ###################" << std::endl;
		std::cout << "throttle: " << iter->second.posY << std::endl;
		std::cout << "brake: " << iter->second.brake << std::endl;
		std::cout << "steering: " << iter->second.steering << std::endl;
		std::cout << "gear: " << iter->second.gear << std::endl;
		std::cout << "engineRpm: " << iter->second.engineRpm << std::endl;
		std::cout << "odometer: " << iter->second.odometer << std::endl;
		std::cout << "################## Acceleration ###################" << std::endl;
		std::cout << "accelX: " << iter->second.accelX << std::endl;
		std::cout << "accelY: " << iter->second.accelY << std::endl;
		std::cout << "accelZ: " << iter->second.accelZ << std::endl;
		std::cout << "################## Angular Velocity ###################" << std::endl;
		std::cout << "angVelX: " << iter->second.angVelX << std::endl;
		std::cout << "angVelY: " << iter->second.angVelY << std::endl;
		std::cout << "angVelZ: " << iter->second.angVelZ << std::endl;
		std::cout << "################## speed of wheels ###################" << std::endl;
		std::cout << "wheelSpeedFL: " << iter->second.wheelSpeedFL << std::endl;
		std::cout << "wheelSpeedFR: " << iter->second.wheelSpeedFR << std::endl;
		std::cout << "wheelSpeedRL: " << iter->second.wheelSpeedRL << std::endl;
		std::cout << "wheelSpeedRR: " << iter->second.wheelSpeedRR << std::endl;
	}
}

TEST_F(GlobalTestNoSync, SetGroundTruthUpdateCB) {
	static bool flagSetGroundTruthUpdateCB = true;
	SimOneAPI::SetGroundTruthUpdateCB([](int mainVehicleId, SimOne_Data_Obstacle *pObstacle) {
		std::cout << "SetGroundTruthUpdateCB timestampe: " << pObstacle->timestamp << std::endl;
		std::cout << "SetGroundTruthUpdateCB obstacleSize: " << pObstacle->obstacleSize << std::endl;
		for (int i = 0; i < pObstacle->obstacleSize; ++i) {
			std::cout << "________________________________________" << std::endl;
			std::cout << "SetGroundTruthUpdateCB theta: " << pObstacle->obstacle[i].theta << std::endl;
			std::cout << "SetGroundTruthUpdateCB posX: " << pObstacle->obstacle[i].posX << std::endl;
			std::cout << "SetGroundTruthUpdateCB type: " << pObstacle->obstacle[i].type << std::endl;
			std::cout << "SetGroundTruthUpdateCB posY: " << pObstacle->obstacle[i].posY << std::endl;
			std::cout << "SetGroundTruthUpdateCB posZ: " << pObstacle->obstacle[i].posZ << std::endl;
			std::cout << "SetGroundTruthUpdateCB velX: " << pObstacle->obstacle[i].velX << std::endl;
			std::cout << "SetGroundTruthUpdateCB velY: " << pObstacle->obstacle[i].velY << std::endl;
			std::cout << "SetGroundTruthUpdateCB velZ: " << pObstacle->obstacle[i].velZ << std::endl;
			std::cout << "SetGroundTruthUpdateCB length: " << pObstacle->obstacle[i].length << std::endl;
			std::cout << "SetGroundTruthUpdateCB width: " << pObstacle->obstacle[i].width << std::endl;
			std::cout << "SetGroundTruthUpdateCB height: " << pObstacle->obstacle[i].height << std::endl;
		}
		flagSetGroundTruthUpdateCB = false;
		EXPECT_GT(pObstacle->obstacleSize, 8);
	});
	while (flagSetGroundTruthUpdateCB) {}
	if (flagSetGroundTruthUpdateCB) {
		GTEST_FATAL_FAILURE_("not call back");
	}
}

TEST_F(GlobalTestNoSync, GetGroundTruthResult) {
	std::unique_ptr<SimOne_Data_Obstacle> Obstacle = std::make_unique<SimOne_Data_Obstacle>();
	std::map<int, SimOne_Data_Obstacle>ObstacleMap;
	//bool resultGetGroundTruth = SimOneAPI::GetGroundTruth(0, Obstacle.get());
	//EXPECT_TRUE(resultGetGroundTruth) << "GetGroundTruth Error" << std::endl;
	while (1) {
		bool resultGetGroundTruth = SimOneAPI::GetGroundTruth(0, Obstacle.get());
		if (Obstacle->frame <= 5) {
			ObstacleMap.insert(map<int, SimOne_Data_Obstacle>::value_type(Obstacle->frame, *Obstacle));
		}
		if (Obstacle->frame > 5) {
			std::this_thread::sleep_for(std::chrono::milliseconds(1000));
			ObstacleMap.insert(map<int, SimOne_Data_Obstacle>::value_type(Obstacle->frame, *Obstacle));
		}
		if (SimOneAPI::GetCaseRunStatus() == SimOne_Case_Status::SimOne_Case_Status_Stop) {
			break;
		}
	}
	std::cout << ObstacleMap.size() << std::endl;
	map<int, SimOne_Data_Obstacle>::iterator iter;
	for (iter = ObstacleMap.begin(); iter != ObstacleMap.end(); iter++) {
		std::cout << "-----------------------------------------frame:--------------------------------------> " << iter->first << std::endl;
		std::cout << "obstacleSize: " << iter->second.obstacleSize << std::endl;
		for (int i = 0; i < iter->second.obstacleSize; i++) {
			std::cout << "id: " << iter->second.obstacle[i].id << std::endl;
			std::cout << "viewId: " << iter->second.obstacle[i].viewId << std::endl;
			std::cout << "type: " << iter->second.obstacle[i].type << std::endl;
			std::cout << "theta: " << iter->second.obstacle[i].theta << std::endl;
			std::cout << "posX: " << iter->second.obstacle[i].posX << std::endl;
			std::cout << "posY: " << iter->second.obstacle[i].posY << std::endl;
			std::cout << "posZ: " << iter->second.obstacle[i].posZ << std::endl;
			std::cout << "oriX: " << iter->second.obstacle[i].oriX << std::endl;
			std::cout << "oriY: " << iter->second.obstacle[i].oriY << std::endl;
			std::cout << "oriZ: " << iter->second.obstacle[i].oriZ << std::endl;
			std::cout << "velX: " << iter->second.obstacle[i].velX << std::endl;
			std::cout << "velY: " << iter->second.obstacle[i].velY << std::endl;
			std::cout << "velZ: " << iter->second.obstacle[i].velZ << std::endl;
			std::cout << "length: " << iter->second.obstacle[i].length << std::endl;
			std::cout << "width: " << iter->second.obstacle[i].width << std::endl;
			std::cout << "height: " << iter->second.obstacle[i].height << std::endl;
		}
	}
}

TEST_F(GlobalTestNoSync, SetSensorDetectionsUpdateCB) {
	static bool numberSetImageUpdateCB = true;
	static bool flagSetSensorDetectionsUpdateCB = true;
	SimOneAPI::SetSensorDetectionsUpdateCB([](int mainVehicleId, const char* sensorId, SimOne_Data_SensorDetections *pGroundtruth) {
		std::cout << "SetSensorDetectionsUpdateCB timestampe: " << pGroundtruth->timestamp << std::endl;
		std::cout << "SetSensorDetectionsUpdateCB obstacleSize: " << pGroundtruth->objectSize << std::endl;
		for (int i = 0; i < pGroundtruth->objectSize; ++i) {
			std::cout << "________________________________________" << std::endl;
			std::cout << "SetSensorDetectionsUpdateCB bbox2dMinY: " << pGroundtruth->objects[i].bbox2dMinY << std::endl;
			std::cout << "SetSensorDetectionsUpdateCB bbox2dMinX: " << pGroundtruth->objects[i].bbox2dMinX << std::endl;
			std::cout << "SetSensorDetectionsUpdateCB  relativePosX: " << pGroundtruth->objects[i].relativePosX << std::endl;
			std::cout << "SetSensorDetectionsUpdateCB type: " << pGroundtruth->objects[i].type << std::endl;
			std::cout << "SetSensorDetectionsUpdateCB posX: " << pGroundtruth->objects[i].posX << std::endl;
			std::cout << "SetSensorDetectionsUpdateCB posY: " << pGroundtruth->objects[i].posY << std::endl;
			std::cout << "SetSensorDetectionsUpdateCB posZ: " << pGroundtruth->objects[i].posZ << std::endl;
			std::cout << "SetSensorDetectionsUpdateCB velX: " << pGroundtruth->objects[i].velX << std::endl;
			std::cout << "SetSensorDetectionsUpdateCB velY: " << pGroundtruth->objects[i].velY << std::endl;
			std::cout << "SetSensorDetectionsUpdateCB velZ: " << pGroundtruth->objects[i].velZ << std::endl;
			std::cout << "SetSensorDetectionsUpdateCB length: " << pGroundtruth->objects[i].length << std::endl;
			std::cout << "SetSensorDetectionsUpdateCB width: " << pGroundtruth->objects[i].width << std::endl;
			std::cout << "SetSensorDetectionsUpdateCB height: " << pGroundtruth->objects[i].height << std::endl;
		}
		flagSetSensorDetectionsUpdateCB = false;
		EXPECT_GT(pGroundtruth->objectSize, 8);
	});
	while (flagSetSensorDetectionsUpdateCB && numberSetImageUpdateCB) {
		numberSetImageUpdateCB = false;
	}
	if (flagSetSensorDetectionsUpdateCB) {
		GTEST_FATAL_FAILURE_("not call back");
	}
}

TEST_F(GlobalTestNoSync, GetSensorDetectionsResult) {
	std::unique_ptr<SimOne_Data_SensorDetections> SensorDetections = std::make_unique<SimOne_Data_SensorDetections>();
	std::map<int, SimOne_Data_SensorDetections> SensorDetectionsMap;
	while (1) {
		bool resultGetRadarDetections = SimOneAPI::GetSensorDetections(0, "objectBasedRadar1", SensorDetections.get());
		if (SensorDetections->frame <= 5) {
			SensorDetectionsMap.insert(map<int, SimOne_Data_SensorDetections>::value_type(SensorDetections->frame, *SensorDetections));
		}
		if (SensorDetections->frame > 5) {
			std::this_thread::sleep_for(std::chrono::milliseconds(1000));
			SensorDetectionsMap.insert(map<int, SimOne_Data_SensorDetections>::value_type(SensorDetections->frame, *SensorDetections));
		}
		if (SimOneAPI::GetCaseRunStatus() == SimOne_Case_Status::SimOne_Case_Status_Stop) {
			break;
		}
	}
	std::cout << SensorDetectionsMap.size() << std::endl;
	map<int, SimOne_Data_SensorDetections>::iterator iter;
	for (iter = SensorDetectionsMap.begin(); iter != SensorDetectionsMap.end(); iter++) {
		std::cout << "frame: " << iter->first << std::endl;
		std::cout << "objectSize: " << iter->second.objectSize << std::endl;
		//EXPECT_GT(iter->second.detectNum, 0);
		for (int i = 0; i < iter->second.objectSize; i++) {
			std::cout << "GetSensorDetections id: " << iter->second.objects[i].id << std::endl;
			std::cout << "GetSensorDetections type: " << iter->second.objects[i].type << std::endl;
			std::cout << "GetSensorDetections posX: " << iter->second.objects[i].posX << std::endl;
			std::cout << "GetSensorDetections posY: " << iter->second.objects[i].posY << std::endl;
			std::cout << "GetSensorDetections posZ: " << iter->second.objects[i].posZ << std::endl;
			std::cout << "GetSensorDetections velX: " << iter->second.objects[i].velX << std::endl;
			std::cout << "GetSensorDetections velY: " << iter->second.objects[i].velY << std::endl;
			std::cout << "GetSensorDetections velZ: " << iter->second.objects[i].velZ << std::endl;
			std::cout << "GetSensorDetections range: " << iter->second.objects[i].range << std::endl;
			std::cout << "GetSensorDetections length: " << iter->second.objects[i].length << std::endl;
			std::cout << "GetSensorDetections width: " << iter->second.objects[i].width << std::endl;
			std::cout << "GetSensorDetections height: " << iter->second.objects[i].height << std::endl;
			std::cout << "GetSensorDetections probability: " << iter->second.objects[i].probability << std::endl;
		}
	}
}

TEST_F(GlobalTestNoSync, GetRadarDetectionsResult) {
	std::unique_ptr<SimOne_Data_RadarDetection> pRadarDetection = std::make_unique<SimOne_Data_RadarDetection>();
	std::map<int, SimOne_Data_RadarDetection> RadarDetectionMap;
	std::unique_ptr<SimOne_Data_SensorConfigurations> SensorConfigurations = std::make_unique<SimOne_Data_SensorConfigurations>();
	//bool resultGetRadarDetections = SimOneAPI::GetRadarDetections(0, "1", pRadarDetection.get());
	//EXPECT_TRUE(resultGetRadarDetections) << "GetRadarDetections Error" << std::endl;
	//char *SensorID;
	//bool resultSensorConfiguration = SimOneAPI::GetSensorConfigurations(SensorConfigurations.get());
	//for (int n = 0; n < SensorConfigurations->dataSize; n++) {
	//	if (SensorConfigurations->data[n].sensorType == "objectBasedRadar") {
	//		SensorID = SensorConfigurations->data[n].sensorId;
	//	}
	//
	//}
	//std::cout << "SensorID---------------------" << SensorID;
	//std::cout<<"sensorid:"
	//char ChsensorID[1];
	//strcpy(ChsensorID, (char *)&SensorID);
	while (1) {
		bool resultGetRadarDetections = SimOneAPI::GetRadarDetections(0, "objectBasedRadar1", pRadarDetection.get());
		if (pRadarDetection->frame <= 5) {
			RadarDetectionMap.insert(map<int, SimOne_Data_RadarDetection>::value_type(pRadarDetection->frame, *pRadarDetection));
		}
		if (pRadarDetection->frame > 5) {
			std::this_thread::sleep_for(std::chrono::milliseconds(1000));
			RadarDetectionMap.insert(map<int, SimOne_Data_RadarDetection>::value_type(pRadarDetection->frame, *pRadarDetection));
		}
		if (SimOneAPI::GetCaseRunStatus() == SimOne_Case_Status::SimOne_Case_Status_Stop) {
			break;
		}
	}
	std::cout << RadarDetectionMap.size() << std::endl;
	map<int, SimOne_Data_RadarDetection>::iterator iter;
	for (iter = RadarDetectionMap.begin(); iter != RadarDetectionMap.end(); iter++) {
		std::cout << "frame: " << iter->first << std::endl;
		std::cout << "detectNum: " << iter->second.detectNum << std::endl;
		//EXPECT_GT(iter->second.detectNum, 0);
		for (int i = 0; i < iter->second.detectNum; i++) {
			std::cout << "GetRadarDetections id: " << iter->second.detections[i].id << std::endl;
			std::cout << "GetRadarDetections subId: " << iter->second.detections[i].subId << std::endl;
			std::cout << "GetRadarDetections type: " << iter->second.detections[i].type << std::endl;
			std::cout << "GetRadarDetections posX: " << iter->second.detections[i].posX << std::endl;
			std::cout << "GetRadarDetections posY: " << iter->second.detections[i].posY << std::endl;
			std::cout << "GetRadarDetections posZ: " << iter->second.detections[i].posZ << std::endl;
			std::cout << "GetRadarDetections velX: " << iter->second.detections[i].velX << std::endl;
			std::cout << "GetRadarDetections velY: " << iter->second.detections[i].velY << std::endl;
			std::cout << "GetRadarDetections velZ: " << iter->second.detections[i].velZ << std::endl;
			std::cout << "GetRadarDetections range: " << iter->second.detections[i].range << std::endl;
			std::cout << "GetRadarDetections rangeRate: " << iter->second.detections[i].rangeRate << std::endl;
			std::cout << "GetRadarDetections azimuth: " << iter->second.detections[i].azimuth << std::endl;
			std::cout << "GetRadarDetections vertical: " << iter->second.detections[i].vertical << std::endl;
			std::cout << "GetRadarDetections snrdb: " << iter->second.detections[i].snrdb << std::endl;
			std::cout << "GetRadarDetections rcsdb: " << iter->second.detections[i].rcsdb << std::endl;
			std::cout << "GetRadarDetections probability: " << iter->second.detections[i].probability << std::endl;
		}
	}
}
TEST_F(GlobalTestNoSync, SetUltrasonicRadarsCB) {
	static bool numberSetUltrasonicRadarsCB = true;
	static bool flagSetUltrasonicRadarsCB = true;
	SimOneAPI::SetUltrasonicRadarsCB([](int mainVehicleId, SimOne_Data_UltrasonicRadars *pUltrasonics) {
		std::cout << "SetUltrasonicRadarsCB frame: " << pUltrasonics->frame << std::endl;
		std::cout << "SetUltrasonicRadarsCB timestamp: " << pUltrasonics->timestamp << std::endl;
		std::cout << "SetUltrasonicRadarsCB UltrasonicRadarsNum: " << pUltrasonics->ultrasonicRadarNum << std::endl;
		for (int i = 0; i < pUltrasonics->ultrasonicRadarNum; i++) {
			std::cout << "ultrasonicRadars id: " << pUltrasonics->ultrasonicRadars[i].sensorId << std::endl;
			std::cout << "ultrasonicRadars obstacleNum: " << pUltrasonics->ultrasonicRadars[i].obstacleNum << std::endl;
			for (int j = 0; j < pUltrasonics->ultrasonicRadars[i].obstacleNum; j++) {
				std::cout << "ultrasonicRadars obstacleRanges: " << pUltrasonics->ultrasonicRadars[i].obstacleDetections[j].obstacleRanges << std::endl;
				std::cout << "ultrasonicRadars x: " << pUltrasonics->ultrasonicRadars[i].obstacleDetections[j].x << std::endl;
				std::cout << "ultrasonicRadars y: " << pUltrasonics->ultrasonicRadars[i].obstacleDetections[j].y << std::endl;
			}

		}
		flagSetUltrasonicRadarsCB = false;
	});

	while (numberSetUltrasonicRadarsCB && flagSetUltrasonicRadarsCB) {
		numberSetUltrasonicRadarsCB = false;
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}
	if (flagSetUltrasonicRadarsCB) {
		GTEST_FATAL_FAILURE_("not call back");
	}
}

TEST_F(GlobalTestNoSync, GetUltrasonicRadarResult) {
	std::unique_ptr<SimOne_Data_UltrasonicRadar> pUltrasonic = std::make_unique<SimOne_Data_UltrasonicRadar>();
	std::map<int, SimOne_Data_UltrasonicRadar> pUltrasonicMap;
	//bool resultGetRadarDetections = SimOneAPI::GetRadarDetections(0, "1", pRadarDetection.get());
	//EXPECT_TRUE(resultGetRadarDetections) << "GetRadarDetections Error" << std::endl;
	while (1) {
		bool pUltrasonicResult = SimOneAPI::GetUltrasonicRadar(0, "1", pUltrasonic.get());
		if (pUltrasonic->frame <= 5) {
			pUltrasonicMap.insert(map<int, SimOne_Data_UltrasonicRadar>::value_type(pUltrasonic->frame, *pUltrasonic));
		}
		if (pUltrasonic->frame > 5) {
			std::this_thread::sleep_for(std::chrono::milliseconds(1000));
			pUltrasonicMap.insert(map<int, SimOne_Data_UltrasonicRadar>::value_type(pUltrasonic->frame, *pUltrasonic));
		}
		if (SimOneAPI::GetCaseRunStatus() == SimOne_Case_Status::SimOne_Case_Status_Stop) {
			break;
		}
	}
	std::cout << pUltrasonicMap.size() << std::endl;
	map<int, SimOne_Data_UltrasonicRadar>::iterator iter;
	for (iter = pUltrasonicMap.begin(); iter != pUltrasonicMap.end(); iter++) {
		std::cout << "frame: " << iter->first << std::endl;
		std::cout << "obstacleNum: " << iter->second.obstacleNum << std::endl;
		std::cout << "sensorId id: " << iter->second.sensorId << std::endl;
		std::cout << "obstacleRanges: " << iter->second.obstacleDetections->obstacleRanges << std::endl;
		std::cout << "x: " << iter->second.obstacleDetections->x << std::endl;
		std::cout << "y : " << iter->second.obstacleDetections->y << std::endl;
	}

}

TEST_F(GlobalTestNoSync, GetUltrasonicRadarsResult) {
	std::unique_ptr<SimOne_Data_UltrasonicRadars> pUltrasonics = std::make_unique<SimOne_Data_UltrasonicRadars>();
	std::map<int, SimOne_Data_UltrasonicRadars> pUltrasonicsMap;
	//bool resultGetRadarDetections = SimOneAPI::GetRadarDetections(0, "1", pRadarDetection.get());
	//EXPECT_TRUE(resultGetRadarDetections) << "GetRadarDetections Error" << std::endl;
	while (1) {
		bool pUltrasonicsResult = SimOneAPI::GetUltrasonicRadars(0, pUltrasonics.get());
		if (pUltrasonics->frame <= 5) {
			pUltrasonicsMap.insert(map<int, SimOne_Data_UltrasonicRadars>::value_type(pUltrasonics->frame, *pUltrasonics));
		}
		if (pUltrasonics->frame > 5) {
			std::this_thread::sleep_for(std::chrono::milliseconds(1000));
			pUltrasonicsMap.insert(map<int, SimOne_Data_UltrasonicRadars>::value_type(pUltrasonics->frame, *pUltrasonics));
		}
		if (SimOneAPI::GetCaseRunStatus() == SimOne_Case_Status::SimOne_Case_Status_Stop) {
			break;
		}
	}
	std::cout << pUltrasonicsMap.size() << std::endl;
	map<int, SimOne_Data_UltrasonicRadars>::iterator iter;
	for (iter = pUltrasonicsMap.begin(); iter != pUltrasonicsMap.end(); iter++) {
		std::cout << "frame: " << iter->first << std::endl;
		std::cout << "ultrasonicRadarNum: " << iter->second.ultrasonicRadarNum << std::endl;
		for (int i = 0; i < iter->second.ultrasonicRadarNum; i++) {
			std::cout << "sensorId id: " << iter->second.ultrasonicRadars[i].sensorId << std::endl;
			std::cout << "obstacleNum: " << iter->second.ultrasonicRadars[i].obstacleNum << std::endl;
			std::cout << "obstacleRanges: " << iter->second.ultrasonicRadars[i].obstacleDetections->obstacleRanges << std::endl;
			std::cout << "x: " << iter->second.ultrasonicRadars[i].obstacleDetections->x << std::endl;
			std::cout << "y : " << iter->second.ultrasonicRadars[i].obstacleDetections->y << std::endl;
		}
	}

}


TEST_F(GlobalTestNoSync, SetRadarDetectionsUpdateCB) {
	static bool numberSetRadarDetectionsUpdateCB = true;
	static bool flagSetRadarDetectionsUpdateCB = true;
	SimOneAPI::SetRadarDetectionsUpdateCB([](int mainVehicleId, const char* sensorId, SimOne_Data_RadarDetection *pDetections) {
		std::cout << "SetRadarDetectionsUpdateCB detectNum: " << pDetections->detectNum << std::endl;
		std::cout << "SetRadarDetectionsUpdateCB timestamp: " << pDetections->timestamp << std::endl;
		std::cout << "SetRadarDetectionsUpdateCB frame: " << pDetections->frame << std::endl;
		std::cout << "_______________________________________________________" << std::endl;
		for (int i = 0; i < pDetections->detectNum; i++)
		{
			std::cout << "*****************************" << std::endl;
			std::cout << "SetRadarDetectionsUpdateCB id: " << pDetections->detections[i].id << std::endl;
			std::cout << "SetRadarDetectionsUpdateCB subId: " << pDetections->detections[i].subId << std::endl;
			std::cout << "SetRadarDetectionsUpdateCB type: " << pDetections->detections[i].type << std::endl;
			std::cout << "SetRadarDetectionsUpdateCB posX: " << pDetections->detections[i].posX << std::endl;
			std::cout << "SetRadarDetectionsUpdateCB posY: " << pDetections->detections[i].posY << std::endl;
			std::cout << "SetRadarDetectionsUpdateCB posZ: " << pDetections->detections[i].posZ << std::endl;
			std::cout << "SetRadarDetectionsUpdateCB velX: " << pDetections->detections[i].velX << std::endl;
			std::cout << "SetRadarDetectionsUpdateCB velY: " << pDetections->detections[i].velY << std::endl;
			std::cout << "SetRadarDetectionsUpdateCB velZ: " << pDetections->detections[i].velZ << std::endl;
			std::cout << "SetRadarDetectionsUpdateCB range: " << pDetections->detections[i].range << std::endl;
			std::cout << "SetRadarDetectionsUpdateCB rangeRate: " << pDetections->detections[i].rangeRate << std::endl;
			std::cout << "SetRadarDetectionsUpdateCB azimuth: " << pDetections->detections[i].azimuth << std::endl;
			std::cout << "SetRadarDetectionsUpdateCB vertical: " << pDetections->detections[i].vertical << std::endl;
			std::cout << "SetRadarDetectionsUpdateCB snrdb: " << pDetections->detections[i].snrdb << std::endl;
			std::cout << "SetRadarDetectionsUpdateCB rcsdb: " << pDetections->detections[i].rcsdb << std::endl;
			std::cout << "SetRadarDetectionsUpdateCB probability: " << pDetections->detections[i].probability << std::endl;
		}
		flagSetRadarDetectionsUpdateCB = false;
		EXPECT_GT(pDetections->detectNum, 0);
	});

	while (numberSetRadarDetectionsUpdateCB && flagSetRadarDetectionsUpdateCB) {
		numberSetRadarDetectionsUpdateCB = false;
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}
	if (flagSetRadarDetectionsUpdateCB) {
		GTEST_FATAL_FAILURE_("not call back");
	}
}

TEST_F(GlobalTestNoSync, GetSensorConfigurationsResult) {
	std::unique_ptr<SimOne_Data_SensorConfigurations> pSensorConfigurations = std::make_unique<SimOne_Data_SensorConfigurations>();
	std::map<int, SimOne_Data_SensorConfigurations> sensorConfigurationsMap;
	//bool sensorConfigurationsitor = SimOneAPI::GetSensorConfigurations(pSensorConfigurations.get());
	//EXPECT_TRUE(sensorConfigurationsitor) << "GetSensorConfigurations Error" << std::endl;
	while (1) {
		bool resultGetRadarDetections = SimOneAPI::GetSensorConfigurations(pSensorConfigurations.get());
		if (SimOneAPI::GetCaseRunStatus() == SimOne_Case_Status::SimOne_Case_Status_Running) {
			sensorConfigurationsMap.insert(map<int, SimOne_Data_SensorConfigurations>::value_type(pSensorConfigurations->dataSize, *pSensorConfigurations));
			std::this_thread::sleep_for(std::chrono::milliseconds(1000));
		}
		if (SimOneAPI::GetCaseRunStatus() == SimOne_Case_Status::SimOne_Case_Status_Stop) {
			break;
		}
	}
	std::cout << sensorConfigurationsMap.size() << std::endl;
	map<int, SimOne_Data_SensorConfigurations>::iterator iter;
	for (iter = sensorConfigurationsMap.begin(); iter != sensorConfigurationsMap.end(); iter++) {
		std::cout << "dataSize: " << iter->first << std::endl;
		for (int i = 0; i < iter->second.dataSize; i++) {
			std::cout << "-----------------------------------------GetSensorConfigurations------------------------------------ " << iter->second.data[i].id << std::endl;
			std::cout << "GetSensorConfigurations type: " << iter->second.data[i].sensorType << std::endl;
			std::cout << "GetSensorConfigurations sensorId: " << iter->second.data[i].sensorId << std::endl;
			std::cout << "GetSensorConfigurations id: " << iter->second.data[i].id << std::endl;
			std::cout << "GetSensorConfigurations mainVehicle: " << iter->second.data[i].mainVehicle << std::endl;
			std::cout << "GetSensorConfigurations x: " << iter->second.data[i].x << std::endl;
			std::cout << "GetSensorConfigurations y: " << iter->second.data[i].y << std::endl;
			std::cout << "GetSensorConfigurations z: " << iter->second.data[i].z << std::endl;
			std::cout << "GetSensorConfigurations roll: " << iter->second.data[i].roll << std::endl;
			std::cout << "GetSensorConfigurations pitch: " << iter->second.data[i].pitch << std::endl;
			std::cout << "GetSensorConfigurations yaw: " << iter->second.data[i].yaw << std::endl;
			std::cout << "GetSensorConfigurations hz: " << iter->second.data[i].hz << std::endl;
		}
	}
}

TEST_F(GlobalTestNoSync, SetEnvironment) {
	std::unique_ptr<SimOne_Data_Environment> Environment = std::make_unique<SimOne_Data_Environment>();
	SimOne_Data_Environment *pEnvironment;

	pEnvironment->timeOfDay = 1000;
	pEnvironment->heightAngle = 90;
	pEnvironment->directionalLight = 0.5f;
	pEnvironment->ambientLight = 0.5f;
	pEnvironment->artificialLight = 0.5f;
	pEnvironment->cloudDensity = 0.5f;
	pEnvironment->fogDensity = 0.5f;
	pEnvironment->rainDensity = 0.5f;
	pEnvironment->snowDensity = 0.5f;
	pEnvironment->groundHumidityLevel = 0.5f;
	pEnvironment->groundDirtyLevel = 0.5f;
	std::cout << "SetEnvironment timeOfDay ([0, 2400]): " << pEnvironment->timeOfDay << std::endl;
	std::cout << "SetEnvironment heightAngle ([0, 90]): " << pEnvironment->heightAngle << std::endl;
	std::cout << "SetEnvironment directionalLight ([0, 1]): " << pEnvironment->directionalLight << std::endl;
	std::cout << "SetEnvironment ambientLight ([0, 1]): " << pEnvironment->ambientLight << std::endl;
	std::cout << "SetEnvironment artificialLight ([0, 1]): " << pEnvironment->artificialLight << std::endl;
	std::cout << "SetEnvironment cloudDensity ([0, 1]): " << pEnvironment->cloudDensity << std::endl;
	std::cout << "SetEnvironment fogDensity ([0, 1]): " << pEnvironment->fogDensity << std::endl;
	std::cout << "SetEnvironment rainDensity ([0, 1]): " << pEnvironment->rainDensity << std::endl;
	std::cout << "SetEnvironment snowDensity ([0, 1]): " << pEnvironment->snowDensity << std::endl;
	std::cout << "SetEnvironment groundHumidityLevel ([0, 1]): " << pEnvironment->groundHumidityLevel << std::endl;
	std::cout << "SetEnvironment groundDirtyLevel ([0, 1]): " << pEnvironment->groundDirtyLevel << std::endl;
	bool setEnvResult = SimOneAPI::SetEnvironment(pEnvironment);
	if (setEnvResult) {
		bool EnvironmentResult = SimOneAPI::GetEnvironment(Environment.get());
		EXPECT_EQ(Environment->timeOfDay, pEnvironment->timeOfDay);
		EXPECT_EQ(Environment->heightAngle, pEnvironment->heightAngle);
		EXPECT_EQ(Environment->directionalLight, pEnvironment->directionalLight);
		EXPECT_EQ(Environment->ambientLight, pEnvironment->ambientLight);
		EXPECT_EQ(Environment->artificialLight, pEnvironment->artificialLight);
		EXPECT_EQ(Environment->cloudDensity, pEnvironment->cloudDensity);
		EXPECT_EQ(Environment->fogDensity, pEnvironment->fogDensity);
		EXPECT_EQ(Environment->rainDensity, pEnvironment->rainDensity);
		EXPECT_EQ(Environment->snowDensity, pEnvironment->snowDensity);
		EXPECT_EQ(Environment->groundHumidityLevel, pEnvironment->groundHumidityLevel);
		EXPECT_EQ(Environment->groundDirtyLevel, pEnvironment->groundDirtyLevel);
	}

}

TEST_F(GlobalTestNoSync, GetEnvironmentResult) {
	std::unique_ptr<SimOne_Data_Environment> pEnvironment = std::make_unique<SimOne_Data_Environment>();
	std::map<int, SimOne_Data_Environment> EnvironmentMap;
	//bool EnvironmentResult = SimOneAPI::GetEnvironment(pEnvironment.get());
	//EXPECT_TRUE(EnvironmentResult) << "GetEnvironmentResult Error" << std::endl;
	int index = 0;
	while (1) {
		bool EnvironmentResult = SimOneAPI::GetEnvironment(pEnvironment.get());
		if (SimOneAPI::GetCaseRunStatus() == SimOne_Case_Status::SimOne_Case_Status_Running) {
			EnvironmentMap.insert(make_pair(index, *pEnvironment));
			std::this_thread::sleep_for(std::chrono::milliseconds(1000));
			index++;
		}
		if (SimOneAPI::GetCaseRunStatus() == SimOne_Case_Status::SimOne_Case_Status_Stop) {
			break;
		}
	}
	std::cout << EnvironmentMap.size() << std::endl;
	map<int, SimOne_Data_Environment>::iterator iter;
	for (iter = EnvironmentMap.begin(); iter != EnvironmentMap.end(); iter++) {
		std::cout << "----------------------------------------环境参数------------------------------------------" << iter->second.timeOfDay << std::endl;
		std::cout << "GetEnvironment timeOfDay ([0, 2400]): " << iter->second.timeOfDay << std::endl;
		std::cout << "GetEnvironment heightAngle ([0, 90]): " << iter->second.heightAngle << std::endl;
		std::cout << "GetEnvironment directionalLight ([0, 1]): " << iter->second.directionalLight << std::endl;
		std::cout << "GetEnvironment ambientLight ([0, 1]): " << iter->second.ambientLight << std::endl;
		std::cout << "GetEnvironment artificialLight ([0, 1]): " << iter->second.artificialLight << std::endl;
		std::cout << "GetEnvironment cloudDensity ([0, 1]): " << iter->second.cloudDensity << std::endl;
		std::cout << "GetEnvironment fogDensity ([0, 1]): " << iter->second.fogDensity << std::endl;
		std::cout << "GetEnvironment rainDensity ([0, 1]): " << iter->second.rainDensity << std::endl;
		std::cout << "GetEnvironment snowDensity ([0, 1]): " << iter->second.snowDensity << std::endl;
		std::cout << "GetEnvironment groundHumidityLevel ([0, 1]): " << iter->second.groundHumidityLevel << std::endl;
		std::cout << "GetEnvironment groundDirtyLevel ([0, 1]): " << iter->second.groundDirtyLevel << std::endl;
	}
}

TEST_F(GlobalTestNoSync, GetTrafficLightResult) {
	std::unique_ptr<SimOne_Data_TrafficLight> pTrafficLight = std::make_unique<SimOne_Data_TrafficLight>();
	std::map<int, SimOne_Data_TrafficLight>TrafficLightMap;
	SSD::SimVector<HDMapStandalone::MSignal>TrafficLightList;
	SimOneAPI::GetTrafficLightList(TrafficLightList);

	//bool EnvironmentResult = SimOneAPI::GetTrafficLight(0, 1, pTrafficLight.get());
	//EXPECT_TRUE(EnvironmentResult) << "GetTrafficLightResult Error" << std::endl;
	while (1) {

		for (int k = 0; k < TrafficLightList.size(); k++)
		{
			long lightID = TrafficLightList[k].id;
			std::cout << "----------------------------lightID:--------------------" << lightID;
			bool TrafficLightResult = SimOneAPI::GetTrafficLight(0, lightID, pTrafficLight.get());
			if (SimOneAPI::GetCaseRunStatus() == SimOne_Case_Status::SimOne_Case_Status_Running) {
				TrafficLightMap.insert(map<int, SimOne_Data_TrafficLight>::value_type(k, *pTrafficLight));
				std::this_thread::sleep_for(std::chrono::milliseconds(1000));
			}
			if (SimOneAPI::GetCaseRunStatus() == SimOne_Case_Status::SimOne_Case_Status_Stop) {
				break;
			}
		}
	}
	std::cout << TrafficLightMap.size() << std::endl;
	map<int, SimOne_Data_TrafficLight>::iterator iter;
	for (iter = TrafficLightMap.begin(); iter != TrafficLightMap.end(); iter++) {
		std::cout << "GetTrafficLight index: " << iter->second.index << std::endl;
		std::cout << "GetTrafficLight opendriveLightId: " << iter->second.opendriveLightId << std::endl;
		std::cout << "GetTrafficLight countDown: " << iter->second.countDown << std::endl;
		std::cout << "GetTrafficLight status: " << iter->second.status << std::endl;
	}
}


TEST_F(GlobalTestNoSync, SetSensorLaneInfoCB) {
	static bool flagSetSensorLaneInfoCB = true;
	SimOneAPI::SetSensorLaneInfoCB([](int mainVehicleId, const char* sensorId, SimOne_Data_LaneInfo *pLaneInfo) {
		std::cout << "SetSensorLaneInfoCB timestamp:" << pLaneInfo->timestamp << std::endl;
		std::cout << "SetSensorLaneInfoCB frame:" << pLaneInfo->frame << std::endl;
		std::cout << "SetSensorLaneInfoCB id:" << pLaneInfo->id << std::endl;
		std::cout << "SetSensorLaneInfoCB laneType:" << pLaneInfo->laneType << std::endl;
		flagSetSensorLaneInfoCB = false;
		/*EXPECT_FLOAT_EQ(0.1, pGps->throttle);
		EXPECT_EQ(4, pGps->gear);*/

	});
	while (flagSetSensorLaneInfoCB) {}
	if (flagSetSensorLaneInfoCB) {
		GTEST_FATAL_FAILURE_("not call back");
	}
}

TEST_F(GlobalTestNoSync, GetSensorLaneInfoResult) {
	std::unique_ptr<SimOne_Data_LaneInfo> pLaneInfo = std::make_unique<SimOne_Data_LaneInfo>();
	std::map<int, SimOne_Data_LaneInfo> pLaneInfoMap;
	//bool LaneInfoResult = SimOneAPI::GetSensorLaneInfo(0, "1", pLaneInfo.get());
	//EXPECT_TRUE(EnvironmentResult) << "GetSensorLaneInfoResult Error" << std::endl;
	while (1) {
		bool LaneInfoResult = SimOneAPI::GetSensorLaneInfo(0, "1", pLaneInfo.get());
		if (SimOneAPI::GetCaseRunStatus() == SimOne_Case_Status::SimOne_Case_Status_Running) {
			pLaneInfoMap.insert(map<int, SimOne_Data_LaneInfo>::value_type(pLaneInfo->id, *pLaneInfo));
			std::this_thread::sleep_for(std::chrono::milliseconds(1000));
		}
		if (SimOneAPI::GetCaseRunStatus() == SimOne_Case_Status::SimOne_Case_Status_Stop) {
			break;
		}
	}
	std::cout << pLaneInfoMap.size() << std::endl;
	map<int, SimOne_Data_LaneInfo>::iterator iter;
	for (iter = pLaneInfoMap.begin(); iter != pLaneInfoMap.end(); iter++) {
		std::cout << "GetSensorLaneInfo id: " << iter->first << std::endl;
		std::cout << "GetSensorLaneInfo laneType: " << iter->second.laneType << std::endl;
		std::cout << "GetSensorLaneInfo laneLeftID: " << iter->second.laneLeftID << std::endl;
		std::cout << "GetSensorLaneInfo laneRightID: " << iter->second.laneRightID << std::endl;
		std::cout << "GetSensorLaneInfo lanePredecessorID: " << iter->second.lanePredecessorID << std::endl;
		std::cout << "GetSensorLaneInfo laneSuccessorID: " << iter->second.laneSuccessorID << std::endl;
	}
}

int main(int argc, char* argv[]) {
	//testing::GTEST_FLAG(output) = "xml:";
	//testing::GTEST_FLAG(filter) = "GlobalTestNoSync.GetGpsResult";
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();

}