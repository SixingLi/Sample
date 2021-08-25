#include <string>
#include <chrono>
#include <fstream>
#include "utilTest.h"

#include "decode.h"
#include <sstream>

const char* ip = "127.0.0.1";
unsigned short portLidar = 6699;
unsigned short infoPort = 7788;
unsigned short port = 7890;
SimOne_Data_Point_Cloud Point_Cloud;
SimOne_Data_Image Image;

//for SimOneSensorAPI.h test
TEST_F(GlobalSMTest, SetGpsUpdateCB) {
	static bool flagSetGpsUpdateCB = true;
	SimOneSM::SetGpsUpdateCB([](int mainVehicleId, SimOne_Data_Gps *pGps) {
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
	if (flagSetGpsUpdateCB){
		GTEST_FATAL_FAILURE_("not call back");
	}
}

TEST_F(GlobalSMTest, SetGroundTruthUpdateCB) {
	static bool flagSetGroundTruthUpdateCB = true;
	SimOneSM::SetGroundTruthUpdateCB([](int mainVehicleId, SimOne_Data_Obstacle *pObstacle) {
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
	if (flagSetGroundTruthUpdateCB){
		GTEST_FATAL_FAILURE_("not call back");
	}
}

TEST_F(GlobalSMTest, SetSensorDetectionsUpdateCB) {
	static bool numberSetImageUpdateCB = true;
	static bool flagSetSensorDetectionsUpdateCB = true;
	SimOneSM::SetSensorDetectionsUpdateCB([](int mainVehicleId, const char* sensorId, SimOne_Data_SensorDetections *pGroundtruth){
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
	if (flagSetSensorDetectionsUpdateCB){
		GTEST_FATAL_FAILURE_("not call back");
	}
}


TEST_F(GlobalSMTest, SetImageUpdateCB) {
	static bool numberSetImageUpdateCB = true;
	static bool flagSetImageUpdateCB = true;
	SimOneSM::SetImageUpdateCB([](int mainVehicleId, const char* sensorId, SimOne_Data_Image *pImage) {
		std::cout << "SetImageUpdateCB frame: " << pImage->frame << std::endl;
		std::cout << "SetImageUpdateCB timestamp: " << pImage->timestamp << std::endl;

		std::cout << "SetImageUpdateCB imageDataSize: " << pImage->imageDataSize << std::endl;
		std::cout << "SetImageUpdateCB width: " << pImage->width << std::endl;
		std::cout << "SetImageUpdateCB height: " << pImage->height << std::endl;
		flagSetImageUpdateCB = false;
		EXPECT_GT(pImage->imageDataSize, 0);
		EXPECT_GE(pImage->width, 1920);
		EXPECT_GE(pImage->height, 1080);

	});

	while (numberSetImageUpdateCB && flagSetImageUpdateCB) {
		numberSetImageUpdateCB = false;
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}
	if (flagSetImageUpdateCB){
		GTEST_FATAL_FAILURE_("not call back");
	}
}

TEST_F(GlobalSMTest, SetPointCloudUpdateCB) {
	static bool numberSetPointCloudUpdateCB = true;
	static bool flagSetPointCloudUpdateCB = true;
	SimOneSM::SetPointCloudUpdateCB([](int mainVehicleId, const char* sensorId, SimOne_Data_Point_Cloud *pPointcloud) {
		std::cout << "SetPointCloudUpdateCB frame:" << pPointcloud->frame << std::endl;
		std::cout << "SetPointCloudUpdateCB timestamp:" << pPointcloud->timestamp << std::endl;

		std::cout << "SetPointCloudUpdateCB pointCloudDataSize: " << pPointcloud->pointCloudDataSize << std::endl;
		std::cout << "SetPointCloudUpdateCB width: " << pPointcloud->width << std::endl;
		std::cout << "SetPointCloudUpdateCB height: " << pPointcloud->height << std::endl;
		flagSetPointCloudUpdateCB = false;
		EXPECT_GT(pPointcloud->pointCloudDataSize, 0);
		EXPECT_GT(pPointcloud->height, 0);
	});

	while (numberSetPointCloudUpdateCB && flagSetPointCloudUpdateCB) {
		numberSetPointCloudUpdateCB = false;
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}
	if (flagSetPointCloudUpdateCB){
		GTEST_FATAL_FAILURE_("not call back");
	}
}

TEST_F(GlobalSMTest, SetUltrasonicRadarsCB) {
	static bool numberSetUltrasonicRadarsCB = true;
	static bool flagSetUltrasonicRadarsCB = true;
	SimOneSM::SetUltrasonicRadarsCB([](int mainVehicleId, SimOne_Data_UltrasonicRadars *pUltrasonics) {
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
	if (flagSetUltrasonicRadarsCB){
		GTEST_FATAL_FAILURE_("not call back");
	}
}


TEST_F(GlobalSMTest, SetRadarDetectionsUpdateCB) {
	static bool numberSetRadarDetectionsUpdateCB = true;
	static bool flagSetRadarDetectionsUpdateCB = true;
	SimOneSM::SetRadarDetectionsUpdateCB([](int mainVehicleId, const char* sensorId, SimOne_Data_RadarDetection *pDetections) {
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
	if (flagSetRadarDetectionsUpdateCB){
		GTEST_FATAL_FAILURE_("not call back");
	}
}

TEST_F(GlobalSMTest, SetStreamingImageCB) {
	static bool numberSetStreamingImageCB = true;
	static bool flagSetStreamingImageCB = true;
	SimOneSM::SetStreamingImageCB(ip, port, [](SimOne_Data_Image *pImage) {
		std::cout << "SetStreamingImageCB imageDataSize: " << pImage->imageDataSize << std::endl;
		std::cout << "SetStreamingImageCB width: " << pImage->width << std::endl;
		std::cout << "SetStreamingImageCB height: " << pImage->height << std::endl;
		flagSetStreamingImageCB = false;
		EXPECT_GT(pImage->imageDataSize, 0);
		EXPECT_GT(pImage->height, 0);
	});
	while (numberSetStreamingImageCB && flagSetStreamingImageCB) {
		numberSetStreamingImageCB = false;
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}
	if (flagSetStreamingImageCB){
		GTEST_FATAL_FAILURE_("not call back");
	}
}

TEST_F(GlobalSMTest, SetStreamingPointCloudUpdateCB) {
	static bool numberSetStreamingPointCloudUpdateCB = true;
	static bool flagSetStreamingPointCloudUpdateCB = true;
	SimOneSM::SetStreamingPointCloudUpdateCB(ip, portLidar, infoPort, [](SimOne_Data_Point_Cloud *pPointCloud) {
		std::cout << "SetStreamingPointCloudUpdateCB pointStep:" << pPointCloud->pointStep << std::endl;
		std::cout << "SetStreamingPointCloudUpdateCB height:" << pPointCloud->height << std::endl;
		std::cout << "SetStreamingPointCloudUpdateCB pointCloudDataSize:" << pPointCloud->pointCloudDataSize << std::endl;
		
		EXPECT_GT(pPointCloud->pointCloudDataSize, 0);
		EXPECT_GE(16, pPointCloud->pointStep);
		flagSetStreamingPointCloudUpdateCB = false;
	});
	while (numberSetStreamingPointCloudUpdateCB && flagSetStreamingPointCloudUpdateCB) {
		numberSetStreamingPointCloudUpdateCB = false;
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}
	if (flagSetStreamingPointCloudUpdateCB){
		GTEST_FATAL_FAILURE_("not call back");
	}
}

TEST_F(GlobalSMTest, OSISetSensorDataUpdateCB) {
	static bool numberOSISetSensorDataUpdateCB = true;
	static bool flagOSISetSensorDataUpdateCB = true;
	SimOne_Data_OSI *groundtruth = new SimOne_Data_OSI();
	SimOneSM::OSISetSensorDataUpdateCB([](int mainVehicleId, int sensorId, SimOne_Data_OSI *pSensorData){
		std::cout << "OSISetSensorDataUpdateCB.dataSize:" << pSensorData->dataSize << std::endl;
		/*osi3::SensorData sensordata;
		std::string s1(pSensorData->data, pSensorData->data + pSensorData->dataSize);

		std::stringstream iss, oss;
		iss << s1;
		base64::decoder decoder;
		decoder.decode(iss, oss);
		std::string OSISetSensorDataUpdateCBData = oss.str();
		if (sensordata.ParseFromString(OSISetSensorDataUpdateCBData))
		{
			osi3::FeatureData featureData = sensordata.feature_data();
			int sensorsize = sensordata.sensor_view_size();
			std::cout << "OSISetSensorDataUpdateCB:" << sensorsize << std::endl;
		}*/

		flagOSISetSensorDataUpdateCB = false;
	});
	while (numberOSISetSensorDataUpdateCB && flagOSISetSensorDataUpdateCB) {
		numberOSISetSensorDataUpdateCB = false;
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}
	if (flagOSISetSensorDataUpdateCB){
		GTEST_FATAL_FAILURE_("not call back");
	}
}

TEST_F(GlobalSMTest, OSISetGroundTruthUpdateCB) {
	static bool numberOSISetGroundTruthUpdateCB = true;
	static bool flagOSISetGroundTruthUpdateCB = true;
	SimOne_Data_OSI *groundtruth = new SimOne_Data_OSI();
	SimOneSM::OSISetGroundTruthUpdateCB([](int mainVehicleId, SimOne_Data_OSI *pGroundtruth) {
		std::cout << "OSISetSensorDOSISetGroundTruthUpdateCBtaUpdateCB.dataSize:" << pGroundtruth->dataSize << std::endl;
		/*osi3::GroundTruth groundtruth;
		std::string s1(pGroundtruth->data, pGroundtruth->data + pGroundtruth->dataSize);

		std::stringstream iss, oss;
		iss << s1;
		base64::decoder decoder;
		decoder.decode(iss, oss);
		std::string osiData = oss.str();
		if (groundtruth.ParseFromString(osiData))
		{
			int obj_size = groundtruth.moving_object().size();
			std::cout << "OSISetGroundTruthUpdateCB:" << obj_size << std::endl;
		}*/

		flagOSISetGroundTruthUpdateCB = false;
	});
	while (numberOSISetGroundTruthUpdateCB && flagOSISetGroundTruthUpdateCB) {
		numberOSISetGroundTruthUpdateCB = false;
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}
	if (flagOSISetGroundTruthUpdateCB){
		GTEST_FATAL_FAILURE_("not call back");
	}
}

TEST_F(GlobalSMTest, SetCameraSensorLaneInfo) {
	static bool numberSetCameraSensorLaneInfo = true;
	static bool flagSetCameraSensorLaneInfo = true;

	SimOneSM::SetCameraSensorLaneInfoCB([](int mainVehicleId, int sensorId, SimOne_Data_LaneInfo *pLaneInfo) {
		std::cout << "SetCameraSensorLaneInfo.id:" << pLaneInfo->id << std::endl;
		std::cout << "SetCameraSensorLaneInfo.laneType:" << pLaneInfo->laneType << std::endl;
		flagSetCameraSensorLaneInfo = false;
	});
	while (numberSetCameraSensorLaneInfo && flagSetCameraSensorLaneInfo) {
		numberSetCameraSensorLaneInfo = false;
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}
	if (flagSetCameraSensorLaneInfo) {
		GTEST_FATAL_FAILURE_("not call back");
	}
}


int main(int argc, char* argv[]) {
	//testing::GTEST_FLAG(output) = "xml:";
	testing::GTEST_FLAG(filter) = "GlobalSMTest.SetCameraSensorLaneInfo";
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();	
	
}