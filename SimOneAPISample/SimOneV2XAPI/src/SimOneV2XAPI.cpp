#include <vector>
#include <thread>
#include <string>
#include <fstream>
#include "utilTest.h"

#include "osi_groundtruth.pb.h"
#include "osi_object.pb.h"
#include "osi_sensorview.pb.h"
#include "osi_sensorview.pb.h"
#include "osi_featuredata.pb.h"
#include "osi_sensordata.pb.h"
#include "decode.h"
#include <sstream>

int CameraSensorId = 10;
int MMWRadarSensorId = 10;
int LiDARSensorId = 10;
int UltrasonicRadarSensorId = 10;
int AllUltrasonicRadarSensorId = 10;
int PerfectPerceptionSensorId = 10;
int V2XSensorId = 10;

const char* ip = "10.2.35.237";
unsigned short portLidar = 6699;
unsigned short infoPort = 7788;
unsigned short port = 7890;


//for SimOneV2XAPI.h test
 TEST_F(GlobalSMTest, SetDrive)
 {
 	std::unique_ptr<SimOne_Data_Control> Control = std::make_unique<SimOne_Data_Control>();
 	Control->throttle = 0.01;
 	Control->steering = 0;
 	Control->brake = 0;
 	Control->gear = EGearMode::EGearMode_Drive;
 	SimOneSM::SetDrive(0, Control.get());
 }

TEST_F(GlobalSMTest, GetGps) {
	std::unique_ptr<SimOne_Data_Gps> pGps = std::make_unique<SimOne_Data_Gps>();
	std::map<int, SimOne_Data_Gps>GpsMap;

	while (GpsMap.size() <= 0) {
		bool resultGetGps = SimOneSM::GetGps(0, pGps.get());
		if (resultGetGps && pGps->timestamp>=3)
		{
			std::cout << "Gps.posX:" << pGps->posX << std::endl;
			GpsMap.insert(map<int, SimOne_Data_Gps>::value_type(pGps->frame, *pGps));
		}
		else
		{
			whileNumber++;
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
		}

		if (whileNumber > 100) {
			std::cout << "already try 100 times" << std::endl;
			GTEST_FATAL_FAILURE_("api return false");
		}

	}
	map<int, SimOne_Data_Gps>::iterator iter;
	for (iter = GpsMap.begin(); iter != GpsMap.end(); iter++) {
		if (iter->first && iter->second.timestamp) {

			std::cout << " frame:" << iter->first << " timestamp:" << iter->second.timestamp << std::endl;
			std::cout << " frame:" << iter->first << " posX:" << iter->second.posX << std::endl;
			std::cout << " frame:" << iter->first << " posY:" << iter->second.posY << std::endl;
			std::cout << " frame:" << iter->first << " oriX:" << iter->second.oriX << std::endl;
			std::cout << " frame:" << iter->first << " oriY:" << iter->second.oriY << std::endl;
			std::cout << " frame:" << iter->first << " velX:" << iter->second.velX << std::endl;
			std::cout << " frame:" << iter->first << " velY:" << iter->second.velY << std::endl;
			std::cout << " frame:" << iter->first << " velZ:" << iter->second.velZ << std::endl;
			std::cout << " frame:" << iter->first << " oriY:" << iter->second.oriY << std::endl;
			std::cout << " frame:" << iter->first << " oriZ:" << iter->second.oriZ << std::endl;
			std::cout << " frame:" << iter->first << " throttle:" << iter->second.throttle << std::endl;
			std::cout << " frame:" << iter->first << " brake:" << iter->second.brake << std::endl;
			std::cout << " frame:" << iter->first << " steering:" << iter->second.steering << std::endl;
			std::cout << " frame:" << iter->first << " gear:" << iter->second.gear << std::endl;
			//ASSERT_EQ(0.01, iter->second.throttle);
			ASSERT_GE(iter->second.velX, 0);
			//ASSERT_GT(iter->second.velX, 0);
			//ASSERT_EQ(4, iter->second.gear);
			break;
		}
		else
			GTEST_FATAL_FAILURE_("timestamp and frame are 0");
	}
}

TEST_F(GlobalSMTest, GetGroundTruth) {
	std::unique_ptr<SimOne_Data_Obstacle> Obstacle = std::make_unique<SimOne_Data_Obstacle>();
	std::map<int, SimOne_Data_Obstacle>ObstacleMap;
	while (ObstacleMap.size() <= 0) {
		bool resultGetGroundTruth = SimOneSM::GetGroundTruth(0, Obstacle.get());
		if (resultGetGroundTruth) {
			ObstacleMap.insert(map<int, SimOne_Data_Obstacle>::value_type(Obstacle->frame, *Obstacle));
			//ASSERT_GE(int(Obstacle.obstacleSize), 8);
		}
	}
	std::cout << "ObstacleMap------------" << ObstacleMap.size() << std::endl;

	map<int, SimOne_Data_Obstacle>::iterator iter;
	for (iter = ObstacleMap.begin(); iter != ObstacleMap.end(); iter++) {
		if (iter->first && iter->second.timestamp) {
			std::cout << " frame:" << iter->first << " obstacleSize:" << iter->second.obstacleSize << std::endl;
			for (int i = 0; i < iter->second.obstacleSize; ++i)
			{
				std::cout << "++++++++++++++++++++++++++++++++++++" << std::endl;
				std::cout << " frame:" << iter->first << " id:" << iter->second.obstacle[i].id << std::endl;
				std::cout << " frame:" << iter->first << " viewId:" << iter->second.obstacle[i].viewId << std::endl;
				std::cout << " frame:" << iter->first << " type:" << iter->second.obstacle[i].type << std::endl;
				std::cout << " frame:" << iter->first << " theta:" << iter->second.obstacle[i].theta << std::endl;
				std::cout << " frame:" << iter->first << " posX:" << iter->second.obstacle[i].posX << std::endl;
				std::cout << " frame:" << iter->first << " posY:" << iter->second.obstacle[i].posY << std::endl;
				std::cout << " frame:" << iter->first << " velX:" << iter->second.obstacle[i].velX << std::endl;
				std::cout << " frame:" << iter->first << " velY:" << iter->second.obstacle[i].velY << std::endl;
				std::cout << " frame:" << iter->first << " length:" << iter->second.obstacle[i].length << std::endl;
				std::cout << " frame:" << iter->first << " width:" << iter->second.obstacle[i].width << std::endl;
			}
			ASSERT_GE(iter->second.obstacleSize, 8);
			break;
		}
		else
			GTEST_FATAL_FAILURE_("timestamp and frame are 0");
	}
}

TEST_F(GlobalSMTest, SetEvent) {
	std::unique_ptr<SimOne_Data_Gps> Gps = std::make_unique<SimOne_Data_Gps>();
	bool resultGps = SimOneSM::GetGps(0, Gps.get());
	std::unique_ptr<SimOne_Data_Vehicle_EventInfo> EventInfo = std::make_unique<SimOne_Data_Vehicle_EventInfo>();

	EventInfo->timestamp = Gps->timestamp;
	EventInfo->type = ESimone_Vehicle_EventInfo_Type::ESimOne_VehicleEventInfo_Forward_Collision_Warning;
	bool resultEventInfo = SimOneSM::SetVehicleEvent(0, EventInfo.get());
	ASSERT_TRUE(resultEventInfo);
}

TEST_F(GlobalSMTest, SetSignalLights) {
	std::unique_ptr<SimOne_Data_Signal_Lights> SignalLights = std::make_unique<SimOne_Data_Signal_Lights>();
	SignalLights->signalLights = SimOne_Signal_Light::ESimOne_Signal_Light_DoubleFlash;
	bool resultSignalLights = SimOneSM::SetSignalLights(0, SignalLights.get());
	ASSERT_TRUE(resultSignalLights);
}

TEST_F(GlobalSMTest, GetTrafficLightList) {
	int timeout = 20;
	if (!SimOneSM::LoadHDMap(timeout))
	{
		std::cout << "Failed to load hdmap!" << std::endl;
	}

	SSD::SimVector<HDMapStandalone::MSignal> TrafficLightList;
	SimOneSM::GetTrafficLightList(TrafficLightList);
	if (TrafficLightList.size() < 1)
	{
		std::cout << "No traffic light exists!" << std::endl;
	}

	for (auto& item : TrafficLightList)
	{
		std::cout << "sign id:" << item.id << std::endl;
		std::cout << "type:" << item.type.GetString() << std::endl;
		std::cout << "subType:" << item.subType.GetString() << std::endl;
		std::cout << "item.pt.x: " << item.pt.x << "  item.pt.y: " << item.pt.y << "  item.pt.z: " << item.pt.z << std::endl;
		std::cout << "heading.pt.x: " << item.heading.x << "  heading.pt.y: " << item.heading.y << "  heading.pt.z: " << item.heading.y << std::endl;
		std::cout << "value:" << item.value.GetString() << std::endl;
		std::cout << "unit:" << item.unit.GetString() << std::endl;
		std::cout << "isDynamic:" << item.isDynamic << std::endl;
		SimOne_Data_TrafficLight trafficLight;
		if (SimOneSM::GetTrafficLight(0, item.id, &trafficLight))
		{
			std::cout << "trafficLight.status:----------------------" << trafficLight.status << std::endl;
		}

	}


}


TEST_F(GlobalSMTest, GetSensorConfigurations) {
	std::unique_ptr<SimOne_Data_SensorConfigurations> SensorConfigurations = std::make_unique<SimOne_Data_SensorConfigurations>();
	ESimOneNodeType typeCheck;

	bool resultSensorConfigurations = SimOneSM::GetSensorConfigurations(SensorConfigurations.get());
	if (resultSensorConfigurations) {
		int size = SensorConfigurations->dataSize;
		std::cout << "SensorConfigurations size" << size << std::endl;
		bool SensorTypeExist = false;
		for (int i = 0; i < size; i++) {
			std::cout << "____________________________" << std::endl;
			std::cout << "sensor id:" << SensorConfigurations->data[i].sensorId << std::endl;
			std::cout << "sensor type:" << SensorConfigurations->data[i].sensorType << std::endl;
			std::cout << "sensor hz:" << SensorConfigurations->data[i].hz << std::endl;
			std::cout << "sensor roll:" << SensorConfigurations->data[i].roll << std::endl;
			std::cout << "sensor pitch:" << SensorConfigurations->data[i].pitch << std::endl;
			std::cout << "sensor yaw:" << SensorConfigurations->data[i].yaw << std::endl;
			std::cout << "sensor x:" << SensorConfigurations->data[i].x << std::endl;
			std::cout << "sensor y:" << SensorConfigurations->data[i].y << std::endl;

			/*typeCheck = SensorConfigurations->data[i].type;

			switch (typeCheck) {
			case ESimOneNodeType::ESimOneNode_Camera:
				SensorTypeExist = true;
				CameraSensorId = SensorConfigurations->data[i].id;
				break;
			case ESimOneNodeType::ESimOneNode_MMWRadar:
				SensorTypeExist = true;
				MMWRadarSensorId = SensorConfigurations->data[i].id;
				break;
			case ESimOneNodeType::ESimOneNode_LiDAR:
				SensorTypeExist = true;
				LiDARSensorId = SensorConfigurations->data[i].id;
				break;
			case ESimOneNodeType::ESimOneNode_UltrasonicRadar:
				UltrasonicRadarSensorId = SensorConfigurations->data[i].id;
				SensorTypeExist = true;
				break;
			case ESimOneNode_AllUltrasonicRadar:
				AllUltrasonicRadarSensorId = SensorConfigurations->data[i].id;
				SensorTypeExist = true;
				break;
			case ESimOneNode_PerfectPerception:
				PerfectPerceptionSensorId = SensorConfigurations->data[i].id;
				SensorTypeExist = true;
				break;
			case ESimOneNode_V2X:
				V2XSensorId = SensorConfigurations->data[i].id;
				SensorTypeExist = true;
				break;
			default:
				SensorTypeExist = false;*/
			//}
			//ASSERT_TRUE(SensorTypeExist);
		}
		//ASSERT_GT(size, 0);
	}
}

TEST_F(GlobalSMTest, SetEnvironment) {
	std::unique_ptr<SimOne_Data_Environment> Environment = std::make_unique<SimOne_Data_Environment>();
	Environment->timeOfDay = 2300;
	Environment->snowDensity = 1.0;
	bool resultSetEnvironment = SimOneSM::SetEnvironment(Environment.get());
	ASSERT_TRUE(resultSetEnvironment);
}

TEST_F(GlobalSMTest, GetEnvironment) {
	std::unique_ptr<SimOne_Data_Environment> Environment = std::make_unique<SimOne_Data_Environment>();
	bool resultGetEnvironment = SimOneSM::GetEnvironment(Environment.get());
	if (resultGetEnvironment) {
		ASSERT_EQ(Environment->timeOfDay, 2300);
	}
}

TEST_F(GlobalSMTest, GetWayPoints) {
	std::unique_ptr<SimOne_Data_WayPoints> WayPoints = std::make_unique<SimOne_Data_WayPoints>();

	bool resultWayPoints = SimOneSM::GetWayPoints(WayPoints.get());
	if (resultWayPoints) {
		std::cout << "wayPointsSize: " << WayPoints->wayPointsSize << std::endl;
		for (size_t i = 0; i < WayPoints->wayPointsSize; ++i) {
			std::cout << "posX: " << WayPoints->wayPoints[i].posX << std::endl;
			std::cout << "posY: " << WayPoints->wayPoints[i].posY << std::endl;
		}
		ASSERT_GE(WayPoints->wayPointsSize, 2);
		float startPosX = WayPoints->wayPoints[0].posX;
		float endPosX = WayPoints->wayPoints[WayPoints->wayPointsSize - 1].posX;
		ASSERT_GT(endPosX, startPosX);
	}
}


TEST_F(GlobalSMTest, GetDriverStatus) {
	std::unique_ptr<SimOne_Data_Driver_Status> DriverStatus = std::make_unique<SimOne_Data_Driver_Status>();
	while (1) {
		bool resultDriverStatus = SimOneSM::GetDriverStatus(0, DriverStatus.get());
		if (resultDriverStatus) {
			std::cout << "get simoneDriver status: " << DriverStatus->driverStatus << std::endl;
		}
		else
		{
			whileNumber++;
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
		}

		if (whileNumber > 100) {
			std::cout << "already try 100 times" << std::endl;
			GTEST_FATAL_FAILURE_("api return false");
		}
	}
}

TEST_F(GlobalSMTest, GetStreamingPointCloud) {
	std::unique_ptr<SimOne_Data_Point_Cloud> Point_Cloud = std::make_unique<SimOne_Data_Point_Cloud>();
	while (1) {
		bool  resultStreamingPointCloud = SimOneSM::GetStreamingPointCloud(ip, portLidar, infoPort, Point_Cloud.get());
		if (resultStreamingPointCloud && Point_Cloud->pointCloudDataSize > 0) {
			std::cout << "GetStreamingPointCloud pointStep: " << Point_Cloud->pointStep << std::endl;
			std::cout << "GetStreamingPointCloud height: " << Point_Cloud->height << std::endl;
			std::cout << "GetStreamingPointCloud pointCloudDataSize: " << Point_Cloud->pointCloudDataSize << std::endl;
			ASSERT_GT(Point_Cloud->pointCloudDataSize, 0);
			ASSERT_GE(16, Point_Cloud->pointStep);
			ASSERT_TRUE(resultStreamingPointCloud);
			break;
		}
		else
		{
			whileNumber++;
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
		}

		if (whileNumber > 100) {
			std::cout << "already try 100 times" << std::endl;
			GTEST_FATAL_FAILURE_("api return false");
		}
	}
}

TEST_F(GlobalSMTest, GetStreamingImage) {
	std::unique_ptr<SimOne_Data_Image> ImageStreaming = std::make_unique<SimOne_Data_Image>();
	std::map<int, SimOne_Data_Image*>GetStreamingImageMap;
	while (GetStreamingImageMap.size() <= 0) {
		bool resultStreamingImage = SimOneSM::GetStreamingImage(ip, port, ImageStreaming.get());
		if (resultStreamingImage && ImageStreaming->imageDataSize>0) {
			GetStreamingImageMap.insert(std::map<int, SimOne_Data_Image*>::value_type(ImageStreaming->frame, ImageStreaming.get()));
			std::cout << "GetStreamingImageMap.size()====================" << GetStreamingImageMap.size() << std::endl;
		}
		else
		{
			whileNumber++;
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
		}

		if (whileNumber > 100) {
			std::cout << "already try 100 times" << std::endl;
			GTEST_FATAL_FAILURE_("api return false");
		}
	}
	std::map<int, SimOne_Data_Image*>::iterator iter;

	for (iter = GetStreamingImageMap.begin(); iter != GetStreamingImageMap.end(); iter++) {
			std::cout << " imageDataSize:" << iter->first << " imageDataSize:" << iter->second->imageDataSize << std::endl;
			std::cout << " width:" << iter->first << " width:" << iter->second->width << std::endl;
			std::cout << " height:" << iter->first << " height:" << iter->second->height << std::endl;
			ASSERT_GT(iter->second->imageDataSize, 0);
			ASSERT_GT(iter->second->height, 0);
			break;
	}
}

TEST_F(GlobalSMTest, GetUltrasonicRadar) {
	// if (UltrasonicRadarSensorId != 10) {
		std::map<int, SimOne_Data_UltrasonicRadar*> UltrasonicRadarMap;
		std::cout << "UltrasonicRadarSensorId: " << UltrasonicRadarSensorId << std::endl;
		std::unique_ptr<SimOne_Data_UltrasonicRadar> UltrasonicRadar = std::make_unique<SimOne_Data_UltrasonicRadar>();

		while (UltrasonicRadarMap.size() <= 0) {
			bool resultUltrasonicRadar = SimOneSM::GetUltrasonicRadar(0, 1, UltrasonicRadar.get());

			if (resultUltrasonicRadar) {
				UltrasonicRadarMap.insert(std::map<int, SimOne_Data_UltrasonicRadar*>::value_type(UltrasonicRadar->frame, UltrasonicRadar.get()));
				std::cout << " GetUltrasonicRadar frame:" << UltrasonicRadar->frame << std::endl;
				std::cout << " GetUltrasonicRadar timestamp:" << UltrasonicRadar->timestamp << std::endl;
			}
			else
			{
				whileNumber++;
				std::this_thread::sleep_for(std::chrono::milliseconds(10));
			}

			if (whileNumber > 100) {
				std::cout << "already try 100 times" << std::endl;
				GTEST_FATAL_FAILURE_("api return false");
			}

		}
		std::map<int, SimOne_Data_UltrasonicRadar*>::iterator iter;

		for (iter = UltrasonicRadarMap.begin(); iter != UltrasonicRadarMap.end(); iter++) {
			if (iter->first && iter->second->timestamp) {
				std::cout << " frame:" << iter->first << " obstacleNum:" << iter->second->obstacleNum << std::endl;
				std::cout << " frame:" << iter->first << " id: " << iter->second->id << std::endl;
				break;
			}
			else
				GTEST_FATAL_FAILURE_("timestamp and frame are 0");
		}
	// }
	// else
	// 	GTEST_FATAL_FAILURE_("not set UltrasonicRadar");
}

TEST_F(GlobalSMTest, GetSensorDetections) {
	std::unique_ptr<SimOne_Data_SensorDetections> SensorDetections = std::make_unique<SimOne_Data_SensorDetections>();

	while (1) {
		bool resultGetSensorDetections = SimOneSM::GetSensorDetections(0, 1, SensorDetections.get());
		if (resultGetSensorDetections) {
			std::cout << "GetSensorDetections timestampe: " << SensorDetections->timestamp << std::endl;
			std::cout << "GetSensorDetections obstacleSize: " << SensorDetections->objectSize << std::endl;
			for (int i = 0; i < SensorDetections->objectSize; ++i) {
				std::cout << "________________________________________" << std::endl;
				std::cout << "GetSensorDetections bbox2dMinY: " << SensorDetections->objects[i].bbox2dMinY << std::endl;
				std::cout << "GetSensorDetections bbox2dMinX: " << SensorDetections->objects[i].bbox2dMinX << std::endl;
				std::cout << "GetSensorDetections  id: " << SensorDetections->objects[i].id << std::endl;
				std::cout << "GetSensorDetections  relativePosX: " << SensorDetections->objects[i].relativePosX << std::endl;
				std::cout << "GetSensorDetections type: " << SensorDetections->objects[i].type << std::endl;
				std::cout << "GetSensorDetections posX: " << SensorDetections->objects[i].posX << std::endl;
				std::cout << "GetSensorDetections posY: " << SensorDetections->objects[i].posY << std::endl;
				std::cout << "GetSensorDetections posZ: " << SensorDetections->objects[i].posZ << std::endl;
				std::cout << "GetSensorDetections velX: " << SensorDetections->objects[i].velX << std::endl;
				std::cout << "GetSensorDetections velY: " << SensorDetections->objects[i].velY << std::endl;
				std::cout << "GetSensorDetections velZ: " << SensorDetections->objects[i].velZ << std::endl;
				std::cout << "GetSensorDetections length: " << SensorDetections->objects[i].length << std::endl;
				std::cout << "GetSensorDetections width: " << SensorDetections->objects[i].width << std::endl;
				std::cout << "GetSensorDetections height: " << SensorDetections->objects[i].height << std::endl;
				std::cout << "GetSensorDetections probability: " << SensorDetections->objects[i].probability << std::endl;
			}
			EXPECT_GT(SensorDetections->objectSize, 8);
			//break;
		}
		else
		{
			whileNumber++;
			std::this_thread::sleep_for(std::chrono::milliseconds(10));
		}

		if (whileNumber > 10000) {
			std::cout << "already try 100 times" << std::endl;
			GTEST_FATAL_FAILURE_("api return false");
		}
	}
	
}

TEST_F(GlobalSMTest, GetImage) {
	std::unique_ptr<SimOne_Data_Image> Image = std::make_unique<SimOne_Data_Image>();
	/*if (CameraSensorId != 10)
	{*/
		std::cout << "CameraSensorId: " << CameraSensorId << std::endl;
		std::map<int, SimOne_Data_Image*> ImageMap;
		
		while (ImageMap.size() <= 0) {
			bool resultGetImage = SimOneSM::GetImage(0, 1, Image.get());
			std::cout << "Image.resultGetImage:" << resultGetImage << std::endl;
			if (resultGetImage) {
				std::cout << "Image.imageDataSize:" << Image->imageDataSize << std::endl;
				ImageMap.insert(std::map<int, SimOne_Data_Image*>::value_type(Image->frame, Image.get()));
				break;
			}
			else
			{
				std::this_thread::sleep_for(std::chrono::milliseconds(100));
				whileNumber++;
			}

			if (whileNumber > 100) {
				std::cout << "already try 100 times" << std::endl;
				GTEST_FATAL_FAILURE_("api return false");
			}
		}

		std::map<int, SimOne_Data_Image*>::iterator iter;
		for (iter = ImageMap.begin(); iter != ImageMap.end(); iter++) {
			if (iter->first && iter->second->timestamp) {
				std::cout << "GetImage imageDataSize: " << iter->second->imageDataSize << std::endl;
				std::cout << "GetImage width: " << iter->second->width << std::endl;
				std::cout << "GetImage height: " << iter->second->height << std::endl;
				ASSERT_GT(iter->second->imageDataSize, 0);
				ASSERT_GE(iter->second->width, 1920);
				ASSERT_GE(iter->second->height, 1080);
				break;
			}
			else
				GTEST_FATAL_FAILURE_("timestamp and frame are 0");
		}
	//}
}

TEST_F(GlobalSMTest, GetPointCloud) {
	
	/*if (LiDARSensorId != 10)
	{*/
		std::cout << "LiDARSensorId: " << LiDARSensorId << std::endl;
		std::unique_ptr<SimOne_Data_Point_Cloud> Point_Cloud = std::make_unique<SimOne_Data_Point_Cloud>();
		std::map<int, SimOne_Data_Point_Cloud*> GetPointCloudMap;
		while (GetPointCloudMap.size() <= 0) {
			bool resultGetPointCloud = SimOneSM::GetPointCloud(0, 1, Point_Cloud.get());
			std::cout << "resultGetPointCloud:" << resultGetPointCloud << std::endl;
			if (resultGetPointCloud) {
				std::cout << "GetPointCloud frame:" << Point_Cloud->frame << std::endl;
				std::cout << "GetPointCloud timestamp:" << Point_Cloud->timestamp << std::endl;
				GetPointCloudMap.insert(std::map<int, SimOne_Data_Point_Cloud*>::value_type(Point_Cloud->frame, Point_Cloud.get()));
			}
			else
			{
				whileNumber++;
				std::this_thread::sleep_for(std::chrono::milliseconds(100));
			}

			if (whileNumber > 100) {
				std::cout << "already try 100 times" << std::endl;
				GTEST_FATAL_FAILURE_("api return false");
			}
		}

		std::map<int, SimOne_Data_Point_Cloud*>::iterator iter;
		for (iter = GetPointCloudMap.begin(); iter != GetPointCloudMap.end(); iter++)
		{
			if (iter->first && iter->second->timestamp) {
				std::cout << "GetPointCloud pointCloudDataSize: " << Point_Cloud->pointCloudDataSize << std::endl;
				std::cout << "GetPointCloud width: " << Point_Cloud->width << std::endl;
				std::cout << "GetPointCloud height: " << Point_Cloud->height << std::endl;

				ASSERT_GT(Point_Cloud->pointCloudDataSize, 0);
				ASSERT_GT(Point_Cloud->height, 0);
				break;
			}
			else
				GTEST_FATAL_FAILURE_("timestamp and frame are 0");
		}
	/*}

	else {
		GTEST_FATAL_FAILURE_("not set lidar");
	}*/

}

TEST_F(GlobalSMTest, GetUltrasonicRadars) {
	std::unique_ptr<SimOne_Data_UltrasonicRadars> Ultrasonics = std::make_unique<SimOne_Data_UltrasonicRadars>();
	std::map<int, SimOne_Data_UltrasonicRadars*> UltrasonicsMap;
	while (UltrasonicsMap.size() <= 0) {
		bool resultUltrasonicRadars = SimOneSM::GetUltrasonicRadars(0, Ultrasonics.get());
		if (resultUltrasonicRadars) {
			UltrasonicsMap.insert(std::map<int, SimOne_Data_UltrasonicRadars*>::value_type(Ultrasonics->frame, Ultrasonics.get()));
			std::cout << "GetUltrasonicRadars frame: " << Ultrasonics->frame << std::endl;
			std::cout << "GetUltrasonicRadars timestamp: " << Ultrasonics->timestamp << std::endl;
		}
		else
		{
			whileNumber++;
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
		}

		if (whileNumber > 100) {
			std::cout << "already try 100 times" << std::endl;
			GTEST_FATAL_FAILURE_("api return false");
		}
	}

	std::map<int, SimOne_Data_UltrasonicRadars*>::iterator iter;
	for (iter = UltrasonicsMap.begin(); iter != UltrasonicsMap.end(); iter++)
	{
		if (iter->first && iter->second->timestamp) {
			for (int i = 0; i < iter->second->UltrasonicRadarsNum; i++) {
				std::cout << "GetUltrasonicRadar id: " << iter->second->ultrasonicRadars[i].id << std::endl;
				std::cout << "GetUltrasonicRadar obstacleNum: " << iter->second->ultrasonicRadars[i].obstacleNum << std::endl;
				for (int j = 0; j < iter->second->ultrasonicRadars[i].obstacleNum; j++) {
					std::cout << "GetUltrasonicRadar obstacleRanges: " << iter->second->ultrasonicRadars[i].obstacledetections[j].obstacleRanges << std::endl;
					std::cout << "GetUltrasonicRadar x: " << iter->second->ultrasonicRadars[i].obstacledetections[j].x << std::endl;
					std::cout << "GetUltrasonicRadar y: " << iter->second->ultrasonicRadars[i].obstacledetections[j].y << std::endl;
				}
			}
		}
		else
			GTEST_FATAL_FAILURE_("timestamp and frame are 0");
	}
}

TEST_F(GlobalSMTest, GetRadarDetections) {
	//if (MMWRadarSensorId != 10) {
		std::unique_ptr<SimOne_Data_RadarDetection> RadarDetection = std::make_unique<SimOne_Data_RadarDetection>();

		std::map<int, SimOne_Data_RadarDetection> RadarDetectionMap;
		while (RadarDetectionMap.size() <= 0) {
			bool resultMMWRadarSensor = SimOneSM::GetRadarDetections(0, 1, RadarDetection.get());
			std::cout << "resultMMWRadarSensor��" << resultMMWRadarSensor << std::endl;
			if (resultMMWRadarSensor && RadarDetection->timestamp > 0) {
				RadarDetectionMap.insert(std::map<int, SimOne_Data_RadarDetection>::value_type(RadarDetection->frame, *RadarDetection));
				break;
			}
			else
			{
				whileNumber++;
				std::this_thread::sleep_for(std::chrono::milliseconds(100));
			}

			if (whileNumber > 100) {
				std::cout << "already try 100 times" << std::endl;
				GTEST_FATAL_FAILURE_("api return false");
			}
		}

		std::map<int, SimOne_Data_RadarDetection>::iterator iter;

		for (iter = RadarDetectionMap.begin(); iter != RadarDetectionMap.end(); iter++) {
			if (iter->first && iter->second.timestamp) {
				std::cout << "GetRadarDetections detectNum: " << iter->second.detectNum << std::endl;
				std::cout << "GetRadarDetections timestamp: " << iter->second.timestamp << std::endl;
				std::cout << "GetRadarDetections frame: " << iter->second.frame << std::endl;
				ASSERT_GT(iter->second.detectNum, 0);
				ASSERT_GT(iter->second.timestamp, 0);

				for (int i = 0; i < iter->second.detectNum; ++i)
				{
					std::cout << "*****************************" << std::endl;
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
			else
				GTEST_FATAL_FAILURE_("timestamp and frame are 0");
		}
	/*}
	else {
		GTEST_FATAL_FAILURE_("not set RadarDetections");
	}*/

}


TEST_F(GlobalSMTest, OSIGetGroundTruth) {
	std::unique_ptr<SimOne_Data_OSI> groundtruth = std::make_unique<SimOne_Data_OSI>();

	while (1) {
		bool resultOSIGetGroundTruth = SimOneSM::OSIGetGroundTruth(0, groundtruth.get());
		if (resultOSIGetGroundTruth) {
			std::cout << "groundtruth.dataSize:" << groundtruth->dataSize << std::endl;
			break;
		}
		else
		{
			whileNumber++;
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
		}

		if (whileNumber > 100) {
			std::cout << "already try 100 times" << std::endl;
			GTEST_FATAL_FAILURE_("api return false");
		}
	}
}

TEST_F(GlobalSMTest, OSIGetSensorData) {
	std::unique_ptr<SimOne_Data_OSI> Sensordata = std::make_unique<SimOne_Data_OSI>();

	while (1) {
		bool resultOSIGetSensorData = SimOneSM::OSIGetSensorData(0, 1, Sensordata.get());
		if (resultOSIGetSensorData) {
			std::cout << "Sensordata.dataSize:" << Sensordata->dataSize << std::endl;
			break;
		}
		else
		{
			whileNumber++;
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
		}

		if (whileNumber > 100) {
			std::cout << "already try 100 times" << std::endl;
			GTEST_FATAL_FAILURE_("api return false");
		}
	}
}


TEST_F(GlobalSMTest, GetCameraSensorLaneInfo) {
	std::unique_ptr<SimOne_Data_LaneInfo> pLaneInfo = std::make_unique<SimOne_Data_LaneInfo>();

	while (1) {
		bool resultGetCameraSensorLaneInfo = SimOneSM::GetCameraSensorLaneInfo(0, 1, pLaneInfo.get());
		if (resultGetCameraSensorLaneInfo) {
			std::cout << "pLaneInfo.id:" << pLaneInfo->id << std::endl;
			std::cout << "pLaneInfo.laneType:" << pLaneInfo->laneType << std::endl;
			std::cout << "pLaneInfo.laneLeftID:" << pLaneInfo->laneLeftID << std::endl;
			std::cout << "pLaneInfo.laneRightID:" << pLaneInfo->laneRightID << std::endl;
			std::cout << "pLaneInfo.lanePredecessorID:" << pLaneInfo->lanePredecessorID << std::endl;
			std::cout << "pLaneInfo.laneSuccessorID:" << pLaneInfo->laneSuccessorID << std::endl;
			std::cout << "pLaneInfo.leftBoundaryType:" << pLaneInfo->leftBoundaryType << std::endl;
			std::cout << "pLaneInfo.rightBoundaryType:" << pLaneInfo->rightBoundaryType << std::endl;
			std::cout << "pLaneInfo.leftBoundaryColor:" << pLaneInfo->leftBoundaryColor << std::endl;
			std::cout << "pLaneInfo.rightBoundaryColor:" << pLaneInfo->rightBoundaryColor << std::endl;
			std::cout << "pLaneInfo.leftBoundaryWidth:" << pLaneInfo->leftBoundaryWidth << std::endl;
			std::cout << "pLaneInfo.rightBoundaryWidth:" << pLaneInfo->rightBoundaryWidth << std::endl;
			std::cout << "pLaneInfo.leftBoundaryPoints:" << pLaneInfo->leftBoundaryPoints << std::endl;
			std::cout << "pLaneInfo.rightBoundaryPoints:" << pLaneInfo->rightBoundaryPoints << std::endl;
			std::cout << "pLaneInfo.centerBoundaryPoints:" << pLaneInfo->centerBoundaryPoints << std::endl;
			std::cout << "pLaneInfo.laneLeftParameter:" << pLaneInfo->laneLeftParameter.C0 << std::endl;
			std::cout << "pLaneInfo.leftBoundaryPoints:" << pLaneInfo->laneRightParameter.C0 << std::endl;
			std::cout << "pLaneInfo.leftBoundaryPoints:" << pLaneInfo->laneCenterParameter.C0 << std::endl;

			std::cout << "--------------------------------------------------" << std::endl;
			//break;
		}
		else
		{
			whileNumber++;
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
		}

		if (whileNumber > 100) {
			std::cout << "already try 100 times" << std::endl;
			GTEST_FATAL_FAILURE_("api return false");
		}
	}
}

TEST_F(GlobalSMTest, SetNetPose) {
	std::unique_ptr<SimOne_Data_Gps> Gps = std::make_unique<SimOne_Data_Gps>();
	bool resultGps = SimOneSM::GetGps(0, Gps.get());
	std::unique_ptr<SimOne_Data_Pose_Control> Pose_Control = std::make_unique<SimOne_Data_Pose_Control>();
	float initPosX = Gps->posX;
	Pose_Control->posX = initPosX + 10;
	bool resultPose = SimOneSM::SetPose(0, Pose_Control.get());
	ASSERT_TRUE(resultPose);

}


int main(int argc, char* argv[]) {
	//testing::GTEST_FLAG(output) = "xml:";
	testing::GTEST_FLAG(filter) = "GlobalSMTest.GetStreamingPointCloud";
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
