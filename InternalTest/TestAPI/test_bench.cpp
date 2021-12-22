#include "test_bench.h"

tester::tester(const char* mv_id):mainVehicleId(mv_id){}

tester::~tester(){}

void tester::Test_InitSimOneAPI(bool isJoinTimeLoop, const char *serverIP)
{
	// const char *mainVehicleId = "0", bool isFrameSync = false, const char *serverIP = "127.0.0.1", int port = 23789, void (*startCase)() = 0, void (*endCase)() = 0, int registerNodeId = 0;
	SimOneAPI::InitSimOneAPI(mainVehicleId.c_str(), isJoinTimeLoop, serverIP);
}

void tester::Test_GetSensorConfigurations()
{
	while (true) {
		std::unique_ptr<SimOne_Data_SensorConfigurations> pConfigs = std::make_unique<SimOne_Data_SensorConfigurations>();
		if (SimOneAPI::GetSensorConfigurations(mainVehicleId.c_str(), pConfigs.get())) {
			for (int i = 0; i < pConfigs->dataSize; i++) {

				std::cout << "mainVehicleId:" << mainVehicleId << ", pConfigs->data[i].sensorId:" << pConfigs->data[i].sensorId << ", pConfigs->data[i].sensorType:" << pConfigs->data[i].sensorType << std::endl;//The Lane's leftLane ID 
			}
		}
		else {
			std::cout << "there is no sensor config in current vehicle!!!" << std::endl;//The Lane's leftLane ID 
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(30));
	}
}

void tester::Test_GetVersion()
{
	std::cout << "Version:" << SimOneAPI::GetVersion() << std::endl;;
}

void tester::Test_SetEnvironment()
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

void tester::Test_GetEnvironment()
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

void tester::Test_GetGroundTruth(bool IsCallBackMode)
{
	if (IsCallBackMode) {
		auto function = [](const char* mainVehicleId, SimOne_Data_Obstacle *pObstacle) {
			std::cout << "mainVehicleId:" << mainVehicleId << ", pDetections->frame:" << pObstacle->frame << ", pDetections->detectNum:" << pObstacle->obstacleSize << std::endl;//The Lane's leftLane ID 
			for (int i = 0; i < pObstacle->obstacleSize; i++) {
				std::cout << "obstacle.type:" << pObstacle->obstacle[i].type << std::endl;;
			}
		};
		SimOneAPI::SetGroundTruthUpdateCB(function);
	}
	else {
		std::unique_ptr<SimOne_Data_Obstacle> pDetections = std::make_unique<SimOne_Data_Obstacle>();
		while (true) {
			bool flag = SimOneAPI::GetGroundTruth(mainVehicleId.c_str(),  pDetections.get());
			if (flag) {
				std::cout << "mainVehicleId:" << mainVehicleId << ", pDetections->frame:" << pDetections->frame << ", pDetections->detectNum:" << pDetections->obstacleSize << std::endl;//The Lane's leftLane ID 
				for (int i = 0; i < pDetections->obstacleSize; i++) {
					std::cout << "obstacle.type:" << pDetections->obstacle[i].type << std::endl;;
				}
			}
			std::this_thread::sleep_for(std::chrono::milliseconds(30));
		}
	}
}

void tester::Test_RadarDetection(bool IsCallBackMode)
{
	if (IsCallBackMode) {
		auto function = [](const char* mainVehicleId, const char* sensorId, SimOne_Data_RadarDetection *pDetections) {
			std::cout <<"mainVehicleId:"<< mainVehicleId<<", pDetections->frame:"<<pDetections->frame << ", pDetections->detectNum:"<<pDetections->detectNum <<std::endl;//The Lane's leftLane ID 
			for (auto i = 0; i < pDetections->detectNum;i++) {
				std::cout <<" test.rcsdb:" << pDetections->detections[i].rcsdb << "," <<" test.rcsdb:"<< pDetections->detections[i].probability << std::endl;//The Lane's leftLane ID 
			}
		};
		SimOneAPI::SetRadarDetectionsUpdateCB(function);
	}
	else {
		std::unique_ptr<SimOne_Data_RadarDetection> pDetections = std::make_unique<SimOne_Data_RadarDetection>();
		while (true) {
			bool flag = SimOneAPI::GetRadarDetections(mainVehicleId.c_str(), "objectBasedRadar1", pDetections.get());
			if (flag) {
				std::cout <<"mainVehicleId:"<< mainVehicleId.c_str() <<", pDetections->frame:"<<pDetections->frame << ", pDetections->detectNum:"<<pDetections->detectNum <<std::endl;//The Lane's leftLane ID 
				for (int i = 0; i < pDetections->detectNum; i++)
				{
					std::cout << "detections.range:" << pDetections->detections[i].range <<",pDetections->detections[i].azimuth:"<< pDetections->detections[i].azimuth <<std::endl;
				}
			}
			std::this_thread::sleep_for(std::chrono::milliseconds(30));
		}
	}
}

void tester::Test_SensorSensorDetection(bool IsCallBackMode)
{
	if (IsCallBackMode) {
		auto function = [](const char* MainVehicleID, const char* sensorId, SimOne_Data_SensorDetections *pGroundtruth) {
			printf("hostVehicle:%s frame:%d objectSize:%d\n", MainVehicleID, pGroundtruth->frame, pGroundtruth->objectSize);
			//std::cout <<"hostVehicle:"<<*mainVehicleId<<"frame:"<< pGroundtruth->frame << "," << pGroundtruth->objectSize << std::endl;//The Lane's leftLane ID 
		};
		SimOneAPI::SetSensorDetectionsUpdateCB(function);
	}
	else {
		std::unique_ptr<SimOne_Data_SensorDetections> pGroundtruth = std::make_unique<SimOne_Data_SensorDetections>();
		while (true) {
			SimOneAPI::GetSensorDetections(mainVehicleId.c_str(), "sensorFusion1", pGroundtruth.get());
			std::cout << pGroundtruth->frame << "," << pGroundtruth->objectSize << std::endl;//The Lane's leftLane ID 
			std::this_thread::sleep_for(std::chrono::milliseconds(10));
		}
	}
}

void tester::Test_SensorLaneInfo(bool IsCallBackMode)
{
	if (IsCallBackMode) {
		auto function = [](const char* mainVehicleId, const char* sensorId, SimOne_Data_LaneInfo *pDetections) {
			std::cout << pDetections->frame << "," << pDetections->laneType << "," << pDetections->laneLeftID << "," << pDetections->laneRightID << "," << pDetections->laneRightID<<std::endl;//The Lane's leftLane ID 
		};
		SimOneAPI::SetSensorLaneInfoCB(function);
	}
	else {
		std::unique_ptr<SimOne_Data_LaneInfo> pDetections = std::make_unique<SimOne_Data_LaneInfo>();
		while (true) {
			SimOneAPI::GetSensorLaneInfo(mainVehicleId.c_str(), "sensorFusion1", pDetections.get());
			std::cout << pDetections->frame <<","<< pDetections->laneType<<","<< pDetections->laneLeftID<<","<< pDetections->laneLeftID <<"," << pDetections->laneRightID<<std::endl;//The Lane's leftLane ID  
			std::this_thread::sleep_for(std::chrono::milliseconds(10));
		}
	}
}

void tester::Test_UltrasonicRadars(bool IsCallBackMode)
{
	if (IsCallBackMode) {
		auto function = [](const char * mainVehicleId, SimOne_Data_UltrasonicRadars *pUltrasonics) {
			if(pUltrasonics->ultrasonicRadars[0].obstacleNum>0)
				std::cout << pUltrasonics->frame << "," << pUltrasonics->ultrasonicRadarNum << "," << pUltrasonics->timestamp << "," << pUltrasonics->ultrasonicRadars[0].obstacleDetections[0].obstacleRanges << "," << pUltrasonics->ultrasonicRadars[0].obstacleDetections[0].x << "," << pUltrasonics->ultrasonicRadars[0].obstacleDetections[0].y << std::endl;
		};
		SimOneAPI::SetUltrasonicRadarsCB(function);
	}
	else {
		std::unique_ptr<SimOne_Data_UltrasonicRadars> pDetections = std::make_unique<SimOne_Data_UltrasonicRadars>();
		while (true) {
			SimOneAPI::GetUltrasonicRadars(mainVehicleId.c_str(), pDetections.get());
			for (int i = 0; i < pDetections->ultrasonicRadarNum; i++) {
				std::cout << pDetections->frame << ", " << pDetections->ultrasonicRadars[i].obstacleNum << ", " << pDetections->ultrasonicRadars[i].obstacleDetections[i].obstacleRanges << ", " << pDetections-> ultrasonicRadars[i].obstacleDetections[i].x << ", " << pDetections->ultrasonicRadars[i].obstacleDetections[i].y << std::endl;
			}
			std::this_thread::sleep_for(std::chrono::milliseconds(10));
		}
	}
}

void tester::Test_UltrasonicRadar()
{
	std::unique_ptr<SimOne_Data_UltrasonicRadar> pDetections = std::make_unique<SimOne_Data_UltrasonicRadar>();
	while (true) {
		SimOneAPI::GetUltrasonicRadar(mainVehicleId.c_str(), "ultrasonic1", pDetections.get());
		for (int i = 0; i < pDetections->obstacleNum; i++) {
			std::cout << pDetections->frame << ", " << pDetections->obstacleDetections[i].x << ", " << pDetections->obstacleDetections[i].obstacleRanges << ", " << pDetections->obstacleDetections[i].x << ", " << pDetections->obstacleDetections[i].y << std::endl;
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
	}
}

void tester::Test_GPS(bool IsCallBackMode)
{
	if (IsCallBackMode) {
		auto function = []( const char* mainVehicleId, SimOne_Data_Gps *pGps){
				std::cout<<"pGps->frame:"<< pGps->frame<<", pGps->posX:"<<pGps->posX<<",pGps->posY"<<pGps->posY<< std::endl;
		};
		 SimOneAPI::SetGpsUpdateCB(function);
	}
	else {
		std::unique_ptr<SimOne_Data_Gps> pGps = std::make_unique<SimOne_Data_Gps>();
		while(1)
		{
			if(SimOneAPI::GetGps(mainVehicleId.c_str(), pGps.get())){
				std::cout<<"pGps->posX:"<<pGps->posX<<",pGps->posY"<<pGps->posY<< std::endl;					
			}else{

				std::cout<<"Get GPS Fail"<< std::endl;
			}
			std::this_thread::sleep_for(std::chrono::milliseconds(10));
		}
	}
}

void tester::Test_V2X(bool IsCallBackMode)
{
	if (IsCallBackMode) {
		auto function = [](const char* mainVehicleId, const char* sensorId, SimOne_Data_V2XNFS *pDetections) {
			std::cout << "########### sensorId:" << sensorId<<" SetV2XInfoUpdateCB strlen= "<<pDetections->V2XMsgFrameSize <<"  "<<pDetections->MsgFrameData << std::endl;
		};
		SimOneAPI::SetV2XInfoUpdateCB(function);
	}
	else {
		std::unique_ptr<SimOne_Data_V2XNFS> pDetections = std::make_unique<SimOne_Data_V2XNFS>();
		while (1) {
			if (SimOneAPI::GetV2XInfo(mainVehicleId.c_str(), "obu1", ESimOne_V2X_MessageFrame_PR::ESimOne_V2X_MessageFrame_PR_bsmFrame, pDetections.get())) {
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

bool tester::Test_HDMap_ALL(const std::vector<std::string> &apiNames)
{
	int success_count = 0;
	if (SimOneAPI::LoadHDMap(3)) {
		for (auto apiName : apiNames) {
			std::cout << "get hdmap data success" << std::endl;
			if (apiName == "GetTrafficSignList") {
				SSD::SimVector<HDMapStandalone::MSignal> list;
				SimOneAPI::GetTrafficSignList(list);
				int listSize = list.size();
				if (listSize > 0) {
					std::cout << ">>>>>>>>>>>>>>>>>>>>>  GetTrafficSignList Size = " << listSize << std::endl;
					success_count++;
				}
			}
			else if (apiName == "GetTrafficLightList") {
				SSD::SimVector<HDMapStandalone::MSignal> list;
				SimOneAPI::GetTrafficLightList(list);
				int listSize = list.size();
				if (listSize > 0) {
					std::cout << ">>>>>>>>>>>>>>>>>>>>>  GetTrafficLightList Size = " << listSize << std::endl;
					success_count++;
				}

				for (auto signal : list){
					std::unique_ptr<SimOne_Data_TrafficLight> pDetections = std::make_unique<SimOne_Data_TrafficLight>();
					if(SimOneAPI::GetTrafficLight(mainVehicleId.c_str(), signal.id, pDetections.get()))
						printf(">>>>>>>>>>>>>>>>>>>>>  SoGetTrafficLights opendriveId = %ld, trafficLigtData.countDown = %d, trafficLigtData.status = %d\n", signal.id, pDetections->countDown, pDetections->status);

				}
			}
			else if (apiName == "GetCrossHatchList") {
				SSD::SimString id;
				SSD::SimVector<HDMapStandalone::MObject> crossHatchList;
				double s, t, s_toCenterLine, t_toCenterLine;
				std::unique_ptr<SimOne_Data_Gps> pDetections = std::make_unique<SimOne_Data_Gps>();
				SimOneAPI::GetGps(mainVehicleId.c_str(), pDetections.get());
				SSD::SimPoint3D pos = { pDetections->posX,pDetections->posY ,pDetections->posZ };
				SimOneAPI::GetNearMostLane(pos, id, s, t, s_toCenterLine, t_toCenterLine);
				SimOneAPI::GetCrossHatchList(id, crossHatchList);
				int listSize = crossHatchList.size();
				if (listSize > 0) {
					std::cout << ">>>>>>>>>>>>>>>>>>>>>  getCrossHatchList Size = " << listSize << std::endl;
					success_count++;
				}
			}
			else if (apiName == "GetLaneLink") {
				SSD::SimString id;
				HDMapStandalone::MLaneLink laneLink;
				double s, t, s_toCenterLine, t_toCenterLine;
				std::unique_ptr<SimOne_Data_Gps> pDetections = std::make_unique<SimOne_Data_Gps>();
				SimOneAPI::GetGps(mainVehicleId.c_str(), pDetections.get());
				SSD::SimPoint3D pos = { pDetections->posX,pDetections->posY ,pDetections->posZ };
				SimOneAPI::GetNearMostLane(pos, id, s, t, s_toCenterLine, t_toCenterLine);
				SimOneAPI::GetLaneLink(id, laneLink);
				std::string laneID = id.GetString();
				int listSize = laneLink.successorLaneNameList.size();
				SSD::SimString leftLaneName = laneLink.leftNeighborLaneName;
				if (listSize > 0 && !leftLaneName.Empty()) {
					std::cout << ">>>>>>>>>>>>>>>>>>>>>  getLaneLink successorLaneNameList Size = " << listSize << std::endl;
					success_count++;
				}
			}
		}
	}
	else {
		std::cout << "#####################  LoadHDMap Data fail!!!" << std::endl;
		std::cout << "#####################  Test_HDMap_ALL fail!!!" << std::endl;
		return false;
	}
	if (success_count == apiNames.size()) {
		std::cout << "#####################  Test_HDMap_ALL success!!!" << std::endl;
		return true;
	}
	else {
		std::cout << "#####################  Test_HDMap_ALL fail!!!" << std::endl;
		return false;
	}
}