#include "test_bench.h"

dumper tester::dbg_data;

tester::tester(const char* mv_id):mainVehicleId(mv_id){}

tester::~tester(){}

void tester::Test_InitSimOneAPI(bool isJoinTimeLoop, const char *serverIP)
{
	// const char *mainVehicleId = "0", bool isFrameSync = false, const char *serverIP = "127.0.0.1", int port = 23789, void (*startCase)() = 0, void (*endCase)() = 0, int registerNodeId = 0;
	SimOneAPI::InitSimOneAPI(mainVehicleId.c_str(), isJoinTimeLoop, serverIP);
}

void tester::Test_TerminateSimOneAPI()
{
	SimOneAPI::TerminateSimOneAPI();
}

void tester::Test_GetCaseInfo()
{
	std::unique_ptr<SimOne_Data_CaseInfo> mpCaseInfo = std::make_unique<SimOne_Data_CaseInfo>();
	if (!SimOneAPI::GetCaseInfo(mpCaseInfo.get()))
	{
		std::cout << "GetCaseInfo Failed!" << std::endl;
		return;
	}
	std::cout << "caseName: " << mpCaseInfo->caseName;
	std::cout << "caseId: " << mpCaseInfo->caseId;
	std::cout << "taskId: " << mpCaseInfo->taskId;
}

void tester::Test_GetCaseRunStatus()
{
	switch (SimOneAPI::GetCaseRunStatus())
	{
	case ESimOne_Case_Status::ESimOne_Case_Status_Unknown:
		std::cout << "ESimOne_Case_Status_Unknown" << std::endl;
		break;
	case ESimOne_Case_Status::ESimOne_Case_Status_Stop:
		std::cout << "ESimOne_Case_Status_Stop" << std::endl;
		break;
	case ESimOne_Case_Status::ESimOne_Case_Status_Running:
		std::cout << "ESimOne_Case_Status_Running" << std::endl;
		break;
	case ESimOne_Case_Status::ESimOne_Case_Status_Pause:
		std::cout << "ESimOne_Case_Status_Pause" << std::endl;
		break;
	default:
		std::cout << "Invalid CaseRunStatus!" << std::endl;
	}
}

void tester::Test_GetMainVehicleList()
{
	std::unique_ptr<SimOne_Data_MainVehicle_Info> MainVehicleListTest = std::make_unique<SimOne_Data_MainVehicle_Info>();
	if(!SimOneAPI::GetMainVehicleList(MainVehicleListTest.get()))
	{
		std::cout << "GetMainVehicleList Failed!" << std::endl;
		return;
	}
	std::cout << "size:" << MainVehicleListTest->size << std::endl;
	for (int i = 0; i < MainVehicleListTest->size; i++)
	{
		std::cout << "id:" << MainVehicleListTest->id_list[i] << std::endl;
		std::cout << "type:" << MainVehicleListTest->type_list[i] << std::endl;
	}
}

void tester::Test_GetMainVehicleStatus(bool IsCallBackMode)
{
	if (IsCallBackMode)
	{
		auto function = [](const char *mainVehicleId, SimOne_Data_MainVehicle_Status *pMainVehicleStatus) {
			std::cout << "mainVehicleId: " << pMainVehicleStatus->mainVehicleId << std::endl;
			std::cout << "mainVehicleStatus: " << pMainVehicleStatus->mainVehicleStatus << std::endl;
		};
		SimOneAPI::SetMainVehicleStatusUpdateCB(function);
	}
	else
	{
		std::unique_ptr<SimOne_Data_MainVehicle_Status> pMainVehicleStatus = std::make_unique<SimOne_Data_MainVehicle_Status>();
		if (!SimOneAPI::GetMainVehicleStatus(mainVehicleId.c_str(), pMainVehicleStatus.get()))
		{
			std::cout << "GetMainVehicleStatus Failed!" << std::endl;
			return;
		}
		std::cout << "mainVehicleId: " << pMainVehicleStatus->mainVehicleId << std::endl;
		std::cout << "mainVehicleStatus: " << pMainVehicleStatus->mainVehicleStatus << std::endl;
	}
}

void tester::Test_GetHDMapData()
{
	std::unique_ptr<SimOne_Data_Map> pHdmap = std::make_unique<SimOne_Data_Map>();
	if (!SimOneAPI::GetHDMapData(pHdmap.get()))
	{
		std::cout << "GetHDMapData Failed!" << std::endl;
		return;
	}
	std::cout << "openDrive: " << pHdmap->openDrive << std::endl;;
	std::cout << "openDriveUrl: " << pHdmap->openDriveUrl << std::endl;;
	std::cout << "opendriveMd5: " << pHdmap->opendriveMd5 << std::endl;
}

void tester::Test_GetSensorConfigurations()
{
	std::unique_ptr<SimOne_Data_SensorConfigurations> pConfigs = std::make_unique<SimOne_Data_SensorConfigurations>();
	while (true)
	{
		bool flag = SimOneAPI::GetSensorConfigurations(mainVehicleId.c_str(), pConfigs.get());
		if (flag)
		{
			dbg_data.dump_sensor_configurations(mainVehicleId.c_str(), pConfigs.get());
		}
		else
		{
			std::cout << "there is no sensor config in current vehicle!!!" << std::endl;
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(30));
	}
}


void tester::Test_GetMessage()
{
	auto function = [](const char* source, const char* target, const char* type, const char* content) {
		std::cout << "source:" << std::string(source) << ", target:" << std::string(target) << ", type:" << std::string(type) << ", content:" << std::string(content) << std::endl;
	};
	SimOneAPI::SetScenarioEventCB(function);
}


void tester::Test_GetVersion()
{
	std::cout << "Version:" << SimOneAPI::GetVersion() << std::endl;;
}

void tester::Test_SetEnvironment()
{
	SimOne_Data_Environment Environment; // std::unique_ptr<SimOne_Data_Environment> pEnvironment = std::make_unique<SimOne_Data_Environment>();
	Environment.timeOfDay = 1000;
	Environment.heightAngle = 90;
	Environment.directionalLight = 0.5f;
	Environment.ambientLight = 0.5f;
	Environment.artificialLight = 0.5f;
	Environment.cloudDensity = 0.5f;
	Environment.fogDensity = 0.5f;
	Environment.rainDensity = 0.5f;
	Environment.snowDensity = 0.5f;
	Environment.groundHumidityLevel = 0.5f;
	Environment.groundDirtyLevel = 0.5f;
       if(!SimOneAPI::SetEnvironment(&Environment))
	{
        	std::cout << "SetEnvironment Failed!" << std::endl; 
	}
	else
	{
		dbg_data.dump_environment(&Environment);
	}
}

void tester::Test_GetEnvironment()
{
	std::unique_ptr<SimOne_Data_Environment> pEnvironment = std::make_unique<SimOne_Data_Environment>();
	bool flag = SimOneAPI::GetEnvironment(pEnvironment.get());
	if (flag)
	{
		dbg_data.dump_environment(pEnvironment.get());
	}
	else
	{
		std::cout << "GetEnvironment Failed!" << std::endl;
		return;
	}
}

void tester::Test_GetHdMapData() {
	std::unique_ptr<SimOne_Data_Map> hdMap = std::make_unique<SimOne_Data_Map>();
	while (true) {
		if (SimOneAPI::GetHDMapData(hdMap.get())) {
			std::cout << "hdMap.OpenDrive:" << hdMap->openDrive<< std::endl;
			std::this_thread::sleep_for(std::chrono::milliseconds(30));
		}
	}
}

void tester::Test_GetGroundTruth(bool IsCallBackMode)
{
	if (IsCallBackMode) {
		auto function = [](const char* mainVehicleId, SimOne_Data_Obstacle *pObstacle) {
			dbg_data.dump_ground_truth(mainVehicleId, pObstacle);
		};
		SimOneAPI::SetGroundTruthUpdateCB(function);
	}
	else {
		std::unique_ptr<SimOne_Data_Obstacle> pDetections = std::make_unique<SimOne_Data_Obstacle>();
		int lastFrame = 0;
		while (true) {
			bool flag = SimOneAPI::GetGroundTruth(mainVehicleId.c_str(),  pDetections.get());
			if (flag && pDetections->frame != lastFrame)
			{
				lastFrame  = pDetections->frame;
				dbg_data.dump_ground_truth(mainVehicleId.c_str(), pDetections.get());
			}
			if (!flag)
			{
				std::cout << "GetGroundTruth Failed!" << std::endl;
			}
			std::this_thread::sleep_for(std::chrono::milliseconds(1));
		}
	}
}

void tester::Test_RadarDetection(bool IsCallBackMode)
{
	if (IsCallBackMode) {
		auto function = [](const char* mainVehicleId, const char* sensorId, SimOne_Data_RadarDetection *pDetections)
		{
			dbg_data.dump_radar_detection(mainVehicleId, sensorId, pDetections);
		};
		SimOneAPI::SetRadarDetectionsUpdateCB(function);
	}
	else
	{
		std::unique_ptr<SimOne_Data_RadarDetection> pDetections = std::make_unique<SimOne_Data_RadarDetection>();
		int lastFrame = 0;
		while (true)
		{
			bool flag = SimOneAPI::GetRadarDetections(mainVehicleId.c_str(), "objectBasedRadar1", pDetections.get());
			if (flag && pDetections->frame != lastFrame)
			{
				lastFrame  = pDetections->frame;
				dbg_data.dump_radar_detection(mainVehicleId.c_str(), "objectBasedRadar1", pDetections.get());
			}
			if (!flag)
			{
				std::cout << "GetRadarDetections Failed!" << std::endl;
			}
			std::this_thread::sleep_for(std::chrono::milliseconds(30));
		}
	}
}

void tester::Test_GetSensorDetections(bool IsCallBackMode)
{
	if (IsCallBackMode)
	{
		auto function = [](const char* MainVehicleID, const char* sensorId, SimOne_Data_SensorDetections* pGroundtruth)
		{
			dbg_data.dump_sensor_detections(MainVehicleID, sensorId, pGroundtruth);
		};
		SimOneAPI::SetSensorDetectionsUpdateCB(function);
	}
	else
	{
		std::unique_ptr<SimOne_Data_SensorDetections> pGroundtruth = std::make_unique<SimOne_Data_SensorDetections>();
		int lastFrame = 0;
		while (true)
		{
			// "sensorFusion1" "objectBasedCamera1" "objectBasedLidar1" "perfectPerception1"
			bool flag = SimOneAPI::GetSensorDetections(mainVehicleId.c_str(), "perfectPerception1", pGroundtruth.get());
			if (flag && pGroundtruth->frame != lastFrame)
			{
				lastFrame = pGroundtruth->frame;
				dbg_data.dump_sensor_detections(mainVehicleId.c_str(), "perfectPerception1", pGroundtruth.get());
			}
			if (!flag)
			{
				std::cout << "GetSensorDetections Failed!" << std::endl;
			}
			std::this_thread::sleep_for(std::chrono::milliseconds(10));
		}
	}
}

void tester::Test_SensorLaneInfo(bool IsCallBackMode)
{
	if (IsCallBackMode) {
		auto function = [](const char* mainVehicleId, const char* sensorId, SimOne_Data_LaneInfo *pDetections) {
			dbg_data.dump_sensor_laneInfo(mainVehicleId, sensorId, pDetections);
		};
		SimOneAPI::SetSensorLaneInfoCB(function);
	}
	else {
		std::unique_ptr<SimOne_Data_LaneInfo> pDetections = std::make_unique<SimOne_Data_LaneInfo>();
		int lastFrame = 0;
		while (true) {
			bool flag = SimOneAPI::GetSensorLaneInfo(mainVehicleId.c_str(), "sensorFusion1", pDetections.get());
			if (flag && pDetections->frame != lastFrame)
			{
				lastFrame = pDetections->frame;
				dbg_data.dump_sensor_laneInfo(mainVehicleId.c_str(), "sensorFusion1", pDetections.get());
			}
			if (!flag)
			{
				std::cout << "GetSensorLaneInfo Failed!" << std::endl;
			}
			std::this_thread::sleep_for(std::chrono::milliseconds(10));
		}
	}
}

void tester::Test_UltrasonicRadars(bool IsCallBackMode)
{
	if (IsCallBackMode) {
		auto function = [](const char* mainVehicleId, SimOne_Data_UltrasonicRadars* pUltrasonics)
		{
			dbg_data.dump_ultrasonic_radars(mainVehicleId, pUltrasonics);
		};
		SimOneAPI::SetUltrasonicRadarsCB(function);
	}
	else
	{
		std::unique_ptr<SimOne_Data_UltrasonicRadars> pDetections = std::make_unique<SimOne_Data_UltrasonicRadars>();
		int lastFrame = 0;
		while (true)
		{
			bool flag = SimOneAPI::GetUltrasonicRadars(mainVehicleId.c_str(), pDetections.get());
			if (flag && pDetections->frame != lastFrame)
			{
				lastFrame  = pDetections->frame;
				dbg_data.dump_ultrasonic_radars(mainVehicleId.c_str(), pDetections.get());
			}
			if (!flag)
			{
				std::cout << "GetUltrasonicRadars Failed!" << std::endl;
			}
			std::this_thread::sleep_for(std::chrono::milliseconds(10));
		}
	}
}

void tester::Test_UltrasonicRadar()
{
	std::unique_ptr<SimOne_Data_UltrasonicRadar> pDetections = std::make_unique<SimOne_Data_UltrasonicRadar>();
	int lastFrame = 0;
	while (true)
	{
		bool flag = SimOneAPI::GetUltrasonicRadar(mainVehicleId.c_str(), "ultrasonic1", pDetections.get());
		if (flag && pDetections->frame != lastFrame)
		{
			lastFrame  = pDetections->frame;
			dbg_data.dump_ultrasonic_radar(mainVehicleId.c_str(), "ultrasonic1",pDetections.get());
		}
		if (!flag)
		{
			std::cout << "GetUltrasonicRadar Failed!" << std::endl;
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
	}
}


void tester::Test_GPS(bool IsCallBackMode)
{
	if (IsCallBackMode) {
		auto function = []( const char* mainVehicleId, SimOne_Data_Gps *pGps){
				dbg_data.dump_gps(mainVehicleId, pGps);
		};
		 SimOneAPI::SetGpsUpdateCB(function);
	}
	else {
		std::unique_ptr<SimOne_Data_Gps> pGps = std::make_unique<SimOne_Data_Gps>();
		int lastFrame = 0;
		while(1)
		{
			bool flag = SimOneAPI::GetGps(mainVehicleId.c_str(), pGps.get());
			if (flag && pGps->frame != lastFrame)
			{
				lastFrame  = pGps->frame;
				dbg_data.dump_gps(mainVehicleId.c_str(), pGps.get());
			}
			if (!flag)
			{
				std::cout<<"Get GPS Fail"<< std::endl;
			}
			std::this_thread::sleep_for(std::chrono::milliseconds(10));
		}
	}
}

void tester::Test_V2XInfo(bool IsCallBackMode)
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
			if (SimOneAPI::GetV2XInfo(mainVehicleId.c_str(), "obu1", ESimOne_V2X_MessageFrame_PR::ESimOne_V2X_MessageFrame_PR_rsiFrame, pDetections.get())) {
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
						dbg_data.dump_traffic_light(mainVehicleId.c_str(), signal.id, pDetections.get());
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

void tester::Test_GetNearLanes()
{
	std::unique_ptr<SimOne_Data_Gps> pGps = std::make_unique<SimOne_Data_Gps>();
	if (!SimOneAPI::GetGps(mainVehicleId.c_str(), pGps.get()))
	{
		std::cout << "Get GPS Fail" << std::endl;
		return;
	}
	SSD::SimPoint3D pos(pGps->posX, pGps->posY, pGps->posZ);
	double distance = 3.0;
	SSD::SimStringVector nearLanes;
	if(!SimOneAPI::GetNearLanes(pos, distance, nearLanes))
	{
		std::cout << "GetNearLanes Failed!" << std::endl;
		return;
	}
	if (nearLanes.size() == 0)
	{
		std::cout << "nearLanes size: " << nearLanes.size() << std::endl;
		return;
	}
	for (int i=0; i<nearLanes.size(); i++)
	{
		std::cout << "nearLanes[" << i << "]: " << nearLanes[i].GetString();
	}
}

void tester::Test_SetVehicleEvent()
{
	SimOne_Data_Vehicle_EventInfo event; 
	// event.type =  ESimone_Vehicle_EventInfo_Type::ESimone_Vehicle_EventInfo_Type_Forward_Collision_Warning; // front_crash_warning
	// event.type = ESimone_Vehicle_EventInfo_Type::ESimone_Vehicle_EventInfo_Type_Backward_Collision_Warning; // back_crash_warning
	// event.type = ESimone_Vehicle_EventInfo_Type::ESimone_Vehicle_EventInfo_Type_Left_Turn_Decision; // turn_left
	// event.type = ESimone_Vehicle_EventInfo_Type::ESimone_Vehicle_EventInfo_Type_Left_Turn_Warning; // left_warning
	// event.type = ESimone_Vehicle_EventInfo_Type::ESimone_Vehicle_EventInfo_Type_Right_Turn_Decision; // turn_right
	// event.type = ESimone_Vehicle_EventInfo_Type::ESimone_Vehicle_EventInfo_Type_Right_Turn_Warning; // right_warning
	// event.type = ESimone_Vehicle_EventInfo_Type::ESimone_Vehicle_EventInfo_Type_Forward_Straight_Decision; // straight_through
	// event.type = ESimone_Vehicle_EventInfo_Type::ESimone_Vehicle_EventInfo_Type_Forward_Straight_Warning; // straight_warning
	// event.type = ESimone_Vehicle_EventInfo_Type::ESimone_Vehicle_EventInfo_Type_Over_Speed_Warning; // overspeeding_warning
	// event.type = ESimone_Vehicle_EventInfo_Type::ESimone_Vehicle_EventInfo_Type_Lane_Change_Decision; // lane_change
	// event.type = ESimone_Vehicle_EventInfo_Type::ESimone_Vehicle_EventInfo_Type_Lane_Change_Warning; // lane_change_warning
	// event.type = ESimone_Vehicle_EventInfo_Type::ESimone_Vehicle_EventInfo_Type_Overtake_Decision; // overtake
	// event.type = ESimone_Vehicle_EventInfo_Type::ESimone_Vehicle_EventInfo_Type_Emergency_Braking_Decision; // emergency_braking
	// event.type = ESimone_Vehicle_EventInfo_Type::ESimone_Vehicle_EventInfo_Type_Accelerate_Decision; // accelerate

	if (! SimOneAPI::SetVehicleEvent(mainVehicleId.c_str(), &event))
	{
		std::cout << "SetVehicleEvent Failed!" << std::endl;
		return;
	}
	std::cout << "SetVehicleEvent Done!" << std::endl;
}
