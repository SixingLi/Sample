#ifndef WITHOUT_SENSOR
#include "Service/SimOneNetService.hpp"
#include "Service/SimOneIOStruct.h"
#include "SimOneSensorAPI.h"
#include <iostream>
#include <thread>
#include <math.h>
#include <stdarg.h>
#pragma warning(disable:4244)


#ifdef __cplusplus
extern "C"
{
#endif
#define MAX_DRIVER_NAME_LEN 10
    static char gDriverNameArray[MAX_MAINVEHICLE_NUM][MAX_DRIVER_NAME_LEN];

	SIMONE_NET_API bool SimOneAPI::SetSensorObjectbasedDataEnable(bool enable) {
		return SimOneAPIService::GetInstance()->SetObjectbasedDataEnable(enable);
	}

	SIMONE_NET_API bool SimOneAPI::SetFrameCB(void(*FrameStart)(int frame), void(*FrameEnd)(int frame)) {
		SimOneAPIService::GetInstance()->SetFrameStartCB(FrameStart);
		SimOneAPIService::GetInstance()->SetFrameStartCB(FrameEnd);
		return true;
	}

	SIMONE_NET_API bool SimOneAPI::GetGps(int mainVehicleId, SimOne_Data_Gps *pGps) {
		return SimOneAPIService::GetInstance()->GetGps(mainVehicleId, pGps);
	}
	SIMONE_NET_API bool SimOneAPI::SetGpsUpdateCB(void(*cb)(int mainVehicleId, SimOne_Data_Gps *pGps)) {
		return SimOneAPIService::GetInstance()->SetGpsUpdateCB(cb);
	}

	SIMONE_NET_API bool SimOneAPI::GetTrafficLight(int mainVehicleId, int opendriveLightId, SimOne_Data_TrafficLight *pTrafficLight) {
		return SimOneAPIService::GetInstance()->GetTrafficLight(mainVehicleId, opendriveLightId, pTrafficLight);
	}

	SIMONE_NET_API bool SimOneAPI::GetGroundTruth(int mainVehicleId, SimOne_Data_Obstacle *pObstacle) {
		return SimOneAPIService::GetInstance()->GetObstacle(mainVehicleId, pObstacle);
	}
	SIMONE_NET_API bool SimOneAPI::SetGroundTruthUpdateCB(void(*cb)(int mainVehicleId, SimOne_Data_Obstacle *pObstacle)) {
		return SimOneAPIService::GetInstance()->SetObstacleUpdateCB(cb);
	}

	SIMONE_NET_API bool SimOneAPI::GetRadarDetections(const int mainVehicleId, const char* sensorId, SimOne_Data_RadarDetection *pDetections) {

		ETaskCommandId commandId = ETaskCommandId_ContiRadarObj;
		int sensorType = Bridge::ESensorType_MMWRadar;
		string key = std::to_string(mainVehicleId).append("_").append(sensorId);

		return SimOneAPIService::GetInstance()->GetTaskData(key, sensorType, (int)commandId, (void*)pDetections);
	}
	SIMONE_NET_API bool SimOneAPI::SetRadarDetectionsUpdateCB(void(*cb)(int mainVehicleId, const char* sensorId, SimOne_Data_RadarDetection *pDetections)) {
		return SimOneAPIService::GetInstance()->SetRadarDetectionsUpdateCB(cb);
	}
	SIMONE_NET_API bool SimOneAPI::GetUltrasonicRadar(int mainVehicleId, const char* sensorId, SimOne_Data_UltrasonicRadar *pUltrasonic) {
		ETaskCommandId commandId = ETaskCommandId_UltrasonicRadarObj;
		int sensorType = Bridge::ESensorType_AllUltrasonicRadar;
		string key = std::to_string(mainVehicleId);
		SimOne_Data_UltrasonicRadars* pTemp = new SimOne_Data_UltrasonicRadars();
		if (!SimOneAPIService::GetInstance()->GetTaskData(key, sensorType, commandId, (void*)pTemp)) {
			delete pTemp;
			return false;
		}
		int num_radars = pTemp->ultrasonicRadarNum;
		for (int i = 0; i < num_radars; i++)
		{
			if (pTemp->ultrasonicRadars[i].sensorId == sensorId)
			{
				memcpy(pUltrasonic, &pTemp->ultrasonicRadars[i], sizeof(SimOne_Data_UltrasonicRadar));
				break;
			}
		}
		delete pTemp;
		return true;
	}
	SIMONE_NET_API bool SimOneAPI::GetUltrasonicRadars(int mainVehicleId, SimOne_Data_UltrasonicRadars *pUltrasonics) {

		ETaskCommandId commandId = ETaskCommandId_UltrasonicRadarObj;
		int sensorType = Bridge::ESensorType_AllUltrasonicRadar;
		string key = std::to_string(mainVehicleId);

		return SimOneAPIService::GetInstance()->GetTaskData(key, sensorType, commandId, (void*)pUltrasonics);
	}
	SIMONE_NET_API bool SimOneAPI::SetUltrasonicRadarsCB(void(*cb)(int mainVehicleId, SimOne_Data_UltrasonicRadars *pUltrasonics)) {
		return SimOneAPIService::GetInstance()->SetUltrasonicRadarsCB(cb);
	}



	SIMONE_NET_API bool SimOneAPI::GetSensorDetections(int mainVehicleId, const char* sensorId, SimOne_Data_SensorDetections *pGroundtruth) {


		string key = std::to_string(mainVehicleId).append("_").append(sensorId);
		map<std::string, int>::iterator it =SimOneAPIService::GetInstance()->mSensorDataTypeMap.find(key);
		if (it == SimOneAPIService::GetInstance()->mSensorDataTypeMap.end())
		{
			logInfo("Get GetSensorDetections not found");
			return false;
		}

		int sensorType = it->second;

		CTaskSensorBase* pTask = TaskSensorManager::getInstance().FindTask(sensorType);
		if (!pTask) {
			return false;
		}

		int commandId = pTask->GetCommandIDFromObj();
		if (commandId < 0) {
			return false;
		}

		if (!pTask->GetData(key, (ETaskCommandId)commandId, pGroundtruth)) {
			return false;
		}

		return true;
	}
	SIMONE_NET_API bool SimOneAPI::SetSensorDetectionsUpdateCB(void(*cb)(int mainVehicleId, const char* sensorId, SimOne_Data_SensorDetections *pGroundtruth)) {
		return SimOneAPIService::GetInstance()->SetSensorDetectionsUpdateCB(cb);
	}

	SIMONE_NET_API bool SimOneAPI::GetSensorConfigurations(SimOne_Data_SensorConfigurations *pSensorConfigurations)
	{
		return SimOneAPIService::GetInstance()->GetSensorConfigurations(pSensorConfigurations);
	}

	SIMONE_NET_API bool SimOneAPI::GetEnvironment(SimOne_Data_Environment *pEnvironment)
	{
		return SimOneAPIService::GetInstance()->GetEnvironment(pEnvironment);
	}

	SIMONE_NET_API bool SimOneAPI::SetEnvironment(SimOne_Data_Environment *pEnvironment)
	{
		return SimOneAPIService::GetInstance()->SetEnvironment(pEnvironment);
	}

	SIMONE_NET_API bool SimOneAPI::GetSensorLaneInfo(int mainVehicleId, const char* sensorId, SimOne_Data_LaneInfo *pLaneInfo) {

		string key = std::to_string(mainVehicleId).append("_").append(sensorId);
		map<std::string, int>::iterator it = SimOneAPIService::GetInstance()->mSensorDataTypeMap.find(key);
		if (it == SimOneAPIService::GetInstance()->mSensorDataTypeMap.end())
		{
			logInfo("Get GetCameraSensorLaneInfo not found");
			return false;
		}

		int sensorType = it->second;

		CTaskSensorBase* pTask = TaskSensorManager::getInstance().FindTask(sensorType);
		if (!pTask) {
			return false;
		}

		int commandId = pTask->GetCommandIDFromLane();
		if (commandId < 0) {
			return false;
		}

		if (!pTask->GetData(key, (ETaskCommandId)commandId, pLaneInfo)) {
			return false;
		}
		return true;
	}

	SIMONE_NET_API bool SimOneAPI::SetSensorLaneInfoCB(void(*cb)(int mainVehicleId, const char* sensorId, SimOne_Data_LaneInfo *pLaneInfo)) {
		return SimOneAPIService::GetInstance()->SetSensorLaneInfoCB(cb);
	}

#ifdef __cplusplus
}
#endif

#endif // !WITHOUT_SENSOR