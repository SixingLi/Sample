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

	SIMONE_API bool SimOneAPI::GetGps(const char* mainVehicleId, SimOne_Data_Gps *pGps) {
		int mainVehId = SimOneAPIService::string2Int(mainVehicleId);
		return SimOneAPIService::GetInstance()->GetGps(mainVehId, pGps);
	}
	SIMONE_API bool SimOneAPI::SetGpsUpdateCB(void(*cb)(const char* mainVehicleId, SimOne_Data_Gps *pGps)) {
		return SimOneAPIService::GetInstance()->SetGpsUpdateCB(cb);
	}

	SIMONE_API bool SimOneAPI::GetTrafficLight(const char* mainVehicleId, int opendriveLightId, SimOne_Data_TrafficLight *pTrafficLight) {
		int mainVehId = SimOneAPIService::string2Int(mainVehicleId);
		return SimOneAPIService::GetInstance()->GetTrafficLight(mainVehId, opendriveLightId, pTrafficLight);
	}

	SIMONE_API bool SimOneAPI::GetGroundTruth(const char* mainVehicleId, SimOne_Data_Obstacle *pObstacle) {
		int mainVehId = SimOneAPIService::string2Int(mainVehicleId);
		return SimOneAPIService::GetInstance()->GetObstacle(mainVehId, pObstacle);
	}
	SIMONE_API bool SimOneAPI::SetGroundTruthUpdateCB(void(*cb)(const char* mainVehicleId, SimOne_Data_Obstacle *pObstacle)) {
		return SimOneAPIService::GetInstance()->SetObstacleUpdateCB(cb);
	}

	SIMONE_API bool SimOneAPI::GetRadarDetections(const char* mainVehicleId, const char* sensorId, SimOne_Data_RadarDetection *pDetections) {

		int mainVehId = SimOneAPIService::string2Int(mainVehicleId);
		ETaskCommandId commandId = ETaskCommandId_ContiRadarObj;
		int sensorType = Bridge::ESensorType_MMWRadar;
		string key = std::to_string(mainVehId).append("_").append(sensorId);

		return SimOneAPIService::GetInstance()->GetTaskData(key, sensorType, (int)commandId, (void*)pDetections);
	}
	SIMONE_API bool SimOneAPI::SetRadarDetectionsUpdateCB(void(*cb)(const char* mainVehicleId, const char* sensorId, SimOne_Data_RadarDetection *pDetections)) {
		return SimOneAPIService::GetInstance()->SetRadarDetectionsUpdateCB(cb);
	}
	SIMONE_API bool SimOneAPI::GetUltrasonicRadar(const char* mainVehicleId, const char* sensorId, SimOne_Data_UltrasonicRadar *pUltrasonic) {
		int mainVehId = SimOneAPIService::string2Int(mainVehicleId);
		ETaskCommandId commandId = ETaskCommandId_UltrasonicRadarObj;
		int sensorType = Bridge::ESensorType_AllUltrasonicRadar;
		string key = std::to_string(mainVehId);
		SimOne_Data_UltrasonicRadars* pTemp = new SimOne_Data_UltrasonicRadars();
		if (!SimOneAPIService::GetInstance()->GetTaskData(key, sensorType, commandId, (void*)pTemp)) {
			delete pTemp;
			return false;
		}
		int num_radars = pTemp->ultrasonicRadarNum;
		for (int i = 0; i < num_radars; i++)
		{
			if (strcmp(pTemp->ultrasonicRadars[i].sensorId,sensorId)==0)
			{
				memcpy(pUltrasonic, &pTemp->ultrasonicRadars[i], sizeof(SimOne_Data_UltrasonicRadar));
				break;
			}
		}
		delete pTemp;
		return true;
	}
	SIMONE_API bool SimOneAPI::GetUltrasonicRadars(const char* mainVehicleId, SimOne_Data_UltrasonicRadars *pUltrasonics) {

		int mainVehId = SimOneAPIService::string2Int(mainVehicleId);
		ETaskCommandId commandId = ETaskCommandId_UltrasonicRadarObj;
		int sensorType = Bridge::ESensorType_AllUltrasonicRadar;
		string key = std::to_string(mainVehId);

		return SimOneAPIService::GetInstance()->GetTaskData(key, sensorType, commandId, (void*)pUltrasonics);
	}
	SIMONE_API bool SimOneAPI::SetUltrasonicRadarsCB(void(*cb)(const char* mainVehicleId, SimOne_Data_UltrasonicRadars *pUltrasonics)) {
		return SimOneAPIService::GetInstance()->SetUltrasonicRadarsCB(cb);
	}



	SIMONE_API bool SimOneAPI::GetSensorDetections(const char* mainVehicleId, const char* sensorId, SimOne_Data_SensorDetections *pGroundtruth) {

		int mainVehId = SimOneAPIService::string2Int(mainVehicleId);
		string key = std::to_string(mainVehId).append("_").append(sensorId);
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
	SIMONE_API bool SimOneAPI::SetSensorDetectionsUpdateCB(void(*cb)(const char* mainVehicleId, const char* sensorId, SimOne_Data_SensorDetections *pGroundtruth)) {
		return SimOneAPIService::GetInstance()->SetSensorDetectionsUpdateCB(cb);
	}

	SIMONE_API bool SimOneAPI::GetSensorConfigurations(SimOne_Data_SensorConfigurations *pSensorConfigurations)
	{
		return SimOneAPIService::GetInstance()->GetSensorConfigurations(pSensorConfigurations);
	}

	SIMONE_API bool SimOneAPI::GetEnvironment(SimOne_Data_Environment *pEnvironment)
	{
		return SimOneAPIService::GetInstance()->GetEnvironment(pEnvironment);
	}

	SIMONE_API bool SimOneAPI::SetEnvironment(SimOne_Data_Environment *pEnvironment)
	{
		return SimOneAPIService::GetInstance()->SetEnvironment(pEnvironment);
	}

	SIMONE_API bool SimOneAPI::GetSensorLaneInfo(const char* mainVehicleId, const char* sensorId, SimOne_Data_LaneInfo *pLaneInfo) {

		int mainVehId = SimOneAPIService::string2Int(mainVehicleId);
		string key = std::to_string(mainVehId).append("_").append(sensorId);
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

	SIMONE_API bool SimOneAPI::SetSensorLaneInfoCB(void(*cb)(const char* mainVehicleId, const char* sensorId, SimOne_Data_LaneInfo *pLaneInfo)) {
		return SimOneAPIService::GetInstance()->SetSensorLaneInfoCB(cb);
	}

#ifdef __cplusplus
}
#endif

#endif // !WITHOUT_SENSOR