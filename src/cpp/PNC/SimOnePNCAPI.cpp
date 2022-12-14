#ifndef WITHOUT_PNC
#include "Service/SimOneNetService.hpp"
#include "Service/SimOneIOStruct.h"
#include "SimOnePNCAPI.h"
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

	SIMONE_API bool SimOneAPI::RegisterVehicleState(const char* mainVehicleId, ESimOne_Data_Vehicle_State *pStateIndics, int size)
	{
		int mainVehId = SimOneAPIService::string2Int(mainVehicleId);
		return SimOneAPIService::GetInstance()->RegisterSimOneVehicleState(mainVehId, pStateIndics, size);
	}

	SIMONE_API bool SimOneAPI::GetVehicleState(const char* mainVehicleId, SimOne_Data_Vehicle_Extra* pVehExtraState)
	{
		int mainVehId = SimOneAPIService::string2Int(mainVehicleId);
		return SimOneAPIService::GetInstance()->GetSimOneVehicleState(mainVehId, pVehExtraState);
	}

	SIMONE_API bool SimOneAPI::SetPose(const char* mainVehicleId, SimOne_Data_Pose_Control *pPose) {
		int mainVehId=SimOneAPIService::string2Int(mainVehicleId);
		return SimOneAPIService::GetInstance()->sendVehicleControlPosReq(mainVehId, pPose);
	}

	SIMONE_API bool SimOneAPI::SetDrive(const char* mainVehicleId, SimOne_Data_Control *pControl) {
		int mainVehId = SimOneAPIService::string2Int(mainVehicleId);
		return SimOneAPIService::GetInstance()->sendVehicleControlReq(mainVehId, pControl, gDriverNameArray[mainVehId]);
	}

	SIMONE_API bool SimOneAPI::SetDriveMode(const char* mainVehicleId, ESimOne_Drive_Mode driveMode) {
		int mainVehId = SimOneAPIService::string2Int(mainVehicleId);
		return SimOneAPIService::GetInstance()->sendVehicleDriveModeReq(mainVehId, driveMode);
	}

	SIMONE_API bool SimOneAPI::SetTrajectory(const char* mainVehicleId, SimOne_Data_Trajectory *Trajectory) {
		int mainVehId = SimOneAPIService::string2Int(mainVehicleId);
		return SimOneAPIService::GetInstance()->sendVehicleTrajectory(mainVehId, Trajectory, gDriverNameArray[mainVehId]);
	}

	SIMONE_API bool SimOneAPI::SetDriveTrajectory(const char* mainVehicleId, SimOne_Data_Control_Trajectory *pControlTrajectory) {
		int mainVehId = SimOneAPIService::string2Int(mainVehicleId);
		return SimOneAPIService::GetInstance()->sendVehicleTrajectoryControlReq(mainVehId, pControlTrajectory, gDriverNameArray[mainVehId]);
	}

	SIMONE_API void SimOneAPI::SetDriverName(const char* mainVehicleId, const char* name) {
		int mainVehId = SimOneAPIService::string2Int(mainVehicleId);
		if (mainVehId < MAX_MAINVEHICLE_NUM) {
			memcpy(gDriverNameArray[mainVehId], name, MAX_DRIVER_NAME_LEN);
		}
	}

	SIMONE_API bool SimOneAPI::SetVehicleEvent(const char* mainVehicleId, SimOne_Data_Vehicle_EventInfo *pEvent) {
		int mainVehId = SimOneAPIService::string2Int(mainVehicleId);
		return SimOneAPIService::GetInstance()->sendVehicleEventInfoReq(mainVehId, pEvent);
	}

	SIMONE_API bool SimOneAPI::GetDriverStatus(const char* mainVehicleId, SimOne_Data_Driver_Status* pDriverStatus) {
		int mainVehId = SimOneAPIService::string2Int(mainVehicleId);
		return SimOneAPIService::GetInstance()->GetDriverStatus(mainVehId, pDriverStatus);
	}

	SIMONE_API bool SimOneAPI::GetControlMode(const char* mainVehicleId, SimOne_Data_Control_Mode* pControlMode) {
		int mainVehId = SimOneAPIService::string2Int(mainVehicleId);
		return SimOneAPIService::GetInstance()->GetControlMode(mainVehId, pControlMode);
	}

	SIMONE_API bool SimOneAPI::GetDriverControl(const char* mainVehicleId, SimOne_Data_Control* pControl)
	{
		int mainVehId = SimOneAPIService::string2Int(mainVehicleId);
		return SimOneAPIService::GetInstance()->GetDriverControl(mainVehId, pControl);
	}

	SIMONE_API bool SimOneAPI::SetSignalLights(const char* mainVehicleId, SimOne_Data_Signal_Lights *pSignalLights)
	{
		int mainVehId = SimOneAPIService::string2Int(mainVehicleId);
		return SimOneAPIService::GetInstance()->sendVehicleSignalLights(mainVehId, pSignalLights);
	}

	SIMONE_API bool SimOneAPI::GetWayPoints(const char* mainVehicleId, SimOne_Data_WayPoints* pWayPoints)
	{
		int mainVehId = SimOneAPIService::string2Int(mainVehicleId);
		return SimOneAPIService::GetInstance()->GetWayPoints(mainVehId, pWayPoints);
	}

	SIMONE_API bool SimOneAPI::SetScenarioEventCB(void(*cb)(const char* source, const char* target, const char* type, const char* content))
	{
		return SimOneAPIService::GetInstance()->SetScenarioEventCB(cb);
	}

#ifdef __cplusplus
}
#endif

#endif // !WITHOUT_PNC

