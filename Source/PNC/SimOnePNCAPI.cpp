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

	SIMONE_NET_API bool SimOneAPI::RegisterSimOneVehicleState(SimOne_Data_Vehicle_State *pStateIndics, int size)
	{
		return SimOneAPIService::GetInstance()->RegisterSimOneVehicleState(pStateIndics, size);
	}

	SIMONE_NET_API bool SimOneAPI::GetSimOneVehicleState(SimOne_Data_Vehicle_Extra* pVehExtraState)
	{
		return SimOneAPIService::GetInstance()->GetSimOneVehicleState(pVehExtraState);
	}

	SIMONE_NET_API bool SimOneAPI::SetPose(int mainVehicleId, SimOne_Data_Pose_Control *pPose) {
		return SimOneAPIService::GetInstance()->sendVehicleControlPosReq(mainVehicleId, pPose);
	}

	SIMONE_NET_API bool SimOneAPI::SetDrive(int mainVehicleId, SimOne_Data_Control *pControl) {
		return SimOneAPIService::GetInstance()->sendVehicleControlReq(mainVehicleId, pControl, gDriverNameArray[mainVehicleId]);
	}

	SIMONE_NET_API bool SimOneAPI::SetDriveTrajectory(int mainVehicleId, SimOne_Data_Control_Trajectory *pControlTrajectory) {
		return SimOneAPIService::GetInstance()->sendVehicleTrajectoryControlReq(mainVehicleId, pControlTrajectory, gDriverNameArray[mainVehicleId]);
	}

	SIMONE_NET_API void SimOneAPI::SetDriverName(int mainVehicleId, const char* name) {
		if (mainVehicleId < MAX_MAINVEHICLE_NUM) {
			memcpy(gDriverNameArray[mainVehicleId], name, MAX_DRIVER_NAME_LEN);
		}
	}

	SIMONE_NET_API bool SimOneAPI::SetVehicleEvent(int mainVehicleId, SimOne_Data_Vehicle_EventInfo *pEvent) {
		return SimOneAPIService::GetInstance()->sendVehicleEventInfoReq(mainVehicleId, pEvent);
	}

	SIMONE_NET_API bool SimOneAPI::SetTrajectory(int mainVehicleId, SimOne_Data_Trajectory *Trajectory) {
		return SimOneAPIService::GetInstance()->sendVehicleTrajectory(mainVehicleId, Trajectory, gDriverNameArray[mainVehicleId]);
	}

	SIMONE_NET_API bool SimOneAPI::GetDriverStatus(const int mainVehicleId, SimOne_Data_Driver_Status* pDriverStatus) {
		return SimOneAPIService::GetInstance()->GetDriverStatus(mainVehicleId, pDriverStatus);
	}

	SIMONE_NET_API bool SimOneAPI::GetDriverControl(const int mainVehicleId, SimOne_Data_Control* pControl)
	{
		return SimOneAPIService::GetInstance()->GetDriverControl(mainVehicleId, pControl);
	}

	SIMONE_NET_API bool SimOneAPI::SetSignalLights(const int mainVehicleId, SimOne_Data_Signal_Lights *pSignalLights)
	{
		return SimOneAPIService::GetInstance()->sendVehicleSignalLights(mainVehicleId, pSignalLights);
	}

	SIMONE_NET_API bool SimOneAPI::GetWayPoints(SimOne_Data_WayPoints* pWayPoints)
	{
		return SimOneAPIService::GetInstance()->GetWayPoints(pWayPoints);
	}

	SIMONE_NET_API bool SimOneAPI::SetScenarioEventCB(void(*cb)(int mainVehicleId, const char* event, const char* data))
	{
		return SimOneAPIService::GetInstance()->SetScenarioEventCB(cb);
	}

#ifdef __cplusplus
}
#endif

#endif // !WITHOUT_PNC

