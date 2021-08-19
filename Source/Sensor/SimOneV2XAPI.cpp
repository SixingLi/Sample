#ifndef WITHOUT_V2X
#include "Service/SimOneNetService.hpp"
#include "Service/SimOneIOStruct.h"
#include "SimOneV2XAPI.h"
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
	SIMONE_NET_API bool SimOneAPI::GetV2XInfo(int mainVehicleId, const char* sensorId, int infoType, SimOne_Data_V2XNFS *pDetections) {
		ETaskCommandId commandId = (ETaskCommandId)(ETaskCommandId_V2XNFSRawBSM + infoType - 1);
		int sensorType = Bridge::ESensorType_V2XNFS;
		string key = std::to_string(mainVehicleId).append("_").append(sensorId).append("_").append(std::to_string(infoType));
		return SimOneAPIService::GetInstance()->GetTaskData(key, sensorType, (int)commandId, (void *)pDetections);
	}

	SIMONE_NET_API bool SimOneAPI::SetV2XInfoUpdateCB(void(*cb)(int mainVehicleId, const char* sensorId, SimOne_Data_V2XNFS *pDetections)) {
		return SimOneAPIService::GetInstance()->SetV2XInfoUpdateCB(cb);
	}
#ifdef __cplusplus
}
#endif

#endif // !WITHOUT_SENSOR