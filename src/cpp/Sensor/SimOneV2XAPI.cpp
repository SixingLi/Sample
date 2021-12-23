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
	SIMONE_API bool SimOneAPI::GetV2XInfo(const char* mainVehicleId, const char* sensorId, int infoType, SimOne_Data_V2XNFS *pDetections) {
		//cout << "mainVehicleId:"<< mainVehicleId<<",sensorId:"<< sensorId<<",infoType:"<< infoType << endl;
		ETaskCommandId commandId = (ETaskCommandId)(ETaskCommandId_V2XNFSRawBSM + infoType - 1);
		int sensorType = Bridge::ESensorType_V2XNFS;
		string temp = mainVehicleId;
		string key = temp.append("_").append(sensorId).append("_").append(std::to_string(infoType));
		return SimOneAPIService::GetInstance()->GetTaskData(key, sensorType, (int)commandId, (void *)pDetections);
	}

	SIMONE_API bool SimOneAPI::SetV2XInfoUpdateCB(void(*cb)(const char* mainVehicleId, const char* sensorId, SimOne_Data_V2XNFS *pDetections)) {
		return SimOneAPIService::GetInstance()->SetV2XInfoUpdateCB(cb);
	}
#ifdef __cplusplus
}
#endif

#endif // !WITHOUT_SENSOR