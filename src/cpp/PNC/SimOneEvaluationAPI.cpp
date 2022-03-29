#include "Service/SimOneNetService.hpp"
#include "Service/SimOneIOStruct.h"
#include "SimOneEvaluationAPI.h"
#include <iostream>
#include <thread>
#include <math.h>
#include <stdarg.h>
#pragma warning(disable:4244)

#ifdef __cplusplus
extern "C"
{
#endif

	SIMONE_API bool SimOneAPI::InitEvaluationService(
								const char* mainVehicleId, 
								const char *serviceIP/* = "127.0.0.1"*/,
								int port/* = 8078*/)
	{
		int mainVehId = SimOneAPIService::string2Int(mainVehicleId);
		return SimOneAPIService::GetInstance()->InitEvaluationService(mainVehId, serviceIP, port);
	}

	SIMONE_API bool SimOneAPI::AddEvaluationRecord(const char* mainVehicleId, const char * jsonString)
	{
		int mainVehId = SimOneAPIService::string2Int(mainVehicleId);
		cybertron::json jsonRecord;
		std::string strJsonString(jsonString);
		if (!JsonReader::loadString(jsonRecord, strJsonString)) {
			logError("Evaluation record format is wrong");
			return false;
		}
		if (!cybertron::JsonReader::hasValue(jsonRecord, "category") || !cybertron::JsonReader::hasValue(jsonRecord, "timestamp")) {
			logError("Evaluation record need contains properties for category and timestamp");
			return false;
		}
		return SimOneAPIService::GetInstance()->AddEvaluationRecord(mainVehId, jsonRecord);
	}

	SIMONE_API bool SimOneAPI::SetJudgeEventCB(void(*cb)(const char* mainVehicleId, SimOne_Data_JudgeEvent *judgeEventDetailInfo))
	{
		return SimOneAPIService::GetInstance()->SetJudgeEventCB(cb);
	}

#ifdef __cplusplus
}
#endif
