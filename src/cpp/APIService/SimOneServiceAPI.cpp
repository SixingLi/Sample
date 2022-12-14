#include "Service/SimOneNetService.hpp"
#include "Service/SimOneIOStruct.h"
#include "SimOneServiceAPI.h"
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

	SIMONE_API bool SimOneAPI::GetMainVehicleStatus(const char* mainVehicleId, SimOne_Data_MainVehicle_Status *pMainVehicleStatus) {
		int mainVehId = SimOneAPIService::string2Int(mainVehicleId);
		return SimOneAPIService::GetInstance()->GetMainVehicleStatus(mainVehId, pMainVehicleStatus);
	}

	SIMONE_API bool SimOneAPI::SetMainVehicleStatusUpdateCB(void(*cb)(const char* mainVehicleId, SimOne_Data_MainVehicle_Status *pMainVehicleStatus)) {
		SimOneAPIService::GetInstance()->SetMainVehicleStatusCB(cb);
		return true;
	}

	SIMONE_API bool SimOneAPI::InitSimOneAPI(const char* mainVehicleId, bool isFrameSync, const char *serverIP, int port, void(*startCase)(), void(*endCase)(), int registerNodeId)
	{
		char *serverAddrStr = getenv("SIMONE_SERVER_ADDR");
		char *serverPortStr = getenv("SIMONE_SERVER_PORT");

		const char *serverAddr = serverAddrStr ? serverAddrStr : serverIP;
		int serverPort = serverPortStr ? atoi(serverPortStr) : port;

		cout << "mainVehicleId:" << mainVehicleId << "isFrameSync:" << isFrameSync<<"serverIP:"<<serverAddr<<",port:"<<serverPort<< endl;
		SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Information,"mainVehicleId:%s,isFrameSync:%d", mainVehicleId, isFrameSync);
		SimOneAPIService::GetInstance()->setServerInfo(serverAddr, serverPort);
		int tryCount = 0;
		if (SimOneAPIService::GetInstance()->Start(startCase, endCase, registerNodeId)&& SimOneAPIService::GetInstance()->SimOneNodeReady()) {
			tryCount = 0;
			while (true)
			{
				if (tryCount > 10) {
					return false;
				}
				tryCount += 1;
				bool bRet = SimOneAPIService::GetInstance()->SubMainVehicle(mainVehicleId, isFrameSync);
				if (!bRet) {
					std::cout << "failed to subscribe main vehicle" << std::endl;
					std::this_thread::sleep_for(std::chrono::milliseconds(100));
					continue;
				}
				break;
			}

			SimOne_Data_MainVehicle_Status mainVehicleStatus;
			tryCount = 0;
			while (true)
			{
				if (tryCount > 10) {
					return false;
				}
				tryCount += 1;
				bool bRet = GetMainVehicleStatus(mainVehicleId, &mainVehicleStatus);
				if (!bRet) {
					std::this_thread::sleep_for(std::chrono::milliseconds(100));
				}
				if (std::string(mainVehicleStatus.mainVehicleId) == std::string(mainVehicleId) && mainVehicleStatus.mainVehicleStatus > 0) {
					std::cout << "mainVehicle is ready" << std::endl;
					break;
				}
				std::this_thread::sleep_for(std::chrono::milliseconds(100));
			}
			//SimOneAPIService::GetInstance()->SimOneNodeReady();
			tryCount = 0;
			while (true) {
				if (tryCount > 10) {
					return false;
				}
				tryCount += 1;
				if (GetCaseRunStatus() == ESimOne_Case_Status::ESimOne_Case_Status_Running) {
					break;
				}
				std::this_thread::sleep_for(std::chrono::milliseconds(100));
			}
			return true;
		}
		else {
			std::cout << "mainVehicle is not ready" << std::endl;
			return false;
		}
	}

	SIMONE_API bool SimOneAPI::TerminateSimOneAPI() {
		return SimOneAPIService::GetInstance()->Stop();
	}

	SIMONE_API  bool SimOneAPI::GetMainVehicleList(SimOne_Data_MainVehicle_Info *pMainVehicleInfo) {
		return SimOneAPIService::GetInstance()->GetMainVehicleList(pMainVehicleInfo);
	}
	SIMONE_API  const char* SimOneAPI::GetVersion() {
		return SimOneAPIService::GetInstance()->GetVersion();
	}

	SIMONE_API  bool SimOneAPI::SendRouteMessage(int length, void* pBuffer, int msgId, int toNodeId, ESimOne_Client_Type toNodeType) {
		return SimOneAPIService::GetInstance()->SendRouteMessage(length, pBuffer, msgId, toNodeId, toNodeType);
	}
	SIMONE_API bool SimOneAPI::ReceiveRouteMessageCB(void(*cb)(int fromId, ESimOne_Client_Type fromType, int length, const void* pBuffer, int commandId)) {
		return SimOneAPIService::GetInstance()->ReceiveRouteMessageCB(cb);
	}
	SIMONE_API ESimOne_Case_Status SimOneAPI::GetCaseRunStatus() {
		return (ESimOne_Case_Status)SimOneAPIService::GetInstance()->GetCaseStatus();
	}
	SIMONE_API bool SimOneAPI::GetCaseInfo(SimOne_Data_CaseInfo *pCaseInfo) {
		return SimOneAPIService::GetInstance()->GetCaseInfo(pCaseInfo);
	}
	SIMONE_API bool SimOneAPI::SetLogOut(ESimOne_LogLevel_Type level, const char *format, ...) {
		va_list ap;
		va_start(ap, format);
		char buf[1024];
		int pos = vsnprintf(buf, sizeof(buf), format, ap);
		SimOneAPIService::GetInstance()->bridgeLogOutput(level, buf);
		va_end(ap);
		return true;
	}

	SIMONE_API int SimOneAPI::Wait() {
		return SimOneAPIService::GetInstance()->wait();
	}

	SIMONE_API void SimOneAPI::NextFrame(int frame) {
		SimOneAPIService::GetInstance()->nextFrame(frame);
	}

	SIMONE_API bool SimOneAPI::SetFrameCB(void(*FrameStart)(int frame), void(*FrameEnd)(int frame)) {
		SimOneAPIService::GetInstance()->SetFrameStartCB(FrameStart);
		SimOneAPIService::GetInstance()->SetFrameStartCB(FrameEnd);
		return true;
	}

	SIMONE_API bool SimOneAPI::GetHDMapData(SimOne_Data_Map *hdMap)
	{
		return SimOneAPIService::GetHDMapData(hdMap);
	}
#ifdef __cplusplus
}
#endif
