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

	SIMONE_API bool SimOneAPI::GetMainVehicleStatus(SimOne_Data_MainVehicle_Status *pMainVehicleStatus) {
		return SimOneAPIService::GetInstance()->GetMainVehicleStatus(pMainVehicleStatus);
	}

	SIMONE_API bool SimOneAPI::SetMainVehicleStatusCB(void(*cb)(SimOne_Data_MainVehicle_Status *pMainVehicleStatus)) {
		SimOneAPIService::GetInstance()->SetMainVehicleStatusCB(cb);
		return true;
	}

	SIMONE_API bool SimOneAPI::InitSimOneAPI(int hostVehicleId, bool isFrameSync, void(*startCase)(), void(*endCase)(), int registerNodeId)
	{
		SetServerInfo();
		if (SimOneAPIService::GetInstance()->Start(startCase, endCase, registerNodeId)&& SimOneAPIService::GetInstance()->SimOneNodeReady()) {
			while (true)
			{
				bool bRet = SimOneAPIService::GetInstance()->SubMainVehicle(hostVehicleId, isFrameSync);
				if (!bRet) {
					std::cout << "failed to subscribe main vehicle" << std::endl;
					std::this_thread::sleep_for(std::chrono::milliseconds(100));
					continue;
				}
				break;
			}

			SimOne_Data_MainVehicle_Status mainVehicleStatus;

			while (true)
			{
				bool bRet = GetMainVehicleStatus(&mainVehicleStatus);
				if (!bRet) {
					std::this_thread::sleep_for(std::chrono::milliseconds(100));
				}
				if (mainVehicleStatus.mainVehicleId == hostVehicleId && mainVehicleStatus.mainVehicleStatus > 0) {
					std::cout << "mainVehicle is ready" << std::endl;
					break;
				}
				std::this_thread::sleep_for(std::chrono::milliseconds(100));
			}
			SimOneAPIService::GetInstance()->SimOneNodeReady();
			while (true) {
				if (GetCaseRunStatus() == ESimOne_Case_Status::ESimOne_Case_Status_Running) {
					break;
				}
			}
			return true;
		}
		else {
			std::cout << "mainVehicle is not ready" << std::endl;
			return false;
		}
	}

	SIMONE_API bool SimOneAPI::StopSimOneNode() {
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
	SIMONE_API bool SimOneAPI::SetServerInfo(const char *serverIP, int port) {
		SimOneAPIService::GetInstance()->setServerInfo(serverIP, port);
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

#ifdef __cplusplus
}
#endif
