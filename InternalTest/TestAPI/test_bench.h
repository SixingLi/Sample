#ifndef TEST_BENCH_H
#define TEST_BENCH_H

#include "SimOneSensorAPI.h"
#include "SimOneV2XAPI.h"
#include "SimOnePNCAPI.h"
#include "SimOneServiceAPI.h"
#include "SimOneHDMapAPI.h"

#include <thread> 
#include <chrono>
#include <iostream>
#include <string.h>
// #include <windows.h>
// using namespace std;

class tester
{
	public:
		tester(const char* mv_id);
		~tester();

		// SimOneServiceAPI
		void Test_InitSimOneAPI(bool isJoinTimeLoop, const char *serverIP);
		void Test_GetVersion();
		// void Test_SendRouteMessage(int length, void* pBuffer, int msgId, int toNodeId, ESimOne_Client_Type toNodeType); ------
		// void Test_ReceiveRouteMessageCB(void(*cb)(int fromId, ESimOne_Client_Type fromType, int length, const void* pBuffer, int commandId)); ------
		// void Test_SetLogOut(ESimOne_LogLevel_Type level, const char *format, ...); ------

		// SimOneV2X
		void Test_V2X(bool IsCallBackMode);

		// SimOneSensor
		void Test_GetSensorConfigurations();
		void Test_SetEnvironment();
		void Test_GetEnvironment();
		void Test_GPS(bool IsCallBackMode);
		void Test_UltrasonicRadar();
		void Test_UltrasonicRadars(bool IsCallBackMode);
		void Test_SensorLaneInfo(bool IsCallBackMode);
		void Test_SensorSensorDetection(bool IsCallBackMode);
		void Test_RadarDetection(bool IsCallBackMode);
		void Test_GetGroundTruth(bool IsCallBackMode);
		bool Test_HDMap_ALL(const std::vector<std::string> &apiNames);

	private:
		std::string mainVehicleId;
};

#endif