#ifndef TEST_BENCH_H
#define TEST_BENCH_H

#include "SimOneSensorAPI.h"
#include "SimOneV2XAPI.h"
#include "SimOnePNCAPI.h"
#include "SimOneServiceAPI.h"
#include "SimOneHDMapAPI.h"
#include "dumper.h"

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

		// Service API
		void Test_GetVersion();
		void Test_GetHdMapData();
		// ------ void Test_SendRouteMessage(int length, void* pBuffer, int msgId, int toNodeId, ESimOne_Client_Type toNodeType);
		// ------ void Test_ReceiveRouteMessageCB(void(*cb)(int fromId, ESimOne_Client_Type fromType, int length, const void* pBuffer, int commandId));
		// ------ void Test_SetLogOut(ESimOne_LogLevel_Type level, const char *format, ...);
		void Test_InitSimOneAPI(bool isJoinTimeLoop, const char *serverIP);
		void Test_TerminateSimOneAPI();
		void Test_GetCaseInfo();
		void Test_GetCaseRunStatus();
		void Test_GetMainVehicleList();
		// ------ int Wait();
		// ------ void NextFrame(int frame);
		// ------ bool SetFrameCB(void(*FrameStart)(int frame), void(*FrameEnd)(int frame));
		void Test_GetMainVehicleStatus(bool IsCallBackMode);
		void Test_GetHDMapData();

		// HDMap API
		bool Test_HDMap_ALL(const std::vector<std::string> &apiNames);
		// LoadHDMap: Test in Test_HDMap_ALL
		// GetNearMostLane: Test in Test_HDMap_ALL
		void Test_GetNearLanes();
		// ------ bool GetNearLanesWithAngle(const SSD::SimPoint3D& pos, const double& distance, const double& headingAngle, const double& angleShift, SSD::SimStringVector& nearLanes);
		// ------ bool GetDistanceToLaneBoundary(const SSD::SimPoint3D& pos, SSD::SimString& id, double& distToLeft, double& distToRight, double& distToLeft2D, double& distToRight2D);
		// ------ bool GetLaneSample(const SSD::SimString &id, HDMapStandalone::MLaneInfo& info);
		// GetLaneLink: Test in Test_HDMap_ALL
		// ------ bool GetLaneType(const SSD::SimString& id, HDMapStandalone::MLaneType& laneType);
		// ------ bool GetLaneWidth(const SSD::SimString& id, const SSD::SimPoint3D& pos, double& width);
		// ------ bool GetLaneST(const SSD::SimString& id, const SSD::SimPoint3D& pos, double& s, double& t);
		// ------ bool GetRoadST(const SSD::SimString& id, const SSD::SimPoint3D& pos, double& s, double& t, double& z);
		// ------ bool GetInertialFromLaneST(const SSD::SimString& id, const double& s, const double& t, SSD::SimPoint3D& inertial, SSD::SimPoint3D& dir);
		// ------ bool ContainsLane(const SSD::SimString& id);
		// ------ void GetParkingSpaceList(SSD::SimVector<HDMapStandalone::MParkingSpace>& parkingSpaceList);
		// ------ bool GenerateRoute(const SSD::SimPoint3DVector& inputPoints, SSD::SimVector<int>& indexOfValidPoints, SSD::SimPoint3DVector& route);
		// ------ bool Navigate(const SSD::SimPoint3DVector& inputPoints, SSD::SimVector<int>& indexOfValidPoints, SSD::SimVector<long>& roadIdList);
		// ------ bool GetRoadMark(const SSD::SimPoint3D& pos, const SSD::SimString& id, HDMapStandalone::MRoadMark& left, HDMapStandalone::MRoadMark& right);
		// GetTrafficLightList: Test in Test_HDMap_ALL
		// ------ void GetTrafficSignList(SSD::SimVector<HDMapStandalone::MSignal>& list);
		// ------ void GetStoplineList(const HDMapStandalone::MSignal& light, const SSD::SimString& id, SSD::SimVector<HDMapStandalone::MObject>& stoplineList);
		// ------ void GetCrosswalkList(const HDMapStandalone::MSignal& light, const SSD::SimString& id, SSD::SimVector<HDMapStandalone::MObject>& crosswalkList);
		// GetCrossHatchList: Test in Test_HDMap_ALL
		// ------ bool GetLaneMiddlePoint(const SSD::SimPoint3D& inputPt, const SSD::SimString& id, SSD::SimPoint3D& targetPoint, SSD::SimPoint3D& dir);
		// ------ bool GetHeights(const SSD::SimPoint3D& inputPt, const double& radius, SSD::SimVector<double>& heights, SSD::SimVector<long>& roadIds, SSD::SimVector<bool>& insideRoadStates);
		// ------ void GetLaneData(SSD::SimVector<HDMapStandalone::MLaneInfo>& data);
		// ------ SSD::SimVector<long> GetJunctionList();
		// ------ double GetRoadLength(const long& roadId);
		// ------ bool GetSectionLaneList(const SSD::SimString& laneId, SSD::SimStringVector& sectionLaneList);
		// ------ bool IsTwoSideRoad(const long& roadId);
		// ------ double GetLaneLength(const SSD::SimString& id);
		// ------ bool IsDriving(const SSD::SimString& id);
		// ------ bool IsInJunction(const SSD::SimString& id, long& juncId);
		// ------ bool IsInsideLane(const SSD::SimPoint3D& inputPt, const SSD::SimString& laneName, HDMapStandalone::MSideState& sideState);
		// ------ bool GetLaneSampleByLocation(const SSD::SimPoint3D& pos, HDMapStandalone::MLaneInfo& info);

		// PNC API
		// ------ bool RegisterVehicleState(const char* mainVehicleId, ESimOne_Data_Vehicle_State *pStateIndics, int size);
		// ------ bool GetVehicleState(const char* mainVehicleId, SimOne_Data_Vehicle_Extra* pVehExtraState);
		// ------ bool SetPose(const char* mainVehicleId, SimOne_Data_Pose_Control *pPose);
		// ------ bool SetDrive(const char* mainVehicleId, SimOne_Data_Control *pControl);
		// ------ bool SetDriveTrajectory(const char* mainVehicleId, SimOne_Data_Control_Trajectory *pControlTrajectory);
		// ------ void SetDriverName(const char* mainVehicleId, const char* name);
		void Test_SetVehicleEvent();
		// ------ bool SetSignalLights(const char* mainVehicleId, SimOne_Data_Signal_Lights *pSignalLights);
		// ------ bool GetDriverStatus(const char* mainVehicleId, SimOne_Data_Driver_Status* pDriverStatus);
		// ------ bool GetControlMode(const char* mainVehicleId, SimOne_Data_Control_Mode* pControlMode);
		// ------ bool GetDriverControl(const char* mainVehicleId, SimOne_Data_Control* pControl);
		// ------ bool GetWayPoints(const char* mainVehicleId, SimOne_Data_WayPoints* pWayPoints);
		// ------ bool SetScenarioEventCB(void(*cb)(const char* mainVehicleId, const char* event, const char* data));
		// ------ bool SetTrafficEventCB(void(*cb)(const char* mainVehicleId, const char* data));

		// Sensor API
		// SimOneSensor
		void Test_GetEnvironment();
		void Test_GPS(bool IsCallBackMode);
		void Test_GetGroundTruth(bool IsCallBackMode);
		void Test_RadarDetection(bool IsCallBackMode);
		void Test_UltrasonicRadar();
		void Test_UltrasonicRadars(bool IsCallBackMode);
		void Test_GetSensorDetections(bool IsCallBackMode);
		void Test_GetSensorConfigurations();
		void Test_SetEnvironment();
		// GetTrafficLight: Test in Test_HDMap_ALL
		void Test_SensorLaneInfo(bool IsCallBackMode);

		// Sensor-V2X API
		void Test_V2XInfo(bool IsCallBackMode);

		static dumper dbg_data;

	private:
		std::string mainVehicleId;
};

#endif