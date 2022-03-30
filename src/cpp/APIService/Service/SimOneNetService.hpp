#pragma once
#include "SimOneIOStruct.h"
#include "SimOneDataStat.h"
#include "cybertron/network/SocketTcpClient.hpp"
#include "cybertron/network/SocketTcpClientSync.hpp"
#include "cybertron/core/Log.hpp"
#include "cybertron/core/UtilConsole.hpp"
#include "cybertron/core/JsonReader.hpp"
#include "cybertron/network/Message.hpp"
#include "Traffic/ScenarioEvent.pb.h"
#include "Traffic/TrafficCommon.pb.h"
#include "Traffic/TrafficEvent.pb.h"
#include "Node/HotArea.pb.h"
#include "Node/Bridge.pb.h"
#include "Sensor/Vehicle.pb.h"
#include "Sensor/SensorCommon.pb.h"
#include "Task/TaskSensorBase.hpp"
#include <map>
#include <string>

#ifndef WITHOUT_HDMAP
#include "SimOneHDMapAPI.h"
#include "public/common/MRoadMark.h"
#include "public/common/MSignal.h"
#include "public/common/MObject.h"
#include "public/common/MParkingSpace.h"
#include "public/common/MTopoGraph.h"
#endif //WITHOUT_HDMAP

#define SOINFO(f_, ...) printf(("SOAPI[I]:" f_ "\n"), __VA_ARGS__)
#define SOWARNING(f_, ...) printf(("SOAPI[W]:" f_ "\n"), __VA_ARGS__)
#define SOERROR(f_, ...) printf(("SOAPI[E]:"  f_ "\n"), __VA_ARGS__)

#define SOINFO_N(f_, ...) printf(("SOAPI[I]:" f_ "\n"))
#define SOWARNING_N(f_, ...) printf(("SOAPI[W]:" f_ "\n"))
#define SOERROR_N(f_, ...) printf(("SOAPI[E]:" f_ "\n"))

#define SOSM_LOGSERVERIP 256
#define SOSM_LOGFILENAME 256
using std::map;
using std::string;
using namespace cybertron;
using namespace SimOneAPI;
class SensorImage;
class SensorLidar;

class SimOneAPIService
{
public:
	static SimOneAPIService * GetInstance()
	{
		static SimOneAPIService instance;
		return &instance;
	}
	static int string2Int(const char* str) {
		if (str==nullptr)
			return 0;
		else {
			return atoi(str);
		}
	}
	SimOne_Data_Map getHdmap() { return mHDMapInfo; };
	bool Start( void(*startCase)(), void(*endCase)(), int registerNodeId);
	bool Stop();
	void zeroMemory();
	bool SubMainVehicle(const char* mainVehicleId, bool isJoinTimeLoop);
	bool SimOneNodeReady();

	bool GetMainVehicleList(SimOne_Data_MainVehicle_Info *pMainVehicleInfo);
	const char* GetVersion();

	bool ReceiveRouteMessageCB(void(*cb)(int fromId, ESimOne_Client_Type fromType,int length, const void* pBuffer, int commandId));
	void(*mpRouteMessageCB)(int fromId, ESimOne_Client_Type fromType,int length, const void* pBuffer, int commandId);

	bool SendRouteMessage(int length, void* pBuffer, int msgId, int toNodeId, ESimOne_Client_Type toNodeType);

	bool GetMainVehicleStatus(int mainVehicleId, SimOne_Data_MainVehicle_Status *pMainVehicleStatus);
	//1
	bool GetEnvironment(SimOne_Data_Environment *pEnvironment);
	bool SetEnvironment(SimOne_Data_Environment *pEnvironment);
	//2
	bool GetWayPoints(int mainVehicleId, SimOne_Data_WayPoints* pWayPoints);
	//3
	bool GetSensorConfigurations(int mainVehicleId, SimOne_Data_SensorConfigurations *pSensorConfigurations);
	std::string GetSensorIdFromId(int id);
	//4 
	bool GetCaseInfo(SimOne_Data_CaseInfo* pCaseInfo);
	//5
	bool GetHDMapInfo(SimOne_Data_Map* pHDMap);
	//6
	int GetCaseStatus();

	void SendMessageEventWriteSimOne(void* pbuffer, int length, Bridge::EOutSimOneMessageType type);
	bool sendMessage(int set_fromId, Bridge::EBridgeClientType fromType, int toId, Bridge::EBridgeClientType toType, int msgId, int length, void* pBuffer);
	//void SendConfigReadReq(Message* pMessage, Bridge::EOutSimOneMessageType type);
	void setEnvironmentInfo(std::uint16_t type, const std::string* msgDataBody);
	void setSensorConfigurationsInfo(std::uint16_t type, const std::string* msgDataBody);
	void setWayPointsInfo(std::uint16_t type, const std::string* msgDataBody);
	void setMapInfo(std::uint16_t type, const std::string* msgDataBody);
	void setCaseInfo(const std::string* msgDataBody);
	void setCaseStatus(const std::string* msgDataBody);
	
public:
	SimOneAPIService();
	~SimOneAPIService();
	
	bool Start();
	//bool Stop();
	bool Init();
	bool ReadLogServerConfigFromFile();
	bool ReadLogServerConfigFromEnv();
	void ReadLogConfigFile();
	void SetStartInfo(bool isJoinTimeLoop);
	void setServerInfo(const char* serverIP, int serverPort);
	static void bridgeLogOutput(ESimOne_LogLevel_Type level, const char *format, ...);
	bool connectSyncBridgeNode();
	bool setDisconnected();
	void onServerMessage(Message& msg);
	void onDisconnected();
	void run();
	int wait();
	void nextFrame(int frame);

	bool IsNeedSendObjectbasedData() {
		return mbNeedSendObjectbasedData;
	}

public:
	bool SetObjectbasedDataEnable(bool enable);
public:
	bool sendNodeRegisterReq(Bridge::EBridgeClientRole role,int id=0);

	bool sendVehicleEventInfoReq(int mainVehicleId, SimOne_Data_Vehicle_EventInfo *pEvent);
	bool sendVehicleControlReq(int mainVehicleId, SimOne_Data_Control *pControl, const char* driverName);
	bool sendVehicleControlPosReq(int mainVehicleId, SimOne_Data_Pose_Control *pPose);
    bool sendVehicleTrajectoryControlReq(int mainVehicleId, SimOne_Data_Control_Trajectory *pControlTraj, const char* driverName);
	bool sendVehicleTrajectory(int mainVehicleId, SimOne_Data_Trajectory *pTrajectory, const char* driverName);
	bool sendVehicleSignalLights(int mainVehicleId, SimOne_Data_Signal_Lights *pSignalLights);

	bool sendMainVehicleMessage(int mainVehicleId, int msgId,const google::protobuf::MessageLite& protobufMsg);
protected:
	bool processMessagesUntil(
		uint16_t desiredMessageIds,
		Message* pDesiredMessages,
		size_t numDesiredMessages=1,
		int timeoutMilliseconds = -1,
		bool msgOnebyOne = true);
protected:
	bool onFromHotAreaGPSData(Bridge::BridgeHotAreaHeader header, const std::string* msgDataBody);
	bool onFromHotAreaObstacleData(Bridge::BridgeHotAreaHeader header, const std::string* msgDataBody);
	bool onFromHotAreaTrafficLightData(Bridge::BridgeHotAreaHeader header, const std::string* msgDataBody);
	bool onFromCaseStart(Message& msg);
	bool onFromCaseEnd(Message& msg);
	bool onFromFrameStart(Message& msg);
	bool onFromFrameEnd(Message& msg);
	bool onFromCaseStatusChange(Message& msg);


	bool onFromMainVehicleDataMessage(Message& msg);
	bool onFromBridgeResultMainVehicleStatus(Message& msg);
	bool onFromBridgeTimeStepForward(Message& msg);
	bool onFromBridgeResultMainVehicleList(Message& msg);
	bool onFromHotAreaDataMessage(Message& msg);
	bool onFromSensorDataMessage(Message& msg);
	bool onFromBridgeDataRouteMessage(Message& msg);
	bool onFromBridgeScenarioEvent(Message& msg);
	bool onFromBridgeTrafficEvent(Message& msg);
	bool onEndTaskReasonFromNodeTime(Message& msg);

	bool onMainVehicleDriverStatus(int MainVehicleId, proto::sensor::DataDriverStatus status);
	bool onMainVehicleDriverControl(int MainVehicleId, proto::sensor::DataVehicleControlState status);
	bool onMainVehicleControlMode(int MainVehicleId, proto::sensor::DataVehicleControlMode mode);

public:
	bool ForwardStep();
    bool RegisterSimOneVehicleState(int mainVehicleId, ESimOne_Data_Vehicle_State *pStateIndics, int size);
    bool GetSimOneVehicleState(int mainVehicleId, SimOne_Data_Vehicle_Extra* pVehExtraState);

	bool GetGps(int mainVehicleId, SimOne_Data_Gps *pGps);
	bool GetObstacle(int mainVehicleId, SimOne_Data_Obstacle *pObstacle);
	bool GetTrafficLight(int mainVehicleId, int opendriveLightId, SimOne_Data_TrafficLight *pTrafficLight);

	
	bool GetTaskData(string key, int sensorType, int commandId, void* pbuffer);

	bool GetDriverStatus(const int mainVehicleId, SimOne_Data_Driver_Status* pDriverStatus);
	bool GetControlMode(const int mainVehicleId, SimOne_Data_Control_Mode* pControlMode);
	bool GetDriverControl(const int mainVehicleId, SimOne_Data_Control* pDriverStatus);

	bool SetStartCaseCB(void(*cb)());
	bool SetEndCaseCB(void(*cb)());
	bool SetMainVehicleStatusCB(void(*cb)(const char* mainVehicleId, SimOne_Data_MainVehicle_Status *pMainVehicleStatus));
	
	bool SetFrameStartCB(void(*cb)(int frame));
	bool SetFrameEndCB(void(*cb)(int frame));

	bool SetGpsUpdateCB(void(*cb)(const char* mainVehicleId, SimOne_Data_Gps *pGps));
	bool SetObstacleUpdateCB(void(*cb)(const char* mainVehicleId, SimOne_Data_Obstacle *pObstacle));
	bool SetTrafficLightUpdateCB(void(*cb)(const char* mainVehicleId, SimOne_Data_TrafficLights *pTrafficLights));

	bool SetImageUpdateCB(void(*cb)(const char* mainVehicleId, const char* sensorId, SimOne_Data_Image *pImage));
	bool SetPointCloudUpdateCB(void(*cb)(const char* mainVehicleId, const char* sensorId, SimOne_Data_Point_Cloud *pPointCloud));
	bool SetRadarDetectionsUpdateCB(void(*cb)(const char* mainVehicleId, const char* sensorId, SimOne_Data_RadarDetection *pDetections));
	bool SetSensorLaneInfoCB(void(*cb)(const char* mainVehicleId, const char* sensorId, SimOne_Data_LaneInfo *pLane));

#ifndef WITHOUT_HDMAP
	static bool GetHDMapData(SimOne_Data_Map* hdMap);
	bool SetV2XInfoUpdateCB(void(*cb)(const char* mainVehicleId, const char* sensorId, SimOne_Data_V2XNFS *pDetections));
#endif

	bool SetSensorDetectionsUpdateCB(void(*cb)(const char* mainVehicleId, const char* sensorId, SimOne_Data_SensorDetections *pGroundtruth));
	bool SetUltrasonicRadarsCB(void(*cb)(const char* mainVehicleId, SimOne_Data_UltrasonicRadars *pUltrasonics));
	bool SetScenarioEventCB(void(*cb)(const char* source, const char* target, const char* type, const char* content));

	// Evaluation related.
	bool SetJudgeEventCB(void(*cb)(const char* mainVehicleId, SimOne_Data_JudgeEvent *judgeEventDetailInfo));
	bool InitEvaluationService(int mainVehicleId, const char *serviceIP, int port, bool withGps);
	bool AddEvaluationRecord(int mainVehicleId, const cybertron::json& jsonRecord);
	void RunEvaluation();

#ifndef WITHOUT_HDMAP
	static bool LoadHDMap(const int& timeOutSeconds);
	static bool GetNearMostLane(const SSD::SimPoint3D& pos, SSD::SimString& id, double& s, double& t, double& s_toCenterLine, double& t_toCenterLine);
	static bool GetNearLanes(const SSD::SimPoint3D& pos, const double& distance, SSD::SimStringVector& nearLanes);
	static bool GetNearLanesWithAngle(const SSD::SimPoint3D& pos, const double& distance,
		const double& headingAngle, const double& angleShift, SSD::SimStringVector& nearLanes);
	static  bool GetDistanceToLaneBoundary(const SSD::SimPoint3D& pos, SSD::SimString& id, double& distToLeft, double& distToRight, double& distToLeft2D, double& distToRight2D);
	static bool GetLaneSample(const SSD::SimString& id, HDMapStandalone::MLaneInfo& info);
	static bool GetLaneLink(const SSD::SimString& id, HDMapStandalone::MLaneLink& laneLink);
	static bool GetLaneType(const SSD::SimString& id, HDMapStandalone::MLaneType& laneType);
	static bool GetLaneWidth(const SSD::SimString& id, const SSD::SimPoint3D& pos, double& width);
	static bool GetLaneST(const SSD::SimString& id, const SSD::SimPoint3D& pos, double& s, double& t);
	static bool GetRoadST(const SSD::SimString& id, const SSD::SimPoint3D& pos, double& s, double& t, double& z);
	static bool GetInertialFromLaneST(const SSD::SimString& id, const double& s, const double& t, SSD::SimPoint3D& inertial, SSD::SimPoint3D& dir);
	static bool ContainsLane(const SSD::SimString& id);
	static void GetParkingSpaceList(SSD::SimVector<HDMapStandalone::MParkingSpace>& parkingSpaceList);
	static bool GenerateRoute(const SSD::SimPoint3DVector& inputPoints, SSD::SimVector<int>& indexOfValidPoints, SSD::SimPoint3DVector& route);
	static bool Navigate(const SSD::SimPoint3DVector& inputPoints, SSD::SimVector<int>& indexOfValidPoints, SSD::SimVector<long>& roadIdList);
	static bool IsOverlapLaneLine(const SSD::SimPoint3D& pos, const double& radius, SSD::SimString& id);
	static bool GetRoadMark(const SSD::SimPoint3D& pos, const SSD::SimString& id, HDMapStandalone::MRoadMark& left, HDMapStandalone::MRoadMark& right);
	static SSD::SimVector<HDMapStandalone::MSignal> GetTrafficLightList();
	static SSD::SimVector<HDMapStandalone::MSignal> GetTrafficSignList();
	static SSD::SimVector<HDMapStandalone::MObject> GetStoplineList(const HDMapStandalone::MSignal& light, const SSD::SimString& id);
	static SSD::SimVector<HDMapStandalone::MObject> GetCrosswalkList(const HDMapStandalone::MSignal& light, const SSD::SimString& id);
	static SSD::SimVector<HDMapStandalone::MObject> GetCrossHatchList(const SSD::SimString& id);
	static  bool GetLaneMiddlePoint(const SSD::SimPoint3D& inputPt, const SSD::SimString& id, SSD::SimPoint3D& targetPoint, SSD::SimPoint3D& dir);
	static bool GetHeights(const SSD::SimPoint3D& inputPt, const double& radius, SSD::SimVector<double>& heights,
		SSD::SimVector<long>& roadIds, SSD::SimVector<bool>& insideRoadStates);


	////////////////////////////////
	//Premium, defined by lx
	//
	static void GetLaneData(SSD::SimVector<HDMapStandalone::MLaneInfo>& data);
	static SSD::SimVector<long> GetJunctionList();
	static double GetRoadLength(const long& roadId);
	static bool GetSectionLaneList(const SSD::SimString& laneId, SSD::SimStringVector& sectionLaneList);
	static bool IsTwoSideRoad(const long& roadId);
	static SimOneAPI::LaneInfo GetForwardLaneInfo(const SSD::SimPoint3D& pos, const SimOneAPI::TyrePosInfo& tyrePosInfo, const double& forward);
	static bool GetTopoGraph(HDMapStandalone::MTopoGraph& topoGraph);
	static double GetLaneLength(const SSD::SimString& id);
	static bool IsDriving(const SSD::SimString& id);
	static bool IsInJunction(const SSD::SimString& id, long& juncId);
	static bool IsInsideLane(const SSD::SimPoint3D& inputPt, const SSD::SimString& laneName, HDMapStandalone::MSideState& sideState);
	static bool GetNearMostLaneWithHeight(const SSD::SimPoint3D& pos, bool drivingOnly, SSD::SimString& id, double& s, double& t,
		double& s_toCenterLine, double& t_toCenterLine, bool& insideLane);
	static bool GetForwardLaneSample(const SSD::SimPoint3D& inputPt, const SSD::SimString& laneName, const double& forward,
		SSD::SimVector<HDMapStandalone::MLaneInfo>& laneInfoList);
	static void GetLaneLineInfo(SSD::SimVector<HDMapStandalone::MLaneLineInfo>& laneLineInfo);
	static void GetSectionList(const long& roadId, SSD::SimStringVector& rightList, SSD::SimStringVector& leftList);


	////////////////////////////////
	//Non-standard gq
	//
	static SSD::SimVector<int> GetLaneIndexList(const SSD::SimPoint3D& pos, int& currentLaneIndex, SSD::SimStringVector& laneIdList);
	static SimOneAPI::EDirectionType_ GetIconType(const SSD::SimPoint3D& pos);
	static bool GetLaneSampleByLocation(const SSD::SimPoint3D& pos, HDMapStandalone::MLaneInfo& info);

#endif
	map<std::string, int> mSensorDataTypeMap;
private:
	//
	void(*mpCaseStart)();
	void(*mpCaseStop)();
	void(*mpMainVehicleChangeStatus)(const char* mainVehicleId, SimOne_Data_MainVehicle_Status *pMainVehicleStatus);
	void(*mpGpsUpdateCB)(const char* mainVehicleId, SimOne_Data_Gps *pGps);
	void(*mpObstacleUpdateCB)(const char* mainVehicleId, SimOne_Data_Obstacle *pObstacle);
	void(*mpTrafficLightUpdateCB)(const char* mainVehicleId, SimOne_Data_TrafficLights *pTrafficLights);

	void(*mpSimOneGpsCB)(SimOne_Data_Gps *pGps);
	void(*mpSimOneGroundTruthCB)(SimOne_Data_Obstacle *pObstacle);
	void(*mpSimOneTrafficLightCB)(SimOne_Data_TrafficLights *pTrafficLights);
	//
	void(*mpStartCase)();
	//
	void(*mpEndCase)();
	//
	void(*mpFrameStart)(int frame);
	//
	void(*mpFrameEnd)(int frame);
	void(*mpScenarioEventCB)(const char* source, const char* target, const char* type, const char* content);
	void(*mpJudgeEventCB)(const char* mainVehicleId, SimOne_Data_JudgeEvent *judgeEventDetailInfo);
	bool mbCaseStartEventAlreadyCallback;
	bool mbCaseStopEventAlreadyCallback;
private:

private:
	// mpDesiredMessageIds and mpDesiredMessages size
	size_t mNumDesiredMessages;
	//typedef std::vector<uint16_t> DesiredMessageIdList;
	//DesiredMessageIdList mDesiredMessageIdsList;
	std::uint16_t mpDesiredMessageIds;
	//Message* mpDesiredMessages;
	Message mpDesiredMessages;
	size_t mNumReceivedDesiredMessages;
	//
	size_t mOneLoopServerMessageCounter;
private:
	cybertron::SocketTcpClientSync* mpClientSync;
	
	enum ENetServiceState
	{
		ENetServiceState_Work,
		ENetServiceState_Stop
	};
	std::atomic<ENetServiceState> mState;
	std::atomic<ENetServiceState> mPendingState;
	typedef map<int, SimOne_Data_Gps> SimOne_Data_GpsMap;
    typedef map<int, SimOne_Data_Vehicle_Extra*> SimOne_Data_Vehicle_ExtraMap;
	typedef map<int, SimOne_Data_Obstacle> SimOne_Data_ObstacleMap;
	typedef map<int, SimOne_Data_Driver_Status> SimOne_Data_Driver_StatusMap;
	typedef map<int, SimOne_Data_Control> SimOne_Data_Driver_ControlMap;
	typedef map<int, SimOne_Data_Control_Mode> SimOne_Data_Control_ModeMap;
	typedef map<int, SimOne_Data_TrafficLights> SimOne_Data_TrafficLightsMap;
	typedef map<int, SimOne_Data_MainVehicle_Info*> SimOne_Data_MainVehicle_InfoMap;
	typedef map<int, SimOne_Data_SensorConfigurations> SimOne_Data_SensorConfigurationsMap;
	typedef map<int, SimOne_Data_WayPoints> SimOne_Data_WayPointsMap;
	typedef map<int, std::vector<cybertron::json>> SimOne_Data_EvaluationRecordsMap;
		
	//  mainVehicleId
	SimOne_Data_GpsMap mLastGPSDataMap;
    SimOne_Data_Vehicle_ExtraMap mLastVehExtraStateMap;
	mutable std::recursive_mutex mLastGPSDataMapLock;
	SimOne_Data_MainVehicle_InfoMap mMainVehicleInfoMap;
	mutable std::recursive_mutex mMainVehicleInfoMapLock;
	//std::unique_lock<std::recursive_mutex> mLastGPSDataMapLock;

	SimOne_Data_ObstacleMap mLastObstacleMap;
	mutable std::recursive_mutex mLastObstacleMapLock;

	SimOne_Data_EvaluationRecordsMap mLastEvaluationRecordsMap;
	mutable std::recursive_mutex mLastEvaluationRecordsMapLock;
	std::string mEvaluationServerUrl;
	int mEvaluationMainVehicleId = 0;
	bool mbEvaluationServerInited = false;
	bool mbEvaluationRecordsReady = false;
	bool mbEvaluationWithGpsData = false;

	SimOne_Data_Driver_StatusMap mLastDriverStatusMap;
	mutable std::recursive_mutex mLastDriverStatusLock;
	SimOne_Data_Driver_ControlMap mLastDriverControlMap;
	mutable std::recursive_mutex mLastDriverControlLock;
	SimOne_Data_Control_ModeMap mLastControlModeMap;
	mutable std::recursive_mutex mLastControlModeLock;
	SimOne_Data_TrafficLightsMap mTrafficLightsMap;
	mutable std::recursive_mutex mTrafficLightsLock;

	const char* mMainVehicleId;
	bool mIsJoinTimeLoop;
	//SimOneDataStat mBridgePerformanceTest;
	bool mbStarted;
	bool mbIsOpenDefaultPerfectSensor;
	SimOne_Data_MainVehicle_Info  mpMainVehicleInfo;
	SimOne_Data_MainVehicle_Status mpMainVehicleStatus;
	SimOne_Data_Environment mEnvironmentData;
	SimOne_Data_SensorConfigurationsMap mSensorConfigurationsMap;
	std::map<int, std::string> mSensorIdMap;
	SimOne_Data_WayPointsMap mWayPointsMap;
	SimOne_Data_CaseInfo mCaseInfo;
	int mCaseStatus;
	SimOne_Data_Map mHDMapInfo;
	private:
#ifndef WITHOUT_HDMAP
		static SimOne_Data_Map mLastHDMap;
		static std::atomic<bool> mbHDMapInited;
		static bool checkHDMapSM();
		static bool startHDMap();
		static void stopHDMap();
		static bool isHDMapSMReady();
#endif
		std::string mServerIP;
		int mServerPort;
		ESimOne_LogLevel_Type mELevel;
		bool mIsOpenLog;
		mutable std::recursive_mutex mTimeStepForwardLock;
		bool mbRecvSync;
		Message mTimeStepForward;
		bool mbRecvThreadRun;
		bool mbRecvThreadExit;
		
		bool mbInWait;

		bool mbNeedSendPhysicalBasedData;
		bool mbNeedSendObjectbasedData;
		bool mbNeedSendOSIData;
		int mMessageTotalCount;
		int mMIntervalPacketCount;

		char mLogServerIp[SOSM_LOGSERVERIP];
		char mLogServerFileName[SOSM_LOGFILENAME];
		int mRegisterNodeId;
		int mLogServerPort;
		bool mbDisplayHotAreaData;
		bool mbDisplaySensorData;
		int mDisPlayFrequency;

		int mFrame;
		bool mbIsWriteLogFile;
};
//extern  SimOneAPIService* gNetService;
