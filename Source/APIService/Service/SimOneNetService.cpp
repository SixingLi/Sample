#include "SimOneNetService.hpp"
#include "UtilUrlRequest.hpp"
#include "UtilString.h"
#include "cybertron/core/JsonReader.hpp"
#include "UtilId.h"
#include "LogApi.hpp"
#include "assert.h"
#include <thread>
#include <string>
#include <fstream>
#include <codecvt>

#pragma warning(disable:4473)
#pragma warning(disable:4101)

#ifndef WITHOUT_HDMAP
#include "public/MLocation.h"
#include "public/MLightAndSign.h"
#include "public/MRouting.h"
#include "SSD/SimPoint2D.h"
#endif

#ifdef CYBERTRON_WIN
#include <windows.h>
#elif CYBERTRON_LINUX
#include <string.h>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <fcntl.h>
#endif

#define MAINVEHICLE_ID_MAX 10
#define ODV0001 "51vr-od-v0001"

#ifndef WITHOUT_HDMAP
SimOne_Data_Map SimOneAPIService::mLastHDMap;
std::atomic<bool> SimOneAPIService::mbHDMapInited(false);
using SimOneAPI::LaneData;
using SimOneAPI::LaneSample;
using SimOneAPI::TyrePosInfo;
using SimOneAPI::LaneInfo;
#endif

SimOneAPIService::SimOneAPIService() :
	mState(ENetServiceState_Stop),
	mPendingState(ENetServiceState_Stop),
	mpGpsUpdateCB(nullptr),
	mpObstacleUpdateCB(nullptr),
	mpTrafficLightUpdateCB(nullptr),
	mpScenarioEventCB(nullptr),
	mpStartCase(nullptr),
	mpEndCase(nullptr),
	mpFrameStart(nullptr),
	mpFrameEnd(nullptr)
{
	mCaseStatus = 0;
	mpSimOneGpsCB = nullptr;
	mpSimOneGroundTruthCB = nullptr;
	mpSimOneTrafficLightCB = nullptr;

	mpStartCase = nullptr;
	mpEndCase = nullptr;
	mpMainVehicleChangeStatus = nullptr;
	mpFrameStart = nullptr;
	mpFrameEnd = nullptr;

	mNumDesiredMessages = 0;
	mpDesiredMessageIds = nullptr;
	mpDesiredMessages = nullptr;
	mNumReceivedDesiredMessages = 0;
	mOneLoopServerMessageCounter = 0;
	mMainVehicleId = 0;
	mIsJoinTimeLoop = false;
	mbStarted = false;
	mbRecvSync = false;
	mServerIP = "127.0.0.1";
	mServerPort = 23789;

	mMessageTotalCount = 0;
    mMIntervalPacketCount = 1000;
	mRegisterNodeId = 0;

	mpClientSync = nullptr;
	mpRouteMessageCB = nullptr;

	zeroMemory();
	Init();
	cybertron::log_enable_net_appender(mLogServerIp, mLogServerPort);
	log_enable_console_appender();
	log_enable_file_appender(mLogServerFileName);
}
SimOneAPIService::~SimOneAPIService()
{
	mSensorDataTypeMap.clear();
}
float clamp(float Min, float input, float Max)
{
	float temp;
	if (input < Min)
	{
		temp = Min;
	}
	else if (input > Max)
	{
		temp = Max;
	}
	else
	{
		temp = input;
	}
	return temp;
}
float calculate_time(float time)
{
	float temp;
	if (int(time) % 100 > 59)
	{
		temp = 59;
	}
	else
	{
		temp = int(time) % 100;
	}
	return temp;
}
void SimOneAPIService::SetStartInfo(bool isJoinTimeLoop) {
	//mMainVehicleId = mainVehicleId;
	mIsJoinTimeLoop = isJoinTimeLoop;
}
void SimOneAPIService::setServerInfo(const char* serverIP, int serverPort) {
	mServerIP = serverIP;
	mServerPort = serverPort;
	return;
}
void SimOneAPIService::bridgeLogOutput(ELogLevel_Type level, const char *format, ...) {
	if (format == nullptr) {
		return;
	}
	va_list ap;
	va_start(ap, format);
	char buf[1024];
	int pos = vsnprintf(buf, sizeof(buf), format, ap);

	auto taskId = GetInstance()->mCaseInfo.taskId;
	//char *taskId = "";
	switch (level)
	{
	case ELogLevel_Type::ELogLevelDebug:
		log_debug(taskId, "SimOneAPI", buf);
		break;
	case ELogLevel_Type::ELogLevelInformation:
		log_info(taskId, "SimOneAPI", buf);
		break;
	case ELogLevel_Type::ELogLevelWarning:
		log_warn(taskId, "SimOneAPI", buf);
		break;
	case ELogLevel_Type::ELogLevelError:
		log_error(taskId, "SimOneAPI", buf);
		break;
	case ELogLevel_Type::ELogLevelFatal:
		log_fatal(taskId, "SimOneAPI", buf);
		break;
	}
	va_end(ap);
}
bool SimOneAPIService::Start(void(*startCase)(), void(*endCase)(), int registerNodeId) {
	SetStartCaseCB(startCase);
	SetEndCaseCB(endCase);
	mRegisterNodeId = registerNodeId;
	return this->Start();
}
bool SimOneAPIService::Stop() {
	mbRecvThreadRun = false;
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));

	if (mpClientSync) {
		mpClientSync->close();
	}

	while (!mbRecvThreadExit)
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}
	delete mpClientSync;
	mpClientSync = nullptr;
	return true;
}
bool SimOneAPIService::SubMainVehicle(int mainVehicleId, bool isJoinTimeLoop) {
	if (!mpClientSync) {
		return false;
	}
	Bridge::BridgeReqSubMainVehicle info;
	info.set_mainvehicleid(mainVehicleId);
	info.set_isjointimeloop(isJoinTimeLoop);
	mpClientSync->send(Bridge::EBridgeReqSubMainVehicle, info);


	Message result_msg;

	Bridge::BridgeResultSubMainVehicle result;
	uint16_t messageId = Bridge::EBridgeResultSubMainVehicle;
	if (!processMessagesUntil(&messageId, &result_msg))
	{
		bridgeLogOutput(ELogLevel_Type::ELogLevelError, "SubMainVehicle Error.");
		return false;
	}

	if (!result_msg.toProtobuf(result))
	{
		return false;
	}
	if (result.mainvehicleid() == mainVehicleId && result.issuccessful() > 0) {
		mMainVehicleId = mainVehicleId;
		bridgeLogOutput(ELogLevel_Type::ELogLevelInformation, "SubMainVehicle successful.");
		//mPendingState = ENetServiceState_Work;
		return true;
	}

	return false;
}
bool SimOneAPIService::SimOneNodeReady() {

	if (!mpClientSync) {
		return false;
	}
	Bridge::BridgeSimOneAPIReady info;
	mpClientSync->send(Bridge::EBridgeSimOneAPIReady, info);
	return true;
}
bool SimOneAPIService::Start()
{
	zeroMemory();
	bool ret = connectSyncBridgeNode();
	if (ret) {
		mbRecvThreadRun = true;
		std::thread thread(&SimOneAPIService::run, this);
		thread.detach();
		mPendingState = ENetServiceState_Work;
	}
	return ret;
}
void SimOneAPIService::zeroMemory() {

	if (mpClientSync) {
		delete mpClientSync;
		mpClientSync = nullptr;
	}
	mState = ENetServiceState_Stop;
	mPendingState = ENetServiceState_Stop;
	mpGpsUpdateCB = nullptr;
	mpObstacleUpdateCB = nullptr;
	mpTrafficLightUpdateCB = nullptr;
	mpScenarioEventCB = nullptr;
	mpStartCase = nullptr;
	mpEndCase = nullptr;
	mpFrameStart = nullptr;
	mpFrameEnd = nullptr;
	mCaseStatus = 0;
	mpSimOneGpsCB = nullptr;
	mpSimOneGroundTruthCB = nullptr;
	mpSimOneTrafficLightCB = nullptr;

	mpStartCase = nullptr;
	mpEndCase = nullptr;
	mpMainVehicleChangeStatus = nullptr;
	mpFrameStart = nullptr;
	mpFrameEnd = nullptr;

	mNumDesiredMessages = 0;
	mpDesiredMessageIds = nullptr;
	mpDesiredMessages = nullptr;
	mNumReceivedDesiredMessages = 0;
	mOneLoopServerMessageCounter = 0;
	mMainVehicleId = 0;
	mIsJoinTimeLoop = false;
	mbStarted = false;
	mbRecvSync = false;
	mbInWait = false;

	mbRecvThreadRun = false;
	mbRecvThreadExit = false;
	mpMainVehicleInfo.size = 0;

	mLastGPSDataMap.clear();
	mMainVehicleInfoMap.clear();
	mLastObstacleMap.clear();
#ifndef WITHOUT_PNC
	mLastDriverStatusMap.clear();
	mLastDriverControlMap.clear();
#endif
	mTrafficLightsMap.clear();
	mSensorIdMap.clear();
#ifndef WITHOUT_SENSOR
	TaskSensorManager::getInstance().ManagerClear();
#endif
	memset(&mWayPoints, 0, sizeof(SimOne_Data_WayPoints));
	memset(&mSensorConfigurations, 0, sizeof(SimOne_Data_SensorConfigurations));
	memset(&mEnvironmentData, 0, sizeof(SimOne_Data_Environment));
	memset(&mCaseInfo, 0, sizeof(SimOne_Data_CaseInfo));
	return;
}
bool SimOneAPIService::Init()
{
	ReadLogConfigFile();
	if (ReadLogServerConfigFromFile())
	{
		return true;
	}
	if(ReadLogServerConfigFromEnv())
	{
		return true;
	}
	memcpy(mLogServerIp, "logcollector", strlen("logcollector"));
	mLogServerPort = 9999;
	return false;
}
void SimOneAPIService::ReadLogConfigFile() {
	cybertron::json jsonSensor;
	if (!JsonReader::loadFile(jsonSensor, "logOutputConfig.json")){
		return;
	}
	else{
		std::string isWriteLogFile = JsonReader::getString(jsonSensor, "API_CONTROL_LOG");
		mbIsWriteLogFile = atoi(isWriteLogFile.c_str());
	}
}
bool SimOneAPIService::ReadLogServerConfigFromFile()
{
	cybertron::json jsonSensor;
	if (!JsonReader::loadFile(jsonSensor, "Config.json")){
		return false;
	}
	else
	{
		JsonReader::loadFile(jsonSensor, "Config.json");
		std::string logServerIp = JsonReader::getString(jsonSensor, "LOG_SERVER_IP");
		memcpy(mLogServerIp, logServerIp.c_str(), strlen(logServerIp.c_str()));

		std::string logServerPort = JsonReader::getString(jsonSensor, "LOG_SERVER_PORT");
		mLogServerPort = atoi(logServerPort.c_str());

		std::string logServerFileName = JsonReader::getString(jsonSensor, "LOG_SERVER_FILENAME");
		memcpy(mLogServerFileName, logServerFileName.c_str(), logServerFileName.size());

		std::string displayHotAreaData = JsonReader::getString(jsonSensor, "LOG_SERVER_HOTAREADATA");
		mbDisplayHotAreaData = atoi(displayHotAreaData.c_str());

		std::string displaySensorData = JsonReader::getString(jsonSensor, "LOG_SERVER_SENSORDATA");
		mbDisplaySensorData = atoi(displaySensorData.c_str());

		std::string disPlayFrequency = JsonReader::getString(jsonSensor, "LOG_SERVER_DISPLAYFREQUENCY");
		mDisPlayFrequency = atoi(disPlayFrequency.c_str());
	}
	return true;
}
bool SimOneAPIService::ReadLogServerConfigFromEnv()
{
	memcpy(mLogServerIp, "logcollector", strlen("logcollector"));
	mLogServerPort = 9999;
	memcpy(mLogServerFileName, "logServer.txt", strlen("logServer.txt"));
	mbDisplayHotAreaData = 0;
	mbDisplaySensorData = 0;
	mDisPlayFrequency = 1000;
	//printf("LOG_SERVER_IP:%s\n", mLogServerIp);
	//printf("LOG_SERVER_PORT:%d\n", mLogServerPort);
	//printf("LOG_SERVER_FILENAME:%s\n", mLogServerFileName);
	//printf("LOG_SERVER_HOTAREADATA:%d\n", mbDisplayHotAreaData);
	//printf("LOG_SERVER_SENSORDATA:%d\n", mbDisplaySensorData);
	//printf("LOG_SERVER_DISPLAYFREQUENCY:%d\n", mDisPlayFrequency);
	return true;
}
bool SimOneAPIService::connectSyncBridgeNode() {
	mbStarted = false;
	try
	{
		bridgeLogOutput(ELogLevel_Type::ELogLevelInformation, "SimOneAPI Version:%s", GetVersion());
		bridgeLogOutput(ELogLevel_Type::ELogLevelInformation, "========connectSyncBridgeNode=========");

		for (;;)
		{
			mpClientSync = new SocketTcpClientSync();
			if (!mpClientSync->connect(mServerIP.c_str(), mServerPort))
			{
				mpClientSync->close();
				bridgeLogOutput(ELogLevel_Type::ELogLevelError, "Initializing node failed, connect time server(%s:%d) failed.==========", mServerIP.c_str(), mServerPort);
				std::this_thread::sleep_for(std::chrono::milliseconds(100));
				delete mpClientSync;
				mpClientSync = nullptr;
				continue;
			}
			bridgeLogOutput(ELogLevel_Type::ELogLevelInformation, "Initializing node succeed, connect time server(%s:%d) succeed.", mServerIP.c_str(), mServerPort);

			if (!this->sendNodeRegisterReq(Bridge::EBridgeClientRole_SimOneOut, mRegisterNodeId)) {
				delete mpClientSync;
				mpClientSync = nullptr;
				return false;
			}
			Message msg;
			int try_count = 0;
			while (true)
			{
				bool bret = mpClientSync->receive(msg);
				if (!bret) {
					delete mpClientSync;
					mpClientSync = nullptr;
					return false;
				}
				uint16_t msg_id = msg.parseMsgId();
				if (msg_id == Bridge::EBridgeNodeRegisterResult) {
					Bridge::BridgeNodeRegisterResult declareResult;
					if (!msg.toProtobuf(declareResult))
					{
						return false;
					}
					if (declareResult.succeeded()) {
						mbStarted = true;

						break;
					}
					else {
						std::this_thread::sleep_for(std::chrono::milliseconds(1500));
						this->sendNodeRegisterReq(Bridge::EBridgeClientRole_SimOneOut, mRegisterNodeId);
						try_count++;
					}
					if (try_count > 30) {
						mbStarted = false;
						break;
					}
				}

				if (msg_id == Bridge::EBridgeMessageFromSimOne2OutWriteReq) {
					Bridge::BridgeMessageFromSimOne2OutWriteReq state;
					if (!msg.toProtobuf(state))
					{
						return false;
					}

					std::uint16_t type = state.header().type();

					switch (type)
					{
#ifndef  WITHOUT_SENSOR
					case Bridge::EOutSimOneMessageType_Environment:
						setEnvironmentInfo(type, &(state.buffer()));
						break;
					case Bridge::EOutSimOneMessageType_SensorConfigurations:
						setSensorConfigurationsInfo(type, &(state.buffer()));
						break;
#endif // ! WITHOUT_SENSOR

#ifndef  WITHOUT_PNC
					case Bridge::EOutSimOneMessageType_WayPoints:
						setWayPointsInfo(type, &(state.buffer()));
						break;
#endif// ! WITHOUT_PNC

#ifndef WITHOUT_HDMAP
					case Bridge::EOutSimOneMessageType_MapInfo:
						setMapInfo(type, &(state.buffer()));
						break;
#endif
					case Bridge::EOutSimOneMessageType_CaseInfo:
						setCaseInfo(&(state.buffer()));
						break;
					case Bridge::EOutSimOneMessageType_CaseStatus:
						setCaseStatus(&(state.buffer()));
						break;
					default:
						break;
					}
				}
				else if (msg_id == Bridge::EBridgeResultMainVehicleList) {
					onFromBridgeResultMainVehicleList(msg);
				}


			}

			break;
		}
	}
	catch (std::exception& e)
	{
		//setMainVehicleDisconnected();
		mbStarted = false;
		bridgeLogOutput(ELogLevel_Type::ELogLevelError, "Exception: %s", e.what());
	}
	catch (...)
	{
		mbStarted = false;
		//setMainVehicleDisconnected();
		bridgeLogOutput(ELogLevel_Type::ELogLevelError, "Exception Unknown!");
	}
	//SimOneDataStat::GetInstance()->Init(mCaseInfo.caseName);
	return mbStarted;
}
bool SimOneAPIService::setDisconnected() {
	if (mpClientSync) {
		mpClientSync->close();
	}
	return true;
}
void SimOneAPIService::onServerMessage(Message& msg) {

	std::uint16_t msgId = msg.parseMsgId();
	bool ret = false;
	mOneLoopServerMessageCounter++;
	// Receive desired messages first
	for (size_t i = 0; i < mNumDesiredMessages; ++i)
	{
		if (mpDesiredMessageIds[i] == msgId)
		{
			if (mpDesiredMessages[i].parseMsgId() == msgId)
			{
				// already received this message
				continue;
			}
			mpDesiredMessages[i] = msg;
			++mNumReceivedDesiredMessages;
			return;
		}
	}
	switch (msgId)
	{
	case Bridge::EBridgeTimeStepForward:
		ret = onFromBridgeTimeStepForward(msg);
		break;
	case Bridge::EBridgeResultMainVehicleStatus:
		onFromBridgeResultMainVehicleStatus(msg);
		break;
	case Bridge::EHotAreaAnyDataMessage:
		ret = onFromHotAreaDataMessage(msg);
		break;
	case Bridge::EBridgeCaseStart:
		ret = onFromCaseStart(msg);
		break;
	case Bridge::EBridgeCaseStatusChange:
		ret = onFromCaseStatusChange(msg);
		break;
	case Bridge::EBridgeCaseEnd:
		ret = onFromCaseEnd(msg);
		break;
	case Bridge::EBridgeFrameStart:
		ret = onFromFrameStart(msg);
		break;
	case Bridge::EBridgeFrameEnd:
		ret = onFromFrameEnd(msg);
		break;
#ifndef WITHOUT_SENSOR
	case Bridge::ESensorAnyDataMessage:
		ret = onFromSensorDataMessage(msg);
		break;
#endif
	case Bridge::EBridgeDataRoute:
		ret = onFromBridgeDataRouteMessage(msg);
		break;
#ifndef WITHOUT_PNC
	case Bridge::EMainVehicleAnyDataMessage:
		ret = onFromMainVehicleDataMessage(msg);
		break;
	case Bridge::EBridgeScenarioEvent:
		ret = onFromBridgeScenarioEvent(msg);
		break;
#endif
	default:
		// ignore
		break;
	}
}
void SimOneAPIService::onDisconnected() {
	return;
}
int SimOneAPIService::wait() {
	mbInWait = true;
	Message msg;
	uint16_t messageId = Bridge::EBridgeTimeStepForward;
	if (mbRecvSync) {
		std::unique_lock<std::recursive_mutex> lock(mTimeStepForwardLock);
		mbRecvSync = false;
		msg = mTimeStepForward;
	}
	else {
		if (!processMessagesUntil(&messageId, &msg))
		{
			bridgeLogOutput(ELogLevel_Type::ELogLevelError, "wait failed, server confirm not received.");
			return -1;
		}
	}

	Bridge::BridgeTimeStepForward result;
	if (!msg.toProtobuf(result))
	{
		bridgeLogOutput(ELogLevel_Type::ELogLevelError, "BridgeTimeStepForward wrong format.");
		return -1;
	}
	int frame = result.framestamp();
	mFrame = frame;
	//{
	//	std::unique_lock<std::recursive_mutex> lock(mTimeStepForwardLock);
	//	mbRecvSync = false;
	//}
	//mbInWait = false;
	return frame;
}
void SimOneAPIService::nextFrame(int frame) {

	Bridge::BridgeTimeStepForwardConfirm msg;
	msg.set_framestamp(frame);
	mpClientSync->send(Bridge::EBridgeTimeStepForwardConfirm, msg);

	return;
}
void SimOneAPIService::run() {

	while (true)
	{
		if (!mbRecvThreadRun) {
			break;
		}
		if (!mbStarted) {
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
			continue;
		}
		if (mPendingState == ENetServiceState_Work &&
			mState == ENetServiceState_Stop)
		{
			mState = ENetServiceState_Work;
			bridgeLogOutput(ELogLevel_Type::ELogLevelInformation, "SimOne API started");
		}
		else if (mPendingState == ENetServiceState_Stop &&
			mState == ENetServiceState_Work)
		{

#ifndef WITHOUT_HDMAP
			stopHDMap();
#endif
			mState = ENetServiceState_Stop;
		}

		if (mState == ENetServiceState_Work)
		{
			if (mbStarted) {
				Message msg;
				int b = mpClientSync->receiveEnhance(msg, 5000);
				if (b == -1) {
					bridgeLogOutput(ELogLevel_Type::ELogLevelInformation, "Close Recv Connecting");
					break;
				}
				if (b == 0) {
					//bridgeLogOutput(ELogLevel_Type::ELogLevelInformation, "Waiting to accept a complete packet");
					continue;
				}
				uint16_t message_id = msg.parseMsgId();
				if (b == 1) {
					onServerMessage(msg);
					mpClientSync->sendHeartbeat();
				}
			}
		}
	}
	mCaseStatus = SimOne_Case_Status_Stop;
	mbRecvThreadExit = true;
	return;
}
bool SimOneAPIService::sendNodeRegisterReq(Bridge::EBridgeClientRole role, int registerNodeId) {
	if (!mpClientSync) {
		return false;
	}

	Bridge::BridgeNodeRegister nodeinfo;
	nodeinfo.set_role(role);
	nodeinfo.set_id(registerNodeId);
	nodeinfo.set_alias("SimOneNetAPI");
	nodeinfo.set_type(Bridge::EBridgeClientType_SimOneDLL);

	mpClientSync->send(Bridge::EBridgeNodeRegisterReq, nodeinfo);
	return true;
}
bool SimOneAPIService::sendMainVehicleMessage(int mainVehicleId, int msgId, const google::protobuf::MessageLite& protobufMsg) {
	if (!mpClientSync) {
		return false;
	}
	if (!mbStarted) {
		return false;
	}
	Bridge::MainVehicleAnyDataMessage mainVehicleMessage;

	Bridge::BridgeMainVehicleHeader *header = mainVehicleMessage.mutable_header();

	header->set_timestamp(0);
	header->set_frame(mFrame);
	header->set_commandid(msgId);
	header->set_mainvehicleid(mainVehicleId);

	std::string* pData = mainVehicleMessage.mutable_buffer();

	pData->resize(protobufMsg.ByteSize());
	if (pData->size() > 0)
	{
		memcpy(&(pData->operator[](0)), (unsigned char*)protobufMsg.SerializeAsString().c_str(), protobufMsg.ByteSize());
	}
	Message msg;
	msg.fromProtobuf(Bridge::EMainVehicleAnyDataMessage, mainVehicleMessage);

	mpClientSync->send(msg);

	return true;
}
void SimOneAPIService::SendMessageEventWriteSimOne(void* pbuffer, int length, Bridge::EOutSimOneMessageType type) {

	Bridge::BridgeMessageFromOut2SimOneWriteReq req;

	Bridge::WriteOutSimOneMessageTypeHeader *header = req.mutable_header();
	header->set_type(type);

	std::string* pData = req.mutable_buffer();
	pData->resize(length);

	if (pData->size() > 0)
	{
		memcpy(&(pData->operator[](0)), (unsigned char*)pbuffer, length);
	}

	Message msg;
	msg.fromProtobuf(Bridge::EBridgeMessageFromOut2SimOneWriteReq, req);
	mpClientSync->send(msg);
}
bool SimOneAPIService::sendMessage(int set_fromId, Bridge::EBridgeClientType fromType, int toId, Bridge::EBridgeClientType toType, int msgId, int length, void* pBuffer) {
	if (!mpClientSync) {
		return false;
	}
	if (!mbStarted) {
		return false;
	}
	Bridge::BridgeDataRoute BridgeDataRouteMessage;

	Bridge::BridgeDataRouteHeader *header = BridgeDataRouteMessage.mutable_header();
	header->set_fromid(set_fromId);
	header->set_fromtype(fromType);
	header->set_fromdesc("Frome SimOneAPI");
	header->set_toid(toId);
	header->set_totype(toType);
	header->set_todesc("To BridgeIO");
	header->set_commandid(msgId);

	std::string* pData = BridgeDataRouteMessage.mutable_buffer();
	pData->resize(length);
	if (pData->size() > 0)
	{
		memcpy(&(pData->operator[](0)), (unsigned char*)pBuffer, length);
	}

	Message msg;
	msg.fromProtobuf(Bridge::EBridgeDataRoute, BridgeDataRouteMessage);

	mpClientSync->send(msg);
	return true;
}
bool SimOneAPIService::GetMainVehicleList(SimOne_Data_MainVehicle_Info *pMainVehicleInfo) {
	memcpy(pMainVehicleInfo, &mpMainVehicleInfo, sizeof(SimOne_Data_MainVehicle_Info));
	return true;
}
bool SimOneAPIService::GetMainVehicleStatus(SimOne_Data_MainVehicle_Status *pMainVehicleStatus) {

	Bridge::BridgeResultMainVehicleStatus mainVehicleInfo;
	mainVehicleInfo.set_mainvehicleid(mMainVehicleId);
	Message req;
	req.fromProtobuf(Bridge::EBridgeReqMainVehicleStatus, mainVehicleInfo);
	mpClientSync->send(req);

	Message result;
	uint16_t messageId = Bridge::EBridgeResultMainVehicleStatus;
	if (!processMessagesUntil(&messageId, &result))
	{
		bridgeLogOutput(ELogLevel_Type::ELogLevelError, "Get MainVehicle Status failed, server confirm not received.");
		return false;
	}
	Bridge::BridgeResultMainVehicleStatus mainVehicleStatus;

	if (!result.toProtobuf(mainVehicleStatus)) {
		return false;
	}
	mpMainVehicleStatus.mainVehicleId = mainVehicleStatus.mainvehicleid();
	mpMainVehicleStatus.mainVehicleStatus = mainVehicleStatus.mainvehiclestatus();
	memcpy(pMainVehicleStatus, &mpMainVehicleStatus, sizeof(SimOne_Data_MainVehicle_Status));
	return true;
}
bool SimOneAPIService::GetCaseInfo(SimOne_Data_CaseInfo* pCaseInfo) {
	if (!pCaseInfo)
		return false;
	memcpy(pCaseInfo, &mCaseInfo, sizeof(SimOne_Data_CaseInfo));
	return true;

}
int SimOneAPIService::GetCaseStatus() {
	return mCaseStatus;
}
const char* SimOneAPIService::GetVersion() {
	return "2021_4_20_21:56:00";
}
bool SimOneAPIService::SendRouteMessage(int length, void* pBuffer, int msgId, int toNodeId, SimOne_ClientType toNodeType) {
	return sendMessage(0, Bridge::EBridgeClientType_SimOneDLL, toNodeId, (Bridge::EBridgeClientType)toNodeType, msgId, length, (void*)pBuffer);
}
bool SimOneAPIService::ReceiveRouteMessageCB(void(*cb)(int fromId, SimOne_ClientType fromType,int length, const void* pBuffer, int commandId)) {
	mpRouteMessageCB = cb;
	return true;
}
bool SimOneAPIService::RegisterSimOneVehicleState(SimOne_Data_Vehicle_State *pStateIndics, int size)
{
    if (!mpClientSync) {
        return false;
    }
    if (!mbStarted) {
        return false;
    }

    HotArea::MainVehicleExtraDataIndics indics;
    indics.set_vehicleid(mMainVehicleId);
    for (int i = 0; i < size; i++) {
        indics.add_extra_state_indics(pStateIndics[i]);
    }
    sendMainVehicleMessage(mMainVehicleId, cybertron::proto::sensor::EDataType_VehicleExtraStateIndics, indics);

    return true;
}
bool SimOneAPIService::GetSimOneVehicleState(SimOne_Data_Vehicle_Extra* pVehExtraState)
{
    std::unique_lock<std::recursive_mutex> lock(mLastGPSDataMapLock);
    if (!pVehExtraState)
        return false;
    SimOne_Data_Vehicle_ExtraMap::iterator it = mLastVehExtraStateMap.find(mMainVehicleId);
    if (it == mLastVehExtraStateMap.end())
    {
        return false;
    }

    memcpy(pVehExtraState, it->second, sizeof(SimOne_Data_Vehicle_Extra));
    return true;
}
#ifndef WITHOUT_SENSOR  //Sensor Compile

bool SimOneAPIService::SetObjectbasedDataEnable(bool enable) {
	if (!mpClientSync) {
		return false;
	}
	if (!mbStarted) {
		return false;
	}
	Bridge::BridgeSensorConfig config;
	config.set_enable(enable);
	Message msg;
	msg.fromProtobuf(Bridge::ESensorObjectbasedDataConfig, config);

	mpClientSync->send(msg);
	SimOneAPIService::GetInstance()->mbNeedSendObjectbasedData = enable;
	bridgeLogOutput(ELogLevel_Type::ELogLevelInformation, "Set Objectbased Data:%d", enable);
	return true;
}

bool SimOneAPIService::GetTaskData(string key, int sensorType, int commandId, void* pbuffer) {
	CTaskSensorBase* pTask = TaskSensorManager::getInstance().FindTask(sensorType);
	if (!pTask) {
		return false;
	}

	if (!pTask->GetData(key, (ETaskCommandId)commandId, pbuffer)) {
		return false;
	}
	return true;
}

bool SimOneAPIService::GetSensorConfigurations(SimOne_Data_SensorConfigurations *pSensorConfigurations) {
	if (!pSensorConfigurations)
		return false;
	memcpy(pSensorConfigurations, &mSensorConfigurations, sizeof(SimOne_Data_SensorConfigurations));
	return true;
}
bool SimOneAPIService::GetEnvironment(SimOne_Data_Environment *pEnvironment) {
	if (!pEnvironment)
		return false;
	memcpy(pEnvironment, &mEnvironmentData, sizeof(SimOne_Data_Environment));
	return true;
}
std::string SimOneAPIService::GetSensorIdFromId(int id)
{
	std::string sensorId;
	auto iter = mSensorIdMap.find(id);
	if (iter != mSensorIdMap.end()) {
		sensorId = iter->second;
	}
	return sensorId;
}

bool SimOneAPIService::GetTrafficLight(int mainVehicleId, int opendriveLightId, SimOne_Data_TrafficLight *pTrafficLight) {
	std::unique_lock<std::recursive_mutex> lock(mTrafficLightsLock);
	if (!pTrafficLight)
		return false;
	SimOne_Data_TrafficLightsMap::iterator it = mTrafficLightsMap.find(mMainVehicleId);
	if (it == mTrafficLightsMap.end())
	{
		return false;
	}

	bool flag = false;
	for (size_t i = 0; i < it->second.trafficlightNum; i++) {
		if (it->second.trafficlights[i].opendriveLightId == opendriveLightId) {
			memcpy(pTrafficLight, &it->second.trafficlights[i], sizeof(SimOne_Data_TrafficLight));
			flag = true;
			break;
		}
	}


	return flag;
}

bool SimOneAPIService::GetGps(int mainVehicleId, SimOne_Data_Gps *pGps) {

	std::unique_lock<std::recursive_mutex> lock(mLastGPSDataMapLock);
	if (!pGps)
		return false;
	SimOne_Data_GpsMap::iterator it = mLastGPSDataMap.find(mainVehicleId);
	if (it == mLastGPSDataMap.end())
	{
		return false;
	}

	memcpy(pGps, &it->second, sizeof(SimOne_Data_Gps));
	return true;
}

bool SimOneAPIService::GetObstacle(SimOne_Data_Obstacle *pObstacle) {
	if (!mbIsOpenDefaultPerfectSensor)
	{

		std::unique_lock<std::recursive_mutex> lock(mLastObstacleMapLock);
		if (!pObstacle)
			return false;
		SimOne_Data_ObstacleMap::iterator it = mLastObstacleMap.find(mMainVehicleId);
		if (it == mLastObstacleMap.end())
		{
			return false;
		}

		memcpy(pObstacle, &it->second, sizeof(SimOne_Data_Obstacle));
	}
	else
	{
		ETaskCommandId commandId = ETaskCommandId_PerfectPerceptionGroundTruth;
		int sensorType = Bridge::ESensorType_PerfectPerception;
		string key = std::to_string(mMainVehicleId);
		if (!GetTaskData(key, sensorType, commandId, (void*)pObstacle)) {
			return false;
		}
	}
	return true;
}

bool SimOneAPIService::GetObstacle(int mainVehicleId, SimOne_Data_Obstacle *pObstacle) {
	if (!mbIsOpenDefaultPerfectSensor)
	{
		std::unique_lock<std::recursive_mutex> lock(mLastObstacleMapLock);
		if (!pObstacle)
			return false;
		SimOne_Data_ObstacleMap::iterator it = mLastObstacleMap.find(mainVehicleId);
		if (it == mLastObstacleMap.end())
		{
			return false;
		}
		memcpy(pObstacle, &it->second, sizeof(SimOne_Data_Obstacle));
	}

	else
	{
		ETaskCommandId commandId = ETaskCommandId_PerfectPerceptionGroundTruth;
		int sensorType = Bridge::ESensorType_PerfectPerception;
		string key = std::to_string(mainVehicleId);
		if (!GetTaskData(key, sensorType, commandId, (void*)pObstacle)) {
			return false;
		}
	}
	return true;
}

bool SimOneAPIService::SetEnvironment(SimOne_Data_Environment *pEnvironment) {
	if (!mpClientSync) {
		return false;
	}
	if (pEnvironment == nullptr) {
		return false;
	}
	SendMessageEventWriteSimOne(pEnvironment, sizeof(SimOne_Data_Environment), Bridge::EOutSimOneMessageType_Environment);

	Message result;
	uint16_t messageId = Bridge::EBridgeMessageFromOut2SimOneWriteResult;
	if (!processMessagesUntil(&messageId, &result))
	{
		bridgeLogOutput(ELogLevel_Type::ELogLevelError, "Set Environment failed, server confirm not received.");
		return false;
	}
	Bridge::BridgeMessageFromOut2SimOneWriteResult setEnvironmentResult;

	if (!result.toProtobuf(setEnvironmentResult)) {
		return false;
	}

	if (setEnvironmentResult.issuccessful())
	{
		memcpy(&mEnvironmentData, pEnvironment, sizeof(SimOne_Data_Environment));
		bridgeLogOutput(ELogLevel_Type::ELogLevelError, "Set Environment failed, server confirm not received.");
		return true;
	}
	return false;
}

bool SimOneAPIService::GetGps(SimOne_Data_Gps *pGps) {

	std::unique_lock<std::recursive_mutex> lock(mLastGPSDataMapLock);
	if (!pGps)
		return false;
	SimOne_Data_GpsMap::iterator it = mLastGPSDataMap.find(mMainVehicleId);
	if (it == mLastGPSDataMap.end())
	{
		return false;
	}

	memcpy(pGps, &it->second, sizeof(SimOne_Data_Gps));
	return true;
}

#endif
#ifndef WITHOUT_PNC
bool SimOneAPIService::onFromMainVehicleDataMessage(Message& msg) {
	Bridge::MainVehicleAnyDataMessage mainVehicleMessage;
	if (!msg.toProtobuf(mainVehicleMessage))
	{
		return false;
	}
	Bridge::BridgeMainVehicleHeader header = mainVehicleMessage.header();
	int mainvehicleid = header.mainvehicleid();
	switch (header.commandid())
	{
	case proto::sensor::EDataType_DriverStatus:
	{
		cybertron::proto::sensor::DataDriverStatus status;
		if (!status.ParseFromString(mainVehicleMessage.buffer())) {
			return false;
		}
		onMainVehicleDriverStatus(mainvehicleid, status);
	}
	break;
	case proto::sensor::EDataType_DriverControl:
	{
		cybertron::proto::sensor::DataVehicleControlState status;
		if (!status.ParseFromString(mainVehicleMessage.buffer())) {
			return false;
		}
		onMainVehicleDriverControl(mainvehicleid, status);
	}
	break;
	default:
		break;
	}
	return true;
}

bool SimOneAPIService::GetDriverStatus(const int mainVehicleId, SimOne_Data_Driver_Status* pDriverStatus) {
	std::unique_lock<std::recursive_mutex> lock(mLastDriverStatusLock);
	if (!pDriverStatus)
		return false;

	SimOne_Data_Driver_StatusMap::iterator it = mLastDriverStatusMap.find(mainVehicleId);
	if (it == mLastDriverStatusMap.end())
	{
		return false;
	}

	memcpy(pDriverStatus, &it->second, sizeof(SimOne_Data_Driver_Status));
	return true;
}

bool SimOneAPIService::GetDriverControl(const int mainVehicleId, SimOne_Data_Control* pDriverControl) {
	std::unique_lock<std::recursive_mutex> lock(mLastDriverControlLock);
	if (!pDriverControl)
		return false;

	SimOne_Data_Driver_ControlMap::iterator it = mLastDriverControlMap.find(mainVehicleId);
	if (it == mLastDriverControlMap.end())
	{
		return false;
	}

	memcpy(pDriverControl, &it->second, sizeof(SimOne_Data_Control));
	return true;
}

bool SimOneAPIService::onFromBridgeScenarioEvent(Message& msg)
{
	Bridge::BridgeScenarioEvent ScenarioEvent;
	if (!msg.toProtobuf(ScenarioEvent))
	{
		return false;
	}
	cybertron::proto::traffic::ScenarioEvent evt;
	if (evt.ParseFromString(ScenarioEvent.buffer()))
	{
		if (evt.has_triggerevent())
		{
			auto& triggerEvt = evt.triggerevent();
			if (mpScenarioEventCB != nullptr)
			{
				mpScenarioEventCB(mMainVehicleId, triggerEvt.event().c_str(), triggerEvt.data().c_str());
				bridgeLogOutput(ELogLevel_Type::ELogLevelInformation, "trigger event:%s trigger data:%s", triggerEvt.event().c_str(), triggerEvt.data().c_str());
			}
		}
	}
	return true;
}
bool SimOneAPIService::onMainVehicleDriverStatus(int mainVehicleId, proto::sensor::DataDriverStatus status) {

	std::unique_lock<std::recursive_mutex> lock(mLastDriverStatusLock);
	SimOne_Data_Driver_Status driverData;
	if (status.status() == cybertron::proto::sensor::EDriver_Controlling) {
		driverData.driverStatus = ESimOne_Driver_Status_Controlling;
	}
	else if (status.status() == cybertron::proto::sensor::EDriver_Disabled) {
		driverData.driverStatus = ESimOne_Driver_Status_Disabled;
	}
	mLastDriverStatusMap[mainVehicleId] = driverData;

	return true;

}

bool SimOneAPIService::onMainVehicleDriverControl(int mainVehicleId, proto::sensor::DataVehicleControlState status) {

	std::unique_lock<std::recursive_mutex> lock(mLastDriverControlLock);
	SimOne_Data_Control driverControlData;
	driverControlData.throttle = status.throttlepercentage();
	driverControlData.brake = status.brake();
	switch (status.brakemode()) {
	case proto::sensor::EBrakeMode::EBrake_PERCENT:
		driverControlData.brakeMode = EBrakeMode_Percent;
		break;
	case proto::sensor::EBrakeMode::EBrake_MASTERCYLINDERPRESSURE:
		driverControlData.brakeMode = EBrakeMode_MasterCylinderPressure;
		break;
	case proto::sensor::EBrakeMode::EBrake_PEDALFORCE:
		driverControlData.brakeMode = EBrakeMode_PedalForce;
		break;
	}
	driverControlData.steering = status.steering();
	switch (status.steeringmode()) {
	case proto::sensor::ESteeringMode::ESteering_PERCENT:
		driverControlData.steeringMode = ESteeringMode_Percent;
		break;
	case proto::sensor::ESteeringMode::ESteering_STEERWHEELANGLE:
		driverControlData.steeringMode = ESteeringMode_SteeringWheelAngle;
		break;
	case proto::sensor::ESteeringMode::ESteering_TORQUE:
		driverControlData.steeringMode = ESteeringMode_Torque;
		break;
	}
	driverControlData.handbrake = status.handbrake();
	driverControlData.isManualGear = status.ismanualgear();
	driverControlData.gear = (EGearMode)status.gearmode();
	mLastDriverControlMap[mainVehicleId] = driverControlData;
	return true;
}

void SimOneAPIService::setWayPointsInfo(std::uint16_t type, const std::string* msgDataBody)
{
	SimOne_Data_WayPoints_Entry * pConf = (SimOne_Data_WayPoints_Entry*)msgDataBody->c_str();
	if (pConf->index >= 100) {
		return;
	}
	mWayPoints.wayPoints[pConf->index] = *pConf;
	mWayPoints.wayPointsSize++;
}

bool SimOneAPIService::SetScenarioEventCB(void(*cb)(int mainVehicleId, const char* event, const char* data))
{
	//Init();
	mpScenarioEventCB = cb;
	return true;
}

bool SimOneAPIService::GetWayPoints(SimOne_Data_WayPoints* pWayPoints) {
	if (!pWayPoints)
		return false;
	memcpy(pWayPoints, &mWayPoints, sizeof(SimOne_Data_WayPoints));
	return true;
}

bool SimOneAPIService::sendVehicleSignalLights(int mainVehicleId, SimOne_Data_Signal_Lights *pSignalLights) {
	if (!mpClientSync) {
		return false;
	}
	if (!mbStarted) {
		return false;
	}
	if (pSignalLights == nullptr) {
		return false;
	}
	cybertron::proto::sensor::DataVehicleSignalLights state;
	state.set_signallights(pSignalLights->signalLights);
	sendMainVehicleMessage(mainVehicleId, proto::sensor::EDataType_SignalLights, state);
	return true;
}

bool SimOneAPIService::sendVehicleTrajectoryControlReq(int mainVehicleId,
	SimOne_Data_Control_Trajectory *pControlTraj,
	const char* driverName) {
	if (!mpClientSync) {
		return false;
	}
	if (!mbStarted) {
		return false;
	}
	if (!pControlTraj) {
		return false;
	}
	cybertron::proto::sensor::DataVehicleControlState state;
	state.set_throttlemode(cybertron::proto::sensor::EThrottle_TRAJECTORY);
	state.set_steeringmode(cybertron::proto::sensor::ESteering_TRAJECTORY);
	for (int i = 0; i < pControlTraj->point_num; i++) {
		auto& p = pControlTraj->points[i];

		::cybertron::proto::common::TrajectoryPoint* traj_point = state.add_trajectorypoint();
		traj_point->mutable_point()->set_x(p.posx);
		traj_point->mutable_point()->set_y(p.posy);
		traj_point->set_speed(p.speed);
		traj_point->set_acc(p.accel);
		traj_point->set_theta(p.theta);
		traj_point->set_kappa(p.kappa);
		traj_point->set_relativetime(p.relative_time);
		traj_point->set_distance(p.s);
	}

	std::string apiDriverName("API");
	if (driverName && driverName[0] != '\0') {
		apiDriverName += "_";
		apiDriverName += driverName;
	}
	state.set_controllername(apiDriverName.data());
	sendMainVehicleMessage(mainVehicleId, proto::sensor::EDataType_VehicleControlState, state);
	return true;
}

bool SimOneAPIService::sendVehicleControlPosReq(int mainVehicleId, SimOne_Data_Pose_Control *pPose) {
	if (!mpClientSync) {
		return false;
	}
	if (!mbStarted) {
		return false;
	}
	cybertron::proto::sensor::DataVehicleBodyState state;
	state.set_fixheight(pPose->autoZ);
	auto pos = state.mutable_position();
	pos->set_x(pPose->posX);
	pos->set_y(pPose->posY);
	pos->set_z(pPose->posZ);
	auto rot = state.mutable_orientation();
	rot->set_x(pPose->oriX);
	rot->set_y(pPose->oriY);
	rot->set_z(pPose->oriZ);

	sendMainVehicleMessage(mainVehicleId, proto::sensor::EDataType_VehicleBodyState, state);
	return true;
}
bool SimOneAPIService::sendVehicleTrajectory(int mainVehicleId, SimOne_Data_Trajectory *pTrajectory, const char* driverName) {

	if (!mpClientSync) {
		return false;
	}
	if (!mbStarted) {
		return false;
	}
	cybertron::proto::sensor::DataVehicleTrajectoryState state;
	state.set_vehicleid(mainVehicleId);

	for (int i = 0; i < pTrajectory->trajectorySize; i++)
	{
		state.add_posx(pTrajectory->trajectory[i].posX);
		state.add_posy(pTrajectory->trajectory[i].posY);
		state.add_speed(pTrajectory->trajectory[i].vel);
	}

	std::string apiDriverName("API");
	if (driverName && driverName[0] != '\0') {
		apiDriverName += "_";
		apiDriverName += driverName;
	}
	state.set_controllername(apiDriverName.data());
	sendMainVehicleMessage(mainVehicleId, proto::sensor::EDataType_VehicleTrajectoryState, state);

	return true;
}


bool SimOneAPIService::sendVehicleEventInfoReq(int mainVehicleId, SimOne_Data_Vehicle_EventInfo *pEvent) {
	if (!mpClientSync) {
		return false;
	}
	if (!mbStarted) {
		return false;
	}
	cybertron::proto::sensor::DataVehicleEventInfo vehicleEventInfoState;

	cybertron::proto::sensor::EVehicleEventType vehicleEventInfoType;

	std::string vehicleEventInfo;

	switch (pEvent->type) {
	case ESimOne_VehicleEventInfo_Forward_Collision_Warning:
		vehicleEventInfoType = cybertron::proto::sensor::EForward_Collision_Warning;
		vehicleEventInfo = "ForwardCollisionWarning";
		break;
	case ESimOne_VehicleEventInfo_Backward_Collision_Warning:
		vehicleEventInfoType = cybertron::proto::sensor::EBackward_Collision_Warning;
		vehicleEventInfo = "BackwardCollisionWarning";
		break;
	case ESimOne_VehicleEventInfo_Left_Turn_Decision:
		vehicleEventInfoType = cybertron::proto::sensor::ELeft_Turn_Decision;
		vehicleEventInfo = "LeftTurnDecision";
		break;
	case ESimOne_VehicleEventInfo_Right_Turn_Warning:
		vehicleEventInfoType = cybertron::proto::sensor::ERight_Turn_Warning;
		vehicleEventInfo = "RightTurnWarning";
		break;
	case ESimOne_VehicleEventInfo_Forward_Straight_Decision:
		vehicleEventInfoType = cybertron::proto::sensor::EForward_Straight_Warning;
		vehicleEventInfo = "ForwardStraightWarning";
		break;
	case ESimOne_VehicleEventInfo_Over_Speed_Warning:
		vehicleEventInfoType = cybertron::proto::sensor::EOver_Speed_Warning;
		vehicleEventInfo = "OverSpeedWarning";
		break;
	case ESimOne_VehicleEventInfo_Lane_Change_Decision:
		vehicleEventInfoType = cybertron::proto::sensor::ELane_Change_Decision;
		vehicleEventInfo = "LaneChangeDecision";
		break;
	case ESimOne_VehicleEventInfo_Lane_Change_Warning:
		vehicleEventInfoType = cybertron::proto::sensor::ELane_Change_Warning;
		vehicleEventInfo = "LaneChangeWarning";
		break;
	case ESimOne_VehicleEventInfo_Overtake_Decision:
		vehicleEventInfoType = cybertron::proto::sensor::EOvertake_Decision;
		vehicleEventInfo = "OvertakeDecision";
		break;
	case ESimOne_VehicleEventInfo_Emergency_Braking_Decision:
		vehicleEventInfoType = cybertron::proto::sensor::EEmergency_Braking_Decision;
		vehicleEventInfo = "EmergencyBrakingDecision";
		break;
	case ESimOne_VehicleEventInfo_Accelerate_Decision:
		vehicleEventInfoType = cybertron::proto::sensor::EAccelerate_Decision;
		vehicleEventInfo = "AccelerateDecision";
		break;
	}

	vehicleEventInfoState.set_type(vehicleEventInfoType);
	vehicleEventInfoState.set_info(vehicleEventInfo);

	sendMainVehicleMessage(mainVehicleId, proto::sensor::EDataType_VehicleEventInfo, vehicleEventInfoState);
	return true;
}
bool SimOneAPIService::sendVehicleControlReq(int mainVehicleId, SimOne_Data_Control *pControl, const char* driverName) {
	if (!mpClientSync) {
		return false;
	}
	if (!mbStarted) {
		return false;
	}
	cybertron::proto::sensor::DataVehicleControlState state;

	switch (pControl->throttleMode) {
	case EThrottleMode_Percent:
		state.set_throttlemode(cybertron::proto::sensor::EThrottleMode::EThrottle_PERCENT);
		break;
	case EThrottleMode_Torque:
		state.set_throttlemode(cybertron::proto::sensor::EThrottleMode::EThrottle_TORQUE);
		break;
	case EThrottleMode_Speed:
		state.set_throttlemode(cybertron::proto::sensor::EThrottleMode::EThrottle_SPEED);
		break;
	case EThrottleMode_Accel:
		state.set_throttlemode(cybertron::proto::sensor::EThrottleMode::EThrottle_ACCEL);
		break;
	case EThrottleMode_EngineAV:
		state.set_throttlemode(cybertron::proto::sensor::EThrottleMode::EThrottle_ENGINEAV);
		break;
	case EThrottleMode_WheelTorque:
		state.set_throttlemode(cybertron::proto::sensor::EThrottleMode::EThrottle_WHEELTORQUE);
		break;
	}
	if (pControl->throttleMode != EThrottleMode_WheelTorque) {
		state.set_throttlepercentage(pControl->throttle);
	}
	else {
		for (int j = 0; j < SO_MAX_WHEEL_NUM; j++) {
			state.add_inputforarraydata(pControl->throttle_input_data[j]);
		}
	}
	switch (pControl->brakeMode) {
	case EBrakeMode_Percent:
		state.set_brakemode(cybertron::proto::sensor::EBrakeMode::EBrake_PERCENT);
		break;
	case EBrakeMode_MasterCylinderPressure:
		state.set_brakemode(cybertron::proto::sensor::EBrakeMode::EBrake_MASTERCYLINDERPRESSURE);
		break;
	case EBrakeMode_PedalForce:
		state.set_brakemode(cybertron::proto::sensor::EBrakeMode::EBrake_PEDALFORCE);
		break;
	case EBrakeMode_WheelCylinderPressure:
		state.set_brakemode(cybertron::proto::sensor::EBrakeMode::EBrake_WHEELCYLINDERPRESSURE);
		break;
	case EBrakeMode_WheelTorque:
		state.set_brakemode(cybertron::proto::sensor::EBrakeMode::EBrake_WHEELTORQUE);
		break;
	}
	if (pControl->brakeMode != EBrakeMode_WheelCylinderPressure &&
		pControl->brakeMode != EBrakeMode_WheelTorque) {
		state.set_brake(pControl->brake);
	}
	else {
		for (int j = 0; j < SO_MAX_WHEEL_NUM; j++) {
			state.add_inputforarraydata(pControl->brake_input_data[j]);
		}
	}
	switch (pControl->steeringMode) {
	case ESteeringMode_Percent:
		state.set_steeringmode(cybertron::proto::sensor::ESteeringMode::ESteering_PERCENT);
		break;
	case ESteeringMode_SteeringWheelAngle:
		state.set_steeringmode(cybertron::proto::sensor::ESteeringMode::ESteering_STEERWHEELANGLE);
		break;
	case ESteeringMode_Torque:
		state.set_steeringmode(cybertron::proto::sensor::ESteeringMode::ESteering_TORQUE);
		break;
	case ESteeringMode_AngularSpeed:
		state.set_steeringmode(cybertron::proto::sensor::ESteeringMode::ESteering_ANGULARSPEED);
		break;
	case ESteeringMode_WheelAngle:
		state.set_steeringmode(cybertron::proto::sensor::ESteeringMode::ESteering_WHEELANGLE);
		break;
	case ESteeringMode_WheelAnglarSpeed:
		state.set_steeringmode(cybertron::proto::sensor::ESteeringMode::ESteering_WHEELANGLARSPEED);
		break;
	}
	if (pControl->steeringMode != ESteeringMode_WheelAngle &&
		pControl->steeringMode != ESteeringMode_WheelAnglarSpeed) {
		state.set_steering(pControl->steering);
	}
	else {
		for (int j = 0; j < SO_MAX_WHEEL_NUM; j++) {
			state.add_inputforarraydata(pControl->steering_input_data[j]);
		}
	}
	state.set_clutch(pControl->clutch);
	state.set_handbrake(pControl->handbrake);
	state.set_ismanualgear(pControl->isManualGear);
	cybertron::proto::sensor::EGearMode gear;

	switch (pControl->gear) {
	case EGearMode_Neutral:
		gear = cybertron::proto::sensor::EGearMode_Neutral;
		break;
	case EGearMode_Drive:
		gear = cybertron::proto::sensor::EGearMode_Drive;
		break;
	case EGearMode_Reverse:
		gear = cybertron::proto::sensor::EGearMode_Reverse;
		break;
	case EGearMode_Parking:
		gear = cybertron::proto::sensor::EGearMode_Parking;
		break;
	case EGearManualMode_1:
		gear = cybertron::proto::sensor::EGearManualMode_1;
		break;
	case EGearManualMode_2:
		gear = cybertron::proto::sensor::EGearManualMode_2;
		break;
	case EGearManualMode_3:
		gear = cybertron::proto::sensor::EGearManualMode_3;
		break;
	case EGearManualMode_4:
		gear = cybertron::proto::sensor::EGearManualMode_4;
		break;
	case EGearManualMode_5:
		gear = cybertron::proto::sensor::EGearManualMode_5;
		break;
	case EGearManualMode_6:
		gear = cybertron::proto::sensor::EGearManualMode_6;
		break;
	case EGearManualMode_7:
		gear = cybertron::proto::sensor::EGearManualMode_7;
		break;
	case EGearManualMode_8:
		gear = cybertron::proto::sensor::EGearManualMode_8;
		break;
	default:
		gear = cybertron::proto::sensor::EGearMode_Neutral;
		break;
	}
	state.set_gearmode(gear);

	std::string apiDriverName("API");
	if (driverName && driverName[0] != '\0') {
		apiDriverName += "_";
		apiDriverName += driverName;
	}
	state.set_controllername(apiDriverName.data());
	std::uint64_t timestamp = std::uint64_t(std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count() / 1000.0);
	sendMainVehicleMessage(mainVehicleId, proto::sensor::EDataType_VehicleControlState, state);
	if (mbIsWriteLogFile)
	{
		bridgeLogOutput(ELogLevel_Type::ELogLevelInformation, "SimOneAPI setDriverControl:frame:%lld throttle:%f brake:%f steering:%f handbrake:%d isManualGear:%d gear:%d", mFrame, state.throttlepercentage(), state.brake(), state.steering(), state.handbrake(), state.ismanualgear(), state.gearmode());
	}
	return true;
}

#endif //PNC
bool SimOneAPIService::processMessagesUntil(
	uint16_t* pDesiredMessageIds,
	Message* pDesiredMessages,
	size_t numDesiredMessages,
	int timeoutMilliseconds,
	bool msgOnebyOne)
{
	if (numDesiredMessages == 0)
	{
		return true;
	}
	mNumDesiredMessages = 0;
	mpDesiredMessageIds = nullptr;
	mpDesiredMessages = nullptr;
	mNumReceivedDesiredMessages = 0;

	if (mpClientSync == nullptr)
	{
		bridgeLogOutput(ELogLevel_Type::ELogLevelError, "Process messages failed, socket invalid.");
		setDisconnected();
		return false;
	}
	if (mpClientSync->getStatus() != ESocketStatus_Connected)
	{
		bridgeLogOutput(ELogLevel_Type::ELogLevelError, "Process messages failed, socket still connecting.");
		setDisconnected();
		return false;
	}

	mNumDesiredMessages = numDesiredMessages;
	mpDesiredMessageIds = pDesiredMessageIds;
	mpDesiredMessages = pDesiredMessages;
	for (size_t i = 0; i < mNumDesiredMessages; ++i)
	{
		mpDesiredMessages[i].clear();
	}

	std::chrono::high_resolution_clock::time_point t0 = std::chrono::high_resolution_clock::now();
	for (; ; )
	{
		mOneLoopServerMessageCounter = 0;
		//mpClient->onTimer(msgOnebyOne);
		if (mpClientSync->getStatus() != ESocketStatus_Connected)
		{
			setDisconnected();
			bridgeLogOutput(ELogLevel_Type::ELogLevelError, "Process messages failed, socket not connected.");
			return false;
		}
		// break if we got the required messages.
		if (mNumReceivedDesiredMessages >= mNumDesiredMessages) {
			return true;
		}
		// batch process, just return
		if (!msgOnebyOne) {
			return false;
		}

		if (timeoutMilliseconds > 0)
		{
			std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
			std::chrono::nanoseconds span = (t1 - t0);
			if (span.count() / 1000000 > timeoutMilliseconds)
			{
				bridgeLogOutput(ELogLevel_Type::ELogLevelWarning, "Process messages failed, timeout. Not received message:");
				for (int i = 0; i < mNumDesiredMessages; i++)
				{
					if (mpDesiredMessages[i].parseMsgId() == mpDesiredMessageIds[i])
						continue;
					bridgeLogOutput(ELogLevel_Type::ELogLevelWarning, "%d", mpDesiredMessageIds[i]);
				}
				mNumDesiredMessages = 0;
				mpDesiredMessageIds = nullptr;
				mpDesiredMessages = nullptr;
				mNumReceivedDesiredMessages = 0;
				return false;
			}
		}
		else if (mOneLoopServerMessageCounter == 0) {
			// wait forever case, if no message handled, sleep for a while to yield CPU.
			std::this_thread::sleep_for(std::chrono::milliseconds(1));
		}
	}

	return true;
}
bool SimOneAPIService::onFromHotAreaGPSData(Bridge::BridgeHotAreaHeader header, const std::string* msgDataBody) {
	if (msgDataBody->size() != sizeof(SimOne_Data_Gps)) {
		bridgeLogOutput(ELogLevel_Type::ELogLevelError, "BridgeIO send GPS size:%d SimOneIOAPI receive GPS size:%d",msgDataBody->size(), sizeof(SimOne_Data_Gps));
		bridgeLogOutput(ELogLevel_Type::ELogLevelError, "SimOne_Data_Gps structure of receiver and transmitter is inconsistent");
		return false;
	}
	SimOne_Data_Gps GPS;
	memcpy(&GPS, msgDataBody->c_str(), msgDataBody->size());
	GPS.timestamp = header.timestamp();
	GPS.frame = header.frame();
	GPS.version = header.version();
	//bridgeLogOutput(ELogLevel_Type::ELogLevelInformation, "==1==HotAreaGPSData frame:%d posX:%f posY:%f posZ:%f throttle:%f brake:%f steering:%f gear:%d", GPS.frame, GPS.posX, GPS.posY, GPS.posZ, GPS.throttle, GPS.brake, GPS.steering, GPS.gear);
	if (mbDisplayHotAreaData)
	{
		static int receiveGpsTotalCount = 0;
		if (receiveGpsTotalCount % mDisPlayFrequency == 0)
		{
			bridgeLogOutput(ELogLevel_Type::ELogLevelInformation, "==1==HotAreaGPSData frame:%d posX:%f posY:%f posZ:%f throttle:%f brake:%f steering:%f gear:%d", GPS.frame, GPS.posX, GPS.posY, GPS.posZ, GPS.throttle, GPS.brake, GPS.steering, GPS.gear);
		}
		receiveGpsTotalCount++;
	}
	int mainVehicleId = header.mainvehicleid();
	SimOne_Data_GpsMap::iterator it = mLastGPSDataMap.find(mainVehicleId);

	if (it != mLastGPSDataMap.end()) {
		mLastGPSDataMap[mainVehicleId] = GPS;
	}
	else
	{
		std::unique_lock<std::recursive_mutex> lock(mLastGPSDataMapLock);
		mLastGPSDataMap[mainVehicleId] = GPS;
	}

	if (mpGpsUpdateCB != NULL)
	{
		mpGpsUpdateCB(mainVehicleId, &GPS);
	}
	if (mpSimOneGpsCB != NULL)
	{
		mpSimOneGpsCB(&GPS);
	}

    SimOne_Data_Vehicle_Extra* pExtraState = nullptr;
    SimOne_Data_Vehicle_ExtraMap::iterator eit = mLastVehExtraStateMap.find(mainVehicleId);
    if (eit != mLastVehExtraStateMap.end()) {
        pExtraState = eit->second;
    }
    else
    {
        pExtraState = new SimOne_Data_Vehicle_Extra;
        mLastVehExtraStateMap[mainVehicleId] = pExtraState;
    }
	pExtraState->dataSize = GPS.extraStateSize;
    if (pExtraState->dataSize > 0)
    {
        memcpy(pExtraState->extra_states, GPS.extraStates, sizeof(GPS.extraStates));
    }

	return true;
}
bool SimOneAPIService::onFromHotAreaObstacleData(Bridge::BridgeHotAreaHeader header, const std::string* msgDataBody) {
	if (!mbIsOpenDefaultPerfectSensor)
	{
		if (msgDataBody->size() != sizeof(SimOne_Data_Obstacle)) {
			bridgeLogOutput(ELogLevel_Type::ELogLevelError, "BridgeIO send Obstacle size:%d SimOneIOAPI receive Obstacle size:%d", msgDataBody->size(), sizeof(SimOne_Data_Gps));
			bridgeLogOutput(ELogLevel_Type::ELogLevelError, "SimOne_Data_Obstacle structure of receiver and transmitter is inconsistent");
			return false;
		}
		SimOne_Data_Obstacle obstacle;
		memcpy(&obstacle, msgDataBody->c_str(), msgDataBody->size());
		obstacle.frame = header.frame();
		obstacle.timestamp = header.timestamp();
		if (mbDisplayHotAreaData)
		{
			static int receiveObstacleTotalCount = 0;
			if (receiveObstacleTotalCount % mDisPlayFrequency == 0)
			{
				bridgeLogOutput(ELogLevel_Type::ELogLevelInformation, "==2==HotAreaObstacleData frame:%d obstaclesize:%d", obstacle.frame, obstacle.obstacleSize);
			}
			receiveObstacleTotalCount++;
		}
		int mainVehicleId = header.mainvehicleid();
		SimOne_Data_ObstacleMap::iterator it = mLastObstacleMap.find(mainVehicleId);

		if (it != mLastObstacleMap.end()) {
			mLastObstacleMap[mainVehicleId] = obstacle;
		}
		else
		{
			std::unique_lock<std::recursive_mutex> lock(mLastObstacleMapLock);
			mLastObstacleMap[mainVehicleId] = obstacle;
		}

		if (mpObstacleUpdateCB != NULL)
		{
			mpObstacleUpdateCB(mainVehicleId, &obstacle);
		}
		if (mpSimOneGroundTruthCB != NULL)
		{
			mpSimOneGroundTruthCB(&obstacle);
		}
		return true;
	}
	return false;
}
bool SimOneAPIService::onFromHotAreaTrafficLightData(Bridge::BridgeHotAreaHeader header, const std::string* msgDataBody) {
	if (msgDataBody->size() != sizeof(SimOne_Data_TrafficLights)) {
		bridgeLogOutput(ELogLevel_Type::ELogLevelError, "BridgeIO send TrafficLights size:%d SimOneIOAPI receive TrafficLights size:%d", msgDataBody->size(), sizeof(SimOne_Data_Gps));
		bridgeLogOutput(ELogLevel_Type::ELogLevelError, "SimOne_Data_TrafficLights structure of receiver and transmitter is inconsistent");
		return false;
	}
	SimOne_Data_TrafficLights trafficLight;
	memcpy(&trafficLight, msgDataBody->c_str(), msgDataBody->size());
	trafficLight.timestamp = header.timestamp();
	trafficLight.frame = header.frame();
	trafficLight.version = header.version();

	if (mbDisplayHotAreaData)
	{
		static int receiveTrafficLightTotalCount = 0;
		if (receiveTrafficLightTotalCount % mDisPlayFrequency == 0)
		{
			bridgeLogOutput(ELogLevel_Type::ELogLevelInformation, "==3==HotAreaTrafficLightData frame:%d trafficlightNum:%d", trafficLight.frame, trafficLight.trafficlightNum);
		}
		receiveTrafficLightTotalCount++;
	}
	int mainVehicleId = header.mainvehicleid();

	SimOne_Data_TrafficLightsMap::iterator it = mTrafficLightsMap.find(mainVehicleId);

	if (it != mTrafficLightsMap.end()) {
		mTrafficLightsMap[mainVehicleId] = trafficLight;
	}
	else
	{
		std::unique_lock<std::recursive_mutex> lock(mTrafficLightsLock);
		mTrafficLightsMap[mainVehicleId] = trafficLight;
	}

	if (mpTrafficLightUpdateCB != NULL)
	{
		mpTrafficLightUpdateCB(mainVehicleId, &trafficLight);
	}
	if (mpSimOneTrafficLightCB != NULL)
	{
		mpSimOneTrafficLightCB(&trafficLight);
	}
	return true;
}
bool SimOneAPIService::onFromCaseStart(Message& msg) {
	bridgeLogOutput(ELogLevel_Type::ELogLevelInformation, "case start");
	if (mpStartCase) {
		Bridge::BridgeCaseStart start;
		if (!msg.toProtobuf(start))
		{
			return false;
		}
		mpStartCase();
	}
	return true;
}
bool SimOneAPIService::onFromCaseEnd(Message& msg) {
	bridgeLogOutput(ELogLevel_Type::ELogLevelInformation, "cases end");
	if (mpEndCase) {
		mpEndCase();
	}
	return true;
}
bool SimOneAPIService::onFromFrameStart(Message& msg) {
	if (mpFrameStart) {
		Bridge::BridgeFrameStart start;
		if (!msg.toProtobuf(start))
		{
			return false;
		}
		mpFrameStart(start.framestamp());
	}

	return true;
}
bool SimOneAPIService::onFromFrameEnd(Message& msg) {
	if (mpFrameEnd) {
		Bridge::BridgeFrameStart start;
		if (!msg.toProtobuf(start))
		{
			return false;
		}
		mpFrameEnd(start.framestamp());
	}
	return true;
}
bool SimOneAPIService::onFromBridgeResultMainVehicleStatus(Message& msg) {


	Bridge::BridgeResultMainVehicleStatus mainVehicleStatus;
	if (!msg.toProtobuf(mainVehicleStatus)) {
		return false;
	}
	mpMainVehicleStatus.mainVehicleId = mainVehicleStatus.mainvehicleid();
	mpMainVehicleStatus.mainVehicleStatus = mainVehicleStatus.mainvehiclestatus();
	if (mpMainVehicleChangeStatus) {
		mpMainVehicleChangeStatus(&mpMainVehicleStatus);
	}
	return true;
}
bool SimOneAPIService::onFromBridgeResultMainVehicleList(Message& msg) {

	Bridge::BridgeResultMainVehicleList mainVehiclelist;

	if (!msg.toProtobuf(mainVehiclelist)) {
		return false;
	}
	for (auto i = 0; i < mainVehiclelist.mainvehicle_info().size(); i++)
	{
		int id = mainVehiclelist.mainvehicle_info(i).id();
		std::string type = mainVehiclelist.mainvehicle_info(i).type();
		mpMainVehicleInfo.id_list[i] = id;
		memcpy(mpMainVehicleInfo.type_list[i], type.c_str(), type.size());

		mpMainVehicleInfo.size++;
	}
	return true;
}
bool SimOneAPIService::onFromBridgeTimeStepForward(Message& msg) {
	std::unique_lock<std::recursive_mutex> lock(mTimeStepForwardLock);
	mbRecvSync = true;
	mTimeStepForward = msg;
	if (!mbInWait) {
		Bridge::BridgeTimeStepForward result;
		if (!msg.toProtobuf(result))
		{
			bridgeLogOutput(ELogLevel_Type::ELogLevelError, "BridgeTimeStepForward wrong format.");
			return false;
		}
		int frame = result.framestamp();
		nextFrame(frame);
	}
	return true;
}
bool SimOneAPIService::onFromHotAreaDataMessage(Message& msg) {
	Bridge::HotAreaAnyDataMessage hotAreaMessage;
	if (!msg.toProtobuf(hotAreaMessage))
	{
		return false;
	}
	Bridge::BridgeHotAreaHeader header = hotAreaMessage.header();
	if (header.frame() <= 1) {
		return false;
	}
	bool ret = false;
	hotAreaMessage.buffer();
	SimOneDataStat::DataContent timeData;
	timeData.frame = header.frame();
	timeData.sensorType = 0;
	timeData.nodeCreateTime = header.currenttime().startnodetime();
	timeData.BridgeIORecvTime = header.currenttime().bridgeiorecvtime();
	timeData.BridgeIOSendTime = header.currenttime().bridgeiosendtime();
	timeData.APIRecvTime = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count() / 1000.0;
	timeData.packageSize = hotAreaMessage.buffer().size();
	if (SimOneDataStat::GetInstance()->getIsOpen()) {
		SimOneDataStat::GetInstance()->addPacketStat(SimOneDataStat::stat_hotarea, &timeData);
	}
	switch (header.commandid())
	{
	case Bridge::EHotAreaGPSData:
		ret = onFromHotAreaGPSData(header, &(hotAreaMessage.buffer()));
		break;
	case Bridge::EHotAreaObstacleData:
		ret = onFromHotAreaObstacleData(header, &(hotAreaMessage.buffer()));
		break;
	case Bridge::EHotAreaTrafficLightData:
		ret = onFromHotAreaTrafficLightData(header, &(hotAreaMessage.buffer()));
		break;
		break;
	default:
		break;
	}
	return ret;
}
#ifndef WITHOUT_SENSOR
bool SimOneAPIService::onFromSensorDataMessage(Message& msg)
{
	Bridge::SensorAnyDataMessage sensorMessage;
	if (!msg.toProtobuf(sensorMessage))
	{
		return false;
	}
	Bridge::BridgeSensorHeader *header = sensorMessage.mutable_header();
	if (header->frame() <= 1) {
		return false;
	}
	std::string channel = header->channel();
	std::uint32_t commanId = header->commandid();
	std::uint32_t sensorType = header->sensortype();
	std::uint32_t sensorId = header->sensorid();
	CTaskSensorBase::SensorContext conext;
	conext.frame = header->frame();
	conext.timestamp = header->timestamp();
	conext.mainVehicleId = header->mainvehicleid();
	conext.sensorId = sensorId;
	conext.sensorType = sensorType;
	conext.commandId = commanId;
	conext.channel = channel;
	conext.time_of_simulation = header->time_of_simulation();
	conext.currentTime.startNodeTime = header->currenttime().startnodetime();
	conext.currentTime.bridgeIORecvTime = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count() / 1000.0;
	string key = std::to_string(conext.mainVehicleId).append("_").append(SimOneAPIService::GetInstance()->GetSensorIdFromId(sensorId));
	if (mSensorDataTypeMap.count(key) == 0)
	{
		mSensorDataTypeMap[key] = sensorType;
	}
	if (mbDisplaySensorData)
	{
		if (mMessageTotalCount % mDisPlayFrequency == 0)
		{
			bridgeLogOutput(ELogLevel_Type::ELogLevelInformation, "receive sensor message:frame:%d commandId:%d sensorType:%d sensord:%d mainVehicleId:%d", header->frame(), commanId, sensorType, sensorId, header->mainvehicleid());
		}
		mMessageTotalCount++;
	}
	if (commanId == cybertron::proto::sensor::EDataType_GroundTruth)
	{
		mbIsOpenDefaultPerfectSensor = true;
	}
	TaskSensorManager::getInstance().Do(conext.sensorType, conext.commandId, &conext, &(sensorMessage.buffer()));
	return true;
}

void SimOneAPIService::setEnvironmentInfo(std::uint16_t type, const std::string* msgDataBody)
{
	if (msgDataBody->size() != sizeof(SimOne_Data_Environment)) {
		return;
	}
	memcpy(&mEnvironmentData, msgDataBody->c_str(), msgDataBody->size());
	bridgeLogOutput(ELogLevel_Type::ELogLevelInformation, "SetEnvironmentInfo ambientLight:%f artificialLight:%f cloudDensity:%f directionalLight:%f fogDensity:%f groundDirtyLevel:%f groundHumidityLevel:%f heightAngle:%f snowDensity:%f timeOfDay:%f rainDensity:%f",
		mEnvironmentData.ambientLight, mEnvironmentData.artificialLight, mEnvironmentData.cloudDensity, mEnvironmentData.directionalLight, mEnvironmentData.fogDensity, mEnvironmentData.groundDirtyLevel, mEnvironmentData.groundHumidityLevel, mEnvironmentData.heightAngle, mEnvironmentData.snowDensity, mEnvironmentData.timeOfDay, mEnvironmentData.rainDensity);
}

void SimOneAPIService::setSensorConfigurationsInfo(std::uint16_t type, const std::string* msgDataBody)
{
	SimOneSensorConfiguration * pConf = (SimOneSensorConfiguration*)msgDataBody->c_str();
	mSensorConfigurations.data[mSensorConfigurations.dataSize] = *pConf;
	mSensorConfigurations.dataSize++;
	mSensorIdMap[pConf->id] = std::string(pConf->sensorId);
	bridgeLogOutput(ELogLevel_Type::ELogLevelInformation, "SensorConfigurations size:%d", mSensorConfigurations.dataSize);
	for (int i = 0; i < mSensorConfigurations.dataSize; i++)
	{
		bridgeLogOutput(ELogLevel_Type::ELogLevelInformation, "SensorConfigurations Id:%d seneorType:%s", mSensorConfigurations.data[i].sensorId, mSensorConfigurations.data[i].sensorType);
	}
}
#endif //Sensor
bool SimOneAPIService::onFromBridgeDataRouteMessage(Message& msg)
{
	bridgeLogOutput(ELogLevel_Type::ELogLevelInformation, "receive test message");
	Bridge::BridgeDataRoute data;
	if (!msg.toProtobuf(data))
	{
		return false;
	}
	int FromId = data.header().fromid();
	auto FromType = data.header().fromtype();
	auto fromDesc = data.header().fromdesc();
	int toId = data.header().toid();
	int toType = data.header().totype();
	auto toDesc = data.header().todesc();
	int commandId = data.header().commandid();
	if (mpRouteMessageCB != nullptr)
	{
		mpRouteMessageCB(FromId, (SimOne_ClientType)FromType,data.buffer().size(),data.mutable_buffer()->c_str(), commandId);
	}
	bridgeLogOutput(ELogLevel_Type::ELogLevelInformation, "formeId:%d frometype:%d fromdesc:%s toId:%d totype:%d todesc:%s", FromId, FromType, fromDesc.c_str(), toId, toType, fromDesc.c_str());
	return true;
}
void SimOneAPIService::setCaseInfo(const std::string* msgDataBody) {

	if (msgDataBody->size() != sizeof(SimOne_Data_CaseInfo)) {
		return;
	}
	memcpy(&mCaseInfo, msgDataBody->c_str(), msgDataBody->size());
	bridgeLogOutput(ELogLevel_Type::ELogLevelInformation, "CaseInfo caseId:%s", mCaseInfo.caseId);
	bridgeLogOutput(ELogLevel_Type::ELogLevelInformation, "CaseInfo caseName:%s", mCaseInfo.caseName);
	bridgeLogOutput(ELogLevel_Type::ELogLevelInformation, "CaseInfo taskId:%s", mCaseInfo.taskId);
}
bool SimOneAPIService::onFromCaseStatusChange(Message& msg) {
	Bridge::BridgeCaseStatusChange data;
	if (!msg.toProtobuf(data))
	{
		return false;
	}
	mCaseStatus = data.status();
	return true;
}
void SimOneAPIService::setCaseStatus(const std::string* msgDataBody) {
	if (msgDataBody->size() != sizeof(int)) {
		return;
	}
	memcpy(&mCaseStatus, msgDataBody->c_str(), msgDataBody->size());
	bridgeLogOutput(ELogLevel_Type::ELogLevelInformation, "case status:%d", mCaseStatus);
	return;
}
bool SimOneAPIService::SetStartCaseCB(void(*cb)()) {
	mpStartCase = cb;
	return true;
}
bool SimOneAPIService::SetEndCaseCB(void(*cb)()) {
	mpEndCase = cb;
	return true;
}
bool SimOneAPIService::SetMainVehicleStatusCB(void(*cb)(SimOne_Data_MainVehicle_Status *pMainVehicleStatus)) {
	mpMainVehicleChangeStatus = cb;
	return true;
}
bool SimOneAPIService::SetFrameStartCB(void(*cb)(int frame)) {
	mpFrameStart = cb;
	return true;
}
bool SimOneAPIService::SetFrameEndCB(void(*cb)(int frame)) {
	mpFrameEnd = cb;
	return true;
}
bool SimOneAPIService::SetObstacleUpdateCB(void(*cb)(int mainVehicleId, SimOne_Data_Obstacle *pObstacle))
{
	//Init();
	mpObstacleUpdateCB = cb;
	return true;
}
#ifndef WITHOUT_SENSOR
bool SimOneAPIService::SetGpsUpdateCB(void(*cb)(int mainVehicleId, SimOne_Data_Gps *pGps))
{
	//Init();
	mpGpsUpdateCB = cb;
	return true;
}

bool SimOneAPIService::SetTrafficLightUpdateCB(void(*cb)(int mainVehicleId, SimOne_Data_TrafficLights *pTrafficLights))
{
	//Init();
	mpTrafficLightUpdateCB = cb;
	return true;
}

bool SimOneAPIService::SetImageUpdateCB(void(*cb)(int mainVehicleId, const char* sensorId, SimOne_Data_Image *pImage))
{
	//	Init();
	TaskSensorManager::getInstance().mpImageUpdateCB = cb;
	return true;
}

bool SimOneAPIService::SetPointCloudUpdateCB(void(*cb)(int mainVehicleId, const char* sensorId, SimOne_Data_Point_Cloud *pPointCloud))
{
	//Init();
	TaskSensorManager::getInstance().mpPointCloudUpdateCB = cb;
	return true;
}

bool SimOneAPIService::SetRadarDetectionsUpdateCB(void(*cb)(int mainVehicleId, const char* sensorId, SimOne_Data_RadarDetection *pDetections))
{
	//Init();
	TaskSensorManager::getInstance().mpRadarDetectionsCB = cb;
	return true;
}

bool SimOneAPIService::SetSensorDetectionsUpdateCB(void(*cb)(int mainVehicleId, const char* sensorId, SimOne_Data_SensorDetections *pGroundtruth))
{
	//Init();
	TaskSensorManager::getInstance().mpSensorDetectionsUpdateCB = cb;
	return true;
}

bool SimOneAPIService::SetUltrasonicRadarsCB(void(*cb)(int mainVehicleId, SimOne_Data_UltrasonicRadars *pUltrasonics))
{
	//Init();
	TaskSensorManager::getInstance().mpUltrasonicRadarsUpdateCB = cb;
	return true;
}

bool SimOneAPIService::SetSensorLaneInfoCB(void(*cb)(int mainVehicleId, const char* sensorId, SimOne_Data_LaneInfo *pLane))
{
	//Init();
	TaskSensorManager::getInstance().mpLaneDetectionsUpdateCB = cb;
	return true;
}

#ifndef WITHOUT_HDMAP
bool SimOneAPIService::SetV2XInfoUpdateCB(void(*cb)(int mainVehicleId, const char* sensorId, SimOne_Data_V2XNFS *pDetections))
{
	//Init();
	TaskSensorManager::getInstance().mpSimOneV2XRawCB = cb;
	return true;
}
#endif
#endif
//Warning! we use same define to remove HDMap service and V2X service.
#ifndef WITHOUT_HDMAP
bool SimOneAPIService::LoadHDMap(const int& timeOutSeconds)
{
	const auto& t1 = std::chrono::steady_clock::now();
	while (!isHDMapSMReady())
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(50));
		const auto& t2 = std::chrono::steady_clock::now();
		auto timeSpan = std::chrono::duration_cast<std::chrono::seconds>(t2 - t1).count();
		if (timeSpan > timeOutSeconds)
		{
			return false;
		}
	}
	return checkHDMapSM();
}

bool SimOneAPIService::GetNearMostLane(const SSD::SimPoint3D& pos, SSD::SimString& id, double& s, double& t, double& s_toCenterLine, double& t_toCenterLine)
{
#ifndef API_TEST_LOADXODR_OFFLINE
	if (!mbHDMapInited)
	{
		bridgeLogOutput(ELogLevel_Type::ELogLevelError, "GetNearMostLane failed, HDMap not initialized yet.");
		return false;
	}
#endif
	return HDMapStandalone::MLocation::GetNearMostLane_V2(pos, id, s, t, s_toCenterLine, t_toCenterLine);
}

bool SimOneAPIService::GetNearLanes(const SSD::SimPoint3D& pos, const double& distance, SSD::SimStringVector& nearLanes)
{
#ifndef API_TEST_LOADXODR_OFFLINE
	if (!mbHDMapInited)
	{
		bridgeLogOutput(ELogLevel_Type::ELogLevelError, "GetNearLanes failed, HDMap not initialized yet.");
		return false;
	}
#endif
	return HDMapStandalone::MLocation::GetNearLanes(pos, distance, nearLanes);
}

bool SimOneAPIService::GetNearLanesWithAngle(const SSD::SimPoint3D& pos, const double& distance,
	const double& headingAngle, const double& angleShift, SSD::SimStringVector& nearLanes)
{
#ifndef API_TEST_LOADXODR_OFFLINE
	if (!mbHDMapInited)
	{
		bridgeLogOutput(ELogLevel_Type::ELogLevelError, "GetNearLanesWithAngle failed, HDMap not initialized yet.");
		return false;
	}
#endif
	return HDMapStandalone::MLocation::GetNearLanes(pos, distance, headingAngle, angleShift, nearLanes);
}

bool SimOneAPIService::GetDistanceToLaneBoundary(const SSD::SimPoint3D& pos, SSD::SimString& id, double& distToLeft, double& distToRight, double& distToLeft2D, double& distToRight2D)
{
#ifndef API_TEST_LOADXODR_OFFLINE
	if (!mbHDMapInited)
	{
		bridgeLogOutput(ELogLevel_Type::ELogLevelError, "GetDistanceToLaneBoundary failed, HDMap not initialized yet.");
		return false;
	}
#endif
	return HDMapStandalone::MLocation::DistanceToLaneBoundary(pos, id, distToLeft, distToRight, distToLeft2D, distToRight2D);
}

bool SimOneAPIService::GetLaneSample(const SSD::SimString& id, HDMapStandalone::MLaneInfo& info)
{
#ifndef API_TEST_LOADXODR_OFFLINE
	if (!mbHDMapInited)
	{
		bridgeLogOutput(ELogLevel_Type::ELogLevelError, "GetLaneSample failed, HDMap not initialized yet.");
		return false;
	}
#endif
	if (!HDMapStandalone::MHDMap::ContainsLane(id))
	{
		return false;
	}
	info = HDMapStandalone::MHDMap::GetLaneSample(id);
	return true;
}

bool SimOneAPIService::GetLaneLink(const SSD::SimString& id, HDMapStandalone::MLaneLink& laneLink)
{
#ifndef API_TEST_LOADXODR_OFFLINE
	if (!mbHDMapInited)
	{
		bridgeLogOutput(ELogLevel_Type::ELogLevelError, "GetLaneLink failed, HDMap not initialized yet.");
		return false;
	}
#endif
	if (!HDMapStandalone::MHDMap::ContainsLane(id))
	{
		return false;
	}
	laneLink = HDMapStandalone::MHDMap::GetLaneLink(id);
	return true;
}

bool SimOneAPIService::GetLaneType(const SSD::SimString& id, HDMapStandalone::MLaneType& laneType)
{
#ifndef API_TEST_LOADXODR_OFFLINE
	if (!mbHDMapInited)
	{
		bridgeLogOutput(ELogLevel_Type::ELogLevelError, "GetLaneType failed, HDMap not initialized yet.");
		return false;
	}
#endif
	if (!HDMapStandalone::MHDMap::ContainsLane(id))
	{
		return false;
	}
	laneType = HDMapStandalone::MHDMap::GetLaneType(id);
	return true;
}

bool SimOneAPIService::GetLaneWidth(const SSD::SimString& id, const SSD::SimPoint3D& pos, double& width)
{
#ifndef API_TEST_LOADXODR_OFFLINE
	if (!mbHDMapInited)
	{
		bridgeLogOutput(ELogLevel_Type::ELogLevelError, "GetLaneWidth failed, HDMap not initialized yet.");
		return false;
	}
#endif
	if (!HDMapStandalone::MHDMap::ContainsLane(id))
	{
		return false;
	}
	width = HDMapStandalone::MLocation::GetLaneWidth(pos, id);
	return true;
}

bool SimOneAPIService::GetLaneST(const SSD::SimString& id, const SSD::SimPoint3D& pos, double& s, double& t)
{
#ifndef API_TEST_LOADXODR_OFFLINE
	if (!mbHDMapInited)
	{
		bridgeLogOutput(ELogLevel_Type::ELogLevelError, "GetLaneST failed, HDMap not initialized yet.");
		return false;
	}
#endif
	if (!HDMapStandalone::MHDMap::ContainsLane(id))
	{
		return false;
	}
	return HDMapStandalone::MLocation::GetLaneST(id, pos, s, t);
}

bool SimOneAPIService::GetRoadST(const SSD::SimString& id, const SSD::SimPoint3D& pos, double& s, double& t, double& z)
{
#ifndef API_TEST_LOADXODR_OFFLINE
	if (!mbHDMapInited)
	{
		bridgeLogOutput(ELogLevel_Type::ELogLevelError, "GetRoadST failed, HDMap not initialized yet.");
		return false;
	}
#endif
	if (!HDMapStandalone::MHDMap::ContainsLane(id))
	{
		return false;
	}
	return HDMapStandalone::MLocation::GetST(id, pos, s, t, z);
}

bool SimOneAPIService::GetInertialFromLaneST(const SSD::SimString& id, const double& s, const double& t, SSD::SimPoint3D& inertial, SSD::SimPoint3D& dir)
{
#ifndef API_TEST_LOADXODR_OFFLINE
	if (!mbHDMapInited)
	{
		bridgeLogOutput(ELogLevel_Type::ELogLevelError, "GetInertialFromLaneST failed, HDMap not initialized yet.");
		return false;
	}
#endif
	if (!HDMapStandalone::MHDMap::ContainsLane(id))
	{
		return false;
	}
	return HDMapStandalone::MLocation::GetInertialFromLaneST(id, s, t, inertial, dir);
}

bool SimOneAPIService::ContainsLane(const SSD::SimString& id)
{
#ifndef API_TEST_LOADXODR_OFFLINE
	if (!mbHDMapInited)
	{
		bridgeLogOutput(ELogLevel_Type::ELogLevelError, "ContainsLane failed, HDMap not initialized yet.");
		return false;
	}
#endif
	return HDMapStandalone::MHDMap::ContainsLane(id);
}

void SimOneAPIService::GetParkingSpaceList(SSD::SimVector<HDMapStandalone::MParkingSpace>& parkingSpaceList)
{
#ifndef API_TEST_LOADXODR_OFFLINE
	if (!mbHDMapInited)
	{
		bridgeLogOutput(ELogLevel_Type::ELogLevelError, "GetParkingSpaceIds failed, HDMap not initialized yet.");
	}
#endif
	//use std::move is to avoid killing performance, that extern "C" has limit to expose C++ object as return value
	parkingSpaceList = std::move(HDMapStandalone::MHDMap::GetParkingSpaceList());
}

bool SimOneAPIService::GenerateRoute(const SSD::SimPoint3DVector& inputPoints, SSD::SimVector<int>& indexOfValidPoints, SSD::SimPoint3DVector& route)
{
#ifndef API_TEST_LOADXODR_OFFLINE
	if (!mbHDMapInited)
	{
		bridgeLogOutput(ELogLevel_Type::ELogLevelError, "GenerateRoute failed, HDMap not initialized yet.");
		return false;
	}
#endif
	HDMapStandalone::MRoutePath routePath;
	if (!HDMapStandalone::MRouting::GenerateRoute(inputPoints, indexOfValidPoints, routePath))
	{
		return false;
	}
	route = std::move(routePath.waypoints);
	return true;
}

bool SimOneAPIService::Navigate(const SSD::SimPoint3DVector& inputPoints, SSD::SimVector<int>& indexOfValidPoints, SSD::SimVector<long>& roadIdList)
{
#ifndef API_TEST_LOADXODR_OFFLINE
	if (!mbHDMapInited)
	{
		bridgeLogOutput(ELogLevel_Type::ELogLevelError, "Navigate failed, HDMap not initialized yet.");
		return false;
	}
#endif
	HDMapStandalone::MRoutePath path;
	SSD::SimVector<HDMapStandalone::MRoutePoint> routePtList;
	if (HDMapStandalone::MRouting::GenerateRoute(inputPoints, indexOfValidPoints, path, routePtList))
	{
		long prevRoadId = -1;
		for (auto& rp : routePtList)
		{
			if (rp.laneId.roadId != prevRoadId)
			{
				roadIdList.push_back(rp.laneId.roadId);
			}
			prevRoadId = rp.laneId.roadId;
		}
		return true;
	}
	return false;
}

bool SimOneAPIService::IsOverlapLaneLine(const SSD::SimPoint3D& pos, const double& radius, SSD::SimString& id)
{
#ifndef API_TEST_LOADXODR_OFFLINE
	if (!mbHDMapInited)
	{
		bridgeLogOutput(ELogLevel_Type::ELogLevelError, "IsOverlapLaneLine failed, HDMap not initialized yet.");
		return false;
	}
#endif
	return HDMapStandalone::MLocation::IsOverlapLaneLine(pos, radius, id);
}

bool SimOneAPIService::GetRoadMark(const SSD::SimPoint3D& pos, const SSD::SimString& id, HDMapStandalone::MRoadMark& left, HDMapStandalone::MRoadMark& right)
{
#ifndef API_TEST_LOADXODR_OFFLINE
	if (!mbHDMapInited)
	{
		bridgeLogOutput(ELogLevel_Type::ELogLevelError, "GetRoadMark failed, HDMap not initialized yet.");
		return false;
	}
#endif
	return HDMapStandalone::MLocation::GetRoadMark(pos, id, left, right);
}

SSD::SimVector<HDMapStandalone::MSignal> SimOneAPIService::GetTrafficLightList()
{
#ifndef API_TEST_LOADXODR_OFFLINE
	if (!mbHDMapInited)
	{
		bridgeLogOutput(ELogLevel_Type::ELogLevelError, "GetTrafficLightList failed, HDMap not initialized yet.");
		return SSD::SimVector<HDMapStandalone::MSignal>();
	}
#endif
	return std::move(HDMapStandalone::MLightAndSign::GetTrafficLightList());
}

SSD::SimVector<HDMapStandalone::MSignal> SimOneAPIService::GetTrafficSignList()
{
#ifndef API_TEST_LOADXODR_OFFLINE
	if (!mbHDMapInited)
	{
		bridgeLogOutput(ELogLevel_Type::ELogLevelError, "GetTrafficSignList failed, HDMap not initialized yet.");
		return SSD::SimVector<HDMapStandalone::MSignal>();
	}
#endif
	return std::move(HDMapStandalone::MLightAndSign::GetTrafficSignList());
}

SSD::SimVector<HDMapStandalone::MObject> SimOneAPIService::GetStoplineList(const HDMapStandalone::MSignal& light, const SSD::SimString& id)
{
#ifndef API_TEST_LOADXODR_OFFLINE
	if (!mbHDMapInited)
	{
		bridgeLogOutput(ELogLevel_Type::ELogLevelError, "GetStoplineList failed, HDMap not initialized yet.");
		return SSD::SimVector<HDMapStandalone::MObject>();
	}
#endif
	return std::move(HDMapStandalone::MLightAndSign::GetStoplineList(light, id));
}

SSD::SimVector<HDMapStandalone::MObject> SimOneAPIService::GetCrosswalkList(const HDMapStandalone::MSignal& light, const SSD::SimString& id)
{
#ifndef API_TEST_LOADXODR_OFFLINE
	if (!mbHDMapInited)
	{
		bridgeLogOutput(ELogLevel_Type::ELogLevelError, "GetCrosswalkList failed, HDMap not initialized yet.");
		return SSD::SimVector<HDMapStandalone::MObject>();
	}
#endif
	return std::move(HDMapStandalone::MLightAndSign::GetCrosswalkList(light, id));
}

SSD::SimVector<HDMapStandalone::MObject> SimOneAPIService::GetCrossHatchList(const SSD::SimString& id)
{
#ifndef API_TEST_LOADXODR_OFFLINE
	if (!mbHDMapInited)
	{
		bridgeLogOutput(ELogLevel_Type::ELogLevelError, "GetCrossHatchList failed, HDMap not initialized yet.");
		return SSD::SimVector<HDMapStandalone::MObject>();
	}
#endif
	return std::move(HDMapStandalone::MHDMap::GetCrossHatchList(id));
}

bool SimOneAPIService::GetLaneMiddlePoint(const SSD::SimPoint3D& inputPt, const SSD::SimString& id, SSD::SimPoint3D& targetPoint, SSD::SimPoint3D& dir)
{
#ifndef API_TEST_LOADXODR_OFFLINE
	if (!mbHDMapInited)
	{
		bridgeLogOutput(ELogLevel_Type::ELogLevelError, "GetLaneMiddlePoint failed, HDMap not initialized yet.");
		return false;
	}
#endif
	return HDMapStandalone::MLocation::GetLaneMiddlePoint(inputPt, id, targetPoint, dir);
}

bool SimOneAPIService::GetHeights(const SSD::SimPoint3D& inputPt, const double& radius, SSD::SimVector<double>& heights,
	SSD::SimVector<long>& roadIds, SSD::SimVector<bool>& insideRoadStates)
{
#ifndef API_TEST_LOADXODR_OFFLINE
	if (!mbHDMapInited)
	{
		bridgeLogOutput(ELogLevel_Type::ELogLevelError, "GetHeights failed, HDMap not initialized yet.");
		return false;
	}
#endif
	return HDMapStandalone::MLocation::GetHeights(inputPt, radius, heights, roadIds, insideRoadStates);
}

///////////////////////////////
//Non-standard, used by GuangQi
//

int GetDirectionTypeCode(const SSD::SimString& type)
{
	std::string strType(type.GetString());
	if (strType == "StraightAheadArrow")
	{
		return SimOneAPI::EDirectionType_::Forward;
	}
	else if (strType == "LeftTurnArrow")
	{
		return SimOneAPI::EDirectionType_::TurnLeft;
	}
	else if (strType == "RightTurnArrow")
	{
		return SimOneAPI::EDirectionType_::TurnRight;
	}
	else if (strType == "UTurnArrow")
	{
		return SimOneAPI::EDirectionType_::TurnBack;
	}
	else if (strType == "StraightOrLeftTurnArrow")
	{
		return SimOneAPI::EDirectionType_::ForwardAndTurnLeft;
	}
	else if (strType == "StraightOrRightTurnArrow")
	{
		return SimOneAPI::EDirectionType_::ForwardAndTurnRight;
	}
	else if (strType == "StraightOrUTurnArrow")
	{
		return SimOneAPI::EDirectionType_::ForwardAndTurnBack;
	}
	else if (strType == "LeftOrUTurnArrow")
	{
		return SimOneAPI::EDirectionType_::TurnLeftAndTurnBack;
	}
	return -1;
}

double GetDistance_(const SSD::SimPoint3D& pt1, const SSD::SimPoint3D& pt2)
{
	return std::sqrt(std::pow((pt1.x - pt2.x), 2) + std::pow((pt1.y - pt2.y), 2) + std::pow((pt1.z - pt2.z), 2));
}

void CalculateLaneIndexInfo_(const SSD::SimString& laneId, SimOneAPI::LaneIndexInfo_& laneIndexInfo)
{
	std::string strLaneId(laneId.GetString());
	int idLane = UtilString::FromString<int>(UtilId::getLaneId(strLaneId));
	if (idLane < 0)
	{
		laneIndexInfo.currentIndex = -idLane - 1;
	}
	else
	{
		laneIndexInfo.currentIndex = idLane - 1;
	}
	const SSD::SimStringVector& laneIdList = HDMapStandalone::MHDMap::GetSectionLaneList(laneId);
	for (auto& id : laneIdList)
	{
		int currentId = UtilString::FromString<int>(UtilId::getLaneId(std::string(id.GetString())));
		int index = 0;
		if (currentId < 0)
		{
			index = -currentId - 1;
		}
		else
		{
			index = currentId - 1;
		}
		laneIndexInfo.indexList.push_back(index);
	}
}

double dot_(const SSD::SimPoint2D& lhs, const SSD::SimPoint2D& rhs)
{
	return lhs.x * rhs.x + lhs.y * rhs.y;
}

double AngleBetween_(const SSD::SimPoint2D& p1, const SSD::SimPoint2D& p2)
{
	double dotVal = dot_(p1, p2);
	if (std::fabs(dotVal) > 1)
	{
		dotVal = dotVal > 0 ? 1 : -1;
	}
	return std::acos(dotVal);
}

double Cross_(const SSD::SimPoint2D& p1, const SSD::SimPoint2D& p2, const SSD::SimPoint2D& p3, const SSD::SimPoint2D& p4)
{
	return (p2.x - p1.x)*(p4.y - p3.y) - (p2.y - p1.y)*(p4.x - p3.x);
}

double GetAngle_(const SSD::SimPoint2D& dirFrom, const SSD::SimPoint2D& dirTo)
{
	double angle = AngleBetween_(dirFrom, dirTo);
	const SSD::SimPoint2D p0;
	const auto& cross = Cross_(p0, dirFrom, p0, dirTo);
	if (cross < 0)
	{
		angle = -angle;  //Just switch sign
	}
	return angle;
}

void GetValidJunctionLanes_(const SSD::SimPoint3D& pos, bool drivingOnly, const double& heading, const SSD::SimString& nearMostLane, bool insideNearMost,
	const long& junctionId, SSD::SimStringVector& laneList)
{
	const double pi = 3.1415926;
	const double kHeightGap = 2.0;
	const double kRadius = 3;
	SSD::SimVector<double> s_toCenterLineList, t_toCenterLineList;
	double angleShift = pi / 6;

	SSD::SimStringVector laneNameList;
	if (!HDMapStandalone::MLocation::GetNearLanes(pos, kRadius, heading, angleShift, laneNameList, s_toCenterLineList, t_toCenterLineList))
	{
		return;
	}

	SSD::SimPoint2D headingDir(std::cos(heading), std::sin(heading));
	SSD::SimPoint3D targetPoint, dir;
	SSD::SimPoint2D dir2D;
	for (auto& laneName : laneNameList)
	{
		if (drivingOnly && !HDMapStandalone::MHDMap::IsDriving(laneName))
		{
			continue;
		}
		long juncId;
		bool inJunc = HDMapStandalone::MHDMap::IsInJunction(laneName, juncId);
		if (!inJunc || juncId != junctionId)
		{
			continue;
		}

		if (!HDMapStandalone::MLocation::GetLaneMiddlePoint(pos, laneName, targetPoint, dir))
		{
			continue;
		}

		if (std::abs(targetPoint.z - pos.z) > kHeightGap)
		{
			continue;
		}
		dir2D.x = dir.x;
		dir2D.y = dir.y;
		dir2D.Normalize();
		double angleBetween = AngleBetween_(headingDir, dir2D);
		if (std::abs(angleBetween) > pi / 5)
		{
			continue;
		}

		if (std::string(laneName.GetString()) == std::string(nearMostLane.GetString()))
		{
			laneList.push_back(laneName);
			continue;
		}
		HDMapStandalone::MSideState sideState;
		//If nearMost is already inside, no need to add more that vehicle is outside of the lane.
		if (insideNearMost && !HDMapStandalone::MLocation::IsInsideLane(pos, laneName, sideState))
		{
			continue;
		}

		laneList.push_back(laneName);
	}
}

SimOneAPI::EDirectionType_ GetIconType_(const SSD::SimPoint3D& pos, const SSD::SimString& idStr)
{
	const auto& signalList = HDMapStandalone::MLightAndSign::GetSignalListOnLaneByType(idStr, SSD::SimString("Graphics"));
	double distMin = std::numeric_limits<double>::max();
	int indexDistMin = -1;
	for (int i = 0; i < (int)signalList.size(); i++)
	{
		auto& item = signalList[i];
		int code = GetDirectionTypeCode(item.subType);
		if (code == -1)
		{
			continue;
		}
		const auto& d = GetDistance_(pos, item.pt);
		if (d < distMin)
		{
			indexDistMin = i;
			distMin = d;
		}
	}
	if (indexDistMin == -1)
	{
		return SimOneAPI::EDirectionType_::Forward;
	}
	auto& bestItem = signalList[indexDistMin];
	int c = GetDirectionTypeCode(bestItem.subType);
	if (c != -1)
	{
		return SimOneAPI::EDirectionType_(c);
	}
	return SimOneAPI::EDirectionType_::Forward;
}

SSD::SimVector<SimOneAPI::LaneSample_> ToLaneSample_(const SSD::SimVector<HDMapStandalone::MLaneInfo>& laneSampleList)
{
	SSD::SimVector<SimOneAPI::LaneSample_> ret;
	int code = 0;
	for (auto& item : laneSampleList)
	{
		code++;
		SimOneAPI::LaneSample_ sample;
		sample.laneCode = code;
		long juncId = -1;
		sample.inJunction = HDMapStandalone::MHDMap::IsInJunction(item.laneName, juncId);
		sample.leftBoundary = std::move(item.leftBoundary);
		sample.rightBoundary = std::move(item.rightBoundary);
		ret.push_back(sample);
	}
	return std::move(ret);
}

SSD::SimVector<SimOneAPI::LaneSample_> GetLaneSampleList_(const SSD::SimPoint3D& pt, const SSD::SimString& laneName, const double& forward)
{
	return ToLaneSample_(HDMapStandalone::MHDMap::GetLaneSample(pt, laneName, forward));
}

SimOneAPI::ELaneLineType_ ToELaneLineType_(HDMapStandalone::MRoadMark roadMark)
{
	SimOneAPI::ELaneLineType_ type;
	switch (roadMark.type)
	{
	case HDMapStandalone::ERoadMarkType::broken:
	{
		if (roadMark.color == HDMapStandalone::ERoadMarkColor::white || roadMark.color == HDMapStandalone::ERoadMarkColor::standard)
		{
			type = SimOneAPI::ELaneLineType_::whiteDotted;
		}
		else if (roadMark.color == HDMapStandalone::ERoadMarkColor::yellow)
		{
			type = SimOneAPI::ELaneLineType_::yellowDotted;
		}
	}
	break;
	case HDMapStandalone::ERoadMarkType::solid:
	{
		if (roadMark.color == HDMapStandalone::ERoadMarkColor::white || roadMark.color == HDMapStandalone::ERoadMarkColor::standard)
		{
			type = SimOneAPI::ELaneLineType_::whiteSolid;
		}
		else if (roadMark.color == HDMapStandalone::ERoadMarkColor::yellow)
		{
			type = SimOneAPI::ELaneLineType_::yellowSolid;
		}
	}
	break;
	default:
		type = SimOneAPI::ELaneLineType_::none;
		break;
	}
	return type;
}

SSD::SimVector<LaneSample> ToLaneSample(const SSD::SimVector<HDMapStandalone::MLaneInfo>& laneSampleList)
{
	SSD::SimVector<LaneSample> ret;
	int code = 0;
	for (auto& item : laneSampleList)
	{
		code++;
		LaneSample sample;
		sample.laneCode = code;
		long juncId = -1;
		sample.inJunction = HDMapStandalone::MHDMap::IsInJunction(item.laneName, juncId);
		sample.leftBoundary = std::move(item.leftBoundary);
		sample.rightBoundary = std::move(item.rightBoundary);
		ret.push_back(sample);
	}
	return std::move(ret);
}

void SimOneAPIService::GetLaneData(SSD::SimVector<HDMapStandalone::MLaneInfo>& data)
{
#ifndef API_TEST_LOADXODR_OFFLINE
	if (!mbHDMapInited)
	{
		SOINFO_N("GetLaneData failed, HDMap not initialized yet.");
		return;
	}
#endif
	data = HDMapStandalone::MHDMap::GetLaneData();
}

SSD::SimVector<long> SimOneAPIService::GetJunctionList()
{
	SSD::SimVector<long> juncionIdList;
#ifndef API_TEST_LOADXODR_OFFLINE
	if (!mbHDMapInited)
	{
		SOINFO_N("GetJunctionList failed, HDMap not initialized yet.");
		return std::move(juncionIdList);
	}
#endif
	return HDMapStandalone::MHDMap::GetJunctionList();
}

double SimOneAPIService::GetRoadLength(const long& roadId)
{
#ifndef API_TEST_LOADXODR_OFFLINE
	if (!mbHDMapInited)
	{
		SOINFO_N("GetRoadLength failed, HDMap not initialized yet.");
		return 0;
	}
#endif
	return HDMapStandalone::MHDMap::GetRoadLength(roadId);
}

bool SimOneAPIService::GetSectionLaneList(const SSD::SimString& laneId, SSD::SimStringVector& sectionLaneList)
{
#ifndef API_TEST_LOADXODR_OFFLINE
	if (!mbHDMapInited)
	{
		SOINFO_N("GetSectionLaneList failed, HDMap not initialized yet.");
		return false;
	}
#endif
	sectionLaneList = HDMapStandalone::MHDMap::GetSectionLaneList(laneId);
	return true;
}

bool SimOneAPIService::IsTwoSideRoad(const long& roadId)
{
#ifndef API_TEST_LOADXODR_OFFLINE
	if (!mbHDMapInited)
	{
		SOINFO_N("IsTwoSideRoad failed, HDMap not initialized yet.");
		return false;
	}
#endif
	return HDMapStandalone::MHDMap::IsTwoSideRoad(roadId);
}

LaneInfo SimOneAPIService::GetForwardLaneInfo(const SSD::SimPoint3D& pos, const TyrePosInfo& tyrePosInfo, const double& forward)
{
	LaneInfo laneInfo;
#ifndef API_TEST_LOADXODR_OFFLINE
	if (!mbHDMapInited)
	{
		bridgeLogOutput(ELogLevel_Type::ELogLevelError, "GetLaneInfo failed, HDMap not initialized yet.");
		return std::move(laneInfo);
	}
#endif
	SSD::SimString idStr;
	double s, t, s_toCenterLine, t_toCenterLine;
	bool insideLane = false;
	bool drivingOnly = true;
	if (HDMapStandalone::MLocation::GetNearMostLaneWithHeight_V2(pos, drivingOnly, idStr, s, t, s_toCenterLine, t_toCenterLine, insideLane))
	{
		laneInfo.currentLane = idStr;
		long juncId = -1;
		if (HDMapStandalone::MHDMap::IsInJunction(idStr, juncId))
		{
			//For lanes in junction, need to cover all of them.
			//Only calculate laneSampleList.
			//
			SSD::SimStringVector laneList;
			SSD::SimPoint2D p1(tyrePosInfo.rearLeft.x, tyrePosInfo.rearLeft.y);
			SSD::SimPoint2D p2(tyrePosInfo.frontLeft.x, tyrePosInfo.frontLeft.y);
			SSD::SimPoint2D dir(tyrePosInfo.frontLeft.x - tyrePosInfo.rearLeft.x, tyrePosInfo.frontLeft.y - tyrePosInfo.rearLeft.y);
			dir.Normalize();
			double angle = GetAngle_(SSD::SimPoint2D(1, 0), dir);
			GetValidJunctionLanes_(pos, drivingOnly, angle, idStr, insideLane, juncId, laneList);
			//dataList
			//

			for (auto& laneId : laneList)
			{
				LaneData data;

				//laneSampleList
				const auto& laneInfoList = HDMapStandalone::MHDMap::GetLaneSample(pos, laneId, forward);
				for (auto& laneInfo : laneInfoList)
				{
					data.laneNameList.push_back(laneInfo.laneName);
				}
				data.laneSampleList = std::move(ToLaneSample(laneInfoList));


				HDMapStandalone::MLocation::GetRoadMark(pos, laneId, data.leftRoadMark, data.rightRoadMark);
				laneInfo.dataList.push_back(data);
			}
			return std::move(laneInfo);
		}

		LaneData data;

		//laneSampleList
		const auto& laneInfoList = HDMapStandalone::MHDMap::GetLaneSample(pos, idStr, forward);
		for (auto& laneInfo : laneInfoList)
		{
			data.laneNameList.push_back(laneInfo.laneName);
		}
		data.laneSampleList = std::move(ToLaneSample(laneInfoList));

		//laneLineTypeInfo
		HDMapStandalone::MLocation::GetRoadMark(pos, idStr, data.leftRoadMark, data.rightRoadMark);
		laneInfo.dataList.push_back(data);
	}
	return std::move(laneInfo);
}

bool SimOneAPIService::GetTopoGraph(HDMapStandalone::MTopoGraph& topoGraph)
{
#ifndef API_TEST_LOADXODR_OFFLINE
	if (!mbHDMapInited)
	{
		bridgeLogOutput(ELogLevel_Type::ELogLevelError, "GetTopoGraph failed, HDMap not initialized yet.");
		return false;
	}
#endif
	return HDMapStandalone::MRouting::GetTopoGraph(topoGraph);
}

double SimOneAPIService::GetLaneLength(const SSD::SimString& id)
{
#ifndef API_TEST_LOADXODR_OFFLINE
	if (!mbHDMapInited)
	{
		bridgeLogOutput(ELogLevel_Type::ELogLevelError, "GetLaneLength failed, HDMap not initialized yet.");
		return 0.0;
	}
#endif
	return HDMapStandalone::MHDMap::GetLaneLength(id);
}

bool SimOneAPIService::IsDriving(const SSD::SimString& id)
{
#ifndef API_TEST_LOADXODR_OFFLINE
	if (!mbHDMapInited)
	{
		bridgeLogOutput(ELogLevel_Type::ELogLevelError, "IsDriving failed, HDMap not initialized yet.");
		return false;
	}
#endif
	return HDMapStandalone::MHDMap::IsDriving(id);
}

bool SimOneAPIService::IsInJunction(const SSD::SimString& id, long& juncId)
{
#ifndef API_TEST_LOADXODR_OFFLINE
	if (!mbHDMapInited)
	{
		bridgeLogOutput(ELogLevel_Type::ELogLevelError, "IsInJunction failed, HDMap not initialized yet.");
		return false;
	}
#endif
	return HDMapStandalone::MHDMap::IsInJunction(id, juncId);
}

bool SimOneAPIService::IsInsideLane(const SSD::SimPoint3D& inputPt, const SSD::SimString& laneName, HDMapStandalone::MSideState& sideState)
{
#ifndef API_TEST_LOADXODR_OFFLINE
	if (!mbHDMapInited)
	{
		bridgeLogOutput(ELogLevel_Type::ELogLevelError, "IsInsideLane failed, HDMap not initialized yet.");
		return false;
	}
#endif
	return HDMapStandalone::MLocation::IsInsideLane(inputPt, laneName, sideState);
}

bool SimOneAPIService::GetNearMostLaneWithHeight(const SSD::SimPoint3D& pos, bool drivingOnly, SSD::SimString& id, double& s, double& t,
	double& s_toCenterLine, double& t_toCenterLine, bool& insideLane)
{
#ifndef API_TEST_LOADXODR_OFFLINE
	if (!mbHDMapInited)
	{
		bridgeLogOutput(ELogLevel_Type::ELogLevelError, "GetNearMostLaneWithHeight failed, HDMap not initialized yet.");
		return false;
	}
#endif
	return HDMapStandalone::MLocation::GetNearMostLaneWithHeight_V2(pos, drivingOnly, id, s, t,
		s_toCenterLine, t_toCenterLine, insideLane);
}

bool SimOneAPIService::GetForwardLaneSample(const SSD::SimPoint3D& inputPt, const SSD::SimString& laneName, const double& forward,
	SSD::SimVector<HDMapStandalone::MLaneInfo>& laneInfoList)
{
#ifndef API_TEST_LOADXODR_OFFLINE
	if (!mbHDMapInited)
	{
		bridgeLogOutput(ELogLevel_Type::ELogLevelError, "GetForwardLaneSample failed, HDMap not initialized yet.");
		return false;
	}
#endif
	laneInfoList = HDMapStandalone::MHDMap::GetLaneSample(inputPt, laneName, forward);
	return true;
}

void SimOneAPIService::GetLaneLineInfo(SSD::SimVector<HDMapStandalone::MLaneLineInfo>& laneLineInfo)
{
#ifndef API_TEST_LOADXODR_OFFLINE
	if (!mbHDMapInited)
	{
		bridgeLogOutput(ELogLevel_Type::ELogLevelError, "GetLaneLineInfo failed, HDMap not initialized yet.");
		return;
	}
#endif
	laneLineInfo = HDMapStandalone::MHDMap::GetLaneLineInfo();
}



void SimOneAPIService::GetSectionList(const long& roadId, SSD::SimStringVector& rightList, SSD::SimStringVector& leftList)
{
#ifndef API_TEST_LOADXODR_OFFLINE
	if (!mbHDMapInited)
	{
		bridgeLogOutput(ELogLevel_Type::ELogLevelError, "GetSectionList failed, HDMap not initialized yet.");
		return;
	}
#endif
	HDMapStandalone::MHDMap::GetSectionList(roadId, rightList, leftList);
}

SSD::SimVector<int> SimOneAPIService::GetLaneIndexList(const SSD::SimPoint3D& pos, int& currentLaneIndex, SSD::SimStringVector& laneIdList)
{
	SSD::SimVector<int> ret;
#ifndef API_TEST_LOADXODR_OFFLINE
	if (!mbHDMapInited)
	{
		bridgeLogOutput(ELogLevel_Type::ELogLevelError, "GetLaneIndexList failed, HDMap not initialized yet.");
		return ret;
	}
#endif
	SSD::SimString laneId;
	double s, t, s_toCenterLine, t_toCenterLine;
	if (HDMapStandalone::MLocation::GetNearMostLane_V2(pos, laneId, s, t, s_toCenterLine, t_toCenterLine))
	{
		std::string strLaneId(laneId.GetString());
		int idLane = UtilString::FromString<int>(UtilId::getLaneId(strLaneId));
		if (idLane < 0)
		{
			currentLaneIndex = -idLane - 1;
		}
		else
		{
			currentLaneIndex = idLane - 1;
		}
		laneIdList = HDMapStandalone::MHDMap::GetSectionLaneList(laneId);
		for (auto& id : laneIdList)
		{
			int currentId = UtilString::FromString<int>(UtilId::getLaneId(std::string(id.GetString())));
			int index = 0;
			if (currentId < 0)
			{
				index = -currentId - 1;
			}
			else
			{
				index = currentId - 1;
			}
			ret.push_back(index);
		}
	}
	return std::move(ret);
}

SimOneAPI::EDirectionType_ SimOneAPIService::GetIconType(const SSD::SimPoint3D& pos)
{
#ifndef API_TEST_LOADXODR_OFFLINE
	if (!mbHDMapInited)
	{
		bridgeLogOutput(ELogLevel_Type::ELogLevelError, "GetIconType failed, HDMap not initialized yet.");
		return SimOneAPI::EDirectionType_::Forward;
	}
#endif
	SSD::SimString laneId;
	double s, t, s_toCenterLine, t_toCenterLine;
	if (HDMapStandalone::MLocation::GetNearMostLane_V2(pos, laneId, s, t, s_toCenterLine, t_toCenterLine))
	{
		return GetIconType_(pos, laneId);
	}
	return SimOneAPI::EDirectionType_::Forward;
}

bool SimOneAPIService::GetLaneSampleByLocation(const SSD::SimPoint3D& pos, HDMapStandalone::MLaneInfo& info)
{
#ifndef API_TEST_LOADXODR_OFFLINE
	if (!mbHDMapInited)
	{
		bridgeLogOutput(ELogLevel_Type::ELogLevelError, "GetLaneSampleByLocation failed, HDMap not initialized yet.");
		return SimOneAPI::EDirectionType_::Forward;
	}
#endif
	SSD::SimString laneId;
	double s, t, s_toCenterLine, t_toCenterLine;
	bool ret = HDMapStandalone::MLocation::GetNearMostLane_V2(pos, laneId, s, t, s_toCenterLine, t_toCenterLine);
	if (ret)
	{
		info = HDMapStandalone::MHDMap::GetLaneSample(laneId);
	}
	return ret;
}
#endif
#ifndef WITHOUT_HDMAP

void SimOneAPIService::setMapInfo(std::uint16_t type, const std::string* msgDataBody) {
	if (msgDataBody->size() != sizeof(SimOne_Data_Map)) {
		return;
	}
	memcpy(&mHDMapInfo, msgDataBody->c_str(), msgDataBody->size());
	bridgeLogOutput(ELogLevel_Type::ELogLevelInformation, "HDMapInfo openDrive:%s", mHDMapInfo.openDrive);
	bridgeLogOutput(ELogLevel_Type::ELogLevelInformation, "HDMapInfo opendriveMd5:%s", mHDMapInfo.opendriveMd5);
	bridgeLogOutput(ELogLevel_Type::ELogLevelInformation, "HDMapInfo openDriveUrl:%s", mHDMapInfo.openDriveUrl);
	return;
}

bool SimOneAPIService::GetHDMapData(SimOne_Data_Map& hdMap)
{
	if (&(GetInstance()->mHDMapInfo) != NULL)
	{
		memcpy(&hdMap, &(GetInstance()->mHDMapInfo), sizeof(SimOne_Data_Map));
		return true;
	}
	return false;
}

bool SimOneAPIService::GetHDMapInfo(SimOne_Data_Map* pHDMap) {

	if (!pHDMap)
		return false;
	memcpy(pHDMap, &mHDMapInfo, sizeof(SimOne_Data_Map));
	return true;
}

bool SimOneAPIService::isHDMapSMReady()
{
	if (&(GetInstance()->mHDMapInfo) != NULL)
	{
		SimOne_Data_Map hdMap;
		memcpy(&hdMap, &(GetInstance()->mHDMapInfo), sizeof(SimOne_Data_Map));
		return true;
	}
	return false;
}

bool SimOneAPIService::checkHDMapSM()
{
	if (&(GetInstance()->mHDMapInfo) != NULL)
	{
		memcpy(&mLastHDMap, &(GetInstance()->mHDMapInfo), sizeof(SimOne_Data_Map));
		return startHDMap();
	}
	return true;
}

bool SimOneAPIService::startHDMap()
{
	bridgeLogOutput(ELogLevel_Type::ELogLevelInformation, "HDMap initializing...");
	bridgeLogOutput(ELogLevel_Type::ELogLevelInformation, "HDMap opendrive: %s", mLastHDMap.openDrive);
	bridgeLogOutput(ELogLevel_Type::ELogLevelInformation, "HDMap opendrive URL: %s", mLastHDMap.openDriveUrl);
	bridgeLogOutput(ELogLevel_Type::ELogLevelInformation, "HDMap opendrive Md5: %s", mLastHDMap.opendriveMd5);
	std::string OD(mLastHDMap.openDrive);
	std::string Url(mLastHDMap.openDriveUrl);
	//string_replace(Url, " ", "%20"); //handle space in file name case

	std::string fileContent;
	{ // Simple example
		try {
			fileContent = UtilUrlRequest::GetUrl(Url);
		}
		catch (const UtilUrlRequest::TException& e) {
			bridgeLogOutput(ELogLevel_Type::ELogLevelError, "Fetching xodr failed: %s", e.what());
		}
	}

	std::string odv001(ODV0001);
	std::vector<char> headChar;
	headChar.resize(odv001.size() + 1);
	headChar[odv001.size()] = '\0';
	if (fileContent.substr(0, odv001.size()) == odv001)
	{
		// Decode xodr with v0001
		std::string odRaw;
		for (std::size_t i = odv001.size(); i < fileContent.size(); i = i + 2)
		{
			char a = fileContent.data()[i];
			char b = fileContent.data()[i + 1];
			char c = b + ((std::uint8_t)a * (std::uint8_t)a) % 256;
			odRaw.append(&c, 1);
		}
		fileContent = odRaw;
	}

	HDMapStandalone::MLoadErrorCode code;
	SSD::SimString content(fileContent.c_str());
	bool ret = HDMapStandalone::MHDMap::LoadDataFromContent(content, code);
	if (!ret)
	{
		bridgeLogOutput(ELogLevel_Type::ELogLevelError, "Failed to load xodr file!");
	}
	mbHDMapInited = true;
	bridgeLogOutput(ELogLevel_Type::ELogLevelInformation, "HDMap Initialize success");
	return ret;
}

void SimOneAPIService::stopHDMap()
{
	memset(&mLastHDMap, 0, sizeof(mLastHDMap));
	mbHDMapInited = false;
	bridgeLogOutput(ELogLevel_Type::ELogLevelInformation, "HDMap stopped");
}
#endif  //WITHOUT_HDMAP
