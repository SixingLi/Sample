#pragma once
#ifndef WITHOUT_SENSOR
#include "cybertron/DefinesCore.hpp"
#include "Sensor/SensorCommon.pb.h"
#include <atomic>
#include <string>
#include "Service/SimOneNetService.hpp"
#include "cybertron/node/NodeHotAreaManager.hpp"
#include "Node/Bridge.pb.h"
#include "cybertron/sensor/util/SensorConfigUtil.hpp"

#pragma warning(disable:4267)
#pragma warning(disable:4244)

enum ETaskCommandId
{
	ETaskCommandId_ImageRaw = 10,
	ETaskCommandId_ImageObj = 11,
	ETaskCommandId_ImageOSI = 12,
	ETaskCommandId_ImageLane = 13,

	ETaskCommandId_ContiRadarObj = 20,
	ETaskCommandId_ContiRadarOSI = 21,

	ETaskCommandId_PerfectPerceptionOSI = 30,
	ETaskCommandId_PerfectPerceptionObj = 31,
	ETaskCommandId_PerfectPerceptionGroundTruth = 32,
	ETaskCommandId_PerfectPerceptionGroundTruthOSI = 33,

	ETaskCommandId_PointCloudRaw = 40,
	ETaskCommandId_PointCloudObj = 41,
	ETaskCommandId_PointCloudOSI = 42,

	ETaskCommandId_UltrasonicRadarObj = 50,
	ETaskCommandId_UltrasonicRadarOSI = 51,

	ETaskCommandId_V2XBSMData = 60,
	ETaskCommandId_V2XRSIData = 61,
	ETaskCommandId_V2XSPAData = 62,
	ETaskCommandId_V2XNFSRawBSM = 63,
	ETaskCommandId_V2XNFSRawMAP = 64,
	ETaskCommandId_V2XNFSRawRSM = 65,
	ETaskCommandId_V2XNFSRawSPAT = 66,
	ETaskCommandId_V2XNFSRawRSI = 67,


	ETaskCommandId_FusionObj = 70,
	ETaskCommandId_FusionLane = 71
};
CYBERTRON_BEGIN
class CTaskSensorBase
{
public:
	struct BridgePerformanceTest
	{
		std::uint64_t startNodeTime = 0;
		std::uint64_t bridgeIORecvTime = 0;
		std::uint64_t bridgeIOSendTime = 0;
		std::uint64_t simOneNetIOTime = 0;
	};

	struct SensorContext {
		std::uint32_t commandId;
		double timestamp;
		double time_of_simulation;
		std::uint32_t frame;
		std::uint32_t version;
		std::uint32_t mainVehicleId;
		std::uint32_t sensorType;
		std::uint32_t sensorId;
		std::string channel;
		BridgePerformanceTest currentTime;
	};
	CTaskSensorBase() { 
		//mbEnable = false;
	}
	virtual ~CTaskSensorBase() {}
	static const char* int2string(int id) {
		return std::to_string(id).c_str();
	}
	void makeSensorHeader(SensorContext* pContext, Bridge::BridgeSensorHeader* pHeader) {
		pHeader->set_timestamp(pContext->timestamp);
		pHeader->set_time_of_simulation(pContext->time_of_simulation);
		pHeader->set_frame(pContext->frame);
		pHeader->set_commandid(pContext->commandId);
		pHeader->set_sensortype(pContext->sensorType);
		pHeader->set_mainvehicleid(pContext->mainVehicleId);
		pHeader->set_sensorid(pContext->sensorId);
		pHeader->set_channel(pContext->channel);
		pHeader->mutable_currenttime()->set_startnodetime(pContext->currentTime.startNodeTime);
		pHeader->mutable_currenttime()->set_bridgeiorecvtime(pContext->currentTime.bridgeIORecvTime);
		pHeader->mutable_currenttime()->set_bridgeiosendtime(std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count() / 1000.0);
	}
	virtual bool  GetData(std::string key, ETaskCommandId commandId, void * pBuffer) = 0;
	virtual uint16_t  Do(std::uint32_t sensorType, std::uint32_t commanId, CTaskSensorBase::SensorContext* pSensorContext, const std::string* pBuffer) = 0;
	virtual int GetCommandIDFromObj() { return -1; };
	virtual int GetCommandIDFromLane() { return -1; };
	std::uint16_t GetSensorDataType() { return mSensorDataType; }
	void Enbale() {
		mbEnable = true;
	}
	bool IsEnbale() {
		return mbEnable;
	}
protected:
	std::uint16_t mSensorDataType;
	Message	mResultMessage;
	bool mbEnable;
};

class TaskSensorManager
{
public:
	TaskSensorManager();
	~TaskSensorManager();
	CTaskSensorBase* FindTask(std::uint16_t taskType);
	void ManagerInit();
	void ManagerClear();
	static TaskSensorManager& getInstance();
	void Do(std::uint32_t type, std::uint32_t commanId,CTaskSensorBase::SensorContext* pSensorContext,const std::string* pBuffer);

	void(*mpImageUpdateCB)(const char* mainVehicleId, const char* sensorId, SimOne_Data_Image *pImage);
	void(*mpPointCloudUpdateCB)(const char* mainVehicleId, const char* sensorId, SimOne_Data_Point_Cloud *pPointCloud);
	void(*mpRadarDetectionsCB)(const char* mainVehicleId, const char* sensorId, SimOne_Data_RadarDetection *pDetections);
	void(*mpSensorFusionUpdateCB)(const char* mainVehicleId, const char* sensorId, SimOne_Data_SensorFusionObstacles *pDetections);
	void(*mpSensorDetectionsUpdateCB)(const char* mainVehicleId, const char* sensorId, SimOne_Data_SensorDetections *pGroundtruth);
	void(*mpLaneDetectionsUpdateCB)(const char* mainVehicleId, const char* sensorId, SimOne_Data_LaneInfo *pLane);
	void(*mpUltrasonicRadarsUpdateCB)(const char* mainVehicleId, SimOne_Data_UltrasonicRadars *pUltrasonics);
	void(*mpUltrasonicRadarUpdateCB)(const char* mainVehicleId, const char* sensorId, SimOne_Data_UltrasonicRadar *pUltrasonics);
	void(*mpObstacleUpdateCB)(const char* mainVehicleId, SimOne_Data_Obstacle *pObstacle);
	void(*mpSimOneGroundTruthCB)(SimOne_Data_Obstacle *pObstacle);

#ifndef WITHOUT_HDMAP
	void(*mpSimOneV2XRawCB)(const char* mainVehicleId, const char* sensorId, SimOne_Data_V2XNFS *pV2XDetections);
#endif

protected:
	typedef  std::map<uint16_t, CTaskSensorBase*> TaskSensorMap;
	TaskSensorMap mTaskSensorDataTypeMap;
};
CYBERTRON_END
#endif
