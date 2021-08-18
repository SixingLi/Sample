#ifndef WITHOUT_SENSOR
#ifndef WITHOUT_HDMAP
#include "TaskV2XNFS.hpp"
#include "cybertron/core/Log.hpp"
#include "cybertron/network/Message.hpp"
#include "cybertron/core/UtilString.hpp"
#include "cybertron/core/JsonReader.hpp"
#include <iostream>

using namespace std;
CYBERTRON_BEGIN

TaskV2XNFS::TaskV2XNFS()
{

}

TaskV2XNFS::~TaskV2XNFS()
{

}

uint16_t TaskV2XNFS::Do(std::uint32_t sensorType, std::uint32_t commanId, CTaskSensorBase::SensorContext* pSensorContext, const std::string* pBuffer)
{
	if (commanId != cybertron::proto::sensor::EDataType_V2XNFS) {
		return -1;
	}
	cybertron::proto::sensor::V2XDetection V2XDetectionsIn;
	if (!V2XDetectionsIn.ParseFromString(*pBuffer)) {
		return 0;
	}else{
		DoSendMsgFrame(pSensorContext, V2XDetectionsIn.v2xsensormssage());
	}
	return 1;
}

void TaskV2XNFS::DoSendMsgFrame(SensorContext* pSensorContext, const std::string &pBuffer) {

	SimOne_Data_V2XNFS V2xMsgBSMFrame;
	SimOne_Data_V2XNFS V2xMsgRSMFrame;
	SimOne_Data_V2XNFS V2xMsgMAPFrame;
	SimOne_Data_V2XNFS V2xMsgRSIFrame;
	SimOne_Data_V2XNFS V2xMsgSPATFrame;

	SimOne_Data_V2XNFS *pV2xMsgBSMFrame = NULL;
	SimOne_Data_V2XNFS *pV2xMsgRSMFrame = NULL;
	SimOne_Data_V2XNFS *pV2xMsgMAPFrame = NULL;
	SimOne_Data_V2XNFS *pV2xMsgRSIFrame = NULL;
	SimOne_Data_V2XNFS *pV2xMsgSPATFrame = NULL;

	std::string v2xInfoType = pSensorContext->channel;
	const std::string sensorId = SimOneAPIService::GetInstance()->GetSensorIdFromId(pSensorContext->sensorId);
	const string sensorKey = std::to_string(pSensorContext->mainVehicleId).append("_").append(sensorId).append("_").append(v2xInfoType);
	
	if(v2xInfoType==std::to_string(MessageFrame_PR_bsmFrame)){
		std::unique_lock<std::recursive_mutex> lock(mLastV2XDetectionBSMMapLock);
		SimOne_Data_V2XDetectionMap::iterator it = mLastV2XDetectionBSMMap.find(sensorKey);
		if (it != mLastV2XDetectionBSMMap.end()) {
			pV2xMsgBSMFrame = it->second;
		}
		else
		{
			pV2xMsgBSMFrame = new SimOne_Data_V2XNFS;
			mLastV2XDetectionBSMMap[sensorKey] = pV2xMsgBSMFrame;
		}

		pV2xMsgBSMFrame->timestamp = pSensorContext->timestamp;
		pV2xMsgBSMFrame->frame = pSensorContext->frame;
		pV2xMsgBSMFrame->V2XMsgFrameSize = pBuffer.size();
		strcpy(pV2xMsgBSMFrame->MsgFrameData, pBuffer.c_str());

		if (TaskSensorManager::getInstance().mpSimOneV2XRawCB != NULL)
		{
			TaskSensorManager::getInstance().mpSimOneV2XRawCB(pSensorContext->mainVehicleId, sensorId.c_str(), pV2xMsgBSMFrame);
		}
	}
	else if (v2xInfoType == std::to_string(MessageFrame_PR_mapFrame)) {
		std::unique_lock<std::recursive_mutex> lock(mLastV2XDetectionMAPMapLock);
		SimOne_Data_V2XDetectionMap::iterator it = mLastV2XDetectionMAPMap.find(sensorKey);
		if (it != mLastV2XDetectionMAPMap.end()) {
			pV2xMsgMAPFrame = it->second;
		}
		else
		{
			pV2xMsgMAPFrame = new SimOne_Data_V2XNFS;
			mLastV2XDetectionMAPMap[sensorKey] = pV2xMsgMAPFrame;
		}

		pV2xMsgMAPFrame->timestamp = pSensorContext->timestamp;
		pV2xMsgMAPFrame->frame = pSensorContext->frame;
		pV2xMsgMAPFrame->V2XMsgFrameSize = pBuffer.size();
		strcpy(pV2xMsgMAPFrame->MsgFrameData, pBuffer.c_str());

		if (TaskSensorManager::getInstance().mpSimOneV2XRawCB != NULL)
		{
			TaskSensorManager::getInstance().mpSimOneV2XRawCB(pSensorContext->mainVehicleId, sensorId.c_str(), pV2xMsgMAPFrame);
		}
	}else if (v2xInfoType == std::to_string(MessageFrame_PR_rsmFrame)) {
		std::unique_lock<std::recursive_mutex> lock(mLastV2XDetectionRSMMapLock);
		SimOne_Data_V2XDetectionMap::iterator it = mLastV2XDetectionRSMMap.find(sensorKey);
		if (it != mLastV2XDetectionRSMMap.end()) {
			pV2xMsgRSMFrame = it->second;
		}
		else
		{
			pV2xMsgRSMFrame = new SimOne_Data_V2XNFS;
			mLastV2XDetectionRSMMap[sensorKey] = pV2xMsgRSMFrame;
		}

		pV2xMsgRSMFrame->timestamp = pSensorContext->timestamp;
		pV2xMsgRSMFrame->frame = pSensorContext->frame;
		pV2xMsgRSMFrame->V2XMsgFrameSize = pBuffer.size();
		strcpy(pV2xMsgRSMFrame->MsgFrameData, pBuffer.c_str());

		if (TaskSensorManager::getInstance().mpSimOneV2XRawCB != NULL)
		{
			TaskSensorManager::getInstance().mpSimOneV2XRawCB(pSensorContext->mainVehicleId, sensorId.c_str(), pV2xMsgRSMFrame);
		}
	}else if (v2xInfoType == std::to_string(MessageFrame_PR_spatFrame)) {
		std::unique_lock<std::recursive_mutex> lock(mLastV2XDetectionSPATMapLock);
		SimOne_Data_V2XDetectionMap::iterator it = mLastV2XDetectionSPATMap.find(sensorKey);
		if (it != mLastV2XDetectionSPATMap.end()) {
			pV2xMsgSPATFrame = it->second;
		}
		else
		{
			pV2xMsgSPATFrame = new SimOne_Data_V2XNFS;
			mLastV2XDetectionSPATMap[sensorKey] = pV2xMsgSPATFrame;
		}

		pV2xMsgSPATFrame->timestamp = pSensorContext->timestamp;
		pV2xMsgSPATFrame->frame = pSensorContext->frame;
		pV2xMsgSPATFrame->V2XMsgFrameSize = pBuffer.size();
		strcpy(pV2xMsgSPATFrame->MsgFrameData, pBuffer.c_str());

		if (TaskSensorManager::getInstance().mpSimOneV2XRawCB != NULL)
		{
			TaskSensorManager::getInstance().mpSimOneV2XRawCB(pSensorContext->mainVehicleId, sensorId.c_str(), pV2xMsgSPATFrame);
		}
	}else if (v2xInfoType == std::to_string(MessageFrame_PR_rsiFrame)) {
		std::unique_lock<std::recursive_mutex> lock(mLastV2XDetectionRSIMapLock);
		SimOne_Data_V2XDetectionMap::iterator it = mLastV2XDetectionRSIMap.find(sensorKey);
		if (it != mLastV2XDetectionRSIMap.end()) {
			pV2xMsgRSIFrame = it->second;
		}
		else
		{
			pV2xMsgRSIFrame = new SimOne_Data_V2XNFS;
			mLastV2XDetectionRSIMap[sensorKey] = pV2xMsgRSIFrame;
		}

		pV2xMsgRSIFrame->timestamp = pSensorContext->timestamp;
		pV2xMsgRSIFrame->frame = pSensorContext->frame;
		pV2xMsgRSIFrame->V2XMsgFrameSize = pBuffer.size();
		strcpy(pV2xMsgRSIFrame->MsgFrameData, pBuffer.c_str());

		if (TaskSensorManager::getInstance().mpSimOneV2XRawCB != NULL)
		{
			TaskSensorManager::getInstance().mpSimOneV2XRawCB(pSensorContext->mainVehicleId, sensorId.c_str(), pV2xMsgRSIFrame);
		}
	}
}

bool TaskV2XNFS::GetData(std::string key, ETaskCommandId commandId, void * pBuffer)
{
#ifndef WITHOUT_HDMAP
	switch (commandId) {
		case ETaskCommandId_V2XNFSRawBSM:
		{
			SimOne_Data_V2XDetectionMap::iterator it = mLastV2XDetectionBSMMap.find(key);
			if (it == mLastV2XDetectionBSMMap.end())
			{
				return false;
			}
			memcpy(pBuffer, it->second, sizeof(SimOne_Data_V2XNFS));
			break;
		}
		case ETaskCommandId_V2XNFSRawRSM: 
		{
			SimOne_Data_V2XDetectionMap::iterator it = mLastV2XDetectionRSMMap.find(key);
			if (it == mLastV2XDetectionRSMMap.end())
			{
				return false;
			}
			memcpy(pBuffer, it->second, sizeof(SimOne_Data_V2XNFS));
			break;
		}

		case ETaskCommandId_V2XNFSRawMAP:
		{
			SimOne_Data_V2XDetectionMap::iterator it = mLastV2XDetectionMAPMap.find(key);
			if (it == mLastV2XDetectionMAPMap.end())
			{
				return false;
			}
			memcpy(pBuffer, it->second, sizeof(SimOne_Data_V2XNFS));
			break;
		}
		case ETaskCommandId_V2XNFSRawRSI:
		{
			SimOne_Data_V2XDetectionMap::iterator it = mLastV2XDetectionRSIMap.find(key);
			if (it == mLastV2XDetectionRSIMap.end())
			{
				return false;
			}
			memcpy(pBuffer, it->second, sizeof(SimOne_Data_V2XNFS));
			break;
		}
		case ETaskCommandId_V2XNFSRawSPAT:
		{
			SimOne_Data_V2XDetectionMap::iterator it = mLastV2XDetectionSPATMap.find(key);
			if (it == mLastV2XDetectionSPATMap.end())
			{
				return false;
			}
			memcpy(pBuffer, it->second, sizeof(SimOne_Data_V2XNFS));
			break;
		}
	}
#endif
	return true;
}

CYBERTRON_END
#endif
#endif