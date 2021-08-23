#ifndef WITHOUT_SENSOR
#include "TaskPerfectPerception.hpp"
#include "cybertron/core/Log.hpp"
#include "cybertron/network/Message.hpp"
#include "cybertron/core/UtilString.hpp"
#include "cybertron/core/JsonReader.hpp"
#pragma warning(disable:4267)
#pragma warning(disable:4244)

CYBERTRON_BEGIN
TaskPerfectPerception::TaskPerfectPerception()
{
	//mSensorDataType = EDataType_ObstacleDetection;
}

TaskPerfectPerception::~TaskPerfectPerception()
{

}

bool TaskPerfectPerception::processObstacleDetection(SensorContext* pSensorContext, const std::string* pBuffer) {
	if (!mPerfectPerceptionDetections.ParseFromString(*pBuffer)) {
		return false;
	}
	const std::string sensorId = SimOneAPIService::GetInstance()->GetSensorIdFromId(pSensorContext->sensorId);
	const string sensorKey = std::to_string(pSensorContext->mainVehicleId).append("_").append(sensorId);
	//Detections
	if (SimOneAPIService::GetInstance()->IsNeedSendObjectbasedData())
	{

		SimOne_Data_SensorDetections *pPerfectPerceptionGroundTruth = NULL;
		SimOne_Data_SensorDetectionsMap::iterator it = mLastSensorDetectionsMap.find(sensorKey);
		if (it != mLastSensorDetectionsMap.end()) {
			pPerfectPerceptionGroundTruth = it->second;
		}
		else
		{
			pPerfectPerceptionGroundTruth = new SimOne_Data_SensorDetections;

		}

		pPerfectPerceptionGroundTruth->frame = pSensorContext->frame;
		pPerfectPerceptionGroundTruth->timestamp = pSensorContext->timestamp;
		pPerfectPerceptionGroundTruth->objectSize = mPerfectPerceptionDetections.obstacles().size();
		for (auto i = 0; i < mPerfectPerceptionDetections.obstacles().size(); i++)
		{
			pPerfectPerceptionGroundTruth->objects[i].id = mPerfectPerceptionDetections.obstacles(i).id();
			pPerfectPerceptionGroundTruth->objects[i].type = (SimOne_Obstacle_Type)mPerfectPerceptionDetections.obstacles(i).type();
			pPerfectPerceptionGroundTruth->objects[i].posX = mPerfectPerceptionDetections.obstacles(i).center().x();
			pPerfectPerceptionGroundTruth->objects[i].posY = mPerfectPerceptionDetections.obstacles(i).center().y();
			pPerfectPerceptionGroundTruth->objects[i].posZ = mPerfectPerceptionDetections.obstacles(i).center().z();
			pPerfectPerceptionGroundTruth->objects[i].oriX = mPerfectPerceptionDetections.obstacles(i).rotation().x();
			pPerfectPerceptionGroundTruth->objects[i].oriY = mPerfectPerceptionDetections.obstacles(i).rotation().y();
			pPerfectPerceptionGroundTruth->objects[i].oriZ = mPerfectPerceptionDetections.obstacles(i).rotation().z();
			pPerfectPerceptionGroundTruth->objects[i].length = mPerfectPerceptionDetections.obstacles(i).size().x();
			pPerfectPerceptionGroundTruth->objects[i].width = mPerfectPerceptionDetections.obstacles(i).size().y();
			pPerfectPerceptionGroundTruth->objects[i].height = mPerfectPerceptionDetections.obstacles(i).size().z();
			pPerfectPerceptionGroundTruth->objects[i].range = mPerfectPerceptionDetections.obstacles(i).range();
			pPerfectPerceptionGroundTruth->objects[i].velX = mPerfectPerceptionDetections.obstacles(i).velocity().x();
			pPerfectPerceptionGroundTruth->objects[i].velY = mPerfectPerceptionDetections.obstacles(i).velocity().y();
			pPerfectPerceptionGroundTruth->objects[i].velZ = mPerfectPerceptionDetections.obstacles(i).velocity().z();
			pPerfectPerceptionGroundTruth->objects[i].probability = mPerfectPerceptionDetections.obstacles(i).probability();
			pPerfectPerceptionGroundTruth->objects[i].relativePosX = mPerfectPerceptionDetections.obstacles(i).relativepos().x();
			pPerfectPerceptionGroundTruth->objects[i].relativePosY = mPerfectPerceptionDetections.obstacles(i).relativepos().y();
			pPerfectPerceptionGroundTruth->objects[i].relativePosZ = mPerfectPerceptionDetections.obstacles(i).relativepos().z();
			pPerfectPerceptionGroundTruth->objects[i].relativeRotX = mPerfectPerceptionDetections.obstacles(i).relativerot().x();
			pPerfectPerceptionGroundTruth->objects[i].relativeRotY = mPerfectPerceptionDetections.obstacles(i).relativerot().y();
			pPerfectPerceptionGroundTruth->objects[i].relativeRotZ = mPerfectPerceptionDetections.obstacles(i).relativerot().z();
			pPerfectPerceptionGroundTruth->objects[i].relativeVelX = mPerfectPerceptionDetections.obstacles(i).relativevel().x();
			pPerfectPerceptionGroundTruth->objects[i].relativeVelY = mPerfectPerceptionDetections.obstacles(i).relativevel().y();
			pPerfectPerceptionGroundTruth->objects[i].relativeVelZ = mPerfectPerceptionDetections.obstacles(i).relativevel().z();
		}
		{
			std::unique_lock<std::recursive_mutex> lock(mLastSensorDetectionsMapLock);
			mLastSensorDetectionsMap[sensorKey] = pPerfectPerceptionGroundTruth;
		}
		if (TaskSensorManager::getInstance().mpSensorDetectionsUpdateCB != NULL)
		{
			TaskSensorManager::getInstance().mpSensorDetectionsUpdateCB(pSensorContext->mainVehicleId, sensorId.c_str(), pPerfectPerceptionGroundTruth);
		}
	}
	return true;
}
bool TaskPerfectPerception::processGroundTruth(SensorContext* pSensorContext, const std::string* pBuffer) {
	cybertron::proto::sensor::GroundTruth groundTruthMsg;
	if (!groundTruthMsg.ParseFromString(*pBuffer)) {
		return false;
	}

	SimOne_Data_Obstacle *pObstacle = NULL;
	int mainVehicleId = pSensorContext->mainVehicleId;
	SimOne_Data_ObstacleMap::iterator it = mLastObstacleMap.find(mainVehicleId);

	if (it != mLastObstacleMap.end()) {
		pObstacle = it->second;
	}
	else
	{
		pObstacle = new SimOne_Data_Obstacle;

	}
	pObstacle->obstacleSize = groundTruthMsg.obstacles().size();
	pObstacle->frame = pSensorContext->frame;
	pObstacle->timestamp = pSensorContext->timestamp;
	for (int i = 0; i < groundTruthMsg.obstacles().size(); i++)
	{
		pObstacle->obstacle[i].id = groundTruthMsg.obstacles(i).id();
		pObstacle->obstacle[i].type = (SimOne_Obstacle_Type)groundTruthMsg.obstacles(i).type();
		pObstacle->obstacle[i].theta = groundTruthMsg.obstacles(i).rotation().z();
		pObstacle->obstacle[i].viewId = std::stoi(groundTruthMsg.obstacles(i).subtype());
		pObstacle->obstacle[i].posX = groundTruthMsg.obstacles(i).center().x();
		pObstacle->obstacle[i].posY = groundTruthMsg.obstacles(i).center().y();
		pObstacle->obstacle[i].posZ = groundTruthMsg.obstacles(i).center().z();
		pObstacle->obstacle[i].oriX = groundTruthMsg.obstacles(i).rotation().x();
		pObstacle->obstacle[i].oriY = groundTruthMsg.obstacles(i).rotation().y();
		pObstacle->obstacle[i].oriZ = groundTruthMsg.obstacles(i).rotation().z();
		pObstacle->obstacle[i].velX = groundTruthMsg.obstacles(i).velocity().x();
		pObstacle->obstacle[i].velY = groundTruthMsg.obstacles(i).velocity().y();
		pObstacle->obstacle[i].velZ = groundTruthMsg.obstacles(i).velocity().z();
		pObstacle->obstacle[i].length = groundTruthMsg.obstacles(i).size().x();
		pObstacle->obstacle[i].width = groundTruthMsg.obstacles(i).size().y();
		pObstacle->obstacle[i].height = groundTruthMsg.obstacles(i).size().z();
	}
	{
		std::unique_lock<std::recursive_mutex> lock(mLastObstacleMapLock);
		mLastObstacleMap[mainVehicleId] = pObstacle;
	}
	if (TaskSensorManager::getInstance().mpObstacleUpdateCB != NULL)
	{
		TaskSensorManager::getInstance().mpObstacleUpdateCB(mainVehicleId, pObstacle);
	}
	if (TaskSensorManager::getInstance().mpSimOneGroundTruthCB != NULL)
	{
		TaskSensorManager::getInstance().mpSimOneGroundTruthCB(pObstacle);
	}

	return true;
}
uint16_t TaskPerfectPerception::Do(std::uint32_t sensorType, std::uint32_t commanId, CTaskSensorBase::SensorContext* pSensorContext, const std::string* pBuffer)
{
	switch (commanId)
	{
	case cybertron::proto::sensor::EDataType_ObstacleDetection:
	{
		processObstacleDetection(pSensorContext, pBuffer);
	}
	break;
	case cybertron::proto::sensor::EDataType_GroundTruth:
	{
		processGroundTruth(pSensorContext, pBuffer);
	}
	break;
	default:
		break;
	}
	
	return 0;
}
bool TaskPerfectPerception::GetData(std::string key, ETaskCommandId commandId, void * pBuffer)
{
	if (commandId == ETaskCommandId_PerfectPerceptionObj)
	{
		SimOne_Data_SensorDetectionsMap::iterator it = mLastSensorDetectionsMap.find(key);
		if (it == mLastSensorDetectionsMap.end())
		{
			return false;
		}
		memcpy(pBuffer, it->second, sizeof(SimOne_Data_SensorDetections));
	}else if (commandId == ETaskCommandId_PerfectPerceptionGroundTruth) {
		int mainVehicle = atoi(key.c_str());
		SimOne_Data_ObstacleMap::iterator it = mLastObstacleMap.find(mainVehicle);
		if (it == mLastObstacleMap.end())
		{
			return false;
		}
		memcpy(pBuffer, it->second, sizeof(SimOne_Data_Obstacle));
		return true;
	}
	else {
		return false;
	}

	return true;
}
CYBERTRON_END
#endif