#ifndef WITHOUT_SENSOR

#include "TaskPointCloud.hpp"
#include "cybertron/core/UtilString.hpp"

#pragma warning(disable:4267)
#pragma warning(disable:4244)
CYBERTRON_BEGIN
TaskPointCloud::TaskPointCloud()
{
	mSensorDataType = ESimOne_Sensor_Data_Type_PointCloudWithGroundTruth;
}

TaskPointCloud::~TaskPointCloud()
{
}

uint16_t TaskPointCloud::Do(std::uint32_t sensorType, std::uint32_t commanId, CTaskSensorBase::SensorContext* pSensorContext, const std::string* pBuffer)
{
	const char* mainVehId = int2string(pSensorContext->mainVehicleId);
	if (commanId != cybertron::proto::sensor::EDataType_PointCloudWithGroundTruth)
	{
		return -1;
	}

	cybertron::proto::sensor::PointCloudWithGroundTruth PointCloudSrcIn;
	if (!PointCloudSrcIn.ParseFromString(*pBuffer)) {
		return 0;
	}
	const std::string sensorId = SimOneAPIService::GetInstance()->GetSensorIdFromId(pSensorContext->sensorId);
	const string sensorKey = std::to_string(pSensorContext->mainVehicleId).append("_").append(sensorId);


	SimOne_Data_SensorDetections *pPointCloudDetections = NULL;
	SimOne_Data_SensorDetectionsMap::iterator it = mLastSensorDetectionsMap.find(sensorKey);
	if (it != mLastSensorDetectionsMap.end()) {
		pPointCloudDetections = it->second;
	}
	else
	{
		pPointCloudDetections = new SimOne_Data_SensorDetections;

	}

	pPointCloudDetections->objectSize = PointCloudSrcIn.ground_truth().obstacles().size();
	pPointCloudDetections->frame = pSensorContext->frame;
	pPointCloudDetections->timestamp = pSensorContext->timestamp;

	for (auto i = 0; i < PointCloudSrcIn.ground_truth().obstacles().size(); i++)
	{
		pPointCloudDetections->objects[i].id = PointCloudSrcIn.ground_truth().obstacles(i).id();
		pPointCloudDetections->objects[i].type = (ESimOne_Obstacle_Type)PointCloudSrcIn.ground_truth().obstacles(i).type();
		pPointCloudDetections->objects[i].posX = PointCloudSrcIn.ground_truth().obstacles(i).center().x();
		pPointCloudDetections->objects[i].posY = PointCloudSrcIn.ground_truth().obstacles(i).center().y();
		pPointCloudDetections->objects[i].posZ = PointCloudSrcIn.ground_truth().obstacles(i).center().z();
		pPointCloudDetections->objects[i].oriX = PointCloudSrcIn.ground_truth().obstacles(i).rotation().x();
		pPointCloudDetections->objects[i].oriY = PointCloudSrcIn.ground_truth().obstacles(i).rotation().y();
		pPointCloudDetections->objects[i].oriZ = PointCloudSrcIn.ground_truth().obstacles(i).rotation().z();
		pPointCloudDetections->objects[i].length = PointCloudSrcIn.ground_truth().obstacles(i).size().x();
		pPointCloudDetections->objects[i].width = PointCloudSrcIn.ground_truth().obstacles(i).size().y();
		pPointCloudDetections->objects[i].height = PointCloudSrcIn.ground_truth().obstacles(i).size().z();
		pPointCloudDetections->objects[i].range = PointCloudSrcIn.ground_truth().obstacles(i).range();
		pPointCloudDetections->objects[i].velX = PointCloudSrcIn.ground_truth().obstacles(i).velocity().x();
		pPointCloudDetections->objects[i].velY = PointCloudSrcIn.ground_truth().obstacles(i).velocity().y();
		pPointCloudDetections->objects[i].velZ = PointCloudSrcIn.ground_truth().obstacles(i).velocity().z();
		pPointCloudDetections->objects[i].probability = PointCloudSrcIn.ground_truth().obstacles(i).probability();
		pPointCloudDetections->objects[i].relativePosX = PointCloudSrcIn.ground_truth().obstacles(i).relativepos().x();
		pPointCloudDetections->objects[i].relativePosY = PointCloudSrcIn.ground_truth().obstacles(i).relativepos().y();
		pPointCloudDetections->objects[i].relativePosZ = PointCloudSrcIn.ground_truth().obstacles(i).relativepos().z();
		pPointCloudDetections->objects[i].relativeRotX = PointCloudSrcIn.ground_truth().obstacles(i).relativerot().x();
		pPointCloudDetections->objects[i].relativeRotY = PointCloudSrcIn.ground_truth().obstacles(i).relativerot().y();
		pPointCloudDetections->objects[i].relativeRotZ = PointCloudSrcIn.ground_truth().obstacles(i).relativerot().z();
		pPointCloudDetections->objects[i].relativeVelX = PointCloudSrcIn.ground_truth().obstacles(i).relativevel().x();
		pPointCloudDetections->objects[i].relativeVelY = PointCloudSrcIn.ground_truth().obstacles(i).relativevel().y();
		pPointCloudDetections->objects[i].relativeVelZ = PointCloudSrcIn.ground_truth().obstacles(i).relativevel().z();
	}
	{
		std::unique_lock<std::recursive_mutex> lock(mLastSensorDetectionsMapLock);
		mLastSensorDetectionsMap[sensorKey] = pPointCloudDetections;
	}
	if (TaskSensorManager::getInstance().mpSensorDetectionsUpdateCB != NULL)
	{
		TaskSensorManager::getInstance().mpSensorDetectionsUpdateCB(mainVehId, sensorId.c_str(), pPointCloudDetections);
	}
	
	return 0;
}

bool TaskPointCloud::GetData(std::string key, ETaskCommandId commandId, void * pBuffer)
{
	if (commandId == ETaskCommandId_PointCloudObj)
	{
		SimOne_Data_SensorDetectionsMap::iterator it = mLastSensorDetectionsMap.find(key);
		if (it == mLastSensorDetectionsMap.end())
		{
			return false;
		}
		memcpy(pBuffer, it->second, sizeof(SimOne_Data_SensorDetections));
	}
	else {
		return false;
	}
	return true;
}
CYBERTRON_END

#endif // !WITHOUT_SENSOR
