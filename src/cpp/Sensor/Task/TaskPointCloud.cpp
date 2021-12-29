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
	static vector<life_time_t> acc_gen;

	std::string tempStr = std::to_string(pSensorContext->mainVehicleId);
	const char* mainVehId = tempStr.c_str();
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
		auto iter = acc_gen.begin();
                while (iter != acc_gen.end())
                {
                    if (iter->objId == pPointCloudDetections->objects[i].id)
                    {
                        int tDiff = pPointCloudDetections->timestamp - iter->pre_Timestamp;
                        if (tDiff < LT_DURATION && tDiff  != 0)
                        {
				pPointCloudDetections->objects[i].accelX = (pPointCloudDetections->objects[i].velX - iter->pre_velX) / (tDiff / 1000.0);
				pPointCloudDetections->objects[i].accelY = (pPointCloudDetections->objects[i].velY - iter->pre_velY) / (tDiff / 1000.0);
				pPointCloudDetections->objects[i].accelZ = (pPointCloudDetections->objects[i].velZ - iter->pre_velZ) / (tDiff / 1000.0);

				iter->pre_velX = pPointCloudDetections->objects[i].velX;
				iter->pre_velY = pPointCloudDetections->objects[i].velY;
				iter->pre_velZ = pPointCloudDetections->objects[i].velZ;
				iter->pre_Timestamp = pPointCloudDetections->timestamp;
				break;
                        }
                        else
                        {
				iter = acc_gen.erase(iter);
				continue;
			}
                    }
                    iter++;
                }
                if (iter == acc_gen.end())
                {
			pPointCloudDetections->objects[i].accelX = 0.0;
			pPointCloudDetections->objects[i].accelY = 0.0;
			pPointCloudDetections->objects[i].accelZ = 0.0;
			life_time_t acc_new;
			acc_new.objId = pPointCloudDetections->objects[i].id;
			acc_new.pre_velX = pPointCloudDetections->objects[i].velX;
			acc_new.pre_velY = pPointCloudDetections->objects[i].velY;
			acc_new.pre_velZ = pPointCloudDetections->objects[i].velZ;
			acc_new.pre_Timestamp = pPointCloudDetections->timestamp;
			acc_gen.push_back(acc_new);
		}
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
