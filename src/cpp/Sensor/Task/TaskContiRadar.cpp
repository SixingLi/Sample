#ifndef WITHOUT_SENSOR
#include "TaskContiRadar.hpp"
#include "cybertron/core/Log.hpp"
#include "cybertron/network/Message.hpp"
#include "cybertron/core/UtilString.hpp"
#pragma warning(disable:4267)
#pragma warning(disable:4244)
CYBERTRON_BEGIN
TaskContiRadar::TaskContiRadar()
{
	//mSensorDataType = EDataType_RadarDetections;
}
TaskContiRadar::~TaskContiRadar()
{
}

uint16_t  TaskContiRadar::Do(std::uint32_t sensorType, std::uint32_t commanId, CTaskSensorBase::SensorContext* pSensorContext, const std::string* pBuffer)
{
	static vector<life_time_t> acc_gen;

	if (commanId != cybertron::proto::sensor::EDataType_RadarDetections)
	{
		return -1;
	}
		
	cybertron::proto::sensor::DataRadarDetections RadarDetectionsIn;
	if (!RadarDetectionsIn.ParseFromString(*pBuffer)) {
		return 0;
	}
	const std::string sensorId = SimOneAPIService::GetInstance()->GetSensorIdFromId(pSensorContext->sensorId);
	const string sensorKey = std::to_string(pSensorContext->mainVehicleId).append("_").append(sensorId);


	SimOne_Data_RadarDetection contiRadarDetections;
	SimOne_Data_RadarDetection *pContiRadarDetections = NULL;

	SimOne_Data_RadarDetectionMap::iterator it = mLastRadarDetectionMap.find(sensorKey);

	if (it != mLastRadarDetectionMap.end()) {
		pContiRadarDetections = it->second;
	}
	else
	{
		pContiRadarDetections = new SimOne_Data_RadarDetection;
	}

	pContiRadarDetections->frame = pSensorContext->frame;
	pContiRadarDetections->timestamp = pSensorContext->timestamp;
	pContiRadarDetections->version = pSensorContext->version;
	pContiRadarDetections->detectNum = RadarDetectionsIn.detections_size();
	for (int i = 0; i < RadarDetectionsIn.detections_size(); i++)
	{
		const cybertron::proto::sensor::RadarDetection &cybObj = RadarDetectionsIn.detections(i);
		pContiRadarDetections->detections[i].id = cybObj.id();
		pContiRadarDetections->detections[i].subId = cybObj.subid();
		pContiRadarDetections->detections[i].type = (ESimOne_Obstacle_Type)cybObj.type();
		pContiRadarDetections->detections[i].posX = cybObj.position().x();
		pContiRadarDetections->detections[i].posY = cybObj.position().y();
		pContiRadarDetections->detections[i].posZ = cybObj.position().z();
		pContiRadarDetections->detections[i].velX = cybObj.velocity().x();
		pContiRadarDetections->detections[i].velY = cybObj.velocity().y();
		pContiRadarDetections->detections[i].velZ = cybObj.velocity().z();
		auto iter = acc_gen.begin();
		while (iter != acc_gen.end())
		{
			if (iter->objId == pContiRadarDetections->detections[i].id)
			{
				int tDiff = pContiRadarDetections->timestamp - iter->pre_Timestamp;
				if (tDiff < LT_DURATION && tDiff  != 0)
				{
					pContiRadarDetections->detections[i].accelX = (pContiRadarDetections->detections[i].velX - iter->pre_velX) / (tDiff / 1000.0);
					pContiRadarDetections->detections[i].accelY = (pContiRadarDetections->detections[i].velY - iter->pre_velY) / (tDiff / 1000.0);
					pContiRadarDetections->detections[i].accelZ = (pContiRadarDetections->detections[i].velZ - iter->pre_velZ) / (tDiff / 1000.0);
					iter->pre_velX = pContiRadarDetections->detections[i].velX;
					iter->pre_velY = pContiRadarDetections->detections[i].velY;
					iter->pre_velZ = pContiRadarDetections->detections[i].velZ;
					iter->pre_Timestamp = pContiRadarDetections->timestamp;
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
			pContiRadarDetections->detections[i].accelX = 0.0;
			pContiRadarDetections->detections[i].accelY = 0.0;
			pContiRadarDetections->detections[i].accelZ = 0.0;
			life_time_t acc_new;
			acc_new.objId = pContiRadarDetections->detections[i].id;
			acc_new.pre_velX = pContiRadarDetections->detections[i].velX;
			acc_new.pre_velY = pContiRadarDetections->detections[i].velY;
			acc_new.pre_velZ = pContiRadarDetections->detections[i].velZ;
			acc_new.pre_Timestamp = pContiRadarDetections->timestamp;
			acc_gen.push_back(acc_new);
		}
		pContiRadarDetections->detections[i].oriX = cybObj.rotation().x();
		pContiRadarDetections->detections[i].oriY = cybObj.rotation().y();
		pContiRadarDetections->detections[i].oriZ = cybObj.rotation().z();
		pContiRadarDetections->detections[i].length = cybObj.size().x();
		pContiRadarDetections->detections[i].width = cybObj.size().y();
		pContiRadarDetections->detections[i].height = cybObj.size().z();
		pContiRadarDetections->detections[i].range = cybObj.range();
		pContiRadarDetections->detections[i].rangeRate = cybObj.rangerate();
		pContiRadarDetections->detections[i].azimuth = cybObj.azimuth();
		pContiRadarDetections->detections[i].vertical = cybObj.vertical();
		pContiRadarDetections->detections[i].snrdb = cybObj.snrdb();
		pContiRadarDetections->detections[i].rcsdb = cybObj.rcsdb();
		pContiRadarDetections->detections[i].probability = cybObj.probability();
	}
	std::unique_lock<std::recursive_mutex> lock(mLastRadarDetectionMapLock);
	mLastRadarDetectionMap[sensorKey] = pContiRadarDetections;
	if (TaskSensorManager::getInstance().mpRadarDetectionsCB != NULL)
	{
		std::string tempStr = std::to_string(pSensorContext->mainVehicleId);
		const char* mainVehId = tempStr.c_str();
		TaskSensorManager::getInstance().mpRadarDetectionsCB(mainVehId, sensorId.c_str(), pContiRadarDetections);
	}
	
	return true;
}
 bool TaskContiRadar::GetData(std::string key, ETaskCommandId commandId, void * pBuffer)
{
	if (commandId == ETaskCommandId_ContiRadarObj)
	{
		SimOne_Data_RadarDetectionMap::iterator it = mLastRadarDetectionMap.find(key);
		if (it == mLastRadarDetectionMap.end())
		{
			return false;
		}
		memcpy(pBuffer, it->second, sizeof(SimOne_Data_RadarDetection));
		
	}else {
		return false;
	}
	return true;
}
CYBERTRON_END
#endif