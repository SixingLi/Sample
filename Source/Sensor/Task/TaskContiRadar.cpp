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
		TaskSensorManager::getInstance().mpRadarDetectionsCB(pSensorContext->mainVehicleId, sensorId.c_str(), pContiRadarDetections);
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