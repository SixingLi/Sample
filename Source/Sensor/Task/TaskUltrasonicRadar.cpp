#ifndef WITHOUT_SENSOR
#include "TaskUltrasonicRadar.hpp"
#include "cybertron/core/UtilString.hpp"

CYBERTRON_BEGIN
TaskUltrasonicRadar::TaskUltrasonicRadar() 
{
	//mSensorDataType = EDataType_UltrasonicRadar;
}

TaskUltrasonicRadar::~TaskUltrasonicRadar()
{

}

uint16_t TaskUltrasonicRadar::Do(std::uint32_t sensorType, std::uint32_t commanId, CTaskSensorBase::SensorContext* pSensorContext, const std::string* pBuffer)
{
	if (commanId != cybertron::proto::sensor::EDataType_UltrasonicRadar)
	{
		return -1;
	}
	cybertron::proto::sensor::UltrasonicRadars DataIn;
	if (!DataIn.ParseFromString(*pBuffer)) {
		return 0;
	}
	//Detections
	if (SimOneAPIService::GetInstance()->IsNeedSendObjectbasedData())
		{
			SimOne_Data_UltrasonicRadars ultrasonicRadarsMuch;
			SimOne_Data_UltrasonicRadars *pUltrasonicRadarsMuch = NULL;
			int mainVehicleId = pSensorContext->mainVehicleId;
			//int mainVehicleId
			SimOne_Data_UltrasonicRadarsMap::iterator it = mLastUltrasonicRadarsMap.find(mainVehicleId);
			if (it != mLastUltrasonicRadarsMap.end()) {
				pUltrasonicRadarsMuch = it->second;
			}
			else
			{
				pUltrasonicRadarsMuch = new SimOne_Data_UltrasonicRadars;
			
			}

			pUltrasonicRadarsMuch->ultrasonicRadarNum = DataIn.radars().size();
			pUltrasonicRadarsMuch->timestamp = pSensorContext->timestamp;
			pUltrasonicRadarsMuch->frame = pSensorContext->frame;
			for (int i = 0; i < DataIn.radars().size(); i++)
			{
				pUltrasonicRadarsMuch->ultrasonicRadars[i].frame = pSensorContext->frame;
				pUltrasonicRadarsMuch->ultrasonicRadars[i].timestamp = pSensorContext->timestamp;
				std::string sensorId = SimOneAPIService::GetInstance()->GetSensorIdFromId(DataIn.radars(i).header().id());
				memset(pUltrasonicRadarsMuch->ultrasonicRadars[i].sensorId, 0, SENSOR_IDTYPE_MAX);
				memcpy(pUltrasonicRadarsMuch->ultrasonicRadars[i].sensorId, sensorId.c_str(), sensorId.size());
				pUltrasonicRadarsMuch->ultrasonicRadars[i].obstacleNum = (DataIn.radars(i).ultrasonicradardetections().size() < SOSM_OBSTACLE_SIZE_MAX) ? DataIn.radars(i).ultrasonicradardetections().size() : SOSM_OBSTACLE_SIZE_MAX;
					for (int j = 0; j < pUltrasonicRadarsMuch->ultrasonicRadars[i].obstacleNum; j++)
					{
						pUltrasonicRadarsMuch->ultrasonicRadars[i].obstacleDetections[j].obstacleRanges = DataIn.radars(i).ultrasonicradardetections(j).obstaclerange();
						pUltrasonicRadarsMuch->ultrasonicRadars[i].obstacleDetections[j].x = DataIn.radars(i).ultrasonicradardetections(j).x();
						pUltrasonicRadarsMuch->ultrasonicRadars[i].obstacleDetections[j].y = DataIn.radars(i).ultrasonicradardetections(j).y();
					}
			}
			{
				std::unique_lock<std::recursive_mutex> lock(mLastUltrasonicRadarsMapLock);
				mLastUltrasonicRadarsMap[mainVehicleId] = pUltrasonicRadarsMuch;
			}
			if (TaskSensorManager::getInstance().mpUltrasonicRadarsUpdateCB != NULL)
			{
				TaskSensorManager::getInstance().mpUltrasonicRadarsUpdateCB(mainVehicleId, pUltrasonicRadarsMuch);
			}
	}
	return true;
}
bool TaskUltrasonicRadar::GetData(std::string key, ETaskCommandId commandId, void * pBuffer)
{

	if (commandId == ETaskCommandId_UltrasonicRadarObj)
	{
		int mainVehicle = atoi(key.c_str());
		SimOne_Data_UltrasonicRadarsMap::iterator it = mLastUltrasonicRadarsMap.find(mainVehicle);
		if (it == mLastUltrasonicRadarsMap.end())
		{
			return false;
		}
		memcpy(pBuffer, it->second, sizeof(SimOne_Data_UltrasonicRadars));
	}else {
		return false;
	}
	return true;
}
CYBERTRON_END
#endif