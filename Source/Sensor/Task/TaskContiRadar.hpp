#pragma once
#ifndef WITHOUT_SENSOR

#include "cybertron/network/CybertronAsyncConn.hpp"
#include "Sensor/RadarDetections.pb.h"
#include "TaskSensorBase.hpp"
#include <mutex>


CYBERTRON_BEGIN
class TaskContiRadar : public CTaskSensorBase
{
public:
	static TaskContiRadar * GetInstance()
	{
		static TaskContiRadar instance;
		return &instance;
	}
	TaskContiRadar();
	~TaskContiRadar();

	virtual int GetCommandIDFromObj() { return ETaskCommandId_ContiRadarObj; };
	virtual uint16_t  Do(std::uint32_t sensorType, std::uint32_t commanId, CTaskSensorBase::SensorContext* pSensorContext, const std::string* pBuffer);
	virtual bool GetData(std::string key, ETaskCommandId commandId, void * pBuffer);
protected:

private:
	typedef map<string, SimOne_Data_RadarDetection*> SimOne_Data_RadarDetectionMap;
	SimOne_Data_RadarDetectionMap mLastRadarDetectionMap;
	mutable std::recursive_mutex mLastRadarDetectionMapLock;
	void(*mpRadarDetectionsCB)(int mainVehicleId, int sensorId, SimOne_Data_RadarDetection *pDetections);
};
CYBERTRON_END


#endif // !WITHOUT_SENSOR
