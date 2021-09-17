#pragma once
#ifndef WITHOUT_SENSOR
#include "cybertron/network/CybertronAsyncConn.hpp"
#include "cybertron/node/NodeHotAreaManager.hpp"

#include "TaskSensorBase.hpp"
#include "Sensor/UltrasonicRadar.pb.h"
#include <mutex>
CYBERTRON_BEGIN
class TaskUltrasonicRadar : public CTaskSensorBase
{
public:
	static TaskUltrasonicRadar * GetInstance()
	{
		static TaskUltrasonicRadar instance;
		return &instance;
	};
	TaskUltrasonicRadar();
	~TaskUltrasonicRadar();
	virtual int GetCommandIDFromRaw() { return -1; };
	virtual int GetCommandIDFromOSI() { return ETaskCommandId_UltrasonicRadarOSI; };
	virtual int GetCommandIDFromObj() { return -1; };
	virtual uint16_t  Do(std::uint32_t sensorType, std::uint32_t commanId, CTaskSensorBase::SensorContext* pSensorContext, const std::string* pBuffer);
	virtual bool GetData(std::string key, ETaskCommandId commandId, void * pBuffer);

private:
	SimOne_Data_SensorConfigurations pSensorConfigurations;
	typedef map<int, SimOne_Data_UltrasonicRadars*> SimOne_Data_UltrasonicRadarsMap;
	SimOne_Data_UltrasonicRadarsMap mLastUltrasonicRadarsMap;
	mutable std::recursive_mutex mLastUltrasonicRadarsMapLock;
	void(*mpUltrasonicRadarsUpdateCB)(int mainVehicleId, SimOne_Data_UltrasonicRadars *pUltrasonics);
};
CYBERTRON_END
#endif