#pragma once
#ifndef WITHOUT_SENSOR
#include "Sensor/ObstacleDetect.pb.h"
#include "TaskSensorBase.hpp"
#include <mutex>

CYBERTRON_BEGIN
class TaskPerfectPerception : public CTaskSensorBase
{
public:
	static TaskPerfectPerception * GetInstance()
	{
		static TaskPerfectPerception instance;
		return &instance;
	}
	TaskPerfectPerception();
	~TaskPerfectPerception();

	virtual int GetCommandIDFromObj() { return ETaskCommandId_PerfectPerceptionObj; };

	virtual uint16_t  Do(std::uint32_t sensorType, std::uint32_t commanId, CTaskSensorBase::SensorContext* pSensorContext, const std::string* pBuffer);
	virtual bool GetData(std::string key, ETaskCommandId commandId, void * pBuffer);

	bool processObstacleDetection(SensorContext* pSensorContext, const std::string* pBuffer);
	bool processGroundTruth(SensorContext* pSensorContext, const std::string* pBuffer);
protected:


private:
	cybertron::proto::sensor::ObstacleDetect mPerfectPerceptionDetections;
	bool flag = false;

	typedef map<string, SimOne_Data_SensorDetections*> SimOne_Data_SensorDetectionsMap;

	// mainVehicleId sensorId
	SimOne_Data_SensorDetectionsMap mLastSensorDetectionsMap;
	mutable std::recursive_mutex mLastSensorDetectionsMapLock;
	   	
	
	Bridge::BridgeObstacleInfo mLastBridgeObstacleInfo;

	typedef map<int, SimOne_Data_Obstacle*> SimOne_Data_ObstacleMap;
	SimOne_Data_ObstacleMap mLastObstacleMap;
	mutable std::recursive_mutex mLastObstacleMapLock;

	void(*mpSensorDetectionsUpdateCB)(int mainVehicleId, int sensorId, SimOne_Data_SensorDetections *pGroundTruth);
};
CYBERTRON_END
#endif