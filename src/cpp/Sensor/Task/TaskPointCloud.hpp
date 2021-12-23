#pragma once
#ifndef WITHOUT_SENSOR
#include "cybertron/node/NodeHotAreaManager.hpp"
#include "Sensor/PointCloud.pb.h"
#include "TaskSensorBase.hpp"
#include <mutex>

CYBERTRON_BEGIN
class TaskPointCloud : public CTaskSensorBase
{
public:

	static TaskPointCloud * GetInstance()
	{
		static TaskPointCloud instance;
		return &instance;
	}
	TaskPointCloud();
	~TaskPointCloud();
	virtual int GetCommandIDFromRaw() { return -1; };
	virtual int GetCommandIDFromOSI() { return ETaskCommandId_PointCloudOSI; };
	virtual int GetCommandIDFromObj() { return ETaskCommandId_PointCloudObj; };

	virtual uint16_t  Do(std::uint32_t sensorType, std::uint32_t commanId, CTaskSensorBase::SensorContext* pSensorContext, const std::string* pBuffer);
	virtual bool GetData(std::string key, ETaskCommandId commandId, void * pBuffer);

private:

	typedef map<string, SimOne_Data_Point_Cloud*> SimOne_Data_Point_CloudMap;
	typedef map<string, SimOne_Data_SensorDetections*> SimOne_Data_SensorDetectionsMap;

	// mainVehicleId sensorId
	SimOne_Data_Point_CloudMap mLastPointCloudMap;
	mutable std::recursive_mutex mLastPointCloudMapLock;
	SimOne_Data_SensorDetectionsMap mLastSensorDetectionsMap;
	mutable std::recursive_mutex mLastSensorDetectionsMapLock;

	void(*mpPointCloudUpdateCB)(int mainVehicleId, int sensorId, SimOne_Data_Point_Cloud *pPointCloud);
	void(*mpSensorDetectionsUpdateCB)(int mainVehicleId, int sensorId, SimOne_Data_SensorDetections *pGroundtruth);
};
CYBERTRON_END
#endif