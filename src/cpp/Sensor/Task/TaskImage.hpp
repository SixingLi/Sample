#pragma once
#ifndef WITHOUT_SENSOR
#include "cybertron/network/Message.hpp"
#include "Sensor/Image.pb.h"
#include "TaskSensorBase.hpp"
#include <mutex>
CYBERTRON_BEGIN
class TaskImage :public CTaskSensorBase
{
public:
	static TaskImage * GetInstance()
	{
		static TaskImage instance;
		return &instance;
	}
	TaskImage();
	~TaskImage();

	virtual int GetCommandIDFromObj() { return ETaskCommandId_ImageObj; };
	virtual int GetCommandIDFromLane() { return ETaskCommandId_ImageLane; };
	virtual int GetCommandIDFromRoadMark() { return ETaskCommandId_ImageRoadMark; };

	virtual uint16_t  Do(std::uint32_t sensorType, std::uint32_t commanId, CTaskSensorBase::SensorContext* pSensorContext, const std::string* pBuffer);
	virtual bool GetData(std::string key, ETaskCommandId commandId, void * pBuffer);

	std::ofstream log_taskImage;
private:
	//mutable std::recursive_mutex mImageMutex;
	bool flag = false;

	typedef map<string, SimOne_Data_SensorDetections*> SimOne_Data_SensorDetectionsMap;
	typedef map<string, SimOne_Data_LaneInfo*> SimOne_Data_Object_LaneMap;
	typedef map<string, SimOne_Data_RoadMarkInfo*> SimOne_Data_Object_RoadMarkMap;

	SimOne_Data_SensorDetectionsMap mLastSensorDetectionsMap;
	SimOne_Data_Object_LaneMap mLastObjectLaneMap;
	SimOne_Data_Object_RoadMarkMap mLastObjectRoadMarkMap;

};

CYBERTRON_END
#endif