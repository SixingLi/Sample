#pragma once
#ifndef WITHOUT_SENSOR
#include "cybertron/network/Message.hpp"
#include "Sensor/SensorFusion.pb.h"
#include "TaskSensorBase.hpp"
#include <mutex>
CYBERTRON_BEGIN
class TaskFusion :public CTaskSensorBase
{
public:
	static TaskFusion * GetInstance()
	{
		static TaskFusion instance;
		return &instance;
	}
	TaskFusion();
	~TaskFusion();

	virtual int GetCommandIDFromObj() { return ETaskCommandId_FusionObj; };
	virtual int GetCommandIDFromLane() { return ETaskCommandId_FusionLane; };

	virtual uint16_t  Do(std::uint32_t sensorType, std::uint32_t commanId, CTaskSensorBase::SensorContext* pSensorContext, const std::string* pBuffer) ;
	virtual bool GetData(std::string key, ETaskCommandId commandId, void * pBuffer);
	std::ofstream log_taskFusion;
private:
	void SetLaneLineInfo(SimOne_Data_LaneLineInfo &lineInfo, const cybertron::proto::sensor::LaneLine &line, std::ofstream& log_taskImage);
	bool flag = false;

	typedef map<string, SimOne_Data_SensorDetections*> SimOne_Data_SensorDetectionsMap;
	SimOne_Data_SensorDetectionsMap mLastSensorDetectionsMap;
	typedef map<string, SimOne_Data_LaneInfo*> SimOne_Data_Object_LaneMap;
	SimOne_Data_Object_LaneMap mLastObjectLaneMap;
};

CYBERTRON_END
#endif