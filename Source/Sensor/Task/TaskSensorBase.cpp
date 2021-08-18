#ifndef WITHOUT_SENSOR
#include "TaskSensorBase.hpp"
#include "TaskContiRadar.hpp"
#include "TaskImage.hpp"
#include "TaskFusion.hpp"
#include "TaskPerfectPerception.hpp"
#include "TaskPointCloud.hpp"
#include "TaskUltrasonicRadar.hpp"
#ifndef WITHOUT_HDMAP
#include "TaskV2XNFS.hpp"
#endif
CYBERTRON_BEGIN
TaskSensorManager::TaskSensorManager()
{
	//mpApp = app;
	mpImageUpdateCB = nullptr;
	mpPointCloudUpdateCB = nullptr;
	mpRadarDetectionsCB = nullptr;
	mpSensorDetectionsUpdateCB = nullptr;
	mpUltrasonicRadarsUpdateCB = nullptr;
	mpOSISensorDataUpdateCB = nullptr;

	mpObstacleUpdateCB = nullptr;
	mpSimOneGroundTruthCB = nullptr;

#ifndef WITHOUT_HDMAP
	mpSimOneV2XRawCB = nullptr;
#endif

	ManagerInit();
}


TaskSensorManager::~TaskSensorManager()
{
}
void TaskSensorManager::ManagerClear() {
	mpImageUpdateCB = nullptr;
	mpSensorFusionUpdateCB = nullptr;
	mpPointCloudUpdateCB = nullptr;
	mpRadarDetectionsCB = nullptr;
	mpSensorDetectionsUpdateCB = nullptr;
	mpUltrasonicRadarsUpdateCB = nullptr;
	mpOSISensorDataUpdateCB = nullptr;
	mpObstacleUpdateCB = nullptr;
	mpSimOneGroundTruthCB = nullptr;
#ifndef WITHOUT_HDMAP
	mpSimOneV2XRawCB = nullptr;
#endif
}
void TaskSensorManager::ManagerInit() {	
	CTaskSensorBase* pTask = new TaskImage();
	mTaskSensorDataTypeMap[Bridge::ESensorType_Camera] = pTask;

    pTask = new TaskFusion();
	mTaskSensorDataTypeMap[Bridge::ESensorType_SensorFusion] = pTask;

	pTask = new TaskContiRadar();
	mTaskSensorDataTypeMap[Bridge::ESensorType_MMWRadar] = pTask;

	pTask = new TaskPerfectPerception();
	mTaskSensorDataTypeMap[Bridge::ESensorType_PerfectPerception] = pTask;

	pTask = new TaskPointCloud();
	mTaskSensorDataTypeMap[Bridge::ESensorType_LiDAR] = pTask;

	pTask = new TaskUltrasonicRadar();
	mTaskSensorDataTypeMap[Bridge::ESensorType_AllUltrasonicRadar] = pTask;
#ifndef WITHOUT_HDMAP
	pTask = new TaskV2XNFS();
	mTaskSensorDataTypeMap[Bridge::ESensorType_V2XNFS] = pTask;
#endif
}
CTaskSensorBase* TaskSensorManager::FindTask(std::uint16_t taskType) {
	auto it = mTaskSensorDataTypeMap.find(taskType);
	if (it == mTaskSensorDataTypeMap.end()) {
		return nullptr;
	}
	return it->second;
}
void TaskSensorManager::Do(std::uint32_t sensorType, std::uint32_t commanId, CTaskSensorBase::SensorContext* pSensorContext, const std::string* pBuffer){
	
	TaskSensorMap::iterator it = mTaskSensorDataTypeMap.find(sensorType);
	if (it != mTaskSensorDataTypeMap.end()) {
		
			it->second->Do(sensorType,commanId,pSensorContext, pBuffer);
	}
}

TaskSensorManager &TaskSensorManager::getInstance()
{
	static TaskSensorManager me;
	return me;
}
CYBERTRON_END
#endif