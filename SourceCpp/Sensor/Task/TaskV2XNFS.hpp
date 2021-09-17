#pragma once
#ifndef WITHOUT_SENSOR
#ifndef WITHOUT_HDMAP
#include "cybertron/network/CybertronAsyncConn.hpp"
#include "TaskSensorBase.hpp"
#include "Sensor/ObstacleDetect.pb.h"
#include "Sensor/V2XNFS.pb.h"
#include <mutex>
CYBERTRON_BEGIN

class TaskV2XNFS : public CTaskSensorBase
{
public:
	static TaskV2XNFS * GetInstance()
	{
		static TaskV2XNFS instance;
		return &instance;
	};
	TaskV2XNFS();
	~TaskV2XNFS();

	virtual int GetCommandIDFromObj() { return -1; };
	virtual uint16_t  Do(std::uint32_t sensorType, std::uint32_t commanId, CTaskSensorBase::SensorContext* pSensorContext, const std::string* pBuffer);
	void DoSendMsgFrame(SensorContext* pSensorContext, const std::string &pBuffer);
	virtual bool GetData(std::string key, ETaskCommandId commandId, void * pBuffer);

protected:
private:
	bool flag = false;
	typedef map<string, SimOne_Data_V2XNFS*> SimOne_Data_V2XDetectionMap;
	SimOne_Data_V2XDetectionMap mLastV2XDetectionBSMMap;
	SimOne_Data_V2XDetectionMap mLastV2XDetectionRSMMap;
	SimOne_Data_V2XDetectionMap mLastV2XDetectionMAPMap;
	SimOne_Data_V2XDetectionMap mLastV2XDetectionRSIMap;
	SimOne_Data_V2XDetectionMap mLastV2XDetectionSPATMap;


	mutable std::recursive_mutex mLastV2XDetectionBSMMapLock;
	mutable std::recursive_mutex mLastV2XDetectionRSMMapLock;
	mutable std::recursive_mutex mLastV2XDetectionMAPMapLock;
	mutable std::recursive_mutex mLastV2XDetectionRSIMapLock;
	mutable std::recursive_mutex mLastV2XDetectionSPATMapLock;

	void(*mpSimOneV2XRawCB)(const char* mainVehicleId, int sensorId, SimOne_Data_V2XNFS *pV2XDetections);
};
CYBERTRON_END

#endif // !WITHOUT_HDMAP
#endif // !WITHOUT_SENSOR
