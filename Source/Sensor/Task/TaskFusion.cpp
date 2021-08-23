#ifndef WITHOUT_SENSOR
#include "TaskFusion.hpp"
#include "cybertron/core/Log.hpp"
#include "cybertron/core/UtilString.hpp"

#pragma warning(disable:4267)
#pragma warning(disable:4244)
CYBERTRON_BEGIN

TaskFusion::TaskFusion()
{
	//mSensorDataType = EDataType_FusionWithGroundTruth;
	//mpFusionUpdateCB = nullptr;
	std::ofstream log_taskFusion;
}

TaskFusion::~TaskFusion()
{
}

void TaskFusion::SetLaneLineInfo(SimOne_Data_LaneLineInfo &lineInfo, const cybertron::proto::sensor::LaneLine &line, std::ofstream& log_taskImage) {


	if (line.linetype())
	{
		lineInfo.lineColor = (ESimOneData_BoundaryColor)line.linecolor();
		lineInfo.lineID = line.id();
		lineInfo.lineType = (ESimOneData_BoundaryType)line.linetype();
		lineInfo.linewidth = line.linewidth();
		for (int i = 0; i < line.linepoints().size(); i++)
		{
			lineInfo.linePoints[i].x = line.linepoints(i).x();
			lineInfo.linePoints[i].y = line.linepoints(i).y();
			lineInfo.linePoints[i].z = line.linepoints(i).z();

		}
		lineInfo.linecurveParameter.C0 = line.linecurveparameter().c0();
		lineInfo.linecurveParameter.C1 = line.linecurveparameter().c1();
		lineInfo.linecurveParameter.C2 = line.linecurveparameter().c2();
		lineInfo.linecurveParameter.C3 = line.linecurveparameter().c3();
		lineInfo.linecurveParameter.length = line.linecurveparameter().length();
		lineInfo.linecurveParameter.endPoints.x = line.linecurveparameter().endtpoints().x();
		lineInfo.linecurveParameter.endPoints.y = line.linecurveparameter().endtpoints().y();
		lineInfo.linecurveParameter.endPoints.z = line.linecurveparameter().endtpoints().z();
		lineInfo.linecurveParameter.firstPoints.x = line.linecurveparameter().firstpoints().x();
		lineInfo.linecurveParameter.firstPoints.y = line.linecurveparameter().firstpoints().y();
		lineInfo.linecurveParameter.firstPoints.z = line.linecurveparameter().firstpoints().z();

	}
}

uint16_t  TaskFusion::Do(std::uint32_t sensorType, std::uint32_t commanId, CTaskSensorBase::SensorContext* pSensorContext, const std::string* pBuffer)
{
	if (commanId != cybertron::proto::sensor::EDataType_SensorFusion)
	{
		return -1;
	}

	cybertron::proto::sensor::SensorFusion FusionDataSrc;
	if (!FusionDataSrc.ParseFromString(*pBuffer)) {
		return false;
	}
	const std::string sensorId = SimOneAPIService::GetInstance()->GetSensorIdFromId(pSensorContext->sensorId);
	const string sensorKey = std::to_string(pSensorContext->mainVehicleId).append("_").append(sensorId);


	//ESensorFusionDetections
	SimOne_Data_SensorDetections *pFusionDetections = NULL;
	SimOne_Data_SensorDetectionsMap::iterator it = mLastSensorDetectionsMap.find(sensorKey);
	if (it != mLastSensorDetectionsMap.end()) {
		pFusionDetections = it->second;
	}
	else
	{
		pFusionDetections = new SimOne_Data_SensorDetections;
		//std::unique_lock<std::recursive_mutex> lock(mLastSensorDetectionsMapLock);
				
	}

	pFusionDetections->timestamp = pSensorContext->timestamp;
	pFusionDetections->frame = pSensorContext->frame;
	pFusionDetections->objectSize = FusionDataSrc.ground_truth().obstacles().size();
	for (auto i = 0; i < FusionDataSrc.ground_truth().obstacles().size(); i++)
	{
		pFusionDetections->objects[i].id = FusionDataSrc.ground_truth().obstacles(i).id();
		pFusionDetections->objects[i].type = (SimOne_Obstacle_Type)FusionDataSrc.ground_truth().obstacles(i).type();
		pFusionDetections->objects[i].posX = FusionDataSrc.ground_truth().obstacles(i).center().x();
		pFusionDetections->objects[i].posY = FusionDataSrc.ground_truth().obstacles(i).center().y();
		pFusionDetections->objects[i].posZ = FusionDataSrc.ground_truth().obstacles(i).center().z();
		pFusionDetections->objects[i].oriX = FusionDataSrc.ground_truth().obstacles(i).rotation().x();
		pFusionDetections->objects[i].oriY = FusionDataSrc.ground_truth().obstacles(i).rotation().y();
		pFusionDetections->objects[i].oriZ = FusionDataSrc.ground_truth().obstacles(i).rotation().z();
		pFusionDetections->objects[i].length = FusionDataSrc.ground_truth().obstacles(i).size().x();
		pFusionDetections->objects[i].width = FusionDataSrc.ground_truth().obstacles(i).size().y();
		pFusionDetections->objects[i].height = FusionDataSrc.ground_truth().obstacles(i).size().z();
		pFusionDetections->objects[i].range = FusionDataSrc.ground_truth().obstacles(i).range();
		pFusionDetections->objects[i].velX = FusionDataSrc.ground_truth().obstacles(i).velocity().x();
		pFusionDetections->objects[i].velY = FusionDataSrc.ground_truth().obstacles(i).velocity().y();
		pFusionDetections->objects[i].velZ = FusionDataSrc.ground_truth().obstacles(i).velocity().z();
		pFusionDetections->objects[i].probability = FusionDataSrc.ground_truth().obstacles(i).probability();
		pFusionDetections->objects[i].relativePosX = FusionDataSrc.ground_truth().obstacles(i).relativepos().x();
		pFusionDetections->objects[i].relativePosY = FusionDataSrc.ground_truth().obstacles(i).relativepos().y();
		pFusionDetections->objects[i].relativePosZ = FusionDataSrc.ground_truth().obstacles(i).relativepos().z();
		pFusionDetections->objects[i].relativeRotX = FusionDataSrc.ground_truth().obstacles(i).relativerot().x();
		pFusionDetections->objects[i].relativeRotY = FusionDataSrc.ground_truth().obstacles(i).relativerot().y();
		pFusionDetections->objects[i].relativeRotZ = FusionDataSrc.ground_truth().obstacles(i).relativerot().z();
		pFusionDetections->objects[i].relativeVelX = FusionDataSrc.ground_truth().obstacles(i).relativevel().x();
		pFusionDetections->objects[i].relativeVelY = FusionDataSrc.ground_truth().obstacles(i).relativevel().y();
		pFusionDetections->objects[i].relativeVelZ = FusionDataSrc.ground_truth().obstacles(i).relativevel().z();

		if (FusionDataSrc.ground_truth().obstacles(i).bbox2d().size() >= 2)
		{
			pFusionDetections->objects[i].bbox2dMinX = FusionDataSrc.ground_truth().obstacles(i).bbox2d(0).x();
			pFusionDetections->objects[i].bbox2dMinY = FusionDataSrc.ground_truth().obstacles(i).bbox2d(0).y();
			pFusionDetections->objects[i].bbox2dMaxX = FusionDataSrc.ground_truth().obstacles(i).bbox2d(1).x();
			pFusionDetections->objects[i].bbox2dMaxY = FusionDataSrc.ground_truth().obstacles(i).bbox2d(1).y();
		}
		if (pFusionDetections->objectSize >= SOSM_SENSOR_DETECTIONS_OBJECT_SIZE_MAX)
		{
			break;
		}
	}
	mLastSensorDetectionsMap[sensorKey] = pFusionDetections;
	if (TaskSensorManager::getInstance().mpSensorDetectionsUpdateCB != NULL)
	{
		TaskSensorManager::getInstance().mpSensorDetectionsUpdateCB(pSensorContext->mainVehicleId, sensorId.c_str(), pFusionDetections);
	}
	

	if (FusionDataSrc.lane().lanelines_size())
	{
		SimOne_Data_LaneInfo *pLaneInfo = NULL;

		SimOne_Data_Object_LaneMap::iterator it = mLastObjectLaneMap.find(sensorKey);
		if (it != mLastObjectLaneMap.end()) {
			pLaneInfo = it->second;
		}
		else
		{
			pLaneInfo = new SimOne_Data_LaneInfo;
		}

		pLaneInfo->timestamp = pSensorContext->timestamp;
		pLaneInfo->frame = pSensorContext->frame;
		pLaneInfo->id = FusionDataSrc.lane().id();
		pLaneInfo->laneType = (ESimOneLaneType)FusionDataSrc.lane().type();
		pLaneInfo->laneLeftID = FusionDataSrc.lane().laneleftid();
		pLaneInfo->laneRightID = FusionDataSrc.lane().lanerightid();
		int antecessorSize = FusionDataSrc.lane().laneantecessorid().size();
		for (int i = 0; i < antecessorSize; i++)
		{
			pLaneInfo->lanePredecessorID[i] = FusionDataSrc.lane().laneantecessorid(i);
		}
		int successorSize = FusionDataSrc.lane().lanesuccessorid().size();
		for (int i = 0; i < successorSize; i++)
		{
			pLaneInfo->laneSuccessorID[i] = FusionDataSrc.lane().lanesuccessorid(i);
		}
		if (FusionDataSrc.lane().lanelines().size() > 0)
		{
			SetLaneLineInfo(pLaneInfo->l_Line, FusionDataSrc.lane().lanelines(0), log_taskFusion);
			SetLaneLineInfo(pLaneInfo->c_Line, FusionDataSrc.lane().lanelines(1), log_taskFusion);
			SetLaneLineInfo(pLaneInfo->r_Line, FusionDataSrc.lane().lanelines(2), log_taskFusion);
			SetLaneLineInfo(pLaneInfo->ll_Line, FusionDataSrc.lane().lanelines(3), log_taskFusion);
			SetLaneLineInfo(pLaneInfo->rr_Line, FusionDataSrc.lane().lanelines(4), log_taskFusion);
		}

		mLastObjectLaneMap[sensorKey] = pLaneInfo;
		if (TaskSensorManager::getInstance().mpLaneDetectionsUpdateCB != NULL)
		{
			TaskSensorManager::getInstance().mpLaneDetectionsUpdateCB(pSensorContext->mainVehicleId, sensorId.c_str(), pLaneInfo);
		}

	}
	return 0;
}


bool TaskFusion::GetData(std::string key, ETaskCommandId commandId, void* pBuffer)
{

	if (commandId == ETaskCommandId_FusionObj)
	{
		SimOne_Data_SensorDetectionsMap::iterator it = mLastSensorDetectionsMap.find(key);
		if (it == mLastSensorDetectionsMap.end())
		{
			return false;
		}
		memcpy(pBuffer, it->second, sizeof(SimOne_Data_SensorDetections));
		
	}
	else if (commandId == ETaskCommandId_FusionLane) {
		SimOne_Data_Object_LaneMap::iterator it = mLastObjectLaneMap.find(key);
		if (it == mLastObjectLaneMap.end())
		{
			return false;
		}
		memcpy(pBuffer, it->second, sizeof(SimOne_Data_LaneInfo));

	}
	else {
		return false;
	}
	return true;
}
CYBERTRON_END
#endif