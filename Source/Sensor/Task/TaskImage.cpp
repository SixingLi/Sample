#ifndef WITHOUT_SENSOR
#include "TaskImage.hpp"
#include "cybertron/core/Log.hpp"
#include "cybertron/core/UtilString.hpp"

#pragma warning(disable:4267)
#pragma warning(disable:4244)
CYBERTRON_BEGIN


TaskImage::TaskImage()
{
	log_taskImage.open("log_taskImage.txt", std::ios::trunc);
}

TaskImage::~TaskImage()
{
}


void SetLaneLineInfo(SimOne_Data_LaneLineInfo &lineInfo, const cybertron::proto::sensor::LaneLine &line, std::ofstream& log_taskImage) {


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

uint16_t  TaskImage::Do(std::uint32_t sensorType, std::uint32_t commanId, CTaskSensorBase::SensorContext* pSensorContext, const std::string* pBuffer)
{
	if (commanId != cybertron::proto::sensor::EDataType_ImageWithGroundTruth)
	{
		return -1;
	}

	cybertron::proto::sensor::ImageWithGroundTruth ImageDataSrc;
	if (!ImageDataSrc.ParseFromString(*pBuffer)) {
		return false;
	}
	const std::string sensorId = SimOneAPIService::GetInstance()->GetSensorIdFromId(pSensorContext->sensorId);
	const string sensorKey = std::to_string(pSensorContext->mainVehicleId).append("_").append(sensorId);

	//imageDetections

	if (SimOneAPIService::GetInstance()->IsNeedSendObjectbasedData())
	{
		SimOne_Data_SensorDetections *pImageDetections = NULL;
		SimOne_Data_SensorDetectionsMap::iterator it = mLastSensorDetectionsMap.find(sensorKey);
		if (it != mLastSensorDetectionsMap.end()) {
			pImageDetections = it->second;
		}
		else
		{
			pImageDetections = new SimOne_Data_SensorDetections;
		}

		pImageDetections->timestamp = pSensorContext->timestamp;
		pImageDetections->frame = pSensorContext->frame;
		pImageDetections->objectSize = ImageDataSrc.ground_truth().obstacles().size();
		for (auto i = 0; i < ImageDataSrc.ground_truth().obstacles().size(); i++)
		{
			pImageDetections->objects[i].id = ImageDataSrc.ground_truth().obstacles(i).id();
			pImageDetections->objects[i].type = (SimOne_Obstacle_Type)ImageDataSrc.ground_truth().obstacles(i).type();
			pImageDetections->objects[i].posX = ImageDataSrc.ground_truth().obstacles(i).center().x();
			pImageDetections->objects[i].posY = ImageDataSrc.ground_truth().obstacles(i).center().y();
			pImageDetections->objects[i].posZ = ImageDataSrc.ground_truth().obstacles(i).center().z();
			pImageDetections->objects[i].oriX = ImageDataSrc.ground_truth().obstacles(i).rotation().x();
			pImageDetections->objects[i].oriY = ImageDataSrc.ground_truth().obstacles(i).rotation().y();
			pImageDetections->objects[i].oriZ = ImageDataSrc.ground_truth().obstacles(i).rotation().z();
			pImageDetections->objects[i].length = ImageDataSrc.ground_truth().obstacles(i).size().x();
			pImageDetections->objects[i].width = ImageDataSrc.ground_truth().obstacles(i).size().y();
			pImageDetections->objects[i].height = ImageDataSrc.ground_truth().obstacles(i).size().z();
			pImageDetections->objects[i].range = ImageDataSrc.ground_truth().obstacles(i).range();
			pImageDetections->objects[i].velX = ImageDataSrc.ground_truth().obstacles(i).velocity().x();
			pImageDetections->objects[i].velY = ImageDataSrc.ground_truth().obstacles(i).velocity().y();
			pImageDetections->objects[i].velZ = ImageDataSrc.ground_truth().obstacles(i).velocity().z();
			pImageDetections->objects[i].probability = ImageDataSrc.ground_truth().obstacles(i).probability();
			pImageDetections->objects[i].relativePosX = ImageDataSrc.ground_truth().obstacles(i).relativepos().x();
			pImageDetections->objects[i].relativePosY = ImageDataSrc.ground_truth().obstacles(i).relativepos().y();
			pImageDetections->objects[i].relativePosZ = ImageDataSrc.ground_truth().obstacles(i).relativepos().z();
			pImageDetections->objects[i].relativeRotX = ImageDataSrc.ground_truth().obstacles(i).relativerot().x();
			pImageDetections->objects[i].relativeRotY = ImageDataSrc.ground_truth().obstacles(i).relativerot().y();
			pImageDetections->objects[i].relativeRotZ = ImageDataSrc.ground_truth().obstacles(i).relativerot().z();
			pImageDetections->objects[i].relativeVelX = ImageDataSrc.ground_truth().obstacles(i).relativevel().x();
			pImageDetections->objects[i].relativeVelY = ImageDataSrc.ground_truth().obstacles(i).relativevel().y();
			pImageDetections->objects[i].relativeVelZ = ImageDataSrc.ground_truth().obstacles(i).relativevel().z();

			if (ImageDataSrc.ground_truth().obstacles(i).bbox2d().size() >= 2)
			{
				pImageDetections->objects[i].bbox2dMinX = ImageDataSrc.ground_truth().obstacles(i).bbox2d(0).x();
				pImageDetections->objects[i].bbox2dMinY = ImageDataSrc.ground_truth().obstacles(i).bbox2d(0).y();
				pImageDetections->objects[i].bbox2dMaxX = ImageDataSrc.ground_truth().obstacles(i).bbox2d(1).x();
				pImageDetections->objects[i].bbox2dMaxY = ImageDataSrc.ground_truth().obstacles(i).bbox2d(1).y();
			}
			if (pImageDetections->objectSize >= SOSM_SENSOR_DETECTIONS_OBJECT_SIZE_MAX)
			{
				break;
			}
		}
		mLastSensorDetectionsMap[sensorKey] = pImageDetections;
		if (TaskSensorManager::getInstance().mpSensorDetectionsUpdateCB != NULL)
		{
			TaskSensorManager::getInstance().mpSensorDetectionsUpdateCB(pSensorContext->mainVehicleId, sensorId.c_str(), pImageDetections);
		}
	}
	
	if (SimOneAPIService::GetInstance()->IsNeedSendObjectbasedData() && ImageDataSrc.lane().lanelines_size())
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
		pLaneInfo->id = ImageDataSrc.lane().id();
		pLaneInfo->laneType = (ESimOneLaneType)ImageDataSrc.lane().type();
		pLaneInfo->laneLeftID = ImageDataSrc.lane().laneleftid();
		pLaneInfo->laneRightID = ImageDataSrc.lane().lanerightid();
		int antecessorSize = ImageDataSrc.lane().laneantecessorid().size();
		for (int i = 0; i < antecessorSize; i++)
		{
			pLaneInfo->lanePredecessorID[i] = ImageDataSrc.lane().laneantecessorid(i);
		}
		int successorSize = ImageDataSrc.lane().lanesuccessorid().size();
		for (int i = 0; i < successorSize; i++)
		{
			pLaneInfo->laneSuccessorID[i] = ImageDataSrc.lane().lanesuccessorid(i);
		}
		if (ImageDataSrc.lane().lanelines().size() > 0)
		{



			// log_taskImage << "----------------------------------------------------------------ImageDataSrc.lane().lanelines().size():  " << ImageDataSrc.lane().lanelines().size() << " ------------------------------------------------------------\n";

			SetLaneLineInfo(pLaneInfo->l_Line, ImageDataSrc.lane().lanelines(0), log_taskImage);
			SetLaneLineInfo(pLaneInfo->c_Line, ImageDataSrc.lane().lanelines(1), log_taskImage);
			SetLaneLineInfo(pLaneInfo->r_Line, ImageDataSrc.lane().lanelines(2), log_taskImage);
			SetLaneLineInfo(pLaneInfo->ll_Line, ImageDataSrc.lane().lanelines(3), log_taskImage);
			SetLaneLineInfo(pLaneInfo->rr_Line, ImageDataSrc.lane().lanelines(4), log_taskImage);
			//  log_taskImage << "#################### ImageDataSrc.lane().lanelines(0).linetype():  " << ImageDataSrc.lane().lanelines(0).linetype() << " ###############\n";

		}

		mLastObjectLaneMap[sensorKey] = pLaneInfo;
		if (TaskSensorManager::getInstance().mpLaneDetectionsUpdateCB != NULL)
		{
			TaskSensorManager::getInstance().mpLaneDetectionsUpdateCB(pSensorContext->mainVehicleId, sensorId.c_str(), pLaneInfo);
		}

	}
	
	return 0;
}
bool TaskImage::GetData(std::string key, ETaskCommandId commandId, void* pBuffer)
{

	if (commandId == ETaskCommandId_ImageObj)
	{
		SimOne_Data_SensorDetectionsMap::iterator it = mLastSensorDetectionsMap.find(key);
		if (it == mLastSensorDetectionsMap.end())
		{
			return false;
		}
		memcpy(pBuffer, it->second, sizeof(SimOne_Data_SensorDetections));

	}
	else if (commandId == ETaskCommandId_ImageLane) {
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
