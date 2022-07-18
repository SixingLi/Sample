﻿// ==========================================================================
// Copyright (C) 2018 - 2021 Beijing 51WORLD Digital Twin Technology Co., Ltd. 
// , and/or its licensors.  All rights reserved.
//
// The coded instructions, statements, computer programs, and/or related 
// material (collectively the "Data") in these files contain unpublished
// information proprietary to Beijing 51WORLD Digital Twin Technology Co., Ltd. 
// ("51WORLD") and/or its licensors,  which is protected by the People's 
// Republic of China and/or other countries copyright law and by 
// international treaties.
//
// The Data may not be disclosed or distributed to third parties or be
// copied or duplicated, in whole or in part, without the prior written
// consent of 51WORLD.
//
// The copyright notices in the Software and this entire statement,
// including the above license grant, this restriction and the following
// disclaimer, must be included in all copies of the Software, in whole
// or in part, and all derivative works of the Software, unless such copies
// or derivative works are solely in the form of machine-executable object
// code generated by a source language processor.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND.
// 51WORLD DOES NOT MAKE AND HEREBY DISCLAIMS ANY EXPRESS OR IMPLIED
// WARRANTIES INCLUDING, BUT NOT LIMITED TO, THE WARRANTIES OF
// NON-INFRINGEMENT, MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE,
// OR ARISING FROM A COURSE OF DEALING, USAGE, OR TRADE PRACTICE. IN NO
// EVENT WILL 51WORLD AND/OR ITS LICENSORS BE LIABLE FOR ANY LOST
// REVENUES, DATA, OR PROFITS, OR SPECIAL, DIRECT, INDIRECT, OR
// CONSEQUENTIAL DAMAGES, EVEN IF 51WORLD AND/OR ITS LICENSORS HAS
// BEEN ADVISED OF THE POSSIBILITY OR PROBABILITY OF SUCH DAMAGES.
// ==========================================================================
#pragma once
#ifndef WITHOUT_SENSOR
#ifdef BUILD_SIMONE_API
#if defined(WIN32) || defined(_WIN32)
#define SIMONE_API __declspec(dllexport)
#elif defined(__linux__) || defined(__linux)
#define SIMONE_API __attribute__((visibility("default")))
#endif
#else
#define SIMONE_API
#endif

#include <string>
#include "Service/SimOneIOStruct.h"

//namespace HDMapStandalone
//{
//    struct MLaneInfo;
//	struct MLaneLink;
//}

#ifdef __cplusplus
extern "C"
{
#endif

	namespace SimOneAPI {

		/*!
		获取主车GPS信息
		\li function:
		*	GetGps
		\li brief:
		*	Get main vehicle GPS
		@param[in]
		*	mainVehicleId: Id of the main vehicle
		@param[out]
		*	pGps: GPS data
		@return
		*	Success or not
		*/
		SIMONE_API bool GetGps(const char* mainVehicleId, SimOne_Data_Gps *pGps);

		/*!
		主车GPS更新回调
		\li function:
		*	SetGpsUpdateCB
		\li brief:
		*	Register the callback func applying for GPS info
		@param[in]
		*	cb: GPS data update callback function
		*	param[out]
		*	mainVehicleId: Id of the main vehicle
		*	pGps: GPS data
		@return
		*	Success or not
		*/
		SIMONE_API bool SetGpsUpdateCB(void(*cb)(const char* mainVehicleId, SimOne_Data_Gps *pGps));

		/*!
		得到仿真场景中的物体的真值
		\li function:
		*	GetGroundTruth
		\li brief:
		*	Get Ground Truth Data of Objects(Obstacles) from simulation scene.
		@param[in]
		*	mainVehicleId: Id of the main vehicle
		@param[out]
		*	pObstacle: Obstacle data
		@return
		*	Success or not
		*/
		SIMONE_API bool GetGroundTruth(const char* mainVehicleId, SimOne_Data_Obstacle *pObstacle);

		/*!
		得到仿真场景中的物体的真值的更新回调
		\li function:
		*	SetObstacleUpdateCB
		\li brief:
		*	Register the callback func applying for obstacle info
		@param
		*   mainVehicleId: Vehicle index, configure order of web UI, starts from 0
		@param[in]
		*	cb: Obstacle data update callback function
		*	param[out]
		*	mainVehicleId: Id of the main vehicle
		*	pObstacle: Obstacle data
		@return
		*	Success or not
		*/
		SIMONE_API bool SetGroundTruthUpdateCB(void(*cb)(const char* mainVehicleId, SimOne_Data_Obstacle *pObstacle));

		/*!
		得到毫米波雷达目标信息
		\li function:
		*	GetRadarDetections
		\li brief:
		*	Get millimeter wave radar detections.
		@param[in]
		*	mainVehicleId: Id of the main vehicle
		@param[in]
		*	sensorId: Sensor Index
		@param[out]
		*	pDetections: Radar detections
		@return
		*	Success or not
		*/
		SIMONE_API bool GetRadarDetections(const char* mainVehicleId, const char* sensorId, SimOne_Data_RadarDetection *pDetections);

		/*!
		毫米波雷达目标信息回调
		\li function:
		*	SetRadarDetectionsUpdateCB
		\li brief:
		*	Register the callback func applying for Millimeter wave radar detections
		@param[in]
		*	cb: Radar detections update callback function
		*	param[out]
		*	mainVehicleId: Id of the main vehicle
		*	sensorId: Sensor Index
		*	pDetections: Detections data
		@return
		*	Success or not
		*/
		SIMONE_API bool SetRadarDetectionsUpdateCB(void(*cb)(const char* mainVehicleId, const char* sensorId, SimOne_Data_RadarDetection *pDetections));

		/*!
		获得一个超声波雷达信息
		\li function:
		*	GetUltrasonicRadar
		\li brief:
		*	Get UltrasonicRadar imformations
		@param[in]
		*	mainVehicleId: Id of the main vehicle
		@param[in]
		*	sensorId: Sensor Index
		@param[out]
		*	pUltrasonic: ultrasonic data in SimOne_Data_UltrasonicRadar format
		@return
		*	Success or not
		*/
		SIMONE_API bool GetUltrasonicRadar(const char* mainVehicleId, const char* sensorId, SimOne_Data_UltrasonicRadar *pUltrasonic);

		/*!
		获得所有超声波雷达信息
		\li function:
		*	GetUltrasonicRadars
		\li brief:
		*	Get UltrasonicRadars imfomations
		@param[in]
		*	mainVehicleId: Id of the main vehicle
		@param[out]
		*	pUltrasonics: ultrasonics data in SimOne_Data_UltrasonicRadars format
		@return
		*	Success or not
		*/
		SIMONE_API bool GetUltrasonicRadars(const char* mainVehicleId, SimOne_Data_UltrasonicRadars *pUltrasonics);

		/*!
		超生波雷达真值信息更新回调
		\li function:
		*	SetUltrasonicRadarsCB
		\li brief:
		*	Register the callback func applying for ultrasonics radar detections
		@param[in]
		*	cb: Ultrasonics data update callback function
		*	param[out]
		*	mainVehicleId: Id of the main vehicle
		*	pUltrasonics: Ultrasonics detections
		@return
		*	Success or not
		*/
		SIMONE_API bool SetUltrasonicRadarsCB(void(*cb)(const char* mainVehicleId, SimOne_Data_UltrasonicRadars *pUltrasonics));

		/*!
		获取传感器检测到物体的对应真值
		\li function:
		*	GetSensorDetections
		\li brief:
		*	Get Ground Truth objects for current sensor, support camera, lidar and perfect perception sensors.
		@param[in]
		*	mainVehicleId: Id of the main vehicle
		@param[in]
		*	sensorId: Sensor Index
		@param[out]
		*	pGroundtruth: SimOne_Data_SensorDetections data
		@return
		*	Success or not
		*/
		SIMONE_API bool GetSensorDetections(const char* mainVehicleId, const char* sensorId, SimOne_Data_SensorDetections *pGroundtruth);

		/*!
		传感器真值信息更新回调
		\li function:
		*	SetSensorDetectionsUpdateCB
		\li brief:
		*	Register the callback func applying for GroundTruth of current sensor, support camera, lidar and perfect perception sensors.
		@param[in]
		*	cb: Groundtruth data fetch callback function
		*	param[out]
		*	mainVehicleId: Id of the main vehicle
		*	sensorId: Sensor Index
		*	pGroundtruth: Groundtruth data
		@return
		*	Success or not
		*/
		SIMONE_API bool SetSensorDetectionsUpdateCB(void (*cb)(const char *mainVehicleId, const char *sensorId, SimOne_Data_SensorDetections *pGroundtruth));

		/*!
		得到所有传感器的配置信息（Id、类型、频率、位置和朝向等）
		\li function:
		*	GetSensorConfigurations
		\li brief:
		*	Get Sensor's position information
		@param[in]
		*	mainVehicleId: Id of the main vehicle
		@param[out]
		*	pSensorConfigurations: SensorConfigurations data
		@return
		*	Success or not
		*/
		SIMONE_API bool GetSensorConfigurations(const char* mainVehicleId, SimOne_Data_SensorConfigurations *pSensorConfigurations);

		/*!
		获取当前环境相关信息（天气、光照、地面等）
		\li function:
		*	GetEnvironment
		\li brief:
		*	Get current Environment
		@param[in]
		*	pEnvironment: SimOne_Data_Environment data
		@return
		*	Success or not
		*/
		SIMONE_API bool GetEnvironment(SimOne_Data_Environment *pEnvironment);

		/*!
		设置当前环境相关信息（天气、光照、地面等）
		\li function:
		*	SetEnvironment
		\li brief:
		*	Set Current Environment
		@param[out]
		*	pEnvironment: SimOne_Data_Environment data
		@return
		*	Success or not
		*/
		SIMONE_API bool SetEnvironment(SimOne_Data_Environment *pEnvironment);

		/*!
		得到仿真场景中的交通灯的真值
		\li function:
		*	GetTrafficLight
		\li brief:
		*	Get traffic lights Data of Objects(light) from simulation scene.
		@param[in]
		*	mainVehicleId: Id of the main vehicle
		@param[in]
		*	opendriveLightId: traffic light Id on opendrive
		@param[out]
		*	pTrafficLights: light data
		@return
		*	Success or not
		*/
		SIMONE_API bool GetTrafficLight(const char* mainVehicleId, int opendriveLightId, SimOne_Data_TrafficLight* pTrafficLight);

		/*!
		获取传感器检测到车道与车道线数据
		\li function:
		*	GetSensorLaneInfo
		\li brief:
		*	Get LaneInfo for current sensor, support camera and fusion sensor
		@param[in]
		*	mainVehicleId: Id of the main vehicle
		@param[in]
		*	sensorId: Sensor Index
		@param[out]
		*	pLaneInfo: SimOne_Data_LaneInfo data
		@return
		*	Success or not
		*/
		SIMONE_API bool GetSensorLaneInfo(const char* mainVehicleId, const char* sensorId, SimOne_Data_LaneInfo *pLaneInfo);

		/*!
		获取传感器检测到车道与车道线数据回调
		\li function:
		*	SetSensorLaneInfoCB
		\li brief:
		*	Register the callback func applying for LaneInfo from current sensor, support camera, and fusion sensors.
		@param
		*   mainVehicleId: Vehicle index, configure order of web UI, starts from 0
		@param[in]
		*	cb: Groundtruth data fetch callback function
		*	param[out]
		*	mainVehicleId: Id of the main vehicle
		*	sensorId: Sensor Index
		*	pLaneInfo: LaneInfo data
		@return
		*	Success or not
		*/
		SIMONE_API bool SetSensorLaneInfoCB(void(*cb)(const char* mainVehicleId, const char* sensorId, SimOne_Data_LaneInfo *pLaneInfo));

		SIMONE_API bool GetSensorRoadMarkInfo(const char* mainVehicleId, const char* sensorId, SimOne_Data_RoadMarkInfo *pRoadMarkInfo);

		SIMONE_API bool SetSensorRoadMarkInfoCB(void(*cb)(const char* mainVehicleId, const char* sensorId, SimOne_Data_RoadMarkInfo *pRoadMarkInfo));
		}
#ifdef __cplusplus
}
#endif
#endif
