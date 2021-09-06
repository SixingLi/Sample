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
#ifndef WITHOUT_PNC
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

#ifdef __cplusplus
extern "C"
{
#endif

	namespace SimOneAPI {

		/*!
		注册SimOne_Data_Gps包含的状态以外的主车状态信息
		\li function:
		*	RegisterSimOneVehicleState
		\li brief:
		*	Register states of main vehicle dynamics
		@param[pStateIndics]
		*   pStateIndics: array of state names
		@param[size]
		*   size: state number in pStateIndics
		@return
		*	Success or not
		*/
		SIMONE_API bool RegisterVehicleState(ESimOne_Data_Vehicle_State *pStateIndics, int size);

		/*!
		获取通过RegisterSimOneVehicleState注册的主车状态信息
		\li function:
		*	GetSimOneVehicleState
		\li brief:
		*	Get states of main vehicle dynamics which are registered by RegisterSimOneVehicleState
		@param[pVehExtraState]
		*   pVehExtraState: states of main vehicle dynamics
		@return
		*	Success or not
		*/
		SIMONE_API bool GetVehicleState(SimOne_Data_Vehicle_Extra* pVehExtraState);

		/*!
		设置主车位置
		\li function:
		*	SetPose
		\li brief:
		*	Set main vehicle pose
		@param
		*   mainVehicleId: Vehilcle index, configure order of web UI, starts from 0
		@param[in]
		*   pPose: Pose to set(input)
		@return
		*	Success or not
		*/
		SIMONE_API bool SetPose(int mainVehicleId, SimOne_Data_Pose_Control *pPose);


		/*!
		主车控制
		\li function:
		*	SetDrive
		\li brief:
		*	Set vehicle drive control
		@param
		*   mainVehicleId: Vehilcle index, configure order of web UI, starts from 0
		@param[in]
		*   pControl: vehicle control data(input)
		@return
		*	Success or not
		*/
		SIMONE_API bool SetDrive(int mainVehicleId, SimOne_Data_Control *pControl);

		/*!
		主车控制
		\li function:
		*	SetDrive by planning trajectory. Not to use with SetDrive at the same time
		\li brief:
		*	Set vehicle drive control by planning trajectory
		@param
		*   mainVehicleId: Vehilcle index, configure order of web UI, starts from 0
		@param[in]
		*   pControlTrajectory: vehicle planning trajectory (input)
		@return
		*	Success or not
		*/
		SIMONE_API bool SetDriveTrajectory(int mainVehicleId, SimOne_Data_Control_Trajectory *pControlTrajectory);

		/*!
		设置主车控制器的名字
		\li function:
		*	SetDriverName
		\li brief:
		*	Set vehicle driver's name
		@param
		*   mainVehicleId: Vehilcle index, configure order of web UI, starts from 0
		@param[in]
		*   name: vehicle driver name, max length is 8
		*/
		SIMONE_API void SetDriverName(int mainVehicleId, const char* name);


		/*!
		设置主车预警消息
		\li function:
		*	SetVehicleEvent
		\li brief:
		*	Set vehicle event information
		@param
		*   mainVehicleId: Vehicle index, configuration order from scenario editing, starting from 0
		@param[in]
		*   pVehicleEvent: vehicle event information data(input)
		@return
		*	Success or not
		*/
		SIMONE_API bool SetVehicleEvent(int mainVehicleId, SimOne_Data_Vehicle_EventInfo *pEvent);

		/*!
		预测轨迹设置
		\li function:
		*	SetTrajectory
		\li brief:
		*	Set vehicle waypoints
		@param
		*   mainVehicleId: Vehicle index, configuration order from scenario editing, starting from 0
		@param[in]
		*   pTrajectory: vehicle trajectory data(input)
		@return
		*	Success or not
		*/
		SIMONE_API bool SetTrajectory(int mainVehicleId, SimOne_Data_Trajectory *Trajectory);


		/*!
		设置车辆信号灯状态
		\li function:
		*	SetSignalLights
		\li brief:
		*	Set signal lights
		@param
		*   mainVehicleId: Vehicle index, configuration order from scenario editing, starting from 0
		@param[in]
		*	pTurnSignal: SimOne_Data_Turn_Signal data (output)
		@return
		*	Success or not
		*/
		SIMONE_API bool SetSignalLights(const int mainVehicleId, SimOne_Data_Signal_Lights *pSignalLights);

		/*!
		获取SimOneDriver运行状态
		\li function:
		*	GetDriverStatus
		\li brief:
		*	Get SimOneDriver status
		@param
		*   mainVehicleId: Vehicle index, configuration order from scenario editing, starting from 0
		@param[out]
		*   pDriverStatus: SimOneDriver status data(output)
		@return
		*	Success or not
		*/
		SIMONE_API bool GetDriverStatus(const int mainVehicleId, SimOne_Data_Driver_Status* pDriverStatus);

		/*!
		获取SimOneDriver控制信号
		\li function:
		*	GetDriverControl
		\li brief:
		*	Get SimOneDriver drive control
		@param
		*   mainVehicleId: Vehicle index, configure order of web UI, starts from 0
		@param[out]
		*   pControl: vehicle control data from SimOneDriver(output)
		@return
		*	Success or not
		*/
		SIMONE_API bool GetDriverControl(const int mainVehicleId, SimOne_Data_Control* pControl);

		/*!
		获取案例主车路径点
		\li function:
		*	GetWayPoints
		\li brief:
		*	Get MainVehile WayPoints
		@param
		*   mainVehicleId: Vehicle index, configure order of web UI, starts from 0
		@param[out]
		*   pWayPoints: MainVehicle WayPoints data(output)
		@return
		*	Success or not
		*/
		SIMONE_API bool GetWayPoints(SimOne_Data_WayPoints* pWayPoints);



		/*!
		场景事件回调
		\li function:
		*	SetScenarioEventCB
		\li brief:
		*	Add scenario event callback
		@param[in]
		*   cb: scenario event callback function
		@param
		*	mainVehicleId: Vehilcle index, configure order of web UI, starts from 0
		*	event: Command sent to the mainVehicle
		*	data: Not used yet
		@return
		*	Success or not
		*/
		SIMONE_API bool SetScenarioEventCB(void(*cb)(int mainVehicleId, const char* event, const char* data));

	}
#ifdef __cplusplus
}
#endif
#endif