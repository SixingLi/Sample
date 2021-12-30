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

	namespace SimOneAPI{
	
		/*!
		获取当前库的版本号
		\li function:
		*	GetVersion
		\li brief:
		*	Get the version number of the current library
		@param
		*	None
		@return
		*	version number
		*/
		SIMONE_API const char* GetVersion();

		/*!
		节点通信数据发送API
		\li function:
		*	SendRouteMessage
		\li brief:
		*	send data to other node.
		@param[in]
		*	length: send data length.
		@param[in]
		*	pBuffer: send data
		@param[in]
		*	msgId: message id
		@param[in]
		*	toNodeId: the node's id which the message send to
		@param[in]
		*	toNodeType: the node's type which the message send to
		@return
		*	Success or not
		*/
		SIMONE_API bool SendRouteMessage(int length, void* pBuffer, int msgId, int toNodeId, ESimOne_Client_Type toNodeType);
	
		/*!
		节点通信数据接收API
		\li function:
		*	ReceiveRouteMessageCB
		\li brief:
		*	API Recvice data from other node.
		@param
		*	cb: call back func registered for receiving message
		*	param[out]:
		*		fromId: the node's id which the message sent from
		*	param[out]:
		*		fromType: the node's type which the message sent from
		*	param[out];
		*		length: the data length recevied 
		*	param[out];
		*		pBuffer: the data content recevied 
		*	param[out];
		*		commandId: not in use
		@return
		*	Success or not
		*/
		SIMONE_API bool ReceiveRouteMessageCB(void(*cb)(int fromId, ESimOne_Client_Type fromType, int length, const void* pBuffer, int commandId));

		/*!
		日志设置接口
		\li function:
		*	SetLogOut
		\li brief:
		*	Set log interface
		@param[in]
		*	level: warning,error,info flag:true/false
		@param[in]
		*	format: output format
		@return
		*	Success or not
		*/
		SIMONE_API bool SetLogOut(ESimOne_LogLevel_Type level, const char *format, ...);

		/*!
		初始化SimOne API
		\li function:
		*	InitSimOneAPI
		\li brief:
		*	Initialize SimOneAPI for autonomous driving algorithm
		@param[in]
		*	hostVehicleId: host vehicle ID(from 0 to 9)
		@param[in]
		*	isFrameSync: synchronize frame or not
		@param[in]
		*	serverIP: BridgeIO server ip
		@param[in]
		*	port: BridgeIO server port
		@param[in]
		*	startCase: callback func which being called before case start
		@param[in]
		*	endCase: callback func which being called after case end
		@param[in]
		*	registerNodeId: not in use
		@return
		*	None
		*/
		SIMONE_API bool InitSimOneAPI(const char* mainVehicleId = "0", bool isFrameSync =false, const char *serverIP = "127.0.0.1", int port = 23789,void(*startCase)()=0, void(*endCase)()=0, int registerNodeId=0);

		/*!
		退出API node
		\li function:
		*	GetVersion
		\li brief:
		*	Stop SimOne API node
		@param
		*	None
		@return
		*	Success or not
		*/
		SIMONE_API bool TerminateSimOneAPI();

		/*!
		获取案例详情
		\li function:
		*	GetCaseInfo
		\li brief:
		*	Get case information
		@param[out]
		*	pCaseInfo: caseName,caseId,taskId,sessionId
		@return
		*	Success or not
		*/
		SIMONE_API bool GetCaseInfo(SimOne_Data_CaseInfo *pCaseInfo);

		/*!
		获取案例运行情况（运行中，停止）
		\li function:
		*	GetCaseRunStatus
		\li brief:
		*	Get case running status
		@param
		*	None
		@return
		*	Stop,Running
		*/
		SIMONE_API ESimOne_Case_Status GetCaseRunStatus();

		/*!
		获取主车信息列表，只需要获取一次
		\li function:
		*	GetMainVehicleList
		\li brief:
		*	Get the main vehicle information list
		@param[out]
		*	pMainVehicleInfo: mainvehicle id/num/type data(output)
		@return
		*	Success or not
		*/
		SIMONE_API bool GetMainVehicleList(SimOne_Data_MainVehicle_Info *pMainVehicleInfo);

		/*!
		获取当前帧值
		\li function:
		*	Wait
		\li brief:
		*	Get the current frame value
		@param
		*	None
		@return
		*	frame value
		*/
		SIMONE_API int Wait();

		/*!
		进行下一帧
		\li function:
		*	NextFrame
		\li brief:
		*	Go to the next frame
		@param[in]
		*	frame: current frame value
		@return
		*	None
		*/
		SIMONE_API void NextFrame(int frame);

		/*!
		仿真场景中每帧的回调,每帧开始和结束时调用回调函数
		\li function:
		*	SetFrameCB
		\li brief:
		*	Register the callback func which being called at the beginning and end of each frame.
		@param[in]
		*	FrameStart: callback func which being called at the beginning of the frame
		*	frame: current frame index
		@param[in]
		*	FrameEnd: callback func which being called at the end of the frame
		*	frame: current frame index
		@return
		*	Success or not
		*/
		SIMONE_API bool SetFrameCB(void(*FrameStart)(int frame), void(*FrameEnd)(int frame));

		/*!
		获取主车状态信息
		\li function:
		*	GetMainVehicleStatus
		\li brief:
		*	Get the status information of the mainvehicle
		@param[in]
		*	mainVehicleId: Id of the main vehicle
		@param[out]
		*	pMainVehicleStatus: status data which applied for
		@return
		*	Success or not
		*/
		SIMONE_API bool GetMainVehicleStatus(const char* mainVehicleId, SimOne_Data_MainVehicle_Status *pMainVehicleStatus);

		/*!
		获取主车状态信息回调
		\li function:
		*	SetMainVehicleStatusCB
		\li brief:
		*	Register the callback func applying for status info of the mainvehicle
		@param[in]
		*	cb: callback func applying for status info of the mainvehicle
		*	param[out]
		*	mainVehicleId: Id of the main vehicle
		*	pMainVehicleStatus: VehicleStatus data
		@return
		*	Success or not
		*/
		SIMONE_API bool SetMainVehicleStatusUpdateCB(void(*cb)(const char* mainVehicleId, SimOne_Data_MainVehicle_Status *pMainVehicleStatus));
	
		/*!
		获取高精度地图标识
		\li function:
		*	GetHDMapData
		\li brief:
		*	Get the hdmap data which is designated by configuring on SimOne web app.
		@param[out]
		*	hdMap: SimOne_Data_Map data
		@return
		*	Success or not
		*/
		SIMONE_API bool GetHDMapData(SimOne_Data_Map* hdMap);

	}
#ifdef __cplusplus
}
#endif
