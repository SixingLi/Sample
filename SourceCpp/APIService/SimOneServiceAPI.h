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
		节点通信发送数据API
		\li function:
		*	SendRouteMessage
		\li brief:
		*	node send data to other node.
		@param
		*	length: send data length.
		@param
		*	pBuffer: send data
		@return
		*	Success or not
		*/
		SIMONE_API bool SendRouteMessage(int length, void* pBuffer, int msgId, int toNodeId, ESimOne_Client_Type toNodeType);
	
		/*!
		节点通信接收数据API
		\li function:
		*	ReceiveRouteMessageCB
		\li brief:
		*	API Recvice data from other node.
		@param
		*	length: the data length recevied 
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
		@param
		*	level:warning,error,info flag:true/false
		@return
		*	Success or not
		*/
		SIMONE_API bool SetLogOut(ESimOne_LogLevel_Type level, const char *format, ...);

		/*!
		设置服务的地址和端口号
		\li function:
		*	SetServerInfo
		\li brief:
		*	Set the address and port number of the service
		@param
		*	serverIP:set connection bridgeio of IP address,port:set connection bridgeio of IP port
		@return
		*	Success or not
		*/
		SIMONE_API bool SetServerInfo(const char *serverIP = "127.0.0.1", int port = 23789);

		/*!
		SimOne API主入口
		\li function:
		*	InitSimOneAPI
		\li brief:
		*	Initialize SimOneAPI for autonomous driving algorithm
		@param
		*	hostVehicleId: host vehicle ID(from 0 to 9)
		@param
		*   isFrameSync: synchronize frame or not
		@return
		*	None
		*/
		SIMONE_API bool InitSimOneAPI(const char* mainVehicleId = "0", bool isFrameSync =false, const char *serverIP = "127.0.0.1", int port = 23789,void(*startCase)()=0, void(*endCase)()=0, int registerNodeId=0);

		/*!
		停止API node
		\li function:
		*	GetVersion
		\li brief:
		*	Stop SimOne API node
		@param
		*	None
		@return
		*	Success or not
		*/
		SIMONE_API bool StopSimOneNode();

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
		*	pMainVehicleInfo:mainvehicle id/num/type data(output)
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
		*   None
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
		@param
		*   frame: current frame value
		@return
		*	None
		*/
		SIMONE_API void NextFrame(int frame);


		/*!
		仿真场景中每帧的回调,每帧开始和结束时调用回调函数
		\li function:
		*	SetFrameCB
		\li brief:
		*	The callback function is called at the beginning and end of each frame.
		@param
		*   FrameStart:Callback function at the beginning of the frame 
		@param
		*   FrameEnd:Callback function at the end of the frame
		@return
		*	Success or not
		*/
		SIMONE_API bool SetFrameCB(void(*FrameStart)(int frame), void(*FrameEnd)(int frame));

		/*!
		获取主车状态信息
		\li function:
		*	GetMainVehicleStatus
		\li brief:
		*	Get the status information of mainvehicle
		@param
		*	pMainVehicleStatus[out]:id,status
		@return
		*	Success or not
		*/
		SIMONE_API bool GetMainVehicleStatus(SimOne_Data_MainVehicle_Status *pMainVehicleStatus);

		/*!
		获取主车状态信息回调
		\li function:
		*	SetMainVehicleStatusCB
		\li brief:
		*	Get the status information of mainvehicle callback
		@param
		*	cb: VehicleStatus data fetch callback function
		@return
		*	Success or not
		*/
		SIMONE_API bool SetMainVehicleStatusUpdateCB(void(*cb)(SimOne_Data_MainVehicle_Status *pMainVehicleStatus));
	
		/*!
		获取高精度地图标识
		\li function:
		*	GetHDMapData
		\li brief:
		*	Get hdmap data which is configured by SimOne web app.
		@param
		*   hdMap: SimOne_Data_Map.
		@return
		*	True when get HDMap data success, else returns false.
		*/
		SIMONE_API bool GetHDMapData(SimOne_Data_Map& hdMap);

	}
#ifdef __cplusplus
}
#endif
