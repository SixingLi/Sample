from SimOneServiceAPI import *
from SimOneSensorAPI import *
from SimOneV2XAPI import *
from SimOnePNCAPI import *
from SimOneStreamingAPI import *

import time 

def start():
	print("start")

def stop():
	print("stop")

def SoV2XCB(mainVehicleId, sensorId, Data_V2XNFS):
	if Data_V2XNFS:
		print("Data_V2XNFS:{0}, SensorID:{1}, Data_V2XNFS_Size:{2}, Data_V2XNFS_Frame: {3}".format(mainVehicleId,sensorId,Data_V2XNFS[0].V2XMsgFrameSize,Data_V2XNFS[0].MsgFrameData))

def SoMainVehicleStaus(mainVehicleId, data):
	if data:
		print("mainVehicleId:{0},data:{1}".format(mainVehicleId,data.mainVehicleStatus))

def SoSetSensorDetectionUpdateCBTest(mainVehicleId, sensorId, data):
	if data:
		# print("mainVehicleId:{0},SensorId:{1}, data:{2}".format(mainVehicleId,sensorId,data[0].objectSize))
		for i in range(data[0].objectSize):
			print("data[0][{0}].type:{1}".format(i,data[0].objects[i].type.value))

Flag = False
if __name__ == '__main__':
	mainVehicleID = '0'
	try:
		if SoInitSimOneAPI(mainVehicleID, 0, "10.66.9.111")==1:
			print("################## API init success!!!")
			Flag =True
		else:
			print("################## API init fail!!!")
	except Exception as e:
		print(e)
		pass

	# SoApiSetV2XInfoUpdateCB(SoV2XCB)
	# SoAPISetMainVehicleStatusUpdateCB(SoMainVehicleStaus)
	SoApiSetSensorDetectionsUpdateCB(SoSetSensorDetectionUpdateCBTest)

	while Flag:
		# waypoint = SimOne_Data_WayPoints()
		# SoGetWayPoints(mainVehicleID,waypoint)
		# vehicleState = (ESimOne_Data_Vehicle_State * 3)(
		# 	ESimOne_Data_Vehicle_State.ESimOne_Data_Vehicle_State_SO_M_SW, 
		# 	ESimOne_Data_Vehicle_State.ESimOne_Data_Vehicle_State_S0_Vz_SM, 
		# 	ESimOne_Data_Vehicle_State.ESimOne_Data_Vehicle_State_SO_My_DR_L1);
		# vehicleStatelen = len(vehicleState)
		# if(SoRegisterVehicleState(mainVehicleID,vehicleState,vehicleStatelen)):
		# 	print("RegisterVehicleState Success")

		# vehicleExtraState = SimOne_Data_Vehicle_Extra();
		# if(SoGetVehicleState(mainVehicleID,vehicleExtraState)):
		# 	for i in range(0,vehicleStatelen):
		# 		print("SoGetVehicleState Success: state{0}:= {1}".format(i,vehicleExtraState.extra_states[i]))
			
		# gpsData = SimOne_Data_Gps()
		# if SoGetGps(mainVehicleID, gpsData):
		# 	print("timestamp:{0},posX:{1},posY:{2},brake:{3},steering:{4}".format(gpsData.timestamp,gpsData.posX,gpsData.posY,gpsData.brake,gpsData.steering))
		
		# obstacleData = SimOne_Data_Obstacle()
		# if SoGetGroundTruth(mainVehicleID,obstacleData):
		# 	print("Size:{0}".format(obstacleData.obstacleSize))

		# v2xData = SimOne_Data_V2XNFS()
		# if SoGetV2XInfo(mainVehicleID,"obu1",1,v2xData):
		# 	print("Size:{0},MsgFrameData:{1}".format(v2xData.V2XMsgFrameSize,v2xData.MsgFrameData))

		
		# planeInfo = SimOne_Data_LaneInfo()
		# if SoGetSensorLaneInfo(mainVehicleID,"objectBasedCamera1",planeInfo):
		# 	llaneid = planeInfo.lanseLeftID;
		# 	print("planeInfo.laneLeftID:{0}".format(llaneid))

		# pMainVehicleInfo = SimOne_Data_MainVehicle_Info();
		# if SoGetMainVehicleList(pMainVehicleInfo):
		# 	print("	pMainVehicleInfo.size:{0}, pMainVehicleInfo.idList:{1}".format(pMainVehicleInfo.size,pMainVehicleInfo.id_list[0]))
		# 	for index in range(pMainVehicleInfo.size):
		# 		print("pMainVehicleInfo.id:		{0}".format(pMainVehicleInfo.id_list[index].value))
		# 	for indextype in range(pMainVehicleInfo.size):
		# 		print("pMainVehicleInfo.type:		{0}".format(pMainVehicleInfo.type_list[indextype].value))

		# pSensorConfigs = SimOne_Data_SensorConfigurations()
		# if SoGetSensorConfigurations(mainVehicleID, pSensorConfigs):
		# 	for index in range(pSensorConfigs.dataSize):
		# 		print("pSensorConfig.sensorId:{0}, pSensorConfig.SensorType:{1}".format(pSensorConfigs.data[index].sensorId,pSensorConfigs.data[index].sensorType))
		
		# pHdmapData = SimOne_Data_Map()
		# if SoGetHDMapData(pHdmapData):
		# 	print("pHdmapData.openDrive:{0},pHdmapData.openDriveUrl:{1}".format(pHdmapData.openDrive,pHdmapData.openDriveUrl))

		# ImageData = SimOne_Data_Image()
		# Ip = "10.66.9.244"
		# Port = 13944
		# if SoGetStreamingImage(Ip, Port, ImageData):
		# 	print("ImageData.width:{0},ImageData.height:{1}, ImageData.format:{2}, ImageData.imagedataSize:{3}".format(ImageData.width, ImageData.height, ImageData.format, ImageData.imagedataSize))

		time.sleep(0.1)
		pass
