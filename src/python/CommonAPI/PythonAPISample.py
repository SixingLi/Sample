from SimOneServiceAPI import *
from SimOneSensorAPI import *
from SimOneV2XAPI import *
from SimOnePNCAPI import *

import time 

def start():
	print("start")

def stop():
	print("stop")

def SoV2XCB(mainVehicleId, sensorId, Data_V2XNFS):
	if Data_V2XNFS:
		print("Data_V2XNFS:{0}, SensorID:{1}, Data_V2XNFS_Size:{2}, Data_V2XNFS_Frame: {3}".format(mainVehicleId,sensorId,Data_V2XNFS[0].V2XMsgFrameSize,Data_V2XNFS[0].MsgFrameData))

def SoSensorDetectionsCB(mainVehicleId, sensorId, Data_Groundtruth):
	if Data_Groundtruth:
		print("mainVehicleId: {0}".format(mainVehicleId))
		print("SensorID: {0}".format(sensorId))
		objSize = Data_Groundtruth[0].objectSize
		print("objectSize: {0}".format(objSize))
		for i in range(0,objSize):
			print("objects[{0}].id: {1}".format(i, Data_Groundtruth[0].objects[i].id))
			print("objects[{0}].type: {1}".format(i, Data_Groundtruth[0].objects[i].type))
			print("objects[{0}].posX: {1}".format(i, Data_Groundtruth[0].objects[i].posX))
			print("objects[{0}].posY: {1}".format(i, Data_Groundtruth[0].objects[i].posY))
			print("objects[{0}].posZ: {1}".format(i, Data_Groundtruth[0].objects[i].posZ))
			print("objects[{0}].oriX: {1}".format(i, Data_Groundtruth[0].objects[i].oriX))
			print("objects[{0}].oriY: {1}".format(i, Data_Groundtruth[0].objects[i].oriY))
			print("objects[{0}].oriZ: {1}".format(i, Data_Groundtruth[0].objects[i].oriZ))
			print("objects[{0}].length: {1}".format(i, Data_Groundtruth[0].objects[i].length))
			print("objects[{0}].width: {1}".format(i, Data_Groundtruth[0].objects[i].width))
			print("objects[{0}].height: {1}".format(i, Data_Groundtruth[0].objects[i].height))
			print("objects[{0}].range: {1}".format(i, Data_Groundtruth[0].objects[i].range))
			print("objects[{0}].velX: {1}".format(i, Data_Groundtruth[0].objects[i].velX))
			print("objects[{0}].velY: {1}".format(i, Data_Groundtruth[0].objects[i].velY))
			print("objects[{0}].velZ: {1}".format(i, Data_Groundtruth[0].objects[i].velZ))
			print("objects[{0}].probability: {1}".format(i, Data_Groundtruth[0].objects[i].probability))
			print("objects[{0}].relativePosX: {1}".format(i, Data_Groundtruth[0].objects[i].relativePosX))
			print("objects[{0}].relativePosY: {1}".format(i, Data_Groundtruth[0].objects[i].relativePosY))
			print("objects[{0}].relativePosZ: {1}".format(i, Data_Groundtruth[0].objects[i].relativePosZ))
			print("objects[{0}].relativeRotX: {1}".format(i, Data_Groundtruth[0].objects[i].relativeRotX))
			print("objects[{0}].relativeRotY: {1}".format(i, Data_Groundtruth[0].objects[i].relativeRotY))
			print("objects[{0}].relativeRotZ: {1}".format(i, Data_Groundtruth[0].objects[i].relativeRotZ))
			print("objects[{0}].relativeVelX: {1}".format(i, Data_Groundtruth[0].objects[i].relativeVelX))
			print("objects[{0}].relativeVelY: {1}".format(i, Data_Groundtruth[0].objects[i].relativeVelY))
			print("objects[{0}].relativeVelZ: {1}".format(i, Data_Groundtruth[0].objects[i].relativeVelZ))
			print("objects[{0}].bbox2dMinX: {1}".format(i, Data_Groundtruth[0].objects[i].bbox2dMinX))
			print("objects[{0}].bbox2dMinY: {1}".format(i, Data_Groundtruth[0].objects[i].bbox2dMinY))
			print("objects[{0}].bbox2dMaxX: {1}".format(i, Data_Groundtruth[0].objects[i].bbox2dMaxX))
			print("objects[{0}].bbox2dMaxY: {1}".format(i, Data_Groundtruth[0].objects[i].bbox2dMaxY))

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

	SoApiSetSensorDetectionsUpdateCB(SoSensorDetectionsCB)

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
		# 	llaneid = planeInfo.laneLeftID;
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
		

		time.sleep(1) # 0.1
		pass
