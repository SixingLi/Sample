from SimOneServiceAPI import *
from SimOneSensorAPI import *
import HDMapAPI

import time 

def start():
	print("start")

def stop():
	print("stop")
Flag = False
if __name__ == '__main__':
	mainVehicleID = '0'
	success_count = 0
	# apiNames=["GetTrafficSignList","GetTrafficLightList","GetCrossHatchList","GetLaneLink","getTrafficLightList", "getNearLanes"]
	apiNames=["getNearLanes"]

	try:
		if SoInitSimOneAPI(mainVehicleID, 0, "127.0.0.1")==1:
			print("################## API init success!!!")
			Flag =True
		else:
			print("################## API init fail!!!")
	except Exception as e:
		print(e)
		pass
	if Flag:
		if HDMapAPI.loadHDMap(10):
			somapdata = SimOne_Data_Map()
			try:
				print("get hdmap data success")
				for apiName in apiNames:
					if apiName == "getNearLanes":
						gpsData = SimOne_Data_Gps()
						if SoGetGps(mainVehicleID, gpsData):
							pt=HDMapAPI.pySimPoint3D(gpsData.posX,gpsData.posY,gpsData.posZ)
							lanesInfo=HDMapAPI.getNearLanes(pt, 3.0)
							if lanesInfo.exists:
								idListSize = lanesInfo.laneIdList.Size()
								if idListSize>0:
									print(">>>>>>>>>>>>>>>>>>>>>  getNearLanes Size = {0}".format(idListSize))
								for i in range(idListSize):
									laneId = lanesInfo.laneIdList.GetElement(i)
									print(">>>>>>>>>>>>>>>>>>>>>  getNearLanes  laneId = {0}".format(laneId.GetString()))
					if apiName == "getTrafficSignList":
						signList=HDMapAPI.getTrafficSignList()
						signSize = signList.Size()
						if signSize!=0:
							print(">>>>>>>>>>>>>>>>>>>>>  getTrafficSignList Size = {0}".format(signSize))
							success_count+=1
					if apiName == "GetLaneLink":
						gpsData = SimOne_Data_Gps()
						if SoGetGps(mainVehicleID, gpsData):
							pt=HDMapAPI.pySimPoint3D(gpsData.posX,gpsData.posY,gpsData.posX)
							info=HDMapAPI.getNearMostLane(pt)
							laneLinkInfo = HDMapAPI.getLaneLink(info.laneId)
							HDMapAPI.pyLaneLink
							if laneLinkInfo.exists:
								laneId = laneLinkInfo.laneLink.leftNeighborLaneId.GetString()
								print(">>>>>>>>>>>>>>>>>>>>>  getLaneLink leftNeighborLaneId = {0}".format(laneId))
								success_count+=1
					if apiName == "getCrossHatchListame":
						gpsData = SimOne_Data_Gps()
						if SoGetGps(mainVehicleID, gpsData):
							pt=HDMapAPI.pySimPoint3D(gpsData.posX,gpsData.posY,gpsData.posX)
							info=HDMapAPI.getNearMostLane(pt)
							HatchList = HDMapAPI.getCrossHatchList(info.laneId)
							hatchSize = HatchList.Size()
							if hatchSize>0:
								print(">>>>>>>>>>>>>>>>>>>>>  GetCrossHatchList size = {0}".format(hatchSize))
								success_count+=1
					if apiName == "getTrafficLightList":
						taffficLightList = HDMapAPI.pySignalSimVector(HDMapAPI.getTrafficLightList())
						taffficLightListSize = taffficLightList.Size()
						if taffficLightListSize>0:
							print(">>>>>>>>>>>>>>>>>>>>>  getTrafficLightList size = {0}".format(taffficLightListSize))
							success_count+=1
						for i in range(taffficLightListSize):
							pySignali = taffficLightList.GetElement(i)
							trafficLigtData = SimOne_Data_TrafficLight()
							print(">>>>>>>>>>>>>>>>>>>>>  SoGetTrafficLights opendriveId = {0}".format(pySignali.id))
							if SoGetTrafficLights(mainVehicleID,pySignali.id,trafficLigtData):
								print(">>>>>>>>>>>>>>>>>>>>>  SoGetTrafficLights opendriveId = {0}, trafficLigtData.countDown = {1}, trafficLigtData.status = {2}".format(pySignali.id,trafficLigtData.countDown,trafficLigtData.status.value))

					time.sleep(0.3)
				if success_count==len(apiNames):
					print("#####################  Test all api success")
			except Exception as e:
				print(e)	
		pass