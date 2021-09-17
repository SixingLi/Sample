from SimOneServiceAPI import *
from SimOneSensorAPI import *
from SimOneV2XAPI import *

import time 

def start():
	print("start")

def stop():
	print("stop")
Flag = False
if __name__ == '__main__':
	try:
		if SoInitSimOneAPI()==1:
			print("################## API init success!!!")
			Flag =True
		else:
			print("################## API init fail!!!")
	except Exception as e:
		print(e)
		pass
	while Flag:
		gpsData = SimOne_Data_Gps()
		if SoGetGps('0', gpsData):
			print("timestamp:{0},posX:{1},posY:{2},brake:{3},steering:{4}".format(gpsData.timestamp,gpsData.posX,gpsData.posY,gpsData.brake,gpsData.steering))
		
		obstacleData = SimOne_Data_Obstacle()
		if SoGetGroundTruth("0",obstacleData):
			print("Size:{0}".format(obstacleData.obstacleSize))

		v2xData = SimOne_Data_V2XNFS()
		if SoGetV2XInfo("0","v2x",1,v2xData):
			print("Size:{0},MsgFrameData:{1}".format(v2xData.V2XMsgFrameSize,v2xData.MsgFrameData))
		time.sleep(0.3)
		pass