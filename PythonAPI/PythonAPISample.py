from SimOneServiceAPI import *
from SimOneSensorAPI import *
from SimOneV2XAPI import *

import time 

def start():
	print("start")

def stop():
	print("stop")

if __name__ == '__main__':
	try:
		if SoInitSimOneAPI(start,stop)==1:
			print("################## API init success!!!")
		else:
			print("################## API init fail!!!")
	except Exception as e:
		print(e)
		pass
	while True:
		gpsData = SimOne_Data_Gps()
		SoGetGps(0, gpsData)
		print("timestamp:{0},posX:{1},posY:{2},brake:{3},steering:{4}".format(gpsData.timestamp,gpsData.posX,gpsData.posY,gpsData.brake,gpsData.steering))
		time.sleep(0.3)
		v2xData = SimOne_Data_V2XNFS()
		SoGetV2XInfo("0","v2x",1,v2xData)
		print("Size:{0},MsgFrameData:{1}".format(v2xData.V2XMsgFrameSize,v2xData.MsgFrameData))
		pass