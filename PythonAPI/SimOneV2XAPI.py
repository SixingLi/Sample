from SimOneIOStruct import *

SimOne_V2XInfoUpdateCbFuncType = CFUNCTYPE(c_void_p, c_int, POINTER(SimOne_Data_V2XNFS))

SIMONEAPI_V2XInfo_CB = None

def _api_v2xInfo_cb(mainVehicleId, evt, data):
	global SIMONEAPI_V2XInfo_CB
	if SIMONEAPI_V2XInfo_CB is None:
		return
	SIMONEAPI_V2XInfo_CB(mainVehicleId, evt, data)

simoneapi_v2xInfo_cb_func = SimOne_V2XInfoUpdateCbFuncType(_api_v2xInfo_cb)


def SoGetV2XInfo(mainVehicleId, sensorId, infoType, detectionData):
	print(sensorId)
	return SimoneAPI.GetV2XInfo(mainVehicleId, sensorId, infoType, pointer(detectionData))

def SoApiSetV2XInfoUpdateCB(cb):
	global SIMONEAPI_V2XInfo_CB
	SimoneAPI.SetV2XInfoUpdateCB(simoneapi_v2xInfo_cb_func)
	SIMONEAPI_V2XInfo_CB = cb
