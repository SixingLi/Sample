from SimOneIOStruct import *

SimOne_V2XInfoUpdateCbFuncType = CFUNCTYPE(c_void_p, c_char_p, c_char_p, POINTER(SimOne_Data_V2XNFS))

SIMONEAPI_V2XInfo_CB = None

def _api_v2xInfo_cb(mainVehicleId, sennsorId, data):
	global SIMONEAPI_V2XInfo_CB
	if SIMONEAPI_V2XInfo_CB is None:
		return
	SIMONEAPI_V2XInfo_CB(mainVehicleId, sennsorId, data)

simoneapi_v2xInfo_cb_func = SimOne_V2XInfoUpdateCbFuncType(_api_v2xInfo_cb)


def SoGetV2XInfo(mainVehicleId, sensorId, infoType, detectionData):
	_mainVehicleId = create_string_buffer(mainVehicleId.encode(), 256)
	_sensorId = create_string_buffer(sensorId.encode(), 256)
	SimoneAPI.GetV2XInfo.restype = c_bool
	return SimoneAPI.GetV2XInfo(_mainVehicleId, _sensorId, infoType, pointer(detectionData))

def SoApiSetV2XInfoUpdateCB(cb):
	global SIMONEAPI_V2XInfo_CB
	SimoneAPI.SetV2XInfoUpdateCB(simoneapi_v2xInfo_cb_func)
	SIMONEAPI_V2XInfo_CB = cb
