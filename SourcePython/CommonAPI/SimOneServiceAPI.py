from SimOneIOStruct import *


SimOne_StartCaseFuncType = CFUNCTYPE(c_void_p)
SimOne_StopCaseFuncType = CFUNCTYPE(c_void_p)
SimOne_MainVehicleStatusUpdateFuncType = CFUNCTYPE(c_void_p, POINTER(SimOne_Data_MainVehicle_Status))
SimOne_FrameStartFuncType = CFUNCTYPE(c_void_p, c_int)
SimOne_FrameEndFuncType = CFUNCTYPE(c_void_p, c_int)

G_API_StartCase_CB = None
G_API_StopCase_CB = None
G_API_MainVehicleChangeStatusCB =None
G_API_FrameStart_CB = None
G_API_FrameStop_CB = None

def _api_startcase_cb():
	global G_API_StartCase_CB
	if G_API_StartCase_CB is None:
		return
	G_API_StartCase_CB()

def _api_stopcase_cb():
	global G_API_StopCase_CB
	if G_API_StopCase_CB is None:
		return
	G_API_StopCase_CB()

def _api_mainvehiclestatusupdate_cb(mainVehicleId, data):
	global G_API_MainVehicleChangeStatusCB
	if G_API_MainVehicleChangeStatusCB is None:
		return
	G_API_MainVehicleChangeStatusCB(mainVehicleId, data)

def _api_framestart_cb(frame):
	global G_API_FrameStart_CB
	if G_API_FrameStart_CB is None:
		return
	G_API_FrameStart_CB(frame)

def _api_framestop_cb(frame):
	global G_API_FrameStop_CB
	if G_API_FrameStop_CB is None:
		return
	G_API_FrameStop_CB(frame)

api_startcase_cb = SimOne_StartCaseFuncType(_api_startcase_cb)
api_stopcase_cb = SimOne_StopCaseFuncType(_api_stopcase_cb)
api_mainvehiclestatusupdate_cb = SimOne_MainVehicleStatusUpdateFuncType(_api_mainvehiclestatusupdate_cb)
api_framestart_cb = SimOne_FrameStartFuncType(_api_framestart_cb)
api_framestop_cb = SimOne_FrameEndFuncType(_api_framestop_cb)

def SoAPIGetVersion():
	SimoneAPI.GetVersion.restype = c_char_p
	return SimoneAPI.GetVersion()

def SoSetLogOut(logLevel,*args):
	print(logLevel)
	list = ""
	for arg in args:
		list+=arg
	logStr = bytes(list,encoding='utf-8')
	return SimoneAPI.SetLogOut(logLevel,logStr)

def SoInitSimOneAPI(mainVehicleId='0',isFrameSync = 0,serverIP = '127.0.0.1',port=23789,startcase = 0, stopcase= 0,registerNodeId=0):
	_input = create_string_buffer(serverIP.encode(), 256)
	global G_API_StartCase_CB
	global G_API_StopCase_CB
	if startcase == 0:
		startcase = None
	if stopcase == 0:
		stopcase = None
	G_API_StartCase_CB = startcase
	G_API_StopCase_CB = stopcase
	ret = SimoneAPI.InitSimOneAPI(mainVehicleId,isFrameSync,_input,port,startcase,stopcase,registerNodeId)
	return ret

def SoTerminateSimOneAPI():
	SimoneAPI.TerminateSimOneAPI.restype = c_bool
	return SimoneAPI.TerminateSimOneAPI()

def SoAPIGetCaseInfo(data):
	SimoneAPI.GetCaseInfo.restype = c_bool
	return SimoneAPI.GetCaseInfo(pointer(data))

def SoGetCaseRunStatus():
	SimoneAPI.GetCaseRunStatus.restype = c_int
	return SimoneAPI.GetCaseRunStatus()

def SoGetMainVehicleList(data):
	SimoneAPI.GetMainVehicleList.restype = c_bool
	return SimoneAPI.GetMainVehicleList(pointer(data), True)

def SoAPIWait():
	SimoneAPI.Wait.restype = c_int
	return SimoneAPI.Wait()

def SoAPINextFrame(frame):
	SimoneAPI.NextFrame.restype = c_void_p
	return SimoneAPI.NextFrame(frame)

def SoAPISetFrameCB(startcb, stopcb):
	SimoneAPI.SetFrameCB.restype = c_bool
	global G_API_FrameStart_CB
	global G_API_FrameStop_CB
	if startcb == 0:
		startcb = None
	if stopcb == 0:
		stopcb = None
	G_API_FrameStart_CB = startcb
	G_API_FrameStop_CB = stopcb
	ret = SimoneAPI.SetFrameCB(api_framestart_cb, api_framestop_cb)
	return ret

def SoGetMainVehicleStatus(mainVehicleId, data):
	SimoneAPI.GetMainVehicleStatus.restype = c_bool
	return SimoneAPI.GetMainVehicleStatus(mainVehicleId, pointer(data))

def SoAPISetMainVehicleStatusCB(cb):
	SimoneAPI.SetMainVehicleStatusCB.restype = c_bool
	global G_API_MainVehicleChangeStatusCB
	if cb == 0:
		cb = None
	G_API_MainVehicleChangeStatusCB = cb
	return SimoneAPI.SetMainVehicleStatusCB(api_mainvehiclestatusupdate_cb)
