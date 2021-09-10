from SimOneIOStruct import *

SimOne_ScenarioEventCBType = CFUNCTYPE(c_void_p, c_int, c_char_p, c_char_p)

G_API_ScenarioEvent_CB = None

def _api_scenarioEvent_cb(mainVehicleId, evt, data):
	global G_API_ScenarioEvent_CB
	if G_API_ScenarioEvent_CB is None:
		return
	G_API_ScenarioEvent_CB(mainVehicleId, evt, data)

api_scenarioEvent_cb = SimOne_ScenarioEventCBType(_api_scenarioEvent_cb)

def SoRegisterVehicleState(data):
    SimoneAPI.RegisterSimOneVehicleState.restype = c_bool
    return SimoneAPI.RegisterSimOneVehicleState(pointer(data), len(data))

def SoGetVehicleState(data):
	SimoneAPI.GetSimOneVehicleState.restype = c_bool
	return SimoneAPI.GetSimOneVehicleState(pointer(data))

def SoSetPose(mainVehicleId, poseControl):
	return SimoneAPI.SetPose(mainVehicleId, pointer(poseControl))

def SoSetDrive(mainVehicleId, driveControl):
	return SimoneAPI.SetDrive(mainVehicleId, pointer(driveControl))

def SoSetDriverName(mainVehicleId, driverName):
	_input = create_string_buffer(driverName.encode(), 256)
	SimoneAPI.SetDriverName.restype = c_bool
	return SimoneAPI.SetDriverName(mainVehicleId, _input)

def SoSetVehicleEvent(mainVehicleId, vehicleEventInfo):
	return SimoneAPI.SetVehicleEvent(mainVehicleId, pointer(vehicleEventInfo))

def SoSetSignalLights(mainVehicleId, pSignalLight):
	return SimoneAPI.SetSignalLights(mainVehicleId, pointer(pSignalLight))

def SoGetDriverStatus(mainVehicleId, driverStatusData):
	return SimoneAPI.GetDriverStatus(mainVehicleId, pointer(driverStatusData))

def SoGetDriverControl(mainVehicleId, driverControlData):
	return SimoneAPI.GetDriverControl(mainVehicleId, pointer(driverControlData))

def SoGetWayPoints(wayPointsData):
	return SimoneAPI.GetWayPoints(pointer(wayPointsData))

def SoAPISetScenarioEventCB(cb):
	if cb == 0:
		cb = None
	global G_API_ScenarioEvent_CB
	G_API_ScenarioEvent_CB = cb
	SimoneAPI.SetScenarioEventCB.restype = c_bool
	ret = SimoneAPI.SetScenarioEventCB(api_scenarioEvent_cb)
	return ret
