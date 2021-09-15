from SimOneIOStruct import *

SimOne_ScenarioEventCBType = CFUNCTYPE(c_void_p, c_char_p, c_char_p, c_char_p)

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
	SimoneAPI.SetPose.restype = c_bool
	_mainVehicleId = create_string_buffer(mainVehicleId.encode(), 256)
	return SimoneAPI.SetPose(_mainVehicleId, pointer(poseControl))

def SoSetDrive(mainVehicleId, driveControl):
	SimoneAPI.SetDrive.restype = c_bool
	_mainVehicleId = create_string_buffer(mainVehicleId.encode(), 256)
	return SimoneAPI.SetDrive(_mainVehicleId, pointer(driveControl))

def SoSetDriverName(mainVehicleId, driverName):
	SimoneAPI.SetDriverName.restype = c_bool
	_driverName = create_string_buffer(driverName.encode(), 256)
	_mainVehicleId = create_string_buffer(mainVehicleId.encode(), 256)
	return SimoneAPI.SetDriverName(_mainVehicleId, _driverName)

def SoSetVehicleEvent(mainVehicleId, vehicleEventInfo):
	SimoneAPI.SetVehicleEvent.restype = c_bool
	_mainVehicleId = create_string_buffer(mainVehicleId.encode(), 256)
	return SimoneAPI.SetVehicleEvent(_mainVehicleId, pointer(vehicleEventInfo))

def SoSetSignalLights(mainVehicleId, pSignalLight):
	SimoneAPI.SetSignalLights.restype = c_bool
	_mainVehicleId = create_string_buffer(mainVehicleId.encode(), 256)
	return SimoneAPI.SetSignalLights(_mainVehicleId, pointer(pSignalLight))

def SoGetDriverStatus(mainVehicleId, driverStatusData):
	SimoneAPI.GetDriverStatus.restype = c_bool
	_mainVehicleId = create_string_buffer(mainVehicleId.encode(), 256)
	return SimoneAPI.GetDriverStatus(_mainVehicleId, pointer(driverStatusData))

def SoGetDriverControl(mainVehicleId, driverControlData):
	SimoneAPI.GetDriverControl.restype = c_bool
	_mainVehicleId = create_string_buffer(mainVehicleId.encode(), 256)
	return SimoneAPI.GetDriverControl(_mainVehicleId, pointer(driverControlData))

def SoGetWayPoints(wayPointsData):
	SimoneAPI.GetWayPoints.restype = c_bool
	return SimoneAPI.GetWayPoints(pointer(wayPointsData))

def SoAPISetScenarioEventCB(cb):
	if cb == 0:
		cb = None
	global G_API_ScenarioEvent_CB
	G_API_ScenarioEvent_CB = cb
	SimoneAPI.SetScenarioEventCB.restype = c_bool
	ret = SimoneAPI.SetScenarioEventCB(api_scenarioEvent_cb)
	return ret