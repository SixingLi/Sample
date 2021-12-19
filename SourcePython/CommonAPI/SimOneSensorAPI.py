from SimOneIOStruct import *

GpsCbFuncType = CFUNCTYPE(c_void_p, c_char_p, POINTER(SimOne_Data_Gps))
GroundTruthCbFuncType = CFUNCTYPE(c_void_p, c_char_p, POINTER(SimOne_Data_Obstacle))
SensorLaneInfoCbFuncType = CFUNCTYPE(c_void_p,c_char_p, c_char_p, POINTER(SimOne_Data_LaneInfo))
UltrasonicsCbFuncType = CFUNCTYPE(c_void_p, c_char_p, POINTER(SimOne_Data_UltrasonicRadars))
RadarDetectionCbFuncType = CFUNCTYPE(c_void_p, c_char_p, c_char_p,POINTER(SimOne_Data_RadarDetection))
SensorDetectionsCbFuncType = CFUNCTYPE(c_void_p, c_char_p, c_char_p,POINTER(SimOne_Data_SensorDetections))

SIMONEAPI_GPS_CB = None
SIMONEAPI_GROUNDTRUTH_CB = None
SIMONEAPI_SENSORLANEINFO_CB = None
SIMONEAPI_ULTRASONICS_CB = None
SIMONEAPI_RADARDETECTION_CB = None
SIMONEAPI_SENSOR_DETECTIONS_CB = None

def _simoneapi_gps_cb(mainVehicleId, data):
	global SIMONEAPI_GPS_CB
	SIMONEAPI_GPS_CB(mainVehicleId, data)

def _simoneapi_groundtruth_cb(mainVehicleId, data):
	global SIMONEAPI_GROUNDTRUTH_CB
	SIMONEAPI_GROUNDTRUTH_CB(mainVehicleId, data)

def _simoneapi_sensorlaneinfo_cb(mainVehicleId, sensorId, data):
	global SIMONEAPI_SENSORLANEINFO_CB
	SIMONEAPI_SENSORLANEINFO_CB(mainVehicleId, sensorId, data)

def _simoneapi_ultrasonics_cb(mainVehicleId, data):
	global SIMONEAPI_ULTRASONICS_CB
	SIMONEAPI_ULTRASONICS_CB(mainVehicleId, data)

def _simoneapi_radardetection_cb(mainVehicleId, sensorId, data):
	global SIMONEAPI_RADARDETECTION_CB
	SIMONEAPI_RADARDETECTION_CB(mainVehicleId, sensorId, data)

def _simoneapi_sensor_detections_cb(mainVehicleId, sensorId, data):
	global SIMONEAPI_SENSOR_DETECTIONS_CB
	SIMONEAPI_SENSOR_DETECTIONS_CB(mainVehicleId, sensorId, data)

simoneapi_gps_cb_func = GpsCbFuncType(_simoneapi_gps_cb)
simoneapi_groundtruth_cb_func = GroundTruthCbFuncType(_simoneapi_groundtruth_cb)
simoneapi_sensorlaneinfo_cb_func = SensorLaneInfoCbFuncType(_simoneapi_sensorlaneinfo_cb)
simoneapi_ultrasonics_cb_func = UltrasonicsCbFuncType(_simoneapi_ultrasonics_cb)
simoneapi_radardetection_cb_func = RadarDetectionCbFuncType(_simoneapi_radardetection_cb)
simoneapi_sensor_detections_cb_func = SensorDetectionsCbFuncType(_simoneapi_sensor_detections_cb)


def SoGetGps(mainVehicleId, gpsData):
	SimoneAPI.GetGps.restype = c_bool
	_mainVehicleId = create_string_buffer(mainVehicleId.encode(), 256)
	return SimoneAPI.GetGps(_mainVehicleId, pointer(gpsData))

def SoApiSetGpsUpdateCB(cb):
	global SIMONEAPI_GPS_CB
	SimoneAPI.SetGpsUpdateCB(simoneapi_gps_cb_func)
	SIMONEAPI_GPS_CB = cb

def SoGetGroundTruth(mainVehicleId, obstacleData):
	SimoneAPI.GetGroundTruth.restype = c_bool
	_mainVehicleId = create_string_buffer(mainVehicleId.encode(), 256)
	ret = SimoneAPI.GetGroundTruth(_mainVehicleId, pointer(obstacleData))
	return ret

def SoApiSetGroundTruthUpdateCB(cb):
	global SIMONEAPI_GROUNDTRUTH_CB
	SimoneAPI.SetGroundTruthUpdateCB(simoneapi_groundtruth_cb_func)
	SIMONEAPI_GROUNDTRUTH_CB = cb

def SoGetRadarDetections(mainVehicleId, sensorId, detectionData):
	SimoneAPI.GetRadarDetections.restype = c_bool
	_mainVehicleId = create_string_buffer(mainVehicleId.encode(), 256)
	_sensorId = create_string_buffer(sensorId.encode(), 256)
	return SimoneAPI.GetRadarDetections(_mainVehicleId, _sensorId, pointer(detectionData))

def SoApiSetRadarDetectionsUpdateCB(cb):
	global SIMONEAPI_RADARDETECTION_CB
	SimoneAPI.SetRadarDetectionsUpdateCB(simoneapi_radardetection_cb_func)
	SIMONEAPI_RADARDETECTION_CB = cb

def SoGetUltrasonicRadar(mainVehicleId, sensorId, ultrasonics):
	SimoneAPI.GetUltrasonicRadar.restype = c_bool
	_mainVehicleId = create_string_buffer(mainVehicleId.encode(), 256)
	_sensorId = create_string_buffer(sensorId.encode(), 256)
	return SimoneAPI.GetUltrasonicRadar(_mainVehicleId, _sensorId, pointer(ultrasonics))

def SoGetUltrasonicRadars(mainVehicleId, ultrasonics):
	SimoneAPI.GetUltrasonicRadar.restype = c_bool
	_mainVehicleId = create_string_buffer(mainVehicleId.encode(), 256)
	return SimoneAPI.GetUltrasonicRadars(_mainVehicleId, pointer(ultrasonics))

def SoApiSetUltrasonicRadarsCB(cb):
	global SIMONEAPI_ULTRASONICS_CB
	SimoneAPI.SetUltrasonicRadarsCB(simoneapi_ultrasonics_cb_func)
	SIMONEAPI_ULTRASONICS_CB = cb

def SoGetSensorDetections(mainVehicleId, sensorId, sensorDetections):
	_mainVehicleId = create_string_buffer(mainVehicleId.encode(), 256)
	_sensorId = create_string_buffer(sensorId.encode(), 256)
	SimoneAPI.GetSensorDetections.restype = c_bool
	return SimoneAPI.GetSensorDetections(_mainVehicleId, _sensorId, pointer(sensorDetections))

def SoApiSetSensorDetectionsUpdateCB(cb):
	global SIMONEAPI_SENSOR_DETECTIONS_CB
	SimoneAPI.SetSensorDetectionsUpdateCB(simoneapi_sensor_detections_cb_func)
	SIMONEAPI_SENSOR_DETECTIONS_CB = cb

def SoGetSensorConfigurations(mainVehicleId, sensorConfigurations):
	SimoneAPI.GetSensorConfigurations.restype = c_bool
	_mainVehicleId = create_string_buffer(mainVehicleId.encode(), 256)
	return SimoneAPI.GetSensorConfigurations(_mainVehicleId, pointer(sensorConfigurations))

def SoGetEnvironment(pEnvironment):
	SimoneAPI.GetEnvironment.restype = c_bool
	return SimoneAPI.GetEnvironment(pointer(pEnvironment))

def SoSetEnvironment(pEnvironment):
	SimoneAPI.SetEnvironment.restype = c_bool
	return SimoneAPI.SetEnvironment(pointer(pEnvironment))

def SoGetTrafficLights(mainVehicleId, opendriveLightId, trafficLight):
	SimoneAPI.GetTrafficLight.restype = c_bool
	_mainVehicleId = create_string_buffer(mainVehicleId.encode(), 256)
	return SimoneAPI.GetTrafficLight(_mainVehicleId, opendriveLightId, pointer(trafficLight))

def SoGetSensorLaneInfo(mainVehicleId, sensorId,pLaneInfo):
	SimoneAPI.GetSensorLaneInfo.restype = c_bool
	_mainVehicleId = create_string_buffer(mainVehicleId.encode(), 256)
	_sensorId = create_string_buffer(sensorId.encode(), 256)
	return SimoneAPI.GetSensorLaneInfo(_mainVehicleId, _sensorId, pointer(pLaneInfo))

def SoApiSetSensorLaneInfoCB(cb):
	global SIMONEAPI_SENSORLANEINFO_CB
	SimoneAPI.SetSensorLaneInfoCB(simoneapi_sensorlaneinfo_cb_func)
	SIMONEAPI_SENSORLANEINFO_CB = cb

