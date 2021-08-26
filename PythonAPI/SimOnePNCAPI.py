from SimOneIOStruct import * 
# 新的接口的回调函数
SimOne_StartCaseFuncType = CFUNCTYPE(c_void_p)
SimOne_StopCaseFuncType = CFUNCTYPE(c_void_p)
SimOne_MainVehicleStatusUpdateFuncType = CFUNCTYPE(c_void_p, POINTER(SimOne_Data_MainVehicle_Status))

SimOne_FrameStartFuncType = CFUNCTYPE(c_void_p, c_int)
SimOne_FrameEndFuncType = CFUNCTYPE(c_void_p, c_int)

SimOne_GpsCbFuncType = CFUNCTYPE(c_void_p, POINTER(SimOne_Data_Gps))
SimOne_ObstacleFuncType = CFUNCTYPE(c_void_p, POINTER(SimOne_Data_Obstacle))
SimOne_TrafficLightFuncType = CFUNCTYPE(c_void_p, POINTER(SimOne_Data_TrafficLights))
SimOne_ScenarioEventCBType = CFUNCTYPE(c_void_p, c_int, c_char_p, c_char_p)
# 新的接口的回调函数到这里截止

GpsCbFuncType = CFUNCTYPE(c_void_p, c_int, POINTER(SimOne_Data_Gps))
GroundTruthCbFuncType = CFUNCTYPE(c_void_p, c_int, POINTER(SimOne_Data_Obstacle))
SensorInfoCbFuncType = CFUNCTYPE(c_void_p,c_int, c_int, POINTER(SimOne_Data_LaneInfo))
UltrasonicsCbFuncType = CFUNCTYPE(c_void_p, c_int, POINTER(SimOne_Data_UltrasonicRadars))
ImageCbFuncType = CFUNCTYPE(c_void_p, c_int, c_int,POINTER(SimOne_Data_Image))
StreamingImageCbFuncType = CFUNCTYPE(c_void_p,POINTER(SimOne_Data_Image))
PointCloudCbFuncType = CFUNCTYPE(c_void_p, c_int, c_int,POINTER(SimOne_Data_Point_Cloud))
StreamingPointCloudCbFuncType = CFUNCTYPE(c_void_p,POINTER(SimOne_Data_Point_Cloud))
RadarDetectionCbFuncType = CFUNCTYPE(c_void_p, c_int, c_int,POINTER(SimOne_Data_RadarDetection))
SensorDetectionsCbFuncType = CFUNCTYPE(c_void_p, c_int, c_int,POINTER(SimOne_Data_SensorDetections))
OSIGroundTruthCbFuncType = CFUNCTYPE(c_void_p, c_int,POINTER(SimOne_Data_OSI))
OSISensorDataCbFuncType = CFUNCTYPE(c_void_p, c_int, c_int,POINTER(SimOne_Data_OSI))

G_API_StartCase_CB = None
G_API_StopCase_CB = None
G_API_MainVehicleStatusCB = None
G_API_FrameStart_CB = None
G_API_FrameStop_CB = None
G_API_Gps_CB = None
G_API_Obstacle_CB = None
G_API_TrafficLight_CB = None
G_API_ScenarioEvent_CB = None


SIMONEAPI_GPS_CB = None
SIMONEAPI_GROUNDTRUTH_CB = None
SIMONEAPI_SENSORLANEINFO_CB = None
SIMONEAPI_IMAGE_CB = None
SIMONEAPI_StreamingIMAGE_CB = None
SIMONEAPI_POINTCLOUD_CB = None
SIMONEAPI_RADARDETECTION_CB = None
SIMONEAPI_SENSOR_DETECTIONS_CB = None
SIMONEAPI_OSI_GROUNDTRUTH_CB = None
SIMONEAPI_OSI_SENSORDATA_CB = None
SIMONEAPI_ULTRASONICS_CB = None


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


def _api_gps_cb(data):
	global G_API_Gps_CB
	if G_API_Gps_CB is None:
		return
	G_API_Gps_CB(data)


def _api_obstacle_cb(data):
	global G_API_Obstacle_CB
	if G_API_Obstacle_CB is None:
		return
	G_API_Obstacle_CB(data)


def _api_trafficLight_cb(data):
	global G_API_TrafficLight_CB
	if G_API_TrafficLight_CB is None:
		return
	G_API_TrafficLight_CB(data)
    
def _api_scenarioEvent_cb(mainVehicleId, evt, data):
	global G_API_ScenarioEvent_CB
	if G_API_ScenarioEvent_CB is None:
		return
	G_API_ScenarioEvent_CB(mainVehicleId, evt, data)


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


def _simoneapi_image_cb(mainVehicleId, sensorId, data):
	global SIMONEAPI_IMAGE_CB
	SIMONEAPI_IMAGE_CB(mainVehicleId, sensorId, data)


def _simoneapi_streamingimage_cb(data):
	global SIMONEAPI_StreamingIMAGE_CB
	SIMONEAPI_StreamingIMAGE_CB(data)


def _simoneapi_pointcloud_cb(mainVehicleId, sensorId, data):
	global SIMONEAPI_POINTCLOUD_CB
	SIMONEAPI_POINTCLOUD_CB(mainVehicleId, sensorId, data)


def _simoneapi_streamingpointcloud_cb(data):
	global SIMONEAPI_StreamingPOINTCLOUD_CB
	SIMONEAPI_StreamingPOINTCLOUD_CB( data)


def _simoneapi_radardetection_cb(mainVehicleId, sensorId, data):
	global SIMONEAPI_RADARDETECTION_CB
	SIMONEAPI_RADARDETECTION_CB(mainVehicleId, sensorId, data)


def _simoneapi_sensor_detections_cb(mainVehicleId, sensorId, data):
	global SIMONEAPI_SENSOR_DETECTIONS_CB
	SIMONEAPI_SENSOR_DETECTIONS_CB(mainVehicleId, sensorId, data)


def _simoneapi_osi_groundtruth_cb(mainVehicleId, data):
	global SIMONEAPI_OSI_GROUNDTRUTH_CB
	SIMONEAPI_OSI_GROUNDTRUTH_CB(mainVehicleId, data)


def _simoneapi_osi_sensordata_cb(mainVehicleId,sensorId, data):
	global SIMONEAPI_OSI_SENSORDATA_CB
	SIMONEAPI_OSI_SENSORDATA_CB(mainVehicleId,sensorId, data)


api_startcase_cb = SimOne_StartCaseFuncType(_api_startcase_cb)
api_stopcase_cb = SimOne_StopCaseFuncType(_api_stopcase_cb)
api_mainvehiclestatusupdate_cb = SimOne_MainVehicleStatusUpdateFuncType(_api_mainvehiclestatusupdate_cb)
api_framestart_cb = SimOne_FrameStartFuncType(_api_framestart_cb)
api_framestop_cb = SimOne_FrameEndFuncType(_api_framestop_cb)
api_gps_cb = SimOne_GpsCbFuncType(_api_gps_cb)
api_obstacle_cb = SimOne_ObstacleFuncType(_api_obstacle_cb)
api_trafficLight_cb = SimOne_TrafficLightFuncType(_api_trafficLight_cb)
api_scenarioEvent_cb = SimOne_ScenarioEventCBType(_api_scenarioEvent_cb)

simoneapi_gps_cb_func = GpsCbFuncType(_simoneapi_gps_cb)
simoneapi_groundtruth_cb_func = GroundTruthCbFuncType(_simoneapi_groundtruth_cb)
simoneapi_sensorlaneinfo_cb_func = SensorInfoCbFuncType(_simoneapi_sensorlaneinfo_cb)
simoneapi_ultrasonics_cb_func = UltrasonicsCbFuncType(_simoneapi_ultrasonics_cb)
simoneapi_image_cb_func = ImageCbFuncType(_simoneapi_image_cb)
simoneapi_streamingimage_cb_func = StreamingImageCbFuncType(_simoneapi_streamingimage_cb)
simoneapi_pointcloud_cb_func = PointCloudCbFuncType(_simoneapi_pointcloud_cb)
simoneapi_streamingpointcloud_cb_func = StreamingPointCloudCbFuncType(_simoneapi_streamingpointcloud_cb)
simoneapi_radardetection_cb_func = RadarDetectionCbFuncType(_simoneapi_radardetection_cb)
simoneapi_sensor_detections_cb_func = SensorDetectionsCbFuncType(_simoneapi_sensor_detections_cb)
simoneapi_osi_groundtruth_cb_func = OSIGroundTruthCbFuncType(_simoneapi_osi_groundtruth_cb)
simoneapi_osi_sensordata_cb_func = OSISensorDataCbFuncType(_simoneapi_osi_sensordata_cb)


# 新的API
# 获取版本号
def SoAPIGetVersion():
	SimoneIOAPI.GetVersion.restype = c_char_p
	return SimoneIOAPI.GetVersion()


def SoAPISetupPerformance(isOpen, intervalPacketCount, filePath):
	SimoneIOAPI.SetupPerformance.restype = c_bool
	return SimoneIOAPI.SetupPerformance(isOpen, intervalPacketCount, filePath)


def SoAPISetupLogLevel(level, flag):
	SimoneIOAPI.SetupLogLevel.restype = c_bool
	return SimoneIOAPI.SetupLogLevel(level, flag)


def SoAPISetServerInfo(server='127.0.0.1', port=23789):
	_input = create_string_buffer(server.encode(), 256)
	SimoneIOAPI.SetServerInfo.restype = c_bool
	return SimoneIOAPI.SetServerInfo(_input, port)


# new
def SoAPIStartSimOneNode(startcase, stopcase):
	SimoneIOAPI.StartSimOneNode.restype = c_bool
	global G_API_StartCase_CB
	global G_API_StopCase_CB
	if startcase == 0:
		startcase = None
	if stopcase == 0:
		stopcase = None
	G_API_StartCase_CB = startcase
	G_API_StopCase_CB = stopcase
	ret = SimoneIOAPI.StartSimOneNode(api_startcase_cb, api_stopcase_cb)
	return ret


def SoAPIStopSimOneNode():
	SimoneIOAPI.StopSimOneNode.restype = c_bool
	ret = SimoneIOAPI.StopSimOneNode()
	return ret


def SoAPISimOneNodeReady():
	SimoneIOAPI.SimOneNodeReady.restype = c_bool
	ret = SimoneIOAPI.SimOneNodeReady()
	return ret


def SoAPIGetCaseInfo(data):
	SimoneIOAPI.GetCaseInfo.restype = c_bool
	return SimoneIOAPI.GetCaseInfo(pointer(data))


def SoAPIGetCaseRunStatus():
	SimoneIOAPI.GetCaseRunStatus.restype = c_int
	return SimoneIOAPI.GetCaseRunStatus()


def SoAPIGetMainVehicleList(data):
	SimoneIOAPI.GetMainVehicleList.restype = c_bool
	return SimoneIOAPI.GetMainVehicleList(pointer(data), True)


# new
def SoAPISubMainVehicle(mainVehicleId, isJoinTimeLoop):
	SimoneIOAPI.SubMainVehicle.restype = c_bool
	ret = SimoneIOAPI.SubMainVehicle(mainVehicleId, isJoinTimeLoop)
	return ret


# new
def SoAPIGetMainVehicleStatus(data):
	SimoneIOAPI.GetMainVehicleStatus.restype = c_bool
	return SimoneIOAPI.GetMainVehicleStatus(pointer(data))


# new
def SoAPISetMainVehicleStatusCB(cb):
	SimoneIOAPI.SetMainVehicleStatusCB.restype = c_bool
	global G_API_MainVehicleStatusCB
	if cb == 0:
		cb = None

	G_API_MainVehicleStatusCB = cb
	return SimoneIOAPI.SetMainVehicleStatusCB(api_mainvehiclestatusupdate_cb)


def SoAPIWait():
	SimoneIOAPI.Wait.restype = c_int
	return SimoneIOAPI.Wait()


def SoAPINextFrame(frame):
	SimoneIOAPI.NextFrame.restype = c_void_p
	return SimoneIOAPI.NextFrame(frame)


def SoAPIGetSimOneGps(data):
	SimoneIOAPI.GetSimOneGps.restype = c_bool
	return SimoneIOAPI.GetSimOneGps(pointer(data))

def SoAPIRegisterSimOneVehicleState(data):
    SimoneIOAPI.RegisterSimOneVehicleState.restype = c_bool
    return SimoneIOAPI.RegisterSimOneVehicleState(pointer(data), len(data))

def SoAPIGetSimOneVehicleState(data):
	SimoneIOAPI.GetSimOneVehicleState.restype = c_bool
	return SimoneIOAPI.GetSimOneVehicleState(pointer(data))

def SoAPIGetSimOneGroundTruth(data):
	SimoneIOAPI.GetSimOneGroundTruth.restype = c_bool
	return SimoneIOAPI.GetSimOneGroundTruth(pointer(data))


def SoAPISetFrameCB(startcb, stopcb):
	SimoneIOAPI.SetFrameCB.restype = c_bool
	global G_API_FrameStart_CB
	global G_API_FrameStop_CB
	if startcb == 0:
		startcb = None
	if stopcb == 0:
		stopcb = None

	G_API_FrameStart_CB = startcb
	G_API_FrameStop_CB = stopcb

	ret = SimoneIOAPI.SetFrameCB(api_framestart_cb, api_framestop_cb)
	return ret


def SoAPISetSimOneGpsCB(cb):
	if cb == 0:
		cb = None
	global G_API_Gps_CB

	G_API_Gps_CB = cb
	SimoneIOAPI.SetSimOneGpsCB.restype = c_bool
	ret = SimoneIOAPI.SetSimOneGpsCB(api_gps_cb)

	return ret

# new
def SoAPISetSimOneGroundTruthCB(cb):
	if cb == 0:
		cb = None
	global G_API_Obstacle_CB
	G_API_Obstacle_CB = cb
	SimoneIOAPI.SetSimOneGroundTruthCB.restype = c_bool
	ret = SimoneIOAPI.SetSimOneGroundTruthCB(api_obstacle_cb)

	return ret


def SoAPISetSensorPhysicalbasedDataEnable(enable):
	SimoneIOAPI.SetSensorPhysicalbasedDataEnable.restype = c_void_p
	return SimoneIOAPI.SetSensorPhysicalbasedDataEnable(enable)


def SoAPISetSensorObjectbasedDataEnable(enable):
	SimoneIOAPI.SetSensorObjectbasedDataEnable.restype = c_void_p
	return SimoneIOAPI.SetSensorObjectbasedDataEnable(enable)


def SoAPISetSensorOSIDataEnable(enable):
	SimoneIOAPI.SetSensorOSIDataEnable.restype = c_void_p
	return SimoneIOAPI.SetSensorOSIDataEnable(enable)
    
# new
def SoAPISetScenarioEventCB(cb):
	if cb == 0:
		cb = None
	global G_API_ScenarioEvent_CB
	G_API_ScenarioEvent_CB = cb
	SimoneIOAPI.SetScenarioEventCB.restype = c_bool
	ret = SimoneIOAPI.SetScenarioEventCB(api_scenarioEvent_cb)

	return ret
    
# 新的API到截止到这里


def SoApiStart():
	SimoneIOAPI.Start.restype = c_bool
	return SimoneIOAPI.Start()


def SoOSIStart():
	SimoneIOAPI.OSIStart.restype = c_bool
	return SimoneIOAPI.OSIStart()


def SoApiStop():
	SimoneIOAPI.Stop.restype = c_bool
	return SimoneIOAPI.Stop()


def SoApiSetPose(mainVehicleId, poseControl):
	return SimoneIOAPI.SetPose(mainVehicleId, pointer(poseControl))


def SoApiSetDrive(mainVehicleId, driveControl):
	return SimoneIOAPI.SetDrive(mainVehicleId, pointer(driveControl))

def SoSetDriverName(mainVehicleId, driverName):
	_input = create_string_buffer(driverName.encode(), 256)
	SimoneIOAPI.SetDriverName.restype = c_bool
	return SimoneIOAPI.SetDriverName(mainVehicleId, _input)

def SoApiSetVehicleEvent(mainVehicleId, vehicleEventInfo):
	return SimoneIOAPI.SetVehicleEvent(mainVehicleId, pointer(vehicleEventInfo))


def SoGetGps(mainVehicleId, gpsData):
	return SimoneIOAPI.GetGps(mainVehicleId, pointer(gpsData))


def SoApiSetGpsUpdateCB(cb):
	global SIMONEAPI_GPS_CB
	SimoneIOAPI.SetGpsUpdateCB(simoneapi_gps_cb_func)
	SIMONEAPI_GPS_CB = cb


def SoGetGroundTruth(mainVehicleId, obstacleData):
	return SimoneIOAPI.GetGroundTruth(mainVehicleId, pointer(obstacleData))


def SoGetTrafficLights(mainVehicleId, opendriveLightId, trafficLight):
	return SimoneIOAPI.GetTrafficLight(mainVehicleId, opendriveLightId, pointer(trafficLight))


def SoGetSensorConfigurations(sensorConfigurations):
	return SimoneIOAPI.GetSensorConfigurations(pointer(sensorConfigurations))


def SoGetUltrasonicRadar(mainVehicleId, sensorId, ultrasonics):
	return SimoneIOAPI.GetUltrasonicRadar(mainVehicleId, sensorId, pointer(ultrasonics))

def SoGetSensorLaneInfo(mainVehicleId, sensorId,pLaneInfo):
	return SimoneIOAPI.GetSensorLaneInfo(mainVehicleId, sensorId, pointer(pLaneInfo))

def SoGetUltrasonicRadars(mainVehicleId, ultrasonics):
	return SimoneIOAPI.GetUltrasonicRadars(mainVehicleId, pointer(ultrasonics))


def SoApiSetGroundTruthUpdateCB(cb):
	global SIMONEAPI_GROUNDTRUTH_CB
	SimoneIOAPI.SetGroundTruthUpdateCB(simoneapi_groundtruth_cb_func)
	SIMONEAPI_GROUNDTRUTH_CB = cb

def SoApiSetSensorLaneInfoCB(cb):
	global SIMONEAPI_SENSORLANEINFO_CB
	SimoneIOAPI.SetSensorLaneInfoCB(simoneapi_sensorlaneinfo_cb_func)
	SIMONEAPI_SENSORLANEINFO_CB = cb

def SoApiSetUltrasonicRadarsCB(cb):
	global SIMONEAPI_ULTRASONICS_CB
	SimoneIOAPI.SetUltrasonicRadarsCB(simoneapi_ultrasonics_cb_func)
	SIMONEAPI_ULTRASONICS_CB = cb


def SoGetImage(mainVehicleId, sensorId, imageData):
	SimoneIOAPI.GetImage.restype = c_bool
	return SimoneIOAPI.GetImage(mainVehicleId, sensorId, pointer(imageData))


def SoGetStreamingImage(ip, port, imageData):
	_input = create_string_buffer(ip.encode(), 256)
	SimoneStreamingIOAPI.GetStreamingImage.restype = c_bool
	return SimoneStreamingIOAPI.GetStreamingImage(_input, port, pointer(imageData))


def SoApiSetImageUpdateCB(cb):
	global SIMONEAPI_IMAGE_CB
	SimoneIOAPI.SetImageUpdateCB(simoneapi_image_cb_func)
	SIMONEAPI_IMAGE_CB = cb


def SoApiSetStreamingImageCB(ip, port, cb):
	global SIMONEAPI_StreamingIMAGE_CB
	_input = create_string_buffer(ip.encode(), 256)
	SimoneStreamingIOAPI.SetStreamingImageCB(_input, port, simoneapi_streamingimage_cb_func)
	SIMONEAPI_StreamingIMAGE_CB = cb


def SoGetPointCloud(mainVehicleId, sensorId, pointCloudData):
	return SimoneIOAPI.GetPointCloud(mainVehicleId, sensorId, pointer(pointCloudData))


def SoGetStreamingPointCloud(ip, port,infoPort, pointCloudData):
	_input = create_string_buffer(ip.encode(), 256)
	SimoneStreamingIOAPI.GetStreamingPointCloud.restype = c_bool
	return SimoneStreamingIOAPI.GetStreamingPointCloud(_input, port, infoPort,pointer(pointCloudData))


def SoApiSetPointCloudUpdateCB(cb):
	global SIMONEAPI_POINTCLOUD_CB
	SimoneIOAPI.SetPointCloudUpdateCB(simoneapi_pointcloud_cb_func)
	SIMONEAPI_POINTCLOUD_CB = cb


def SoApiSetStreamingPointCloudUpdateCB(ip, port,infoPort, cb):
	global SIMONEAPI_StreamingPOINTCLOUD_CB
	_input = create_string_buffer(ip.encode(), 256)
	SimoneStreamingIOAPI.SetStreamingPointCloudUpdateCB(_input, port,infoPort, simoneapi_streamingpointcloud_cb_func)
	SIMONEAPI_StreamingPOINTCLOUD_CB = cb


def SoGetRadarDetections(mainVehicleId, sensorId, detectionData):
	return SimoneIOAPI.GetRadarDetections(mainVehicleId, sensorId, pointer(detectionData))


def SoApiSetRadarDetectionsUpdateCB(cb):
	global SIMONEAPI_RADARDETECTION_CB
	SimoneIOAPI.SetRadarDetectionsUpdateCB(simoneapi_radardetection_cb_func)
	SIMONEAPI_RADARDETECTION_CB = cb


def SoOSIGetSensorData(mainVehicleId,sensorId, pSensorData):
	return SimoneIOAPI.OSIGetSensorData(mainVehicleId, sensorId, pointer(pSensorData))


def SoGetSensorDetections(mainVehicleId, sensorId, sensorDetections):
	return SimoneIOAPI.GetSensorDetections(mainVehicleId, sensorId, pointer(sensorDetections))


def SoGetEnvironment(pEnvironment):
	return SimoneIOAPI.GetEnvironment(pointer(pEnvironment))


def SoSetEnvironment(pEnvironment):
	return SimoneIOAPI.SetEnvironment(pointer(pEnvironment))


def SoSetSignalLights(mainVehicleId, pSignalLight):
	return SimoneIOAPI.SetSignalLights(mainVehicleId, pointer(pSignalLight))


def SoApiSetSensorDetectionsUpdateCB(cb):
	global SIMONEAPI_SENSOR_DETECTIONS_CB
	SimoneIOAPI.SetSensorDetectionsUpdateCB(simoneapi_sensor_detections_cb_func)
	SIMONEAPI_SENSOR_DETECTIONS_CB = cb


def SoOSIGetGroundTruth(mainVehicleId, groundtruthData):
	return SimoneIOAPI.OSIGetGroundTruth(mainVehicleId, pointer(groundtruthData))


def SoApiOSISetGroundTruthUpdateCB(cb):
	global SIMONEAPI_OSI_GROUNDTRUTH_CB
	SimoneIOAPI.OSISetGroundTruthUpdateCB(simoneapi_osi_groundtruth_cb_func)
	SIMONEAPI_OSI_GROUNDTRUTH_CB = cb


def SoApiOSISetSensorDataUpdateCB(cb):
	global SIMONEAPI_OSI_SENSORDATA_CB
	SimoneIOAPI.OSISetSensorDataUpdateCB(simoneapi_osi_sensordata_cb_func)
	SIMONEAPI_OSI_SENSORDATA_CB = cb


def SoGetDriverStatus(mainVehicleId, driverStatusData):
	return SimoneIOAPI.GetDriverStatus(mainVehicleId, pointer(driverStatusData))

def SoGetDriverControl(mainVehicleId, driverControlData):
	return SimoneIOAPI.GetDriverControl(mainVehicleId, pointer(driverControlData))

def SoGetWayPoints(wayPointsData):
	return SimoneIOAPI.GetWayPoints(pointer(wayPointsData))

def SoBridgeLogOutput(logLevel,*args):
	print(logLevel)
	list = ""
	for arg in args:
		list+=arg
	logStr = bytes(list,encoding='utf-8')
	return SimoneIOAPI.bridgeLogOutput(logLevel,logStr)
