from ctypes import *
import os
import platform

global HDMapModule, SimoneAPI, SimoneAPI_dll,SimoneStreamingAPI

sys = platform.system()
if sys == "Windows":
	HDMapModule = "HDMapModule.dll"
	SimoneAPI_dll = "SimOneAPI.dll"
	SimoneStreamingAPI_dll = "SimOneStreamingAPI.dll"

elif sys == "Linux":
	HDMapModule = "libHDMapModule.so"
	SimoneAPI_dll = "libSimOneAPI.so"
	SimoneStreamingAPI_dll = "libSimOneStreamingAPI.so"

SimOneLibPaths = [
"",
"../Build/build/bin/debug/",
"Win64/",
"../Build/build_debug/bin/debug/"
]

# Append full paths
SimOneLibFullPaths = []
SimOneIOStructRoot = os.getcwd()
for path in SimOneLibPaths:
	SimOneLibFullPaths.append(SimOneIOStructRoot+"/"+path)
SimOneLibPaths += SimOneLibFullPaths

LoadDllSuccess = False
for path in SimOneLibPaths:
	if(LoadDllSuccess == True):
		break
	try:# python v3.8+
		CDLL(path+HDMapModule, winmode=DEFAULT_MODE)
		SimoneAPI = CDLL(path + SimoneAPI_dll, winmode=DEFAULT_MODE)
		SimoneStreamingAPI = CDLL(path + SimoneStreamingAPI_dll, winmode=DEFAULT_MODE)
		LoadDllSuccess = True
	except Exception as e:
		pass
		print(e)

	if LoadDllSuccess:
		print("load libary sucessful")
		break
	try:
		# python 2.7 - 3.7
		CDLL(path+HDMapModule)
		SimoneAPI = CDLL(path + SimoneAPI_dll)
		SimoneStreamingAPI = CDLL(path + SimoneStreamingAPI_dll)
		LoadDllSuccess = True
	except Exception as e:
		pass
		print(e)


if LoadDllSuccess == False:
	print("[SimOne API ERROR] Load SimOneAPI.dll/ or.so failed. unable to start")
else:
	print("[SimOne API Load Success] Ready to start")

#print(SimoneStreamingAPI)
MAX_MAINVEHICLE_NUM = 10
MAX_MAINVEHICLE_NAME_LEN = 64
TOTAL_LEN = 640


class SimOne_Data_MainVehicle_Info(Structure):
	_fields_ = [
		('size', c_int),
		('id_list', c_int*MAX_MAINVEHICLE_NUM),
		('type_list', c_char*MAX_MAINVEHICLE_NAME_LEN*MAX_MAINVEHICLE_NUM)]


class SimOne_Data_MainVehicle_Status(Structure):
	_fields_ = [
		('mainVehicleId', c_int),
		('mainVehicleStatus', c_int)]


SOSM_CASENAME_LENGT = 256
SOSM_CASEID_LENGT = 256
SOSM_TASKID_LENGT = 256
SOSM_SESSIONID_LENGT = 256
SOSM_EXTRA_STATES_SIZE_MAX = 256

SimOne_Case_Status_Unknow = 0,
SimOne_Case_Status_Stop = 1,
SimOne_Case_Status_Running = 2
SimOne_Case_Status_Pause = 3

class SimOne_Data_CaseInfo(Structure):
	_pack_ = 1
	_fields_ = [
		('caseName', c_char*SOSM_CASENAME_LENGT),
		('caseId', c_char*SOSM_CASEID_LENGT),
		('taskId', c_char*SOSM_TASKID_LENGT)]


class SimOne_Data(Structure):
	_pack_ = 1
	_fields_ = [
	('timestamp', c_longlong),
	('frame', c_int),
	('version', c_int)]
	

class ESimOne_TrafficLight_Status(c_int):
	ESimOne_TrafficLight_Status_Invalid = 0,
	ESimOne_TrafficLight_Status_Red = 1,
	ESimOne_TrafficLight_Status_Green = 2,
	ESimOne_TrafficLight_Status_Yellow = 3,
	ESimOne_TrafficLight_Status_RedBlink = 4,
	ESimOne_TrafficLight_Status_GreenBlink = 5,
	ESimOne_TrafficLight_Status_YellowBlink = 6,
	ESimOne_TrafficLight_Status_Black = 7

class ESimOne_Data_Vehicle_State(c_int):
    ESimOne_X_L1 = 0,
    ESimOne_Y_L1 = 1,
    ESimOne_Z_L1 = 2,
    ESimOne_X_L2 = 3,
    ESimOne_Y_L2 = 4,
    ESimOne_Z_L2 = 5,
    ESimOne_X_R1 = 6,
    ESimOne_Y_R1 = 7,
    ESimOne_Z_R1 = 8,
    ESimOne_X_R2 = 9,
    ESimOne_Y_R2 = 10,
    ESimOne_Z_R2 = 11

SOSM_TRAFFICLIGHT_SIZE_MAX = 100


class SimOne_Data_TrafficLight(Structure):
	_pack_ = 1
	_fields_ = [
	('index', c_int),
	('opendriveLightId', c_int),
	('countDown', c_int),
	('status', ESimOne_TrafficLight_Status)
	]


class SimOne_Data_TrafficLights(SimOne_Data):
	_pack_ = 1
	_fields_ = [
		('trafficlightNum', c_int),
		('trafficlights', SimOne_Data_TrafficLight*SOSM_TRAFFICLIGHT_SIZE_MAX)]


class ESimOne_Signal_Light(c_int):
	ESimOne_Signal_Light_RightBlinker = 1
	ESimOne_Signal_Light_LeftBlinker = (1 << 1)
	ESimOne_Signal_Light_DoubleFlash = (1 << 2)
	ESimOne_Signal_Light_BrakeLight = (1 << 3)
	ESimOne_Signal_Light_FrontLight = (1 << 4)
	ESimOne_Signal_Light_HighBeam = (1 << 5)
	ESimOne_Signal_Light_BackDrive = (1 << 6)


class SimOne_Data_Signal_Lights(SimOne_Data):
	_pack_ = 1
	_fields_ = [
		('signalLights', c_uint)]


class SimOne_Data_Pose_Control(SimOne_Data):
	_pack_ = 1
	_fields_ = [
	('posX', c_float),
	('posY', c_float),
	('posZ', c_float),
	('oriX', c_float),
	('oriY', c_float),
	('oriZ', c_float),
	('autoZ', c_bool)]


class ESimOne_Gear_Mode(c_int):
	EGearMode_Neutral = 0
	EGearMode_Drive = 1      # forward gear for automatic gear
	EGearMode_Reverse = 2
	EGearMode_Parking = 3

	EGearManualMode_1 = 4    # forward gear 1 for manual gear
	EGearManualMode_2 = 5
	EGearManualMode_3 = 6
	EGearManualMode_4 = 7
	EGearManualMode_5 = 8
	EGearManualMode_6 = 9
	EGearManualMode_7 = 10
	EGearManualMode_8 = 11

class ESimOne_Throttle_Mode(c_int):
    EThrottleMode_Percent = 0         # [0, 1]
    EThrottleMode_Torque = 1          # engine torque, N.m
    EThrottleMode_Speed = 2           # vehicle speed, m/s,   in this mode, brake input is ignored
    EThrottleMode_Accel = 3           # vehicle acceleration, m/s^2, in this mode, brake input is ignored
    EThrottleMode_EngineAV = 4        # engine, rpm
    EThrottleMode_WheelTorque = 5      # torques applied to each wheel, array, size is the wheel number, N.m

class ESimOne_Brake_Mode(c_int):
    EBrakeMode_Percent = 0
    EBrakeMode_MasterCylinderPressure = 1 # degree
    EBrakeMode_PedalForce = 2
    EBrakeMode_WheelCylinderPressure = 3   # Mpa for each wheel
    EBrakeMode_WheelTorque = 4             # Nm for each wheel
#
class ESimOne_Steering_Mode(c_int):
    ESteeringMode_Percent = 0
    ESteeringMode_SteeringWheelAngle = 1
    ESteeringMode_Torque = 2
    ESteeringAngularSpeed = 3            # steering wheel angualr speed, degree/s
    ESteeringWheelAngle = 4              # degree for each wheel
    ESteeringWheelAnglarSpeed = 5        # degree/s for each wheel

class ESimOne_LogLevel_Type(c_int):
	ELogLevel_Debug = 0
	ELogLevel_Information = 1
	ELogLevel_Warning = 2
	ELogLevel_Error = 3
	ELogLevel_Fatal = 4

SOSM_MAX_WHEEL_NUM = 20
class SimOne_Data_Control(SimOne_Data):
	_pack_ = 1
	_fields_ = [
    ('EThrottleMode', ESimOne_Throttle_Mode),
	('throttle', c_float),
	('EBrakeMode', ESimOne_Brake_Mode),
	('brake', c_float),
	('ESteeringMode', ESimOne_Steering_Mode),
	('steering', c_float),
	('handbrake', c_bool),
	('isManualGear', c_bool),
	('gear', ESimOne_Gear_Mode), # 0: Neutral; 1: Drive; 2: Reverse; 3: Parking
    ('clutch', c_float),
    ('throttle_input_data', c_float * SOSM_MAX_WHEEL_NUM),
    ('brake_input_data', c_float * SOSM_MAX_WHEEL_NUM),
    ('steering_input_data', c_float * SOSM_MAX_WHEEL_NUM)]

class SimOne_Data_ESP_Control(SimOne_Data):
	_pack_ = 1
	_fields_ = [
	('stopDistance', c_int),
	('velocityLimit', c_float),
	('steering', c_float),
	('steerTorque', c_float),
	('accel', c_float),
	('accelUpperLimit', c_float),
	('accelLowerLimit', c_float),
	('accelUpperComfLimit', c_float),
	('accelLowerComfLimit', c_float),
	('standStill', c_bool),
	('driveOff', c_bool),
	('brakeMode', c_int),
	('vlcShutdown', c_int),
	('gearMode', c_int)]

class ESimone_Vehicle_EventInfo_Type(c_int):
	ESimOne_VehicleEventInfo_Forward_Collision = 0
	ESimOne_VehicleEventInfo_Backward_Collision = 1
	ESimOne_VehicleEventInfo_Left_Turn = 2
	ESimOne_VehicleEventInfo_Right_Turn = 3
	ESimOne_VehicleEventInfo_Forward_Straight = 4
	ESimOne_VehicleEventInfo_Over_Speed = 5


class SimOne_Data_Vehicle_EventInfo(SimOne_Data):
	_pack_ = 1
	_fields_ = [
	('type', ESimone_Vehicle_EventInfo_Type)]


SOSM_TRAJECTORY_SIZE_MAX = 100


class SimOne_Data_Trajectory_Entry(Structure):
	_pack_ = 1
	_fields_ = [
	('posX', c_float), # Trajectory Position X no Opendrive (by meter)
	('posY', c_float), # Trajectory Position Y no Opendrive (by meter)
	('vel', c_float)]  # Velocity (by meter/second)
	

class SimOne_Data_Trajectory(SimOne_Data):
	_pack_ = 1
	_fields_ = [
	('trajectorySize', c_int),
	('trajectory', SimOne_Data_Trajectory_Entry * SOSM_TRAJECTORY_SIZE_MAX)]
	
class SimOne_Data_Gps(SimOne_Data):
	_pack_ = 1
	_fields_ = [
	('posX', c_float), # Position X no Opendrive (by meter)
	('posY', c_float), # Position Y no Opendrive (by meter)
	('posZ', c_float), # Position Z no Opendrive (by meter)
	('oriX', c_float), # Rotation X no Opendrive (by radian)
	('oriY', c_float), # Rotation Y no Opendrive (by radian)
	('oriZ', c_float), # Rotation Z no Opendrive (by radian)
	('velX', c_float), # MainVehicle Velocity X on Opendrive (by meter)
	('velY', c_float), # MainVehicle Velocity Y on Opendrive (by meter)
	('velZ', c_float), # MainVehicle Velocity Z on Opendrive (by meter)
	('throttle', c_float), #MainVehicle Throttle
	('brake', c_float), #MainVehicle brake
	('steering', c_float), #MainVehicle Steering angle
	('gear', c_int), # MainVehicle gear position
	('accelX', c_float), # MainVehilce Acceleration X on Opendrive (by meter)
	('accelY', c_float), # MainVehilce Acceleration Y on Opendrive (by meter)
	('accelZ', c_float), # MainVehilce Acceleration Z on Opendrive (by meter)
	('angVelX', c_float), # MainVehilce Angular Velocity X on Opendrive (by meter)
	('angVelY', c_float), # MainVehilce Angular Velocity Y on Opendrive (by meter)
	('angVelZ', c_float), # MainVehilce Angular Velocity Z on Opendrive (by meter)
	('wheelSpeedFL', c_float), # Speed of front left wheel (by meter/sec)
	('wheelSpeedFR', c_float), # Speed of front right wheel (by meter/sec)
	('wheelSpeedRL', c_float), # Speed of rear left wheel (by meter/sec)
	('wheelSpeedRR', c_float), # Speed of rear right wheel (by meter/sec)
	('engineRpm', c_float), # Speed of engine (by r/min)
	('odometer', c_float),#  odometer in meter.
	('extraStates', c_float*SOSM_EXTRA_STATES_SIZE_MAX),# vehicle states subscripted by MainVehicleExtraDataIndics message
	('extraStateSize', c_int)]


SOSM_OBSTACLE_SIZE_MAX = 100

class ESimOne_Obstacle_Type(c_int):
	ESimOne_Obstacle_Type_Unknown = 0
	ESimOne_Obstacle_Type_Pedestrian = 4
	ESimOne_Obstacle_Type_Pole = 5
	ESimOne_Obstacle_Type_Car = 6
	ESimOne_Obstacle_Type_Static = 7
	ESimOne_Obstacle_Type_Bicycle = 8
	ESimOne_Obstacle_Type_Fence = 9
	ESimOne_Obstacle_Type_RoadMark = 12
	ESimOne_Obstacle_Type_TrafficSign = 13
	ESimOne_Obstacle_Type_TrafficLight = 15
	ESimOne_Obstacle_Type_Rider = 17
	ESimOne_Obstacle_Type_Truck = 18
	ESimOne_Obstacle_Type_Bus = 19
	ESimOne_Obstacle_Type_SpecialVehicle = 20
	ESimOne_Obstacle_Type_Motorcycle = 21
	ESimOne_Obstacle_Type_Dynamic = 22
	ESimOne_Obstacle_Type_GuardRail = 23
	ESimOne_Obstacle_Type_SpeedLimitSign = 26
	ESimOne_Obstacle_Type_BicycleStatic = 27
	ESimOne_Obstacle_Type_RoadObstacle = 29


class SimOne_Data_Obstacle_Entry(Structure):
	_pack_ = 1
	_fields_ = [
	('id', c_int), # Obstacle ID
	('viewId', c_int),  # Obstacle ID
	('type', ESimOne_Obstacle_Type), # Obstacle Type
	('theta', c_float), # Obstacle vertical rotation (by radian)
	('posX', c_float), # Obstacle Position X no Opendrive (by meter)
	('posY', c_float), # Obstacle Position Y no Opendrive (by meter)
	('posZ', c_float), # Obstacle Position Z no Opendrive (by meter)
	('oriX', c_float), # Obstacle Velocity X no Opendrive (by meter)
	('oriY', c_float), # Obstacle Velocity Y no Opendrive (by meter)
	('oriZ', c_float), # Obstacle Velocity Z no Opendrive (by meter)
	('velX', c_float), # Obstacle Velocity X no Opendrive (by meter)
	('velY', c_float), # Obstacle Velocity Y no Opendrive (by meter)
	('velZ', c_float), # Obstacle Velocity Z no Opendrive (by meter)
	('length', c_float), # Obstacle length
	('width', c_float), # Obstacle width
	('height', c_float)] # Obstacle height


class SimOne_Data_Obstacle(SimOne_Data):
	_pack_ = 1
	_fields_ = [
	('obstacleSize', c_int), # Obstacle Size
	('obstacle', SimOne_Data_Obstacle_Entry*SOSM_OBSTACLE_SIZE_MAX)] # Obstacles

SOSM_IMAGE_WIDTH_MAX = 3840
SOSM_IMAGE_HEIGHT_MAX = 2160
SOSM_IMAGE_DATA_SIZE_MAX = SOSM_IMAGE_WIDTH_MAX*SOSM_IMAGE_HEIGHT_MAX*3


class ESimOne_Node_Type(c_int):
	ESimOneNode_Vehicle = 0
	ESimOneNode_Camera = 1
	ESimOneNode_LiDAR = 2
	ESimOneNode_MMWRadar = 3
	ESimOneNode_UltrasonicRadar = 4
	ESimOneNode_AllUltrasonicRadar = 5
	ESimOneNode_GNSSINS = 6
	ESimOneNode_PerfectPerception = 7
	ESimOneNode_V2X = 8

SENSOR_IDTYPE_MAX = 64
class SimOne_Data_SensorConfiguration(Structure):
	_pack_ = 1
	_fields_ = [
	('id', c_int),
	('mainVehicle', c_int), # 
	('sensorId', c_char * SENSOR_IDTYPE_MAX), # Sensor's ID
	('sensorType', c_char * SENSOR_IDTYPE_MAX),# Sensor's ESimOneNodeType
	('x', c_float),# Obstacle Position X no Opendrive (by meter)
	('y', c_float),# Obstacle Position Y no Opendrive (by meter)
	('z', c_float),# Obstacle Position Z no Opendrive (by meter)
	('roll', c_float),# Sensor's Rotation coordinates in x
	('pitch', c_float),# Sensor's Rotation coordinates in y
	('yaw', c_float),# Sensor's Rotation coordinates in z
	('hz', c_int)]#Sensor's frequency


SOSM_SENSOR_CONFIGURATION_SIZE_MAX = 100


class SimOne_Data_SensorConfigurations(Structure):
	_pack_ = 1
	_fields_ =[
		('dataSize', c_int),  # num of Sensors
		('data', SimOne_Data_SensorConfiguration*SOSM_SENSOR_CONFIGURATION_SIZE_MAX)
	]


class ESimOne_Image_Format(c_int):
	ESimOne_Image_Format_RGB = 0


class SimOne_Data_Image(SimOne_Data):
	_pack_ = 1
	_fields_ = [
	('width', c_int), # Image resolution width 1920 max
	('height', c_int), # Image resolution height 1080 max
	('format', ESimOne_Image_Format), # Image format. 0: RGB
	('imagedataSize', c_int), # Image data size
	('imagedata', c_char * SOSM_IMAGE_DATA_SIZE_MAX)] #1920 x 1080 x 3 max


SOSM_POINT_DATA_SIZE_MAX = 64*57600


class SimOne_Data_Point_Cloud(SimOne_Data):
	_pack_ = 1
	_fields_ = [
	('width', c_int),
	('height', c_int),
	('pointStep', c_int),
	('pointCloudDataSize', c_int),
	('pointClouddata', c_char * SOSM_POINT_DATA_SIZE_MAX)]


SOSM_RADAR_SIZE_MAX = 256


class SimOne_Data_RadarDetection_Entry(Structure):
	_pack_ = 1
	_fields_ = [
	('id', c_int),				# Obstacle ID
	('subId', c_int),			# Obstacle Sub ID
	('type', ESimOne_Obstacle_Type),			# Obstacle Type
	('posX', c_float),			# Obstacle Position X no Opendrive (by meter)
	('posY', c_float),			# Obstacle Position Y no Opendrive (by meter)
	('posZ', c_float),			# Obstacle Position Z no Opendrive (by meter)
	('velX', c_float),			# Obstacle Velocity X no Opendrive (by meter)
	('velY', c_float),			# Obstacle Velocity Y no Opendrive (by meter)
	('velZ', c_float),			# Obstacle Velocity Z no Opendrive (by meter)
	('range', c_float),			# Obstacle relative range in meter 
	('rangeRate', c_float),		# Obstacle relative range rate in m/s
	('azimuth', c_float),		# Obstacle azimuth angle
	('vertical', c_float),		# Obstacle vertical angle
	('snrdb', c_float),			# Signal noise ratio
	('rcsdb', c_float),			# Obstacle RCS
	('probability', c_float)]	# detection probability


class SimOne_Data_RadarDetection(SimOne_Data):
	_pack_ = 1
	_fields_ = [
	('detectNum', c_int), # Obstacle Size
	('detections', SimOne_Data_RadarDetection_Entry*SOSM_RADAR_SIZE_MAX)] # Obstacles

class SimOne_Data_UltrasonicRadarDetection_Entry(Structure):
	_pack_ = 1
	_fields_ = [
		('obstacleRanges', c_float),  # Obstacle from Ultrasonic distance
		('x', c_float),  # Obstacle relativelX
		('y', c_float) # Obstacle relativelY
		]

class SimOne_Data_UltrasonicRadar(SimOne_Data):
	_pack_ = 1
	_fields_ = [
		('sensorId', c_char * SENSOR_IDTYPE_MAX),  # Ultrasonic ID
		('obstacleNum', c_int),  # Ultrasonic detect object nums
		('obstacleDetections', SimOne_Data_UltrasonicRadarDetection_Entry * SOSM_OBSTACLE_SIZE_MAX) # object information
		]


SOSM_ULTRASONICRADAR_SIZE_MAX = 100
class SimOne_Data_UltrasonicRadars(SimOne_Data):
	_pack_ = 1
	_fields_ = [
		('ultrasonicRadarNum', c_int), # ultrasonic radar count
		('ultrasonicRadars', SimOne_Data_UltrasonicRadar * SOSM_ULTRASONICRADAR_SIZE_MAX)]# ultrasonic radars


SOSM_SENSOR_DETECTIONS_OBJECT_SIZE_MAX = 256


class SimOne_Obstacle_Type(c_int):
	ESimOne_Obstacle_Type_Unknown = 0
	ESimOne_Obstacle_Type_Pedestrian = 4
	ESimOne_Obstacle_Type_Pole = 5
	ESimOne_Obstacle_Type_Car = 6
	ESimOne_Obstacle_Type_Static = 7
	ESimOne_Obstacle_Type_Bicycle = 8
	ESimOne_Obstacle_Type_Fence = 9
	ESimOne_Obstacle_Type_RoadMark = 12
	ESimOne_Obstacle_Type_TrafficSign = 13
	ESimOne_Obstacle_Type_TrafficLight = 15
	ESimOne_Obstacle_Type_Rider = 17
	ESimOne_Obstacle_Type_Truck = 18
	ESimOne_Obstacle_Type_Bus = 19
	ESimOne_Obstacle_Type_Train = 20
	ESimOne_Obstacle_Type_Motorcycle = 21
	ESimOne_Obstacle_Type_Dynamic = 22
	ESimOne_Obstacle_Type_GuardRail = 23
	ESimOne_Obstacle_Type_SpeedLimitSign = 26
	ESimOne_Obstacle_Type_BicycleStatic = 27
	ESimOne_Obstacle_Type_RoadObstacle = 29

class SimOne_Data_SensorDetections_Entry(Structure):
	_pack_ = 1
	_fields_ = [
	('id', c_int),			# Detection Object ID
	('type', SimOne_Obstacle_Type),		# Detection Object Type
	('posX', c_float),		# Detection Object Position X in meter
	('posY', c_float),		# Detection Object Position Y in meter
	('posZ', c_float),		# Detection Object Position Z in meter
	('oriX', c_float),		# Rotation X in radian
	('oriY', c_float),		# Rotation Y in radian
	('oriZ', c_float),		# Rotation Z in radian
	('length', c_float),	# Detection Object Length in meter
	('width', c_float),		# Detection Object Width in meter
	('height', c_float),	# Detection Object Height in meter
	('range', c_float),		# Detection Object nearest range in meter
	('velX', c_float),		# Detection Object Velocity X
	('velY', c_float),		# Detection Object Velocity Y
	('velZ', c_float),		# Detection Object Velocity Z
	('probability', c_float),	# Detection probability
	('relativePosX', c_float),	# Relative position X in sensor space
	('relativePosY', c_float),	# Relative position Y in sensor space
	('relativePosZ', c_float),	# Relative position Z in sensor space
	('relativeRotX', c_float),	# Relative rotation X in sensor space
	('relativeRotY', c_float),	# Relative rotation Y in sensor space
	('relativeRotZ', c_float),	# Relative rotation Z in sensor space
	('relativeVelX', c_float),	# Relative velocity X in sensor space
	('relativeVelY', c_float),	# Relative velocity Y in sensor space
	('relativeVelZ', c_float),	# Relative velocity Z in sensor space
	('bbox2dMinX', c_float),	# bbox2d minX in pixel if have
	('bbox2dMinY', c_float),	# bbox2d minY in pixel if have
	('bbox2dMaxX', c_float),	# bbox2d maxX in pixel if have
	('bbox2dMaxY', c_float)]	# bbox2d maxY in pixel if have


class SimOne_Data_Environment(Structure):
	_pack_ = 1
	_fields_ = [
	('timeOfDay', c_float),		# Environment timeOfDay
	('heightAngle', c_float),		# Environment heightAngle
	('directionalLight', c_float),		# Environment directionalLight
	('ambientLight', c_float),		# Environment ambientLight
	('artificialLight', c_float),		# Environment artificialLight
	('cloudDensity', c_float),		# Environment cloudDensity
	('fogDensity', c_float),	# Environment fogDensity
	('rainDensity', c_float),		# Environment rainDensity
	('snowDensity', c_float),	# Environment snowDensity
	('groundHumidityLevel', c_float),	# Environment groundHumidityLevel
	('groundDirtyLevel', c_float)]	# Environment groundDirtyLevel


class SimOne_Data_SensorDetections(SimOne_Data):
	_pack_ = 1
	_fields_ = [
	('objectSize', c_int), # Detection Object Size
	('objects', SimOne_Data_SensorDetections_Entry*SOSM_SENSOR_DETECTIONS_OBJECT_SIZE_MAX)] # Detection Objects 


SOSM_OSI_DATA_SIZE_MAX = 1024*1024*8

SOSM_LANE_NAME_MAX = 128
SOSM_LANE_POINT_MAX = 65535
SOSM_NEAR_LANE_MAX = 128
SOSM_LANE_LINK_MAX = 1024
SOSM_PARKING_SPACE_MAX = 65535
SOSM_PARKING_SPACE_KNOTS_MAX = 4


class ESimOne_Lane_Type(c_int):
	ESimOneLaneType_none = 0
	ESimOneLaneType_driving = 1
	ESimOneLaneType_stop = 2
	ESimOneLaneType_shoulder = 3
	ESimOneLaneType_biking = 4
	ESimOneLaneType_sidewalk = 5
	ESimOneLaneType_border = 6
	ESimOneLaneType_restricted = 7
	ESimOneLaneType_parking = 8
	ESimOneLaneType_bidirectional = 9
	ESimOneLaneType_median = 10
	ESimOneLaneType_special1 = 11
	ESimOneLaneType_special2 = 12
	ESimOneLaneType_special3 = 13
	ESimOneLaneType_roadWorks = 14
	ESimOneLaneType_tram = 15
	ESimOneLaneType_rail = 16
	ESimOneLaneType_entry = 17
	ESimOneLaneType_exit = 18
	ESimOneLaneType_offRamp = 19
	ESimOneLaneType_onRamp = 20
	ESimOneLaneType_mwyEntry = 21
	ESimOneLaneType_mwyExit = 22

class ESimOne_Boundary_Type(c_int):
	BoundaryType_none = 0
	BoundaryType_solid = 1
	BoundaryType_broken = 2
	BoundaryType_solid_solid = 3
	BoundaryType_solid_broken = 4
	BoundaryType_broken_solid = 5
	BoundaryType_broken_broken = 6
	BoundaryType_botts_dots = 7
	BoundaryType_grass = 8
	BoundaryType_curb = 9

class ESimOne_Boundary_Color(c_int):
	BoundaryColor_standard = 0
	BoundaryColor_blue = 1
	BoundaryColor_green = 2
	BoundaryColor_red = 3
	BoundaryColor_white = 4
	BoundaryColor_yellow = 5

class SimOneData_Vec3f(Structure):
	_pack_ = 1
	_fields_ = [
	('x', c_float),
	('y', c_float),
	('z', c_float)]

class SimOne_LineCurve_Parameter(Structure):
	_pack_ = 1
	_fields_ = [
	('C0', c_float),
	('C1', c_float),
	('C2', c_float),
	('C3', c_float),
	('firstPoints', SimOneData_Vec3f),
	('endPoints', SimOneData_Vec3f),
	('length', c_float)]

SOSM_SENSOR_LANE_OBJECT_SIZE_MAX = 256
SOSM_SENSOR_Boundary_OBJECT_SIZE_MAX = 80
class SimOne_Data_LaneLineInfo(Structure):
	_pack_ = 1
	_fields_ = [
	('lineID', c_int),
	('lineType', ESimOne_Boundary_Type),
	('lineColor', ESimOne_Boundary_Color),
	('linewidth', c_float),
	('C3', c_float),
	('linePoints', SimOneData_Vec3f * SOSM_SENSOR_Boundary_OBJECT_SIZE_MAX),
	('linecurveParameter', SimOne_LineCurve_Parameter)]


class SimOne_Data_LaneInfo(SimOne_Data):
	_pack_ = 1
	_fields_ = [
		('id', c_int),
		('laneType', ESimOne_Lane_Type),
		('laneLeftID', c_int),
		('laneRightID', c_int),
		('lanePredecessorID', c_int * SOSM_SENSOR_LANE_OBJECT_SIZE_MAX),
		('laneSuccessorID', c_int * SOSM_SENSOR_LANE_OBJECT_SIZE_MAX),
		('l_Line', SimOne_Data_LaneLineInfo),
		('c_Line', SimOne_Data_LaneLineInfo),
		('r_Line', SimOne_Data_LaneLineInfo),
		('ll_Line', SimOne_Data_LaneLineInfo),
		('rr_Line', SimOne_Data_LaneLineInfo)]


SOSM_WAYPOINTS_SIZE_MAX = 100
class SimOne_Data_WayPoints_Entry(Structure):
	_pack_ = 1
	_fields_ = [
	('index', c_int),
	('posX', c_float), # MainVehicle WayPoints X on Opendrive (by meter)
	('posY', c_float),  # MainVehicle WayPoints Y on Opendrive (by meter)
	('heading_x', c_float),
	('heading_y', c_float),
	('heading_z', c_float),
	('heading_w', c_float),]


class SimOne_Data_WayPoints(SimOne_Data):
	_pack_ = 1
	_fields_ = [
	('wayPointsSize', c_int),	# MainVehicle WayPoints size
	('wayPoints', SimOne_Data_WayPoints_Entry*SOSM_WAYPOINTS_SIZE_MAX)]  # WayPoints, 300 max


class ESimOne_Driver_Status(c_int):
	ESimOne_Driver_Status_Unknown = 0,
	ESimOne_Driver_Status_Running = 1,
	ESimOne_Driver_Status_Finished = 2


class SimOne_Data_Driver_Status(SimOne_Data):
	_pack_ = 1
	_fields_ = [
		('driverStatus', ESimOne_Driver_Status)
	]

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
SensorLaneInfoCbFuncType = CFUNCTYPE(c_void_p,c_int, c_char_p, POINTER(SimOne_Data_LaneInfo))
UltrasonicsCbFuncType = CFUNCTYPE(c_void_p, c_int, POINTER(SimOne_Data_UltrasonicRadars))
ImageCbFuncType = CFUNCTYPE(c_void_p, c_int, c_char_p,POINTER(SimOne_Data_Image))
StreamingImageCbFuncType = CFUNCTYPE(c_void_p,POINTER(SimOne_Data_Image))
PointCloudCbFuncType = CFUNCTYPE(c_void_p, c_int, c_char_p,POINTER(SimOne_Data_Point_Cloud))
StreamingPointCloudCbFuncType = CFUNCTYPE(c_void_p,POINTER(SimOne_Data_Point_Cloud))
RadarDetectionCbFuncType = CFUNCTYPE(c_void_p, c_int, c_char_p,POINTER(SimOne_Data_RadarDetection))
SensorDetectionsCbFuncType = CFUNCTYPE(c_void_p, c_int, c_char_p,POINTER(SimOne_Data_SensorDetections))

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
simoneapi_sensorlaneinfo_cb_func = SensorLaneInfoCbFuncType(_simoneapi_sensorlaneinfo_cb)
simoneapi_ultrasonics_cb_func = UltrasonicsCbFuncType(_simoneapi_ultrasonics_cb)
simoneapi_image_cb_func = ImageCbFuncType(_simoneapi_image_cb)
simoneapi_streamingimage_cb_func = StreamingImageCbFuncType(_simoneapi_streamingimage_cb)
simoneapi_pointcloud_cb_func = PointCloudCbFuncType(_simoneapi_pointcloud_cb)
simoneapi_streamingpointcloud_cb_func = StreamingPointCloudCbFuncType(_simoneapi_streamingpointcloud_cb)
simoneapi_radardetection_cb_func = RadarDetectionCbFuncType(_simoneapi_radardetection_cb)
simoneapi_sensor_detections_cb_func = SensorDetectionsCbFuncType(_simoneapi_sensor_detections_cb)


# 新的API
# 获取版本号
def SoAPIGetVersion():
	SimoneAPI.GetVersion.restype = c_char_p
	return SimoneAPI.GetVersion()

def SoAPISetupLogLevel(level, flag):
	SimoneAPI.SetupLogLevel.restype = c_bool
	return SimoneAPI.SetupLogLevel(level, flag)


def SoAPISetServerInfo(server='127.0.0.1', port=23789):
	_input = create_string_buffer(server.encode(), 256)
	SimoneAPI.SetServerInfo.restype = c_bool
	return SimoneAPI.SetServerInfo(_input, port)


# new
def SoAPIInitSimOneAPI(startcase, stopcase):
	global G_API_StartCase_CB
	global G_API_StopCase_CB
	if startcase == 0:
		startcase = None
	if stopcase == 0:
		stopcase = None
	G_API_StartCase_CB = startcase
	G_API_StopCase_CB = stopcase
	ret = SimoneAPI.InitSimOneAPI(0,0)
	return ret


def SoAPIStopSimOneNode():
	SimoneAPI.StopSimOneNode.restype = c_bool
	ret = SimoneAPI.StopSimOneNode()
	return ret


def SoAPIGetCaseInfo(data):
	SimoneAPI.GetCaseInfo.restype = c_bool
	return SimoneAPI.GetCaseInfo(pointer(data))


def SoAPIGetCaseRunStatus():
	SimoneAPI.GetCaseRunStatus.restype = c_int
	return SimoneAPI.GetCaseRunStatus()


def SoAPIGetMainVehicleList(data):
	SimoneAPI.GetMainVehicleList.restype = c_bool
	return SimoneAPI.GetMainVehicleList(pointer(data), True)

# new
def SoAPIGetMainVehicleStatus(data):
	SimoneAPI.GetMainVehicleStatus.restype = c_bool
	return SimoneAPI.GetMainVehicleStatus(pointer(data))


# new
def SoAPISetMainVehicleStatusCB(cb):
	SimoneAPI.SetMainVehicleStatusCB.restype = c_bool
	global G_API_MainVehicleStatusCB
	if cb == 0:
		cb = None

	G_API_MainVehicleStatusCB = cb
	return SimoneAPI.SetMainVehicleStatusCB(api_mainvehiclestatusupdate_cb)


def SoAPIWait():
	SimoneAPI.Wait.restype = c_int
	return SimoneAPI.Wait()

def SoAPINextFrame(frame):
	SimoneAPI.NextFrame.restype = c_void_p
	return SimoneAPI.NextFrame(frame)

def SoAPIRegisterSimOneVehicleState(data):
    SimoneAPI.RegisterSimOneVehicleState.restype = c_bool
    return SimoneAPI.RegisterSimOneVehicleState(pointer(data), len(data))

def SoAPIGetSimOneVehicleState(data):
	SimoneAPI.GetSimOneVehicleState.restype = c_bool
	return SimoneAPI.GetSimOneVehicleState(pointer(data))

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

def SoAPISetScenarioEventCB(cb):
	if cb == 0:
		cb = None
	global G_API_ScenarioEvent_CB
	G_API_ScenarioEvent_CB = cb
	SimoneAPI.SetScenarioEventCB.restype = c_bool
	ret = SimoneAPI.SetScenarioEventCB(api_scenarioEvent_cb)

	return ret
    
# 新的API到截止到这里


def SoApiStop():
	SimoneAPI.Stop.restype = c_bool
	return SimoneAPI.Stop()


def SoApiSetPose(mainVehicleId, poseControl):
	return SimoneAPI.SetPose(mainVehicleId, pointer(poseControl))


def SoApiSetDrive(mainVehicleId, driveControl):
	return SimoneAPI.SetDrive(mainVehicleId, pointer(driveControl))

def SoSetDriverName(mainVehicleId, driverName):
	_input = create_string_buffer(driverName.encode(), 256)
	SimoneAPI.SetDriverName.restype = c_bool
	return SimoneAPI.SetDriverName(mainVehicleId, _input)

def SoApiSetVehicleEvent(mainVehicleId, vehicleEventInfo):
	return SimoneAPI.SetVehicleEvent(mainVehicleId, pointer(vehicleEventInfo))


def SoGetGps(mainVehicleId, gpsData):
	return SimoneAPI.GetGps(mainVehicleId, pointer(gpsData))


def SoApiSetGpsUpdateCB(cb):
	global SIMONEAPI_GPS_CB
	SimoneAPI.SetGpsUpdateCB(simoneapi_gps_cb_func)
	SIMONEAPI_GPS_CB = cb


def SoGetGroundTruth(mainVehicleId, obstacleData):
	return SimoneAPI.GetGroundTruth(mainVehicleId, pointer(obstacleData))


def SoGetTrafficLights(mainVehicleId, opendriveLightId, trafficLight):
	return SimoneAPI.GetTrafficLight(mainVehicleId, opendriveLightId, pointer(trafficLight))


def SoGetSensorConfigurations(sensorConfigurations):
	return SimoneAPI.GetSensorConfigurations(pointer(sensorConfigurations))


def SoGetUltrasonicRadar(mainVehicleId, sensorId, ultrasonics):
	return SimoneAPI.GetUltrasonicRadar(mainVehicleId, sensorId, pointer(ultrasonics))

def SoGetSensorLaneInfo(mainVehicleId, sensorId,pLaneInfo):
	return SimoneAPI.GetSensorLaneInfo(mainVehicleId, sensorId, pointer(pLaneInfo))

def SoGetUltrasonicRadars(mainVehicleId, ultrasonics):
	return SimoneAPI.GetUltrasonicRadars(mainVehicleId, pointer(ultrasonics))


def SoApiSetGroundTruthUpdateCB(cb):
	global SIMONEAPI_GROUNDTRUTH_CB
	SimoneAPI.SetGroundTruthUpdateCB(simoneapi_groundtruth_cb_func)
	SIMONEAPI_GROUNDTRUTH_CB = cb

def SoApiSetSensorLaneInfoCB(cb):
	global SIMONEAPI_SENSORLANEINFO_CB
	SimoneAPI.SetSensorLaneInfoCB(simoneapi_sensorlaneinfo_cb_func)
	SIMONEAPI_SENSORLANEINFO_CB = cb

def SoApiSetUltrasonicRadarsCB(cb):
	global SIMONEAPI_ULTRASONICS_CB
	SimoneAPI.SetUltrasonicRadarsCB(simoneapi_ultrasonics_cb_func)
	SIMONEAPI_ULTRASONICS_CB = cb


def SoGetImage(mainVehicleId, sensorId, imageData):
	SimoneAPI.GetImage.restype = c_bool
	return SimoneAPI.GetImage(mainVehicleId, sensorId, pointer(imageData))


def SoGetStreamingImage(ip, port, imageData):
	_input = create_string_buffer(ip.encode(), 256)
	SimoneStreamingAPI.GetStreamingImage.restype = c_bool
	return SimoneStreamingAPI.GetStreamingImage(_input, port, pointer(imageData))


def SoApiSetImageUpdateCB(cb):
	global SIMONEAPI_IMAGE_CB
	SimoneAPI.SetImageUpdateCB(simoneapi_image_cb_func)
	SIMONEAPI_IMAGE_CB = cb


def SoApiSetStreamingImageCB(ip, port, cb):
	global SIMONEAPI_StreamingIMAGE_CB
	_input = create_string_buffer(ip.encode(), 256)
	SimoneStreamingAPI.SetStreamingImageCB(_input, port, simoneapi_streamingimage_cb_func)
	SIMONEAPI_StreamingIMAGE_CB = cb


def SoGetPointCloud(mainVehicleId, sensorId, pointCloudData):
	return SimoneAPI.GetPointCloud(mainVehicleId, sensorId, pointer(pointCloudData))


def SoGetStreamingPointCloud(ip, port,infoPort, pointCloudData):
	_input = create_string_buffer(ip.encode(), 256)
	SimoneStreamingAPI.GetStreamingPointCloud.restype = c_bool
	return SimoneStreamingAPI.GetStreamingPointCloud(_input, port, infoPort,pointer(pointCloudData))


def SoApiSetPointCloudUpdateCB(cb):
	global SIMONEAPI_POINTCLOUD_CB
	SimoneAPI.SetPointCloudUpdateCB(simoneapi_pointcloud_cb_func)
	SIMONEAPI_POINTCLOUD_CB = cb


def SoApiSetStreamingPointCloudUpdateCB(ip, port,infoPort, cb):
	global SIMONEAPI_StreamingPOINTCLOUD_CB
	_input = create_string_buffer(ip.encode(), 256)
	SimoneStreamingAPI.SetStreamingPointCloudUpdateCB(_input, port,infoPort, simoneapi_streamingpointcloud_cb_func)
	SIMONEAPI_StreamingPOINTCLOUD_CB = cb


def SoGetRadarDetections(mainVehicleId, sensorId, detectionData):
	return SimoneAPI.GetRadarDetections(mainVehicleId, sensorId, pointer(detectionData))


def SoApiSetRadarDetectionsUpdateCB(cb):
	global SIMONEAPI_RADARDETECTION_CB
	SimoneAPI.SetRadarDetectionsUpdateCB(simoneapi_radardetection_cb_func)
	SIMONEAPI_RADARDETECTION_CB = cb


def SoGetSensorDetections(mainVehicleId, sensorId, sensorDetections):
	return SimoneAPI.GetSensorDetections(mainVehicleId, sensorId, pointer(sensorDetections))


def SoGetEnvironment(pEnvironment):
	return SimoneAPI.GetEnvironment(pointer(pEnvironment))


def SoSetEnvironment(pEnvironment):
	return SimoneAPI.SetEnvironment(pointer(pEnvironment))


def SoSetSignalLights(mainVehicleId, pSignalLight):
	return SimoneAPI.SetSignalLights(mainVehicleId, pointer(pSignalLight))


def SoApiSetSensorDetectionsUpdateCB(cb):
	global SIMONEAPI_SENSOR_DETECTIONS_CB
	SimoneAPI.SetSensorDetectionsUpdateCB(simoneapi_sensor_detections_cb_func)
	SIMONEAPI_SENSOR_DETECTIONS_CB = cb

def SoGetDriverStatus(mainVehicleId, driverStatusData):
	return SimoneAPI.GetDriverStatus(mainVehicleId, pointer(driverStatusData))

def SoGetDriverControl(mainVehicleId, driverControlData):
	return SimoneAPI.GetDriverControl(mainVehicleId, pointer(driverControlData))

def SoGetWayPoints(wayPointsData):
	return SimoneAPI.GetWayPoints(pointer(wayPointsData))

def SoBridgeLogOutput(logLevel,*args):
	print(logLevel)
	list = ""
	for arg in args:
		list+=arg
	logStr = bytes(list,encoding='utf-8')
	return SimoneAPI.SetLogOut(logLevel,logStr)
