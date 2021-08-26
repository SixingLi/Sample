from ctypes import *
import os
import platform

global HDMapModule, SimoneIOAPI, SimoneIOAPI_dll,SimoneStreamingIOAPI,SimOneStreamingIOAPI_dll

sys = platform.system()
if sys == "Windows":
	HDMapModule = "HDMapModule.dll"
	SimoneIOAPI_dll = "SimOneAPI.dll"
	SimoneStreamingIOAPI_dll = "SimOneStreamingIOAPI.dll"

elif sys == "Linux":
	HDMapModule = "libHDMapModule.so"
	SimoneIOAPI_dll = "libSimOneAPI.so"
	SimoneStreamingIOAPI_dll = "libSimOneStreamingIOAPI.so"


SimOneLibPaths = [
"",
"../SimOneAPISample/lib/Win64/",
"Win64/",
"../SimOneAPISample/lib/Linux64/"
]
try:
	CDLL("HDMapModule.dll")
except Exception as e:
	print(e)
	pass

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
		SimoneIOAPI = CDLL(path + SimoneIOAPI_dll, winmode=DEFAULT_MODE)
		SimoneStreamingIOAPI = CDLL(path + SimoneStreamingIOAPI_dll, winmode=DEFAULT_MODE)
		LoadDllSuccess = True
	except Exception as e:
		print(e)
		pass


	if LoadDllSuccess:
		print("load libary sucessful")
		break
	try:
		# python 2.7 - 3.7
		CDLL(path+HDMapModule)
		SimoneIOAPI = CDLL(path + SimoneIOAPI_dll)
		SimoneStreamingIOAPI = CDLL(path + SimoneStreamingIOAPI_dll)
		LoadDllSuccess = True
	except Exception as e:
		print(e)
		pass


if LoadDllSuccess == False:
	print("[SimOne API ERROR] Load SimOneAPI.dll/ or.so failed. unable to start!")
else:
	print("[SimOne API Success] Load DLL Success!")

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
		('taskId', c_char*SOSM_TASKID_LENGT),
		('sessionId', c_char*SOSM_SESSIONID_LENGT)]


class SimOne_Data(Structure):
	_pack_ = 1
	_fields_ = [
	('timestamp', c_longlong),
	('frame', c_int),
	('version', c_int)]
	

class SimOne_TrafficLight_Status(c_int):
	ESimOne_TrafficLight_Status_Invalid = 0,
	ESimOne_TrafficLight_Status_Red = 1,
	ESimOne_TrafficLight_Status_Green = 2,
	ESimOne_TrafficLight_Status_Yellow = 3,
	ESimOne_TrafficLight_Status_RedBlink = 4,
	ESimOne_TrafficLight_Status_GreenBlink = 5,
	ESimOne_TrafficLight_Status_YellowBlink = 6,
	ESimOne_TrafficLight_Status_Black = 7

class SimOne_Data_Vehicle_State(c_int):
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
	('status', SimOne_TrafficLight_Status)
	]


class SimOne_Data_TrafficLights(SimOne_Data):
	_pack_ = 1
	_fields_ = [
		('trafficlightNum', c_int),
		('trafficlights', SimOne_Data_TrafficLight*SOSM_TRAFFICLIGHT_SIZE_MAX)]


class SimOne_Signal_Light(c_int):
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


class EGearMode(c_int):
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

class EThrottleMode(c_int):
    EThrottleMode_Percent = 0         # [0, 1]
    EThrottleMode_Torque = 1          # engine torque, N.m
    EThrottleMode_Speed = 2           # vehicle speed, m/s,   in this mode, brake input is ignored
    EThrottleMode_Accel = 3           # vehicle acceleration, m/s^2, in this mode, brake input is ignored
    EThrottleMode_EngineAV = 4        # engine, rpm
    EThrottleMode_WheelTorque = 5      # torques applied to each wheel, array, size is the wheel number, N.m

class EBrakeMode(c_int):
    EBrakeMode_Percent = 0
    EBrakeMode_MasterCylinderPressure = 1 # degree
    EBrakeMode_PedalForce = 2
    EBrakeMode_WheelCylinderPressure = 3   # Mpa for each wheel
    EBrakeMode_WheelTorque = 4             # Nm for each wheel
#
class ESteeringMode(c_int):
    ESteeringMode_Percent = 0
    ESteeringMode_SteeringWheelAngle = 1
    ESteeringMode_Torque = 2
    ESteeringAngularSpeed = 3            # steering wheel angualr speed, degree/s
    ESteeringWheelAngle = 4              # degree for each wheel
    ESteeringWheelAnglarSpeed = 5        # degree/s for each wheel

class ELogLevel_Type(c_int):
	ELogLevel_Debug = 0
	ELogLevel_Information = 1
	ELogLevel_Warning = 2
	ELogLevel_Error = 3
	ELogLevel_Fatal = 4

SOSM_MAX_WHEEL_NUM = 20
class SimOne_Data_Control(SimOne_Data):
	_pack_ = 1
	_fields_ = [
    ('EThrottleMode', EThrottleMode),
	('throttle', c_float),
	('EBrakeMode', EBrakeMode),
	('brake', c_float),
	('ESteeringMode', ESteeringMode),
	('steering', c_float),
	('handbrake', c_bool),
	('isManualGear', c_bool),
	('gear', EGearMode), # 0: Neutral; 1: Drive; 2: Reverse; 3: Parking
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
	('type', SimOne_Obstacle_Type), # Obstacle Type
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


class ESimOneNodeType(c_int):
	ESimOneNode_Vehicle = 0
	ESimOneNode_Camera = 1
	ESimOneNode_LiDAR = 2
	ESimOneNode_MMWRadar = 3
	ESimOneNode_UltrasonicRadar = 4
	ESimOneNode_AllUltrasonicRadar = 5
	ESimOneNode_GNSSINS = 6
	ESimOneNode_PerfectPerception = 7
	ESimOneNode_V2X = 8

SENSOR_TYPE_MAX_LENGHT = 64
class SimOneSensorConfiguration(Structure):
	_pack_ = 1
	_fields_ = [
	('index', c_int),
	('mainVehicle', c_int), # 
	('sensorId', c_int), # Sensor's ID
	('sensorType', c_char * SENSOR_TYPE_MAX_LENGHT),# Sensor's ESimOneNodeType
	('x', c_float),# Obstacle Position X no Opendrive (by meter)
	('y', c_float),# Obstacle Position Y no Opendrive (by meter)
	('z', c_float),# Obstacle Position Z no Opendrive (by meter)
	('roll', c_float),# Sensor's Rotation coordinates in x
	('pitch', c_float),# Sensor's Rotation coordinates in y
	('yaw', c_float),# Sensor's Rotation coordinates in z
	('hz', c_int)]#Sensor's frequency


SOSM_SENSOR_CONFIGURATION_SIZE_MAX = 100


class SimOneSensorConfigurations(Structure):
	_pack_ = 1
	_fields_ =[
		('dataSize', c_int),  # num of Sensors
		('data', SimOneSensorConfiguration*SOSM_SENSOR_CONFIGURATION_SIZE_MAX)
	]


class SimOne_Image_Format(c_int):
	ESimOne_Image_Format_RGB = 0


class SimOne_Data_Image(SimOne_Data):
	_pack_ = 1
	_fields_ = [
	('width', c_int), # Image resolution width 1920 max
	('height', c_int), # Image resolution height 1080 max
	('format', SimOne_Image_Format), # Image format. 0: RGB
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
	('type', c_int),			# Obstacle Type
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
	

SOSM_OBSTACL_SIZE_MAX = 100


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
		('id', c_int),  # Ultrasonic ID
		('obstacleNum', c_int),  # Ultrasonic detect object nums
		('obstacledetections', SimOne_Data_UltrasonicRadarDetection_Entry * SOSM_OBSTACL_SIZE_MAX) # object information
		]


SOSM_ULTRASONICRADAR_SIZE_MAX = 100


class SimOne_Data_UltrasonicRadars(SimOne_Data):
	_pack_ = 1
	_fields_ = [
		('UltrasonicRadarsNum', c_int), # Ultrasonic nums
		('detections', SimOne_Data_UltrasonicRadar * SOSM_ULTRASONICRADAR_SIZE_MAX)]# Ultrasonics information


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


class SimOne_Data_OSI(SimOne_Data):
	_pack_ = 1
	_fields_ = [
	('dataSize', c_int),
	('data', c_char * SOSM_OSI_DATA_SIZE_MAX)]


SOSM_LANE_NAME_MAX = 128
SOSM_LANE_POINT_MAX = 65535
SOSM_NEAR_LANE_MAX = 128
SOSM_LANE_LINK_MAX = 1024
SOSM_PARKING_SPACE_MAX = 65535
SOSM_PARKING_SPACE_KNOTS_MAX = 4


class SimOneLaneName(Structure):
	_pack_ = 1
	_fields_ = [
	('nameSize', c_int),
	('name', c_char * SOSM_LANE_NAME_MAX)]


class ESimOneLaneType(c_int):
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

class ESimOneData_BoundaryType(c_int):
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

class ESimOneData_BoundaryColor(c_int):
	BoundaryColor_standard = 0
	BoundaryColor_blue = 1
	BoundaryColor_green = 2
	BoundaryColor_red = 3
	BoundaryColor_white = 4
	BoundaryColor_yellow = 5


class SimOneNearLanes(Structure):
	_pack_ = 1
	_fields_ = [
	('laneNameSize', c_int),
	('laneName', SimOneLaneName *SOSM_NEAR_LANE_MAX )]


class SimOnePoint(Structure):
	_pack_ = 1
	_fields_ = [
	('x', c_double),
	('y', c_double),
	('z', c_double)]

class SimOneData_Vec3f(Structure):
	_pack_ = 1
	_fields_ = [
	('x', c_float),
	('y', c_float),
	('z', c_float)]

class ESimOneData_LineCurveParameter(Structure):
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
	('lineType', ESimOneData_BoundaryType),
	('lineColor', ESimOneData_BoundaryColor),
	('linewidth', c_float),
	('C3', c_float),
	('linePoints', SimOneData_Vec3f * SOSM_SENSOR_Boundary_OBJECT_SIZE_MAX),
	('linecurveParameter', ESimOneData_LineCurveParameter)]


class SimOne_Data_LaneInfo(SimOne_Data):
	_pack_ = 1
	_fields_ = [
		('id', c_int),
		('laneType', ESimOneLaneType),
		('laneLeftID', c_int),
		('laneRightID', c_int),
		('lanePredecessorID', c_int * SOSM_SENSOR_LANE_OBJECT_SIZE_MAX),
		('laneSuccessorID', c_int * SOSM_SENSOR_LANE_OBJECT_SIZE_MAX),
		('l_Line', SimOne_Data_LaneLineInfo),
		('c_Line', SimOne_Data_LaneLineInfo),
		('r_Line', SimOne_Data_LaneLineInfo),
		('ll_Line', SimOne_Data_LaneLineInfo),
		('rr_Line', SimOne_Data_LaneLineInfo)]




class SimOneLaneInfo(Structure):
	_pack_ = 1
	_fields_ = [
	('laneName', SimOneLaneName),
	('leftBoundarySize', c_int),
	('leftBoundary', SimOnePoint *SOSM_LANE_POINT_MAX ),
	('rightBoundarySize', c_int),
	('rightBoundary', SimOnePoint *SOSM_LANE_POINT_MAX ),
	('centerLineSize', c_int),
	('centerLine', SimOnePoint *SOSM_LANE_POINT_MAX )]


class SimOneLaneLink(Structure):
	_pack_ = 1
	_fields_ = [
	('predecessorLaneNameSize', c_int),
	('predecessorLaneName', SimOneLaneName * SOSM_LANE_LINK_MAX),
	('successorLaneNameSize', c_int),
	('successorLaneName', SimOneLaneName * SOSM_LANE_LINK_MAX),
	('leftNeighborLaneName', SimOneLaneName),
	('rightNeighborLaneName', SimOneLaneName)]


class SimOneParkingSpaceIds(Structure):
	_pack_ = 1
	_fields_ = [
	('idSize', c_int),
	('id', SimOneLaneName *SOSM_PARKING_SPACE_MAX )]


class SimOneParkingSpaceKnots(Structure):
	_pack_ = 1
	_fields_ = [
	('knotSize', c_int),
	('knot', SimOnePoint *SOSM_PARKING_SPACE_KNOTS_MAX )]


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


class SimOne_Driver_Status(c_int):
	ESimOne_Driver_Status_Unknown = 0,
	ESimOne_Driver_Status_Running = 1,
	ESimOne_Driver_Status_Finished = 2


class SimOne_Data_Driver_Status(SimOne_Data):
	_pack_ = 1
	_fields_ = [
		('driverStatus', SimOne_Driver_Status)
	]


