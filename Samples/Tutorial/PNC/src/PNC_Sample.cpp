#include "PNC_Sample.h"

INITIALIZE_EASYLOGGINGPP

Logging::Logger pncapi_sample::log_get_gps("get_gps", "true", "true");
Logging::Logger pncapi_sample::log_get_sensor_detection("get_sensor_detection", "true", "true");
Logging::Logger pncapi_sample::log_get_sensor_laneinfo("get_sensor_laneinfo", "true", "true");
Logging::Logger pncapi_sample::log_scenario_event("scenario_event", "true", "true");

SimOne_Data_Gps pncapi_sample::m_gps;
std::atomic<bool> pncapi_sample::m_flip;

pncapi_sample::pncapi_sample() : log_simone_ini("simone_ini", "true", "true"), log_set_pose_ctl("set_pose_ctl", "true", "true"),
                                 log_set_drive_ctl("set_drive_ctl", "true", "true"), log_set_drive_trajectory("set_drive_trajectory", "true", "true")
{
  m_gps.posX = 0.0;
  m_gps.posY = 0.0;
  m_gps.posZ = 0.0;
  m_gps.oriX = 0.0;
  m_gps.oriY = 0.0;
  m_gps.oriZ = 0.0;
  m_flip = false;
}

pncapi_sample::~pncapi_sample(){}

int64_t pncapi_sample::getCurrentTime()
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return tv.tv_sec * 1000 + tv.tv_usec / 1000; // milliseconds
    // return tv.tv_sec * 1e6 + tv.tv_usec; // microseconds
}

// GPS更新回调
void pncapi_sample::get_gps(const char *mainVehicleId, SimOne_Data_Gps *pGps)
{
  if (!m_flip.load())
  {
    m_gps.frame = pGps->frame;
    m_gps.posX = pGps->posX;                 // Position X on Opendrive (by meter)
    m_gps.posY = pGps->posY;                 // Position Y on Opendrive (by meter)
    m_gps.posZ = pGps->posZ;                 // Position Z on Opendrive (by meter)
    m_gps.oriX = pGps->oriX;                 // Rotation X on Opendrive (by radian)
    m_gps.oriY = pGps->oriY;                 // Rotation Y on Opendrive (by radian)
    m_gps.oriZ = pGps->oriZ;                 // Rotation Z on Opendrive (by radian)
    m_gps.velX = pGps->velX;                 // MainVehicle Velocity X on Opendrive (by meter)
    m_gps.velY = pGps->velY;                 // MainVehicle Velocity Y on Opendrive (by meter)
    m_gps.velZ = pGps->velZ;                 // MainVehicle Velocity Z on Opendrive (by meter)
    m_gps.throttle = pGps->throttle;         //MainVehicle throttle
    m_gps.brake = pGps->brake;               //MainVehicle brake;
    m_gps.steering = pGps->steering;         //MainVehicle Wheel Steering angle (deg)
    m_gps.gear = pGps->gear;                 // MainVehicle gear position
    m_gps.accelX = pGps->accelX;             // MainVehicle Acceleration X on Opendrive (by meter)
    m_gps.accelY = pGps->accelY;             // MainVehicle Acceleration Y on Opendrive (by meter)
    m_gps.accelZ = pGps->accelZ;             // MainVehicle Acceleration Z on Opendrive (by meter)
    m_gps.angVelX = pGps->angVelX;           // MainVehicle Angular Velocity X on Opendrive (by meter)
    m_gps.angVelY = pGps->angVelY;           // MainVehicle Angular Velocity Y on Opendrive (by meter)
    m_gps.angVelZ = pGps->angVelZ;           // MainVehicle Angular Velocity Z on Opendrive (by meter)
    m_gps.wheelSpeedFL = pGps->wheelSpeedFL; // Speed of front left wheel (by meter/sec)
    m_gps.wheelSpeedFR = pGps->wheelSpeedFR; // Speed of front right wheel (by meter/sec)
    m_gps.wheelSpeedRL = pGps->wheelSpeedRL; // Speed of rear left wheel (by meter/sec)
    m_gps.wheelSpeedRR = pGps->wheelSpeedRR; // Speed of rear right wheel (by meter/sec)
    m_gps.engineRpm = pGps->engineRpm;       // Speed of engine (by r/min)
    m_gps.odometer = pGps->odometer;         // odometer in meter.
    m_flip.store(true);
  }

  LOGInfo(log_get_gps) << "posX: " << pGps->posX;
  LOGInfo(log_get_gps) << "posY: " << pGps->posY;
  LOGInfo(log_get_gps) << "posZ: " << pGps->posZ;
  LOGInfo(log_get_gps) << "oriX: " << pGps->oriX;
  LOGInfo(log_get_gps) << "oriY: " << pGps->oriY;
  LOGInfo(log_get_gps) << "oriZ: " << pGps->oriZ;
  LOGInfo(log_get_gps) << "velX: " << pGps->velX;
  LOGInfo(log_get_gps) << "velY: " << pGps->velY;
  LOGInfo(log_get_gps) << "velZ: " << pGps->velZ;
  LOGInfo(log_get_gps) << "throttle: " << pGps->throttle;
  LOGInfo(log_get_gps) << "brake: " << pGps->brake;
  LOGInfo(log_get_gps) << "steering: " << pGps->steering;
  LOGInfo(log_get_gps) << "gear: " << pGps->gear;
  LOGInfo(log_get_gps) << "accelX: " << pGps->accelX;
  LOGInfo(log_get_gps) << "accelY: " << pGps->accelY;
  LOGInfo(log_get_gps) << "accelZ: " << pGps->accelZ;
  LOGInfo(log_get_gps) << "angVelX: " << pGps->angVelX;
  LOGInfo(log_get_gps) << "angVelY: " << pGps->angVelY;
  LOGInfo(log_get_gps) << "angVelZ: " << pGps->angVelZ;
  LOGInfo(log_get_gps) << "wheelSpeedFL: " << pGps->wheelSpeedFL;
  LOGInfo(log_get_gps) << "wheelSpeedFR: " << pGps->wheelSpeedFR;
  LOGInfo(log_get_gps) << "wheelSpeedRL: " << pGps->wheelSpeedRL;
  LOGInfo(log_get_gps) << "wheelSpeedRR: " << pGps->wheelSpeedRR;
  LOGInfo(log_get_gps) << "engineRpm: " << pGps->engineRpm;
  LOGInfo(log_get_gps) << "odometer: " << pGps->odometer;
}

// 通过设置位置点移动主车(无动力学)，点位消息由算法端提供
void pncapi_sample::set_pose_ctl()
{
  if (m_flip.load())
  {
    SimOne_Data_Pose_Control pose_ctl;
    // 基于主车当前位置 X 方向移动一米
    pose_ctl.posX = m_gps.posX + cos(m_gps.oriZ); // Position X on Opendrive (by meter)
    pose_ctl.posY = m_gps.posY + sin(m_gps.oriZ); // Position Y on Opendrive (by meter)
    pose_ctl.posZ = m_gps.posZ;                      // Position Z on Opendrive (by meter)
    pose_ctl.oriX = m_gps.oriX;                      // Rotation X on Opendrive (by radian)
    pose_ctl.oriY = m_gps.oriY;                      // Rotation Y on Opendrive (by radian)
    pose_ctl.oriZ = m_gps.oriZ;                      // Rotation Z on Opendrive (by radian)
    pose_ctl.autoZ = false;                          // Automatically set Z according to scene

    LOGInfo(log_set_pose_ctl) << "posX/Y/Z: [" << pose_ctl.posX << ", " << pose_ctl.posY << ", " << pose_ctl.posZ << "]";
    LOGInfo(log_set_pose_ctl) << "oriX/Y/Z: [" << pose_ctl.oriX << ", " << pose_ctl.oriY << ", " << pose_ctl.oriZ << "]";
    LOGInfo(log_set_pose_ctl) << "------------ set_pose_ctl ------------" << m_gps.frame;

    /*
     * 设置主车位置 API
     * input param:
     *     mainVehicleId: Vehilcle index, configure order of web UI, starts from 0
     * output param:
     *     pPose: Pose to set(input)
     * return: Success or not
    */
    if (!SimOneAPI::SetPose(0, &pose_ctl))
    {
      LOGError(log_set_pose_ctl) << "Set Pose failed!";
    }

    m_flip.store(false);
  }

}

// 通过油门、刹车、方向等消息驱动主车(有动力学)，控制参数由算法端提供
void pncapi_sample::set_drive_ctl()
{
    std::unique_ptr<SimOne_Data_Control> pCtrl = std::make_unique<SimOne_Data_Control>();

    pCtrl->timestamp = getCurrentTime(); // uint64  时间戳，单位us
    pCtrl->throttleMode = ESimOne_Throttle_Mode::ESimOne_Throttle_Mode_Speed; // vehicle speed, m/s,   in this mode, brake input is ignored        
    pCtrl->throttle = 10; // m/s  double 油门开度 0-100。100表示最大油门驱动
    // pCtrl->brakeMode = ESimOne_Brake_Mode_Percent;
    // pCtrl->brake = data.brake_pedal_bar(); // double 制度踏板开度 单位bar
    pCtrl->steeringMode = ESimOne_Steering_Mode::ESimOne_Steering_Mode_SteeringWheelAngle; // steering wheel angle, degree
    pCtrl->steering = -40;
    // pCtrl->handbrake = false;
    pCtrl->isManualGear = false;
    pCtrl->gear = ESimOne_Gear_Mode::ESimOne_Gear_Mode_Drive; // forward gear for automatic gear
	  // pCtrl->clutch;

    LOGInfo(log_set_drive_ctl) << "timestamp: " << pCtrl->timestamp;
    LOGInfo(log_set_drive_ctl) << "throttleMode: " << pCtrl->throttleMode;
    LOGInfo(log_set_drive_ctl) << "throttle: " << pCtrl->throttle;
    LOGInfo(log_set_drive_ctl) << "steeringMode: " << pCtrl->steeringMode;
    LOGInfo(log_set_drive_ctl) << "steering: " << pCtrl->steering;
    LOGInfo(log_set_drive_ctl) << "isManualGear: " << pCtrl->isManualGear;
    LOGInfo(log_set_drive_ctl) << "gear: " << pCtrl->gear;
    LOGInfo(log_set_drive_ctl) << "------ set_drive_control_cb ------";

    /*
		 * 主车控制
		 * input param:
		 *     mainVehicleId: Vehilcle index, configure order of web UI, starts from 0
		 *     pControl: vehicle control data
		 * return: Success or not
		*/
    if (!SimOneAPI::SetDrive(0, pCtrl.get()))
    {
        LOGError(log_set_drive_ctl) << "SimOneAPI::SetDrive Failed!";
    }
}

// 通过规划轨迹点驱动主车，不可同时使用SetDrive
void pncapi_sample::set_drive_trajectory()
{
  std::unique_ptr<SimOne_Data_Control_Trajectory> pTraj = std::make_unique<SimOne_Data_Control_Trajectory>();
  std::unique_ptr<SimOne_Data_Gps> gpsInfo = std::make_unique<SimOne_Data_Gps>();
  int n = 10;

  if (!SimOneAPI::GetGps(0, gpsInfo.get()))
  {
    LOGError(log_set_drive_trajectory) << "SimOneAPI::GetGps Failed!";
    return;
  }

  float posx = gpsInfo->posX; // position x
  float posy = gpsInfo->posY; // position y
  float speed = 10; // m/s
  float accel = 1; // accelelation m/s^2
  float theta = gpsInfo->oriZ; // yaw   rad
  float kappa = 0; // curvature
  float relative_time = 0; // time relative to the first trajectory point
  float s = 0; // distance from the first trajectory point

  pTraj->point_num = n;
  for (int i = 0; i < n; i++)
  {
    pTraj->points[i].posx = posx;
    pTraj->points[i].posy = posy;
    pTraj->points[i].speed = speed;
    pTraj->points[i].accel = accel;
    pTraj->points[i].theta = theta;
    pTraj->points[i].kappa = kappa;
    pTraj->points[i].relative_time = relative_time;
    pTraj->points[i].s = s;

    posx = posx + cos(theta);
    posy = posy + sin(theta);
    relative_time += 1;
    s += 1;
  }
  pTraj->isReverse = false;

  /*
		 * 主车控制(通过规划轨迹，不可同时使用SetDrive)
		 * input param:
		 *     mainVehicleId: Vehilcle index, configure order of web UI, starts from 0
		 *     pControlTrajectory: vehicle planning trajectory
		 * return: Success or not
		*/
  if (!SimOneAPI::SetDriveTrajectory("0", pTraj.get()))
  {
    LOGError(log_set_drive_trajectory) << "SimOneAPI::SetDrive Failed!";
  }
}

// 接的场景事件消息的回调
void pncapi_sample::set_scenario_event(const char *mainVehicleId, const char *event, const char *data)
{
  std::string event_info(event);
  std::string car_flw_prefix("CarFollowing");
  if (!event_info.compare(0, car_flw_prefix.length(), car_flw_prefix))
  {
    // car_following
  }
  LOGInfo(log_scenario_event) << "Get Scenario Event: " << event_info.c_str();
}

// 获取传感器检测到物体的对应真值
void pncapi_sample::get_sensor_detection(const char *mainVehicleId, const char *sensorId, SimOne_Data_SensorDetections *pGroundtruth)
  {
    // LOGInfo(log_get_sensor_detection) << "sensorType: " << pSensorConfigurations->data[j].sensorType; "sensorFusion"
    LOGInfo(log_get_sensor_detection) << "sensorId: " << sensorId;

    if (pGroundtruth->objectSize == 0)
    {
      LOGInfo(log_get_sensor_detection) << "No object detected!";
      return;
    }

    LOGInfo(log_get_sensor_detection) << "timestamp: " << pGroundtruth->timestamp;
    for (int i = 0; i < pGroundtruth->objectSize; ++i)
    {
      LOGInfo(log_get_sensor_detection) << "obstacles[" << i << "].id: " << pGroundtruth->objects[i].id; // Detection Object ID
      switch (pGroundtruth->objects[i].type)                                                             // Detection Object Type
      {
      case ESimOne_Obstacle_Type_Unknown:
        LOGInfo(log_get_sensor_detection) << "obstacles[" << i << "].type: ESimOne_Obstacle_Type_Unknown";
        break;
      case ESimOne_Obstacle_Type_Pedestrian:
        LOGInfo(log_get_sensor_detection) << "obstacles[" << i << "].type: ESimOne_Obstacle_Type_Pedestrian";
        break;
      case ESimOne_Obstacle_Type_Car:
        LOGInfo(log_get_sensor_detection) << "obstacles[" << i << "].type: ESimOne_Obstacle_Type_Car";
        break;
      case ESimOne_Obstacle_Type_Bicycle:
        LOGInfo(log_get_sensor_detection) << "obstacles[" << i << "].type: ESimOne_Obstacle_Type_Bicycle";
        break;
      case ESimOne_Obstacle_Type_BicycleStatic:
        LOGInfo(log_get_sensor_detection) << "obstacles[" << i << "].type: ESimOne_Obstacle_Type_BicycleStatic";
        break;
      case ESimOne_Obstacle_Type_Motorcycle:
        LOGInfo(log_get_sensor_detection) << "obstacles[" << i << "].type: ESimOne_Obstacle_Type_Motorcycle";
        break;
      case ESimOne_Obstacle_Type_Truck:
        LOGInfo(log_get_sensor_detection) << "obstacles[" << i << "].type: ESimOne_Obstacle_Type_Truck";
        break;
      case ESimOne_Obstacle_Type_Pole:
        LOGInfo(log_get_sensor_detection) << "obstacles[" << i << "].type: ESimOne_Obstacle_Type_Pole";
        break;
      case ESimOne_Obstacle_Type_Static:
        LOGInfo(log_get_sensor_detection) << "obstacles[" << i << "].type: ESimOne_Obstacle_Type_Static";
        break;
      case ESimOne_Obstacle_Type_Fence:
        LOGInfo(log_get_sensor_detection) << "obstacles[" << i << "].type: ESimOne_Obstacle_Type_Fence";
        break;
      case ESimOne_Obstacle_Type_RoadMark:
        LOGInfo(log_get_sensor_detection) << "obstacles[" << i << "].type: ESimOne_Obstacle_Type_RoadMark";
        break;
      case ESimOne_Obstacle_Type_TrafficSign:
        LOGInfo(log_get_sensor_detection) << "obstacles[" << i << "].type: ESimOne_Obstacle_Type_TrafficSign";
        break;
      case ESimOne_Obstacle_Type_TrafficLight:
        LOGInfo(log_get_sensor_detection) << "obstacles[" << i << "].type: ESimOne_Obstacle_Type_TrafficLight";
        break;
      case ESimOne_Obstacle_Type_Rider:
        LOGInfo(log_get_sensor_detection) << "obstacles[" << i << "].type: ESimOne_Obstacle_Type_Rider";
        break;
      case ESimOne_Obstacle_Type_Bus:
        LOGInfo(log_get_sensor_detection) << "obstacles[" << i << "].type: ESimOne_Obstacle_Type_Bus";
        break;
      case ESimOne_Obstacle_Type_Train:
        LOGInfo(log_get_sensor_detection) << "obstacles[" << i << "].type: ESimOne_Obstacle_Type_Train";
        break;
      case ESimOne_Obstacle_Type_Dynamic:
        LOGInfo(log_get_sensor_detection) << "obstacles[" << i << "].type: ESimOne_Obstacle_Type_Dynamic";
        break;
      case ESimOne_Obstacle_Type_GuardRail:
        LOGInfo(log_get_sensor_detection) << "obstacles[" << i << "].type: ESimOne_Obstacle_Type_GuardRail";
        break;
      case ESimOne_Obstacle_Type_SpeedLimitSign:
        LOGInfo(log_get_sensor_detection) << "obstacles[" << i << "].type: ESimOne_Obstacle_Type_SpeedLimitSign";
        break;
      case ESimOne_Obstacle_Type_RoadObstacle:
        LOGInfo(log_get_sensor_detection) << "obstacles[" << i << "].type: ESimOne_Obstacle_Type_RoadObstacle";
        break;
      default:
        LOGInfo(log_get_sensor_detection) << "obstacles[" << i << "].type: Invalid Object Type";
      }
      LOGInfo(log_get_sensor_detection) << "obstacles[" << i << "].posX: " << pGroundtruth->objects[i].posX;                 // Detection Object Position X in meter
      LOGInfo(log_get_sensor_detection) << "obstacles[" << i << "].posY: " << pGroundtruth->objects[i].posY;                 // Detection Object Position Y in meter
      LOGInfo(log_get_sensor_detection) << "obstacles[" << i << "].posZ: " << pGroundtruth->objects[i].posZ;                 // Detection Object Position Z in meter
      LOGInfo(log_get_sensor_detection) << "obstacles[" << i << "].oriX: " << pGroundtruth->objects[i].oriX;                 // Rotation X in radian
      LOGInfo(log_get_sensor_detection) << "obstacles[" << i << "].oriY: " << pGroundtruth->objects[i].oriY;                 // Rotation Y in radian
      LOGInfo(log_get_sensor_detection) << "obstacles[" << i << "].oriZ: " << pGroundtruth->objects[i].oriZ;                 // Rotation Z in radian
      LOGInfo(log_get_sensor_detection) << "obstacles[" << i << "].length: " << pGroundtruth->objects[i].length;             // Detection Object Length in meter
      LOGInfo(log_get_sensor_detection) << "obstacles[" << i << "].width: " << pGroundtruth->objects[i].width;               // Detection Object Width in meter
      LOGInfo(log_get_sensor_detection) << "obstacles[" << i << "].height: " << pGroundtruth->objects[i].height;             // Detection Object Height in meter
      LOGInfo(log_get_sensor_detection) << "obstacles[" << i << "].range: " << pGroundtruth->objects[i].range;               // Detection Object relative range in meter
      LOGInfo(log_get_sensor_detection) << "obstacles[" << i << "].obstacle_vel.x: " << pGroundtruth->objects[i].velX;       // Detection Object Velocity X
      LOGInfo(log_get_sensor_detection) << "obstacles[" << i << "].obstacle_vel.y: " << pGroundtruth->objects[i].velY;       // Detection Object Velocity Y
      LOGInfo(log_get_sensor_detection) << "obstacles[" << i << "].obstacle_vel.z: " << pGroundtruth->objects[i].velZ;       // Detection Object Velocity Z
      LOGInfo(log_get_sensor_detection) << "obstacles[" << i << "].probability: " << pGroundtruth->objects[i].probability;   // Detection probability
      LOGInfo(log_get_sensor_detection) << "obstacles[" << i << "].relativePosX: " << pGroundtruth->objects[i].relativePosX; // Relative position X in sensor space
      LOGInfo(log_get_sensor_detection) << "obstacles[" << i << "].relativePosY: " << pGroundtruth->objects[i].relativePosY; // Relative position Y in sensor space
      LOGInfo(log_get_sensor_detection) << "obstacles[" << i << "].relativePosZ: " << pGroundtruth->objects[i].relativePosZ; // Relative position Z in sensor space
      LOGInfo(log_get_sensor_detection) << "obstacles[" << i << "].relativeRotX: " << pGroundtruth->objects[i].relativeRotX; // Relative rotation X in sensor space
      LOGInfo(log_get_sensor_detection) << "obstacles[" << i << "].relativeRotY: " << pGroundtruth->objects[i].relativeRotY; // Relative rotation Y in sensor space
      LOGInfo(log_get_sensor_detection) << "obstacles[" << i << "].relativeRotZ: " << pGroundtruth->objects[i].relativeRotZ; // Relative rotation Z in sensor space
      LOGInfo(log_get_sensor_detection) << "obstacles[" << i << "].relativeVelX: " << pGroundtruth->objects[i].relativeVelX; // Relative velocity X in sensor space
      LOGInfo(log_get_sensor_detection) << "obstacles[" << i << "].relativeVelY: " << pGroundtruth->objects[i].relativeVelY; // Relative velocity Y in sensor space
      LOGInfo(log_get_sensor_detection) << "obstacles[" << i << "].relativeVelZ: " << pGroundtruth->objects[i].relativeVelZ; // Relative velocity Z in sensor space
      LOGInfo(log_get_sensor_detection) << "obstacles[" << i << "].bbox2dMinX: " << pGroundtruth->objects[i].bbox2dMinX;     // bbox2d minX in pixel if have
      LOGInfo(log_get_sensor_detection) << "obstacles[" << i << "].bbox2dMinY: " << pGroundtruth->objects[i].bbox2dMinY;     // bbox2d minY in pixel if have
      LOGInfo(log_get_sensor_detection) << "obstacles[" << i << "].bbox2dMaxX: " << pGroundtruth->objects[i].bbox2dMaxX;     // bbox2d maxX in pixel if have
      LOGInfo(log_get_sensor_detection) << "obstacles[" << i << "].bbox2dMaxY: " << pGroundtruth->objects[i].bbox2dMaxY;     // bbox2d maxY in pixel if have
    }
    LOGInfo(log_get_sensor_detection) << "------ get_sensor_detection ------";
  }


// 获取传感器检测到车道与车道线数据回调
void pncapi_sample::get_sensor_laneInfo(const char* mainVehicleId, const char* sensorId, SimOne_Data_LaneInfo *pLaneInfo)
{
  LOGInfo(log_get_sensor_laneinfo) << "id: " << pLaneInfo->id;
  LOGInfo(log_get_sensor_laneinfo) << "laneType: " << pLaneInfo->laneType;
  LOGInfo(log_get_sensor_laneinfo) << "laneLeftID: " << pLaneInfo->laneLeftID;
  LOGInfo(log_get_sensor_laneinfo) << "laneRightID: " << pLaneInfo->laneRightID;
  for (int i=0; i<sizeof(pLaneInfo->lanePredecessorID) / sizeof(pLaneInfo->lanePredecessorID[0]); i++)
  {
    LOGInfo(log_get_sensor_laneinfo) << "lanePredecessorID[" << i << "]: " << pLaneInfo->lanePredecessorID[i];
  }
  for (int i=0; i<sizeof(pLaneInfo->laneSuccessorID) / sizeof(pLaneInfo->laneSuccessorID[0]); i++)
  {
    LOGInfo(log_get_sensor_laneinfo) << "laneSuccessorID[" << i << "]: " << pLaneInfo->laneSuccessorID[i];
  }
  LOGInfo(log_get_sensor_laneinfo) << "l_Line.lineID: " << pLaneInfo->l_Line.lineID;
  LOGInfo(log_get_sensor_laneinfo) << "l_Line.lineType: " << pLaneInfo->l_Line.lineType;
  LOGInfo(log_get_sensor_laneinfo) << "l_Line.lineColor: " << pLaneInfo->l_Line.lineColor;
  LOGInfo(log_get_sensor_laneinfo) << "l_Line.linewidth: " << pLaneInfo->l_Line.linewidth;

  for (int i=0; i<sizeof(pLaneInfo->l_Line.linePoints) / sizeof(pLaneInfo->l_Line.linePoints[0]); i++)
  {
    LOGInfo(log_get_sensor_laneinfo) << "l_Line.linePoints[" << i << "].x: " << pLaneInfo->l_Line.linePoints[i].x;
    LOGInfo(log_get_sensor_laneinfo) << "l_Line.linePoints[" << i << "].y: " << pLaneInfo->l_Line.linePoints[i].y;
    LOGInfo(log_get_sensor_laneinfo) << "l_Line.linePoints[" << i << "].z: " << pLaneInfo->l_Line.linePoints[i].z;
  }
  LOGInfo(log_get_sensor_laneinfo) << "l_Line.linecurveParameter.C0: " << pLaneInfo->l_Line.linecurveParameter.C0;
  LOGInfo(log_get_sensor_laneinfo) << "l_Line.linecurveParameter.C1: " << pLaneInfo->l_Line.linecurveParameter.C1;
  LOGInfo(log_get_sensor_laneinfo) << "l_Line.linecurveParameter.C2: " << pLaneInfo->l_Line.linecurveParameter.C2;
  LOGInfo(log_get_sensor_laneinfo) << "l_Line.linecurveParameter.C3: " << pLaneInfo->l_Line.linecurveParameter.C3;
  LOGInfo(log_get_sensor_laneinfo) << "l_Line.linecurveParameter.firstPoints.x: " << pLaneInfo->l_Line.linecurveParameter.firstPoints.x;
  LOGInfo(log_get_sensor_laneinfo) << "l_Line.linecurveParameter.firstPoints.y: " << pLaneInfo->l_Line.linecurveParameter.firstPoints.y;
  LOGInfo(log_get_sensor_laneinfo) << "l_Line.linecurveParameter.firstPoints.z: " << pLaneInfo->l_Line.linecurveParameter.firstPoints.z;
  LOGInfo(log_get_sensor_laneinfo) << "l_Line.linecurveParameter.endPoints.x: " << pLaneInfo->l_Line.linecurveParameter.endPoints.x;
  LOGInfo(log_get_sensor_laneinfo) << "l_Line.linecurveParameter.endPoints.y: " << pLaneInfo->l_Line.linecurveParameter.endPoints.y;
  LOGInfo(log_get_sensor_laneinfo) << "l_Line.linecurveParameter.endPoints.z: " << pLaneInfo->l_Line.linecurveParameter.endPoints.z;
  LOGInfo(log_get_sensor_laneinfo) << "l_Line.linecurveParameter.length: " << pLaneInfo->l_Line.linecurveParameter.length;

  LOGInfo(log_get_sensor_laneinfo) << "c_Line.lineID: " << pLaneInfo->c_Line.lineID;
  LOGInfo(log_get_sensor_laneinfo) << "c_Line.lineType: " << pLaneInfo->c_Line.lineType;
  LOGInfo(log_get_sensor_laneinfo) << "c_Line.lineColor: " << pLaneInfo->c_Line.lineColor;
  LOGInfo(log_get_sensor_laneinfo) << "c_Line.linewidth: " << pLaneInfo->c_Line.linewidth;
  for (int i=0; i<sizeof(pLaneInfo->c_Line.linePoints) / sizeof(pLaneInfo->c_Line.linePoints[0]); i++)
  {
    LOGInfo(log_get_sensor_laneinfo) << "c_Line.linePoints[" << i << "].x: " << pLaneInfo->c_Line.linePoints[i].x;
    LOGInfo(log_get_sensor_laneinfo) << "c_Line.linePoints[" << i << "].y: " << pLaneInfo->c_Line.linePoints[i].y;
    LOGInfo(log_get_sensor_laneinfo) << "c_Line.linePoints[" << i << "].z: " << pLaneInfo->c_Line.linePoints[i].z;
  }
  LOGInfo(log_get_sensor_laneinfo) << "c_Line.linecurveParameter.C0: " << pLaneInfo->c_Line.linecurveParameter.C0;
  LOGInfo(log_get_sensor_laneinfo) << "c_Line.linecurveParameter.C1: " << pLaneInfo->c_Line.linecurveParameter.C1;
  LOGInfo(log_get_sensor_laneinfo) << "c_Line.linecurveParameter.C2: " << pLaneInfo->c_Line.linecurveParameter.C2;
  LOGInfo(log_get_sensor_laneinfo) << "c_Line.linecurveParameter.C3: " << pLaneInfo->c_Line.linecurveParameter.C3;
  LOGInfo(log_get_sensor_laneinfo) << "c_Line.linecurveParameter.firstPoints.x: " << pLaneInfo->c_Line.linecurveParameter.firstPoints.x;
  LOGInfo(log_get_sensor_laneinfo) << "c_Line.linecurveParameter.firstPoints.y: " << pLaneInfo->c_Line.linecurveParameter.firstPoints.y;
  LOGInfo(log_get_sensor_laneinfo) << "c_Line.linecurveParameter.firstPoints.z: " << pLaneInfo->c_Line.linecurveParameter.firstPoints.z;
  LOGInfo(log_get_sensor_laneinfo) << "c_Line.linecurveParameter.endPoints.x: " << pLaneInfo->c_Line.linecurveParameter.endPoints.x;
  LOGInfo(log_get_sensor_laneinfo) << "c_Line.linecurveParameter.endPoints.y: " << pLaneInfo->c_Line.linecurveParameter.endPoints.y;
  LOGInfo(log_get_sensor_laneinfo) << "c_Line.linecurveParameter.endPoints.z: " << pLaneInfo->c_Line.linecurveParameter.endPoints.z;
  LOGInfo(log_get_sensor_laneinfo) << "c_Line.linecurveParameter.length: " << pLaneInfo->c_Line.linecurveParameter.length;

  LOGInfo(log_get_sensor_laneinfo) << "r_Line.lineID: " << pLaneInfo->r_Line.lineID;
  LOGInfo(log_get_sensor_laneinfo) << "r_Line.lineType: " << pLaneInfo->r_Line.lineType;
  LOGInfo(log_get_sensor_laneinfo) << "r_Line.lineColor: " << pLaneInfo->r_Line.lineColor;
  LOGInfo(log_get_sensor_laneinfo) << "r_Line.linewidth: " << pLaneInfo->r_Line.linewidth;
  for (int i=0; i<sizeof(pLaneInfo->r_Line.linePoints) / sizeof(pLaneInfo->r_Line.linePoints[0]); i++)
  {
    LOGInfo(log_get_sensor_laneinfo) << "r_Line.linePoints[" << i << "].x: " << pLaneInfo->r_Line.linePoints[i].x;
    LOGInfo(log_get_sensor_laneinfo) << "r_Line.linePoints[" << i << "].y: " << pLaneInfo->r_Line.linePoints[i].y;
    LOGInfo(log_get_sensor_laneinfo) << "r_Line.linePoints[" << i << "].z: " << pLaneInfo->r_Line.linePoints[i].z;
  }
  LOGInfo(log_get_sensor_laneinfo) << "r_Line.linecurveParameter.C0: " << pLaneInfo->r_Line.linecurveParameter.C0;
  LOGInfo(log_get_sensor_laneinfo) << "r_Line.linecurveParameter.C1: " << pLaneInfo->r_Line.linecurveParameter.C1;
  LOGInfo(log_get_sensor_laneinfo) << "r_Line.linecurveParameter.C2: " << pLaneInfo->r_Line.linecurveParameter.C2;
  LOGInfo(log_get_sensor_laneinfo) << "r_Line.linecurveParameter.C3: " << pLaneInfo->r_Line.linecurveParameter.C3;
  LOGInfo(log_get_sensor_laneinfo) << "r_Line.linecurveParameter.firstPoints.x: " << pLaneInfo->r_Line.linecurveParameter.firstPoints.x;
  LOGInfo(log_get_sensor_laneinfo) << "r_Line.linecurveParameter.firstPoints.y: " << pLaneInfo->r_Line.linecurveParameter.firstPoints.y;
  LOGInfo(log_get_sensor_laneinfo) << "r_Line.linecurveParameter.firstPoints.z: " << pLaneInfo->r_Line.linecurveParameter.firstPoints.z;
  LOGInfo(log_get_sensor_laneinfo) << "r_Line.linecurveParameter.endPoints.x: " << pLaneInfo->r_Line.linecurveParameter.endPoints.x;
  LOGInfo(log_get_sensor_laneinfo) << "r_Line.linecurveParameter.endPoints.y: " << pLaneInfo->r_Line.linecurveParameter.endPoints.y;
  LOGInfo(log_get_sensor_laneinfo) << "r_Line.linecurveParameter.endPoints.z: " << pLaneInfo->r_Line.linecurveParameter.endPoints.z;
  LOGInfo(log_get_sensor_laneinfo) << "r_Line.linecurveParameter.length: " << pLaneInfo->r_Line.linecurveParameter.length;

  LOGInfo(log_get_sensor_laneinfo) << "ll_Line.lineID: " << pLaneInfo->ll_Line.lineID;
  LOGInfo(log_get_sensor_laneinfo) << "ll_Line.lineType: " << pLaneInfo->ll_Line.lineType;
  LOGInfo(log_get_sensor_laneinfo) << "ll_Line.lineColor: " << pLaneInfo->ll_Line.lineColor;
  LOGInfo(log_get_sensor_laneinfo) << "ll_Line.linewidth: " << pLaneInfo->ll_Line.linewidth;
  for (int i=0; i<sizeof(pLaneInfo->ll_Line.linePoints) / sizeof(pLaneInfo->ll_Line.linePoints[0]); i++)
  {
    LOGInfo(log_get_sensor_laneinfo) << "ll_Line.linePoints[" << i << "].x: " << pLaneInfo->ll_Line.linePoints[i].x;
    LOGInfo(log_get_sensor_laneinfo) << "ll_Line.linePoints[" << i << "].y: " << pLaneInfo->ll_Line.linePoints[i].y;
    LOGInfo(log_get_sensor_laneinfo) << "ll_Line.linePoints[" << i << "].z: " << pLaneInfo->ll_Line.linePoints[i].z;
  }
  LOGInfo(log_get_sensor_laneinfo) << "ll_Line.linecurveParameter.C0: " << pLaneInfo->ll_Line.linecurveParameter.C0;
  LOGInfo(log_get_sensor_laneinfo) << "ll_Line.linecurveParameter.C1: " << pLaneInfo->ll_Line.linecurveParameter.C1;
  LOGInfo(log_get_sensor_laneinfo) << "ll_Line.linecurveParameter.C2: " << pLaneInfo->ll_Line.linecurveParameter.C2;
  LOGInfo(log_get_sensor_laneinfo) << "ll_Line.linecurveParameter.C3: " << pLaneInfo->ll_Line.linecurveParameter.C3;
  LOGInfo(log_get_sensor_laneinfo) << "ll_Line.linecurveParameter.firstPoints.x: " << pLaneInfo->ll_Line.linecurveParameter.firstPoints.x;
  LOGInfo(log_get_sensor_laneinfo) << "ll_Line.linecurveParameter.firstPoints.y: " << pLaneInfo->ll_Line.linecurveParameter.firstPoints.y;
  LOGInfo(log_get_sensor_laneinfo) << "ll_Line.linecurveParameter.firstPoints.z: " << pLaneInfo->ll_Line.linecurveParameter.firstPoints.z;
  LOGInfo(log_get_sensor_laneinfo) << "ll_Line.linecurveParameter.endPoints.x: " << pLaneInfo->ll_Line.linecurveParameter.endPoints.x;
  LOGInfo(log_get_sensor_laneinfo) << "ll_Line.linecurveParameter.endPoints.y: " << pLaneInfo->ll_Line.linecurveParameter.endPoints.y;
  LOGInfo(log_get_sensor_laneinfo) << "ll_Line.linecurveParameter.endPoints.z: " << pLaneInfo->ll_Line.linecurveParameter.endPoints.z;
  LOGInfo(log_get_sensor_laneinfo) << "ll_Line.linecurveParameter.length: " << pLaneInfo->ll_Line.linecurveParameter.length;

  LOGInfo(log_get_sensor_laneinfo) << "rr_Line.lineID: " << pLaneInfo->rr_Line.lineID;
  LOGInfo(log_get_sensor_laneinfo) << "rr_Line.lineType: " << pLaneInfo->rr_Line.lineType;
  LOGInfo(log_get_sensor_laneinfo) << "rr_Line.lineColor: " << pLaneInfo->rr_Line.lineColor;
  LOGInfo(log_get_sensor_laneinfo) << "rr_Line.linewidth: " << pLaneInfo->rr_Line.linewidth;
  for (int i=0; i<sizeof(pLaneInfo->rr_Line.linePoints) / sizeof(pLaneInfo->rr_Line.linePoints[0]); i++)
  {
    LOGInfo(log_get_sensor_laneinfo) << "rr_Line.linePoints[" << i << "].x: " << pLaneInfo->rr_Line.linePoints[i].x;
    LOGInfo(log_get_sensor_laneinfo) << "rr_Line.linePoints[" << i << "].y: " << pLaneInfo->rr_Line.linePoints[i].y;
    LOGInfo(log_get_sensor_laneinfo) << "rr_Line.linePoints[" << i << "].z: " << pLaneInfo->rr_Line.linePoints[i].z;
  }
  LOGInfo(log_get_sensor_laneinfo) << "rr_Line.linecurveParameter.C0: " << pLaneInfo->rr_Line.linecurveParameter.C0;
  LOGInfo(log_get_sensor_laneinfo) << "rr_Line.linecurveParameter.C1: " << pLaneInfo->rr_Line.linecurveParameter.C1;
  LOGInfo(log_get_sensor_laneinfo) << "rr_Line.linecurveParameter.C2: " << pLaneInfo->rr_Line.linecurveParameter.C2;
  LOGInfo(log_get_sensor_laneinfo) << "rr_Line.linecurveParameter.C3: " << pLaneInfo->rr_Line.linecurveParameter.C3;
  LOGInfo(log_get_sensor_laneinfo) << "rr_Line.linecurveParameter.firstPoints.x: " << pLaneInfo->rr_Line.linecurveParameter.firstPoints.x;
  LOGInfo(log_get_sensor_laneinfo) << "rr_Line.linecurveParameter.firstPoints.y: " << pLaneInfo->rr_Line.linecurveParameter.firstPoints.y;
  LOGInfo(log_get_sensor_laneinfo) << "rr_Line.linecurveParameter.firstPoints.z: " << pLaneInfo->rr_Line.linecurveParameter.firstPoints.z;
  LOGInfo(log_get_sensor_laneinfo) << "rr_Line.linecurveParameter.endPoints.x: " << pLaneInfo->rr_Line.linecurveParameter.endPoints.x;
  LOGInfo(log_get_sensor_laneinfo) << "rr_Line.linecurveParameter.endPoints.y: " << pLaneInfo->rr_Line.linecurveParameter.endPoints.y;
  LOGInfo(log_get_sensor_laneinfo) << "rr_Line.linecurveParameter.endPoints.z: " << pLaneInfo->rr_Line.linecurveParameter.endPoints.z;
  LOGInfo(log_get_sensor_laneinfo) << "rr_Line.linecurveParameter.length: " << pLaneInfo->rr_Line.linecurveParameter.length;
}

// 消息发布
void pncapi_sample::pub()
{
  /*
    * 注册主车GPS更新回调
		* input param:
		*     cb: GPS data update callback function
    * return: Success or not
    */
  if (!SimOneAPI::SetGpsUpdateCB(get_gps))
  {
    printf("\033[1m\033[31m[run_pub]: SetSimOneGpsCB Failed!\033[0m\n");
  }

  /*
		* 注册场景事件回调
		* input param:
		*     cb: scenario event callback function
		* output param:
		*     mainVehicleId: Vehilcle index, configure order of web UI, starts from 0
		*     event: Command sent to the mainVehicle
		*     data: Not used yet
		* return: Success or not
		*/
  if (!SimOneAPI::SetScenarioEventCB(set_scenario_event))
  {
    LOGError(log_scenario_event) << "SimOneSM::SetScenarioEventCB Failed!";
  }

  /*
		* 注册传感器真值信息更新回调
		* input param:
		*   cb: Groundtruth data fetch callback function
		* return: Success or not
		*/
  if (!SimOneAPI::SetSensorDetectionsUpdateCB(get_sensor_detection));
  {
    LOGError(log_get_sensor_detection) << "SimOneAPI::SetSensorDetectionsUpdateCB Failed!";
  }

  /*
    * 注册获取传感器检测到车道与车道线数据回调
		* input param[in]
		*   cb: Groundtruth data fetch callback function
		* return: Success or not
		*/
  if (!SimOneAPI::SetSensorLaneInfoCB(get_sensor_laneInfo))
  {
    LOGError(log_get_sensor_laneinfo) << "SimOneAPI::SetSensorLaneInfoCB Failed!";
  }

  Timer timer_pose_ctl, timer_drive_ctl, timer_drive_trajectory;

  timer_pose_ctl.start(100, std::bind(&pncapi_sample::set_pose_ctl, this));
  // timer_drive_ctl.start(100, std::bind(&pncapi_sample::set_drive_ctl, this));
  // timer_drive_trajectory.start(1000, std::bind(&pncapi_sample::set_drive_trajectory, this));

  while(true)
  {
    if (SimOneAPI::GetCaseRunStatus() == ESimOne_Case_Status::ESimOne_Case_Status_Stop)
    {
      timer_pose_ctl.stop();
      // timer_drive_ctl.stop();
      // timer_drive_trajectory.stop();

      SimOneAPI::TerminateSimOneAPI();
      return;
    }
  }
  
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
}

// SimOne 案例初始化
void pncapi_sample::simone_ini()
{
  /*
		* SimOne API主入口
		* input param:
		*	    hostVehicleId: host vehicle ID(from 0 to 9)
		*     isFrameSync: synchronize frame or not
    * return: Success or not
		*/
  if (!SimOneAPI::InitSimOneAPI("0", true, "10.66.9.111"))
  {
    LOGError(log_simone_ini) << "SimOneAPI::InitSimOneAPI Failed!\n";
  }

  SimOne_Data_CaseInfo caseInfo;
  /*
		* 获取案例详情
		* output param:
		*	    pCaseInfo: caseName,caseId,taskId,sessionId
		* return: Success or not
		*/
  if (!SimOneAPI::GetCaseInfo(&caseInfo))
  {
    LOGError(log_simone_ini) << "SimOneAPI::GetCaseInfo Failed!";
  }
}

int main(int argc, char* argv[])
{
  pncapi_sample sp;
  sp.simone_ini();
  sp.pub();

  return 0;
}
