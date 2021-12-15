#include "SimOnePNCAPI_Sample.h"

INITIALIZE_EASYLOGGINGPP

Logging::Logger pncapi_sample::log_scenario_event("scenario_event", "true", "true");
Logging::Logger pncapi_sample::log_v2x_info("v2x_info", "true", "true");

pncapi_sample::pncapi_sample(): log_simone_ini("simone_ini", "true", "true"), log_set_pose_ctl("set_pose_ctl", "true", "true"),
                                                                  log_set_drive_ctl("set_drive_ctl", "true", "true"), log_set_drive_trajectory("set_drive_trajectory", "true", "true"),
                                                                  log_get_sensor_detection("get_sensor_detection", "true", "true")
{}

pncapi_sample::~pncapi_sample(){}

int64_t pncapi_sample::getCurrentTime()
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return tv.tv_sec * 1000 + tv.tv_usec / 1000; // milliseconds
    // return tv.tv_sec * 1e6 + tv.tv_usec; // microseconds
}

void pncapi_sample::set_pose_ctl()
{
    std::unique_ptr<SimOne_Data_Gps> gpsInfo = std::make_unique<SimOne_Data_Gps>();

    /*
      * 获取主车GPS信息
      * input param:
      *     mainVehicleId: Vehilcle index, configure order of web UI, starts from 0
      * output param:
      *     pGps: GPS data(output)
      * return: Success or not
      */
    if (!SimOneAPI::GetGps(0, gpsInfo.get()))
    {
      LOGError(log_set_pose_ctl) << "SimOneAPI::GetGps Failed!";
      return;
    }

    SimOne_Data_Pose_Control pose_ctl;
    pose_ctl.posX = gpsInfo->posX + 1; // Position X on Opendrive (by meter) 基于主车当前位置 X 方向移动一米
    pose_ctl.posY = gpsInfo->posY; // Position Y on Opendrive (by meter)
    pose_ctl.posZ = gpsInfo->posZ; // Position Z on Opendrive (by meter)
    pose_ctl.oriX = gpsInfo->oriX; // Rotation X on Opendrive (by radian)
    pose_ctl.oriY = gpsInfo->oriY; // Rotation Y on Opendrive (by radian)
    pose_ctl.oriZ = gpsInfo->oriZ; // Rotation Z on Opendrive (by radian)
    pose_ctl.autoZ = false; // Automatically set Z according to scene

    LOGInfo(log_set_pose_ctl) << "posX/Y/Z: [" << pose_ctl.posX << ", " << pose_ctl.posY << ", " << pose_ctl.posZ << "]";
    LOGInfo(log_set_pose_ctl) << "oriX/Y/Z: [" << pose_ctl.oriX << ", " << pose_ctl.oriY << ", " << pose_ctl.oriZ << "]";
    LOGInfo(log_set_pose_ctl) << "------------ set_pose_ctl ------------" << gpsInfo->frame;

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
}

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
    posy = posy = sin(theta);
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

void pncapi_sample::get_sensor_detection()
{
    std::unique_ptr<SimOne_Data_SensorConfigurations> pSensorConfigurations = std::make_unique<SimOne_Data_SensorConfigurations>();
    std::unique_ptr<SimOne_Data_SensorDetections> pSensorDetections = std::make_unique<SimOne_Data_SensorDetections>();

    /*
		  * 得到所有传感器的配置信息（Id、类型、频率、位置和朝向等）
      * input param:
			*    mainVehicleId: Vehicle index, configure order of web UI, starts from 0
		  * output param:
		  *     pSensorConfigurations: SensorConfigurations data
		  * return: Success or not
		  */
    if (!SimOneAPI::GetSensorConfigurations("0", pSensorConfigurations.get()))
    {
      LOGError(log_get_sensor_detection) << "SimOneAPI::GetSensorConfigurations Failed!";
      return;
    }

    int num_sensor = pSensorConfigurations->dataSize;
    for (int j = 0; j < num_sensor; j++)
    {
        LOGInfo(log_get_sensor_detection) << "#" << j << " sensorType: " << pSensorConfigurations->data[j].sensorType;
        if ( strcmp(pSensorConfigurations->data[j].sensorType, "sensorFusion") == 0 )
        {
            std::string sensorId = pSensorConfigurations->data[j].sensorId;
            /*
              * 获取传感器检测到物体的对应真值
		          * input param:
              *     mainVehicleId: Vehicle index, configure order of web UI, starts from 0
              *     sensorId: Sensor Index
              * output param:
              *     pGroundtruth: SimOne_Data_SensorDetections data(output)
              * return: Success or not
              */
            if (!SimOneAPI::GetSensorDetections("0", sensorId.c_str(), pSensorDetections.get()))
            {
              LOGError(log_get_sensor_detection) << "SimOneAPI::GetSensorDetections Failed!";
              return;
            }
            if (pSensorDetections->objectSize == 0)
            {
                LOGInfo(log_get_sensor_detection) << "No object detected!";
                return;
            }

            LOGInfo(log_get_sensor_detection) << "timestamp: " << pSensorDetections->timestamp;
            for (int i = 0; i < pSensorDetections->objectSize; ++i)
            {
              LOGInfo(log_get_sensor_detection) << "obstacles[" << i << "].id: " << pSensorDetections->objects[i].id; // Detection Object ID
              switch (pSensorDetections->objects[i].type) // Detection Object Type
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
              LOGInfo(log_get_sensor_detection) << "obstacles[" << i << "].posX: " << pSensorDetections->objects[i].posX; // Detection Object Position X in meter
              LOGInfo(log_get_sensor_detection) << "obstacles[" << i << "].posY: " << pSensorDetections->objects[i].posY; // Detection Object Position Y in meter
              LOGInfo(log_get_sensor_detection) << "obstacles[" << i << "].posZ: " << pSensorDetections->objects[i].posZ; // Detection Object Position Z in meter
              LOGInfo(log_get_sensor_detection) << "obstacles[" << i << "].oriX: " << pSensorDetections->objects[i].oriX; // Rotation X in radian
              LOGInfo(log_get_sensor_detection) << "obstacles[" << i << "].oriY: " << pSensorDetections->objects[i].oriY; // Rotation Y in radian
              LOGInfo(log_get_sensor_detection) << "obstacles[" << i << "].oriZ: " << pSensorDetections->objects[i].oriZ; // Rotation Z in radian
              LOGInfo(log_get_sensor_detection) << "obstacles[" << i << "].length: " << pSensorDetections->objects[i].length; // Detection Object Length in meter
              LOGInfo(log_get_sensor_detection) << "obstacles[" << i << "].width: " << pSensorDetections->objects[i].width; // Detection Object Width in meter
              LOGInfo(log_get_sensor_detection) << "obstacles[" << i << "].height: " << pSensorDetections->objects[i].height; // Detection Object Height in meter
              LOGInfo(log_get_sensor_detection) << "obstacles[" << i << "].range: " << pSensorDetections->objects[i].range; // Detection Object relative range in meter
              LOGInfo(log_get_sensor_detection) << "obstacles[" << i << "].obstacle_vel.x: " << pSensorDetections->objects[i].velX; // Detection Object Velocity X
              LOGInfo(log_get_sensor_detection) << "obstacles[" << i << "].obstacle_vel.y: " << pSensorDetections->objects[i].velY; // Detection Object Velocity Y
              LOGInfo(log_get_sensor_detection) << "obstacles[" << i << "].obstacle_vel.z: " << pSensorDetections->objects[i].velZ; // Detection Object Velocity Z
              LOGInfo(log_get_sensor_detection) << "obstacles[" << i << "].probability: " << pSensorDetections->objects[i].probability; // Detection probability
              LOGInfo(log_get_sensor_detection) << "obstacles[" << i << "].relativePosX: " << pSensorDetections->objects[i].relativePosX; // Relative position X in sensor space
              LOGInfo(log_get_sensor_detection) << "obstacles[" << i << "].relativePosY: " << pSensorDetections->objects[i].relativePosY; // Relative position Y in sensor space
              LOGInfo(log_get_sensor_detection) << "obstacles[" << i << "].relativePosZ: " << pSensorDetections->objects[i].relativePosZ; // Relative position Z in sensor space
              LOGInfo(log_get_sensor_detection) << "obstacles[" << i << "].relativeRotX: " << pSensorDetections->objects[i].relativeRotX; // Relative rotation X in sensor space
              LOGInfo(log_get_sensor_detection) << "obstacles[" << i << "].relativeRotY: " << pSensorDetections->objects[i].relativeRotY; // Relative rotation Y in sensor space
              LOGInfo(log_get_sensor_detection) << "obstacles[" << i << "].relativeRotZ: " << pSensorDetections->objects[i].relativeRotZ; // Relative rotation Z in sensor space
              LOGInfo(log_get_sensor_detection) << "obstacles[" << i << "].relativeVelX: " << pSensorDetections->objects[i].relativeVelX; // Relative velocity X in sensor space
              LOGInfo(log_get_sensor_detection) << "obstacles[" << i << "].relativeVelY: " << pSensorDetections->objects[i].relativeVelY; // Relative velocity Y in sensor space
              LOGInfo(log_get_sensor_detection) << "obstacles[" << i << "].relativeVelZ: " << pSensorDetections->objects[i].relativeVelZ; // Relative velocity Z in sensor space
              LOGInfo(log_get_sensor_detection) << "obstacles[" << i << "].bbox2dMinX: " << pSensorDetections->objects[i].bbox2dMinX; // bbox2d minX in pixel if have
              LOGInfo(log_get_sensor_detection) << "obstacles[" << i << "].bbox2dMinY: " << pSensorDetections->objects[i].bbox2dMinY; // bbox2d minY in pixel if have
              LOGInfo(log_get_sensor_detection) << "obstacles[" << i << "].bbox2dMaxX: " << pSensorDetections->objects[i].bbox2dMaxX; // bbox2d maxX in pixel if have
              LOGInfo(log_get_sensor_detection) << "obstacles[" << i << "].bbox2dMaxY: " << pSensorDetections->objects[i].bbox2dMaxY; // bbox2d maxY in pixel if have
            }
        }
    }
    LOGInfo(log_get_sensor_detection) << "------ get_sensor_detection ------";
}

void pncapi_sample::set_v2x_info(const char* mainVehicleId, const char* sensorId, SimOne_Data_V2XNFS *pDetections) 
{
  LOGInfo(log_v2x_info) << "V2XMsgFrameSize: " << pDetections->V2XMsgFrameSize;
  LOGInfo(log_v2x_info) << "MsgFrameData : " << pDetections->MsgFrameData ;
  LOGInfo(log_v2x_info) <<  "------------ set_v2x_info ------------";
}

void pncapi_sample::pub()
{
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
		* 获得对应车辆编号V2X中的UPER编码之后的v2x消息更新回调
		* input param:
		*     mainVehicleId: Vehicle index, configure order of web UI, starts from 0
    *     sensorId: Sensor Index
		* output param:
		*   pDetections: V2XASN data in SimOne_Data_V2XNFS format(output)
		* return: Success or not
		*/
  if (!SimOneAPI::SetV2XInfoUpdateCB(set_v2x_info))
  {
    LOGError(log_v2x_info) << "SimOneAPI::SetV2XInfoUpdateCB";
  }

  Timer timer_pose_ctl, timer_drive_ctl, timer_drive_trajectory;

  timer_pose_ctl.start(20, std::bind(&pncapi_sample::set_pose_ctl, this));
  timer_drive_ctl.start(20, std::bind(&pncapi_sample::set_drive_ctl, this));
  timer_drive_trajectory.start(1000, std::bind(&pncapi_sample::set_drive_trajectory, this));

  while(true)
  {
    if (SimOneAPI::GetCaseRunStatus() == ESimOne_Case_Status::ESimOne_Case_Status_Stop)
    {
      timer_pose_ctl.stop();
      timer_drive_ctl.stop();
      timer_drive_trajectory.stop();

      SimOneAPI::TerminateSimOneAPI();
      return;
    }
  }
  
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
}

// RegisterSimOneVehicleState
void pncapi_sample::simone_ini()
{
  /*
		* SimOne API主入口
		* input param:
		*	    hostVehicleId: host vehicle ID(from 0 to 9)
		*     isFrameSync: synchronize frame or not
    * return: Success or not
		*/
  if (!SimOneAPI::InitSimOneAPI("0", false, "127.0.0.1"))
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
