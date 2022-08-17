#include "trans_node_cyber.hpp"

bool cyber_trans_node::mSigTerm = false;
std::ofstream cyber_trans_node::log_img;
std::string cyber_trans_node::bio_ip;
std::string cyber_trans_node::vehicle_id;
bool cyber_trans_node::enable_physical_sensor;
PCD_Param_T cyber_trans_node::mPCDParam;
Img_Param_T cyber_trans_node::mImgParam;
std::string cyber_trans_node::mPCDChannel;

cyber_trans_node::cyber_trans_node():
  mPointCloud(mPCDChannel)
{
  exit_ini();
  mSigTerm = false;
}
cyber_trans_node::~cyber_trans_node(){}

int64_t cyber_trans_node::getCurrentTime()
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return tv.tv_sec * 1000 + tv.tv_usec / 1000; // milliseconds
    // return tv.tv_sec * 1e6 + tv.tv_usec; // microseconds
}

void cyber_trans_node::run_pub()
{
  Timer timer_chassis_imu, timer_conti_radar, timer_gps, timer_obstacles, timer_routing_req, timer_traffic_light;

  timer_chassis_imu.start(16, std::bind(&CW_Chassis_Imu::task_chassis_imu, &mChassisImu));
  timer_conti_radar.start(16, std::bind(&CW_ContiRadar::task_conti_radar, &mContiRadar));
  timer_gps.start(16, std::bind(&CW_Gps::task_gps, &mGps));
  timer_obstacles.start(16, std::bind(&CW_Obstacles::task_obstacles, &mObstacles));
  timer_routing_req.start(32, std::bind(&CW_RoutingReq::task_routing_req, &mRoutingReq));
  timer_traffic_light.start(16, std::bind(&CW_TrafficLight::task_traffic_light, &mTrafficLight));

  if (enable_physical_sensor)
  {
    mPointCloud.task_point_cloud();
    mImage.task_image();
  }

  while (true)
  {
    if (SimOneAPI::GetCaseRunStatus() == ESimOne_Case_Status::ESimOne_Case_Status_Stop || mSigTerm)
    {
      timer_chassis_imu.stop();
      timer_conti_radar.stop();
      timer_gps.stop();
      timer_obstacles.stop();
      timer_routing_req.stop();
      timer_traffic_light.stop();

      std::cout << "Terminate SimOne API ..." << std::endl;
      if (!SimOneAPI::TerminateSimOneAPI())
      {
        std::cout << "SimOneAPI TerminateSimOneAPI Failed!" << std::endl;
      }
      sleep(3);
      break;
    }
  }
}

void cyber_trans_node::run_rcv()
{
  while (true)
  {
    mCtl.task_ctl();
    std::this_thread::sleep_for(std::chrono::milliseconds(16));

    if (SimOneAPI::GetCaseRunStatus() == ESimOne_Case_Status::ESimOne_Case_Status_Stop || mSigTerm)
    {
      break;
    }
  }
}

void cyber_trans_node::monitor_case_status()
{
  while (true)
  {
    if(SimOneAPI::GetCaseRunStatus() ==ESimOne_Case_Status::ESimOne_Case_Status_Stop)
    {
      std::cout << "Terminate SimOne API ..." << std::endl;
      if (!SimOneAPI::TerminateSimOneAPI())
      {
        std::cout << "SimOneAPI TerminateSimOneAPI Failed!" << std::endl;
      }
      sleep(3);
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(3000));
  }
}

void cyber_trans_node::exit_handler(int signo)
{
  fprintf(stderr, "**** Ctrl+C,quit ! \n");
  mSigTerm = true;
}
void cyber_trans_node::exit_ini()
{
  struct sigaction sa, osa;
  sa.sa_handler = exit_handler;
  sigemptyset(&sa.sa_mask);
  sa.sa_flags = 0;
  if (sigaction(SIGINT, &sa, &osa) < 0)
  {
    std::cout << "Set signal ctrl+c failure!" << std::endl;
  }
}
void cyber_trans_node::config_ini()
{
	rr::RrConfig config;
	config.ReadConfig("config.ini");
	bio_ip = config.ReadString("BridgeIO", "BridgeIO_IP", "");

  vehicle_id = config.ReadString("HostVehicle", "Vehicle_ID", "0");

  enable_physical_sensor = config.ReadInt("Sensor", "ENABLED", 0) > 0;

  mImgParam.ip = config.ReadString("Sensor", "IMG_IP", "127.0.0.1");
  mImgParam.port = config.ReadInt("Sensor", "IMG_PORT", 0);

  mPCDParam.ip = config.ReadString("Sensor", "PCD_IP", "127.0.0.1");
  mPCDParam.port = config.ReadInt("Sensor", "PCD_PORT", 0);
  mPCDParam.infoPort = config.ReadInt("Sensor", "PCD_PORT_INFO", 0);
  mPCDParam.frameId = config.ReadString("Sensor", "PCD_FRAMEID", "velodyne128");
  mPCDChannel = config.ReadString("Sensor", "PCD_CHANNEL", "/apollo/sensor/lidar128/compensator/PointCloud2");

	// float lat = config.ReadFloat("MapBase", "BASE_LATITUDE", 0);
	// float log = config.ReadFloat("MapBase", "BASE_LONGITUDE", 0);
	// float alt = config.ReadFloat("MapBase", "BASE_ALTITUDE", 0);
}
void cyber_trans_node::simone_ini()
{
  printf("Connecting BridgeIO With Ip : %s ...\n", bio_ip.c_str());
  if (!SimOneAPI::InitSimOneAPI(vehicle_id.c_str(), false, bio_ip.c_str()))
  {
    printf("SimOneAPI::InitSimOneAPI Failed!\n");
  }

  // ESimOne_Data_Vehicle_State index[INDEX_SIZE];
  // for (int i = 0; i < INDEX_SIZE; i++)
  // {
  //   index[i] = (ESimOne_Data_Vehicle_State)i;
  // }
  // for (int i=0; i<1000; i++)
  // {
  //   if (!SimOneAPI::RegisterVehicleState("0", index, INDEX_SIZE))
  //   {
  //     printf("\033[1m\033[31mSimOneAPI::RegisterVehicleState Faile!\033[0m\n");
  //     std::this_thread::sleep_for(std::chrono::milliseconds(500));
  //     continue;
  //   }
  //   break;
  // }

	if (!SimOneAPI::GetCaseInfo(&caseInfo))
  {
    printf("\033[1m\033[31mSimOneAPI::GetCaseInfo Failed!\033[0m\n");
  }
  // ------------ RegisterSimOneVehicleState End ------------
}
void cyber_trans_node::init()
{
  mPointCloud.set_params(mPCDParam);
  mImage.set_params(mImgParam);

  simone_ini();
}

void cyber_trans_node::run()
{
	init();
	std::thread pub_thread = std::thread(std::bind(&cyber_trans_node::run_pub, this));
	std::thread rcv_thread = std::thread(std::bind(&cyber_trans_node::run_rcv, this));
	pub_thread.join();
	rcv_thread.join();
}

int main(int argc, char *argv[])
{
	cyber_trans_node::config_ini();
  cyber_trans_node tn;
  tn.run();
  return 0;
}
