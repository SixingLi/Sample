#include "SimOnePNCAPI_Sample.h"

INITIALIZE_EASYLOGGINGPP

//for SimOnePNCAPI.h test
//TEST_F(GlobalTestNoSync, RegisterSimOneVehicleState)
//{
//	std::unique_ptr<SimOne_Data_Vehicle_State> VehicleStateTest = std::make_unique<SimOne_Data_Vehicle_State>();
//	bool result = SimOneAPI::RegisterSimOneVehicleState(VehicleStateTest.get(), int size);
//	std::cout << "GetCaseInfo caseName:" << CaseInfo->caseName << std::endl;
//	std::cout << "GetCaseInfo caseId:" << CaseInfo->caseId << std::endl;
//	std::cout << "GetCaseInfo taskId:" << CaseInfo->taskId << std::endl;
//	std::cout << "GetCaseInfo sessionId:" << CaseInfo->sessionId << std::endl;
//	EXPECT_TRUE(result);
//}

// TEST_F(GlobalTestNoSync, GetSimOneVehicleState)
// {
// 	std::unique_ptr<SimOne_Data_Vehicle_Extra> VehicleExtraTest = std::make_unique<SimOne_Data_Vehicle_Extra>();
// 	bool VehicleExtraResult = SimOneAPI::GetVehicleState(VehicleExtraTest.get());
// 	EXPECT_TRUE(VehicleExtraResult);
// 	std::cout << "VehicleExtraTest dataSize:" << VehicleExtraTest->dataSize << std::endl;
// 	std::cout << "VehicleExtraTest extra_states:" << VehicleExtraTest->extra_states << std::endl;
// }
// 
// TEST_F(GlobalTestNoSync, SetNetPose) {
// 	std::unique_ptr<SimOne_Data_Gps> Gps = std::make_unique<SimOne_Data_Gps>();
// 	bool resultGps = SimOneAPI::GetGps(0, Gps.get());
// 	std::unique_ptr<SimOne_Data_Pose_Control> Pose_Control = std::make_unique<SimOne_Data_Pose_Control>();
// 	float initPosX = Gps->posX;
// 	Pose_Control->posX = initPosX + 10;
// 	bool resultPose = SimOneAPI::SetPose(0, Pose_Control.get());
// 	EXPECT_TRUE(resultPose);
// }
// 
// //���ڲ����Ľ�����֤����
// TEST_F(GlobalTestNoSync, SetDrive) {
// 	std::unique_ptr<SimOne_Data_Control> ControlTest = std::make_unique<SimOne_Data_Control>();
// 	ControlTest->throttle = 0.1;
// 	ControlTest->gear = ESimOne_Gear_Mode::EGearMode_Drive;
// 	ControlTest->brake = 0;
// 	ControlTest->steering = 0;
// 	bool SetDriveResult = SimOneAPI::SetDrive(0, ControlTest.get());
// 	EXPECT_TRUE(SetDriveResult) << "SetDrive set Error" << std::endl;
// }
// 
// //�ݲ���������ʹ��
// TEST_F(GlobalTestNoSync, SetDriveTrajectory) {
// 	std::unique_ptr<SimOne_Data_Control_Trajectory> DriveTrajectoryTest = std::make_unique<SimOne_Data_Control_Trajectory>();
// 	DriveTrajectoryTest->point_num = 1;
// 	DriveTrajectoryTest->isReverse = false;
// 	bool DriveTrajectoryTestResult = SimOneAPI::SetDriveTrajectory(0, DriveTrajectoryTest.get());
// 	EXPECT_TRUE(DriveTrajectoryTestResult) << "SetDrive set Error" << std::endl;
// }
// 
// 
// TEST_F(GlobalTestNoSync, SetDriverName) {
// 	std::unique_ptr<SimOne_Data_Control> ControlTest = std::make_unique<SimOne_Data_Control>();
// 	while (1) {
// 		SimOneAPI::SetDriverName(0, "testDriverName");
// 		ControlTest->throttle = 0.1;
// 		ControlTest->gear = ESimOne_Gear_Mode::EGearMode_Drive;
// 		ControlTest->brake = 0;
// 		ControlTest->steering = 0;
// 		bool SetDriveResult = SimOneAPI::SetDrive(0, ControlTest.get());
// 		EXPECT_TRUE(SetDriveResult) << "SetDrive set Error" << std::endl;
// 	}
// 
// }
// 
// TEST_F(GlobalTestNoSync, SetVehicleEvent) {
// 	std::unique_ptr<SimOne_Data_Vehicle_EventInfo> EventInfoTest = std::make_unique<SimOne_Data_Vehicle_EventInfo>();
// 	EventInfoTest->type = ESimone_Vehicle_EventInfo_Type::ESimOne_VehicleEventInfo_Forward_Collision_Warning;
// 	bool EventInfoTestResult = SimOneAPI::SetVehicleEvent(0, EventInfoTest.get());
// 	EXPECT_TRUE(EventInfoTestResult) << "SetVehicleEvent Error" << std::endl;
// }
// 
// TEST_F(GlobalTestNoSync, SetSignalLights) {
// 	std::unique_ptr<SimOne_Data_Signal_Lights> SignalLights = std::make_unique<SimOne_Data_Signal_Lights>();
// 	SignalLights->signalLights = ESimOne_Signal_Light::ESimOne_Signal_Light_DoubleFlash;
// 	bool resultSignalLights = SimOneAPI::SetSignalLights(0, SignalLights.get());
// 	EXPECT_TRUE(resultSignalLights) << "SetSignalLights Error" << std::endl;
// }
// 
// TEST_F(GlobalTestNoSync, GetDriverStatus) {
// 	std::unique_ptr<SimOne_Data_Driver_Status> DriverStatusTest = std::make_unique<SimOne_Data_Driver_Status>();
// 	bool DriverStatusTestResult = SimOneAPI::GetDriverStatus(0, DriverStatusTest.get());
// 	EXPECT_TRUE(DriverStatusTestResult) << "GetDriverStatus Error" << std::endl;
// 	std::cout << "driverStatus:" << DriverStatusTest->driverStatus << std::endl;
// }
// 
// 
// TEST_F(GlobalTestNoSync, GetDriverControl) {
// 	std::unique_ptr<SimOne_Data_Driver_Status> DriverStatusTest = std::make_unique<SimOne_Data_Driver_Status>();
// 	bool DriverStatusTestResult = SimOneAPI::GetDriverStatus(0, DriverStatusTest.get());
// 	EXPECT_TRUE(DriverStatusTestResult) << "GetDriverStatus Error" << std::endl;
// 	std::cout << "driverStatus:" << DriverStatusTest->driverStatus << std::endl;
// }
// 
// 
// TEST_F(GlobalTestSync, GetWayPoints) {
// 	std::unique_ptr<SimOne_Data_WayPoints> WayPointsTest = std::make_unique<SimOne_Data_WayPoints>();
// 	int frame = SimOneAPI::Wait();
// 	std::cout << "wait result:---" << frame << std::endl;
// 
// 	bool WayPointsTestResult = SimOneAPI::GetWayPoints(WayPointsTest.get());
// 	EXPECT_TRUE(WayPointsTestResult) << "GetWayPoints Error" << std::endl;
// 	std::cout << "wayPointsSize:" << WayPointsTest->wayPointsSize << std::endl;
// 	for (int i = 0; i < WayPointsTest->wayPointsSize; i++) {
// 		std::cout << "wayPoints posX:" << WayPointsTest->wayPoints[i].posX << std::endl;
// 		std::cout << "wayPoints posY:" << WayPointsTest->wayPoints[i].posY << std::endl;
// 		std::cout << "wayPoints index:" << WayPointsTest->wayPoints[i].index << std::endl;
// 		std::cout << "wayPoints heading_x:" << WayPointsTest->wayPoints[i].heading_x << std::endl;
// 		std::cout << "wayPoints heading_y:" << WayPointsTest->wayPoints[i].heading_y << std::endl;
// 		std::cout << "wayPoints heading_z:" << WayPointsTest->wayPoints[i].heading_z << std::endl;
// 		std::cout << "wayPoints heading_w:" << WayPointsTest->wayPoints[i].heading_w << std::endl;
// 	}
// 	getchar();
// 	SimOneAPI::NextFrame(frame);
// }
// 
// TEST_F(GlobalTestNoSync, SetScenarioEventCB) {
// 
// 	SimOneAPI::SetScenarioEventCB([](int mainVehicleId, const char* event, const char* data) {
// 		std::cout << "event:------" << event << std::endl;
// 	});
// 	while (1) {
// 		std::this_thread::sleep_for(std::chrono::milliseconds(100));
// 	}
// }

pncapi_sample::pncapi_sample():log_set_pose_ctl("set_pose_ctl", "true", "true"), log_simone_ini("simone_ini", "true", "true")
{}

pncapi_sample::~pncapi_sample(){}

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
      LOGError(log_set_pose_ctl) << "SimOneSM::GetGps Failed!";
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

void pncapi_sample::pub()
{
  Timer timer_pose_ctl;

  timer_pose_ctl.start(20, std::bind(&pncapi_sample::set_pose_ctl, this));

  while(true)
  {
    if (SimOneAPI::GetCaseRunStatus() == ESimOne_Case_Status::ESimOne_Case_Status_Stop)
    {
      timer_pose_ctl.stop();

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
