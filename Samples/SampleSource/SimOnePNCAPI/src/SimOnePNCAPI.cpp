#include <vector>
#include <thread>
#include <string>
#include <fstream>
#include "utilTest.h"

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

TEST_F(GlobalTestNoSync, GetSimOneVehicleState)
{
	std::unique_ptr<SimOne_Data_Vehicle_Extra> VehicleExtraTest = std::make_unique<SimOne_Data_Vehicle_Extra>();
	bool VehicleExtraResult = SimOneAPI::GetVehicleState(VehicleExtraTest.get());
	EXPECT_TRUE(VehicleExtraResult);
	std::cout << "VehicleExtraTest dataSize:" << VehicleExtraTest->dataSize << std::endl;
	std::cout << "VehicleExtraTest extra_states:" << VehicleExtraTest->extra_states << std::endl;
}

TEST_F(GlobalTestNoSync, SetNetPose) {
	std::unique_ptr<SimOne_Data_Gps> Gps = std::make_unique<SimOne_Data_Gps>();
	bool resultGps = SimOneAPI::GetGps(0, Gps.get());
	std::unique_ptr<SimOne_Data_Pose_Control> Pose_Control = std::make_unique<SimOne_Data_Pose_Control>();
	float initPosX = Gps->posX;
	Pose_Control->posX = initPosX + 10;
	bool resultPose = SimOneAPI::SetPose(0, Pose_Control.get());
	EXPECT_TRUE(resultPose);
}

//存在参数的交叉验证过程
TEST_F(GlobalTestNoSync, SetDrive) {
	std::unique_ptr<SimOne_Data_Control> ControlTest = std::make_unique<SimOne_Data_Control>();
	ControlTest->throttle = 0.1;
	ControlTest->gear = ESimOne_Gear_Mode::EGearMode_Drive;
	ControlTest->brake = 0;
	ControlTest->steering = 0;
	bool SetDriveResult = SimOneAPI::SetDrive(0, ControlTest.get());
	EXPECT_TRUE(SetDriveResult) << "SetDrive set Error" << std::endl;
}

//暂不清晰具体使用
TEST_F(GlobalTestNoSync, SetDriveTrajectory) {
	std::unique_ptr<SimOne_Data_Control_Trajectory> DriveTrajectoryTest = std::make_unique<SimOne_Data_Control_Trajectory>();
	DriveTrajectoryTest->point_num = 1;
	DriveTrajectoryTest->isReverse = false;
	bool DriveTrajectoryTestResult = SimOneAPI::SetDriveTrajectory(0, DriveTrajectoryTest.get());
	EXPECT_TRUE(DriveTrajectoryTestResult) << "SetDrive set Error" << std::endl;
}


TEST_F(GlobalTestNoSync, SetDriverName) {
	std::unique_ptr<SimOne_Data_Control> ControlTest = std::make_unique<SimOne_Data_Control>();
	while (1) {
		SimOneAPI::SetDriverName(0, "testDriverName");
		ControlTest->throttle = 0.1;
		ControlTest->gear = ESimOne_Gear_Mode::EGearMode_Drive;
		ControlTest->brake = 0;
		ControlTest->steering = 0;
		bool SetDriveResult = SimOneAPI::SetDrive(0, ControlTest.get());
		EXPECT_TRUE(SetDriveResult) << "SetDrive set Error" << std::endl;
	}

}

TEST_F(GlobalTestNoSync, SetVehicleEvent) {
	std::unique_ptr<SimOne_Data_Vehicle_EventInfo> EventInfoTest = std::make_unique<SimOne_Data_Vehicle_EventInfo>();
	EventInfoTest->type = ESimone_Vehicle_EventInfo_Type::ESimOne_VehicleEventInfo_Forward_Collision_Warning;
	bool EventInfoTestResult = SimOneAPI::SetVehicleEvent(0, EventInfoTest.get());
	EXPECT_TRUE(EventInfoTestResult) << "SetVehicleEvent Error" << std::endl;
}

TEST_F(GlobalTestNoSync, SetSignalLights) {
	std::unique_ptr<SimOne_Data_Signal_Lights> SignalLights = std::make_unique<SimOne_Data_Signal_Lights>();
	SignalLights->signalLights = ESimOne_Signal_Light::ESimOne_Signal_Light_DoubleFlash;
	bool resultSignalLights = SimOneAPI::SetSignalLights(0, SignalLights.get());
	EXPECT_TRUE(resultSignalLights) << "SetSignalLights Error" << std::endl;
}

TEST_F(GlobalTestNoSync, GetDriverStatus) {
	std::unique_ptr<SimOne_Data_Driver_Status> DriverStatusTest = std::make_unique<SimOne_Data_Driver_Status>();
	bool DriverStatusTestResult = SimOneAPI::GetDriverStatus(0, DriverStatusTest.get());
	EXPECT_TRUE(DriverStatusTestResult) << "GetDriverStatus Error" << std::endl;
	std::cout << "driverStatus:" << DriverStatusTest->driverStatus << std::endl;
}


TEST_F(GlobalTestNoSync, GetDriverControl) {
	std::unique_ptr<SimOne_Data_Driver_Status> DriverStatusTest = std::make_unique<SimOne_Data_Driver_Status>();
	bool DriverStatusTestResult = SimOneAPI::GetDriverStatus(0, DriverStatusTest.get());
	EXPECT_TRUE(DriverStatusTestResult) << "GetDriverStatus Error" << std::endl;
	std::cout << "driverStatus:" << DriverStatusTest->driverStatus << std::endl;
}


TEST_F(GlobalTestSync, GetWayPoints) {
	std::unique_ptr<SimOne_Data_WayPoints> WayPointsTest = std::make_unique<SimOne_Data_WayPoints>();
	int frame = SimOneAPI::Wait();
	std::cout << "wait result:---" << frame << std::endl;

	bool WayPointsTestResult = SimOneAPI::GetWayPoints(WayPointsTest.get());
	EXPECT_TRUE(WayPointsTestResult) << "GetWayPoints Error" << std::endl;
	std::cout << "wayPointsSize:" << WayPointsTest->wayPointsSize << std::endl;
	for (int i = 0; i < WayPointsTest->wayPointsSize; i++) {
		std::cout << "wayPoints posX:" << WayPointsTest->wayPoints[i].posX << std::endl;
		std::cout << "wayPoints posY:" << WayPointsTest->wayPoints[i].posY << std::endl;
		std::cout << "wayPoints index:" << WayPointsTest->wayPoints[i].index << std::endl;
		std::cout << "wayPoints heading_x:" << WayPointsTest->wayPoints[i].heading_x << std::endl;
		std::cout << "wayPoints heading_y:" << WayPointsTest->wayPoints[i].heading_y << std::endl;
		std::cout << "wayPoints heading_z:" << WayPointsTest->wayPoints[i].heading_z << std::endl;
		std::cout << "wayPoints heading_w:" << WayPointsTest->wayPoints[i].heading_w << std::endl;
	}
	getchar();
	SimOneAPI::NextFrame(frame);
}

TEST_F(GlobalTestNoSync, SetScenarioEventCB) {

	SimOneAPI::SetScenarioEventCB([](int mainVehicleId, const char* event, const char* data) {
		std::cout << "event:------" << event << std::endl;
	});
	while (1) {
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}
}


int main(int argc, char* argv[]) {

	//testing::GTEST_FLAG(output) = "xml:";
	testing::GTEST_FLAG(filter) = "GlobalTestNoSync.SetScenarioEventCB";
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
