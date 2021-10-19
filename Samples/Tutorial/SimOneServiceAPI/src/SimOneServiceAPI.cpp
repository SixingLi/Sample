#include <iostream>
#include <vector>
#include <thread>
#include <string>

#include <fstream>
#include "utilTest.h"

//for SimOneServiceAPI.h test

//TEST_F(GlobalTestNoSync, GetCaseInfo)
//{
//	if (SimOneAPI::GetCaseRunStatus() == SimOne_Case_Status::SimOne_Case_Status_Running) {
//		std::unique_ptr<SimOne_Data_CaseInfo> caseInfoTest = std::make_unique<SimOne_Data_CaseInfo>();
//		bool caseInfoTestResult = SimOneAPI::GetCaseInfo(caseInfoTest.get());
//		EXPECT_TRUE(caseInfoTestResult) << "GetCaseInfo return false";
//		std::cout << "caseInfoTest caseName" << caseInfoTest->caseName << std::endl;
//		std::cout << "caseInfoTest caseId" << caseInfoTest->caseId << std::endl;
//		std::cout << "caseInfoTest taskId" << caseInfoTest->taskId << std::endl;
//		std::cout << "caseInfoTest sessionId" << caseInfoTest->sessionId << std::endl;
//	}
//	else {
//		std::cout << "SimOneAPI::GetCaseRunStatus() wating......." << SimOneAPI::GetCaseRunStatus() << std::endl;
//		std::this_thread::sleep_for(std::chrono::milliseconds(1000));
//	}
//}


TEST_F(GlobalTestNoSync, GetVersion) {
	std::cout << "GetVersion:------------" << SimOneAPI::GetVersion() << std::endl;;
}

TEST_F(GlobalTestNoSync, GetMainVehicleList) {
	std::unique_ptr<SimOne_Data_MainVehicle_Info> MainVehicleListTest = std::make_unique<SimOne_Data_MainVehicle_Info>();
	bool result = SimOneAPI::GetMainVehicleList(MainVehicleListTest.get());
	std::cout << "size:" << MainVehicleListTest->size << std::endl;
	for (int i = 0; i < MainVehicleListTest->size; i++) {
		std::cout << "id:" << MainVehicleListTest->id_list[i] << std::endl;
		std::cout << "type:" << MainVehicleListTest->type_list[i] << std::endl;
	}
	

	std::cout << "GetMainVehicleList result" << result << std::endl;

}

TEST_F(GlobalTestNoSync, GetMainVehicleStatus) {
	std::unique_ptr<SimOne_Data_MainVehicle_Status> MainVehicleTest = std::make_unique<SimOne_Data_MainVehicle_Status>();
	bool MainVehicleTestResult = SimOneAPI::GetMainVehicleStatus(MainVehicleTest.get());
	EXPECT_TRUE(MainVehicleTestResult) << "GetMainVehicleStatus return false";
	EXPECT_EQ(0, MainVehicleTest->mainVehicleId) << "mainVehicleId error, not 0" << std::endl;
	EXPECT_EQ(0, MainVehicleTest->mainVehicleId) << "mainVehicleStatus error, not 0" << std::endl;
}

TEST_F(GlobalTestNoSync, SetMainVehicleStatusCB) {
	static bool flagSetSimOneGpsCB = true;
	static bool numberSetSimOneGpsCB = true;
	SimOneAPI::SetMainVehicleStatusCB([](SimOne_Data_MainVehicle_Status *pMainVehicleStatus) {
		EXPECT_EQ(0, pMainVehicleStatus->mainVehicleId) << "mainVehicleId error, not 0" << std::endl;
		EXPECT_EQ(0, pMainVehicleStatus->mainVehicleId) << "mainVehicleStatus error, not 0" << std::endl;
	});

	//TEST_F(GlobalTestCallBack,SetSimOneGpsCB) {
	//	static bool flagSetSimOneGpsCB = true;
	//	static bool numberSetSimOneGpsCB = true;
	//	//SimOneAPI::SetFrameCB(0, 0);
	//	SimOneAPI::SetSimOneGpsCB([](SimOne_Data_Gps *pGps) {
	//		float testThrottle = 0.01;
	//		EXPECT_TRUE(pGps->posX);
	//		EXPECT_EQ(testThrottle, pGps->throttle);
	//		EXPECT_EQ(1, pGps->gear);
	//		std::cout << "++++++++++++++++++++++++++++++++++++++++GetSimOneGps" << std::endl;
	//		std::cout << "SetSimOneGpsCB timestamp: " << pGps->timestamp << std::endl;
	//		std::cout << "SetSimOneGpsCB posX: " << pGps->posX << std::endl;
	//		std::cout << "SetSimOneGpsCB posY: " << pGps->posY << std::endl;
	//		std::cout << "SetSimOneGpsCB posZ: " << pGps->posZ << std::endl;
	//		std::cout << "SetSimOneGpsCB oriX: " << pGps->oriX << std::endl;
	//		std::cout << "SetSimOneGpsCB oriY: " << pGps->oriY << std::endl;
	//		std::cout << "SetSimOneGpsCB oriZ: " << pGps->oriZ << std::endl;
	//		std::cout << "SetSimOneGpsCB velX: " << pGps->velX << std::endl;
	//		std::cout << "SetSimOneGpsCB velY: " << pGps->velY << std::endl;
	//		std::cout << "SetSimOneGpsCB velZ: " << pGps->velZ << std::endl;
	//		std::cout << "SetSimOneGpsCB throttle: " << pGps->throttle << std::endl;
	//		std::cout << "SetSimOneGpsCB brake: " << pGps->brake << std::endl;
	//		std::cout << "SetSimOneGpsCB steering: " << pGps->steering << std::endl;
	//		std::cout << "SetSimOneGpsCB gear: " << pGps->gear << std::endl;
	//		std::cout << "SetSimOneGpsCB accelX: " << pGps->accelX << std::endl;
	//		std::cout << "SetSimOneGpsCB accelY: " << pGps->accelY << std::endl;
	//		std::cout << "SetSimOneGpsCB accelZ: " << pGps->accelZ << std::endl;
	//		std::cout << "SetSimOneGpsCB angVelX: " << pGps->angVelX << std::endl;
	//		std::cout << "SetSimOneGpsCB angVelY: " << pGps->angVelY << std::endl;
	//		std::cout << "SetSimOneGpsCB angVelZ: " << pGps->angVelZ << std::endl;
	//		std::cout << "SetSimOneGpsCB wheelSpeedFL: " << pGps->wheelSpeedFL << std::endl;
	//		std::cout << "SetSimOneGpsCB wheelSpeedFR: " << pGps->wheelSpeedFR << std::endl;
	//		std::cout << "SetSimOneGpsCB wheelSpeedRL: " << pGps->wheelSpeedRL << std::endl;
	//		std::cout << "SetSimOneGpsCB wheelSpeedRR: " << pGps->wheelSpeedRR << std::endl;
	//		flagSetSimOneGpsCB = false;
	//		std::cout << "flagSetSimOneGpsCB: " << flagSetSimOneGpsCB << std::endl;
	//		//number++;
	//	});
	//	while(flagSetSimOneGpsCB && numberSetSimOneGpsCB){
	//		numberSetSimOneGpsCB= false;
	//	}
	//	if (flagSetSimOneGpsCB){
	//		GTEST_FATAL_FAILURE_("not call back");
	//	}
	//
	//
	//}
	//
	//TEST_F(GlobalTestCallBack, GetSimOneGroundTruth) {
	//	static bool flagGetSimOneGroundTruth = true;
	//	static bool numberGetSimOneGroundTruth = true;
	//	//SimOneAPI::SetFrameCB(0, 0);
	//	SimOneAPI::SetSimOneGroundTruthCB([](SimOne_Data_Obstacle *pObstacle) {
	//		EXPECT_EQ(pObstacle->obstacleSize, 9);
	//
	//		std::cout << "GetSimOneGroundTruth timestamp: " << pObstacle->timestamp << std::endl;
	//		std::cout << "GetSimOneGroundTruth frame: " << pObstacle->frame << std::endl;
	//		std::cout << "GetSimOneGroundTruth obstacleSize: " << pObstacle->obstacleSize << std::endl;
	//
	//		for (int i = 0; i < pObstacle->obstacleSize; ++i) {
	//			std::cout << "*********************" << std::endl;
	//			std::cout << "id: " << pObstacle->obstacle[i].id << std::endl;
	//			std::cout << "viewId: " << pObstacle->obstacle[i].viewId << std::endl;
	//			std::cout << "theta: " << pObstacle->obstacle[i].theta << std::endl;
	//			std::cout << "type: " << pObstacle->obstacle[i].type << std::endl;
	//			std::cout << "posX: " << pObstacle->obstacle[i].posX << std::endl;
	//			std::cout << "posY: " << pObstacle->obstacle[i].posY << std::endl;
	//			std::cout << "posZ: " << pObstacle->obstacle[i].posZ << std::endl;
	//			std::cout << "velX: " << pObstacle->obstacle[i].velX << std::endl;
	//			std::cout << "velY: " << pObstacle->obstacle[i].velY << std::endl;
	//			std::cout << "velZ: " << pObstacle->obstacle[i].velZ << std::endl;
	//			std::cout << "length: " << pObstacle->obstacle[i].length << std::endl;
	//			std::cout << "width: " << pObstacle->obstacle[i].width << std::endl;
	//			std::cout << "height: " << pObstacle->obstacle[i].height << std::endl;
	//		}
	//		flagGetSimOneGroundTruth = false;
	//		//number++;
	//	});
	//	while (flagGetSimOneGroundTruth && numberGetSimOneGroundTruth) {
	//		numberGetSimOneGroundTruth=false;
	//	}
	//	if (flagGetSimOneGroundTruth){
	//		GTEST_FATAL_FAILURE_("not call back");
	//	}
	//}
}

int main(int argc, char* argv[]) {
	//testing::GTEST_FLAG(output) = "xml:";
	testing::GTEST_FLAG(filter) = "GlobalTestNoSync.GetMainVehicleList";
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();

}
