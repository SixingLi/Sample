#include <vector>
#include <thread>
#include <string>
#include <fstream>
#include "utilTest.h"

//#include "osi_groundtruth.pb.h"
//#include "osi_object.pb.h"
//#include "osi_sensorview.pb.h"
//#include "osi_sensorview.pb.h"
//#include "osi_featuredata.pb.h"
//#include "osi_sensordata.pb.h"
//#include "decode.h"
//#include <sstream>

//int CameraSensorId = 10;
//int MMWRadarSensorId = 10;
//int LiDARSensorId = 10;
//int UltrasonicRadarSensorId = 10;
//int AllUltrasonicRadarSensorId = 10;
//int PerfectPerceptionSensorId = 10;
//int V2XSensorId = 10;

const char* ip = "127.0.0.1";
unsigned short portLidar = 6699;
unsigned short infoPort = 7788;
unsigned short port = 7890;
using namespace std;

//for SimOneV2XAPI.h test
TEST_F(GlobalTestNoSync, SetV2XInfoUpdateCB) {
	static bool flagSetV2XInfoUpdateCB = true;
	SimOneAPI::SetV2XInfoUpdateCB([](int mainVehicleId, const char* sensorId, SimOne_Data_V2XNFS *pDetections) {
		std::cout << "SetV2XInfoUpdateCB timestamp:" << pDetections->timestamp << std::endl;
		std::cout << "SetV2XInfoUpdateCB frame:" << pDetections->frame << std::endl;
		std::cout << "SetV2XInfoUpdateCB V2XMsgFrameSize:" << pDetections->V2XMsgFrameSize << std::endl;
		std::cout << "SetV2XInfoUpdateCB MsgFrameData:" << pDetections->MsgFrameData << std::endl;
		flagSetV2XInfoUpdateCB = false;
		EXPECT_GT(0, pDetections->V2XMsgFrameSize);
		/*EXPECT_FLOAT_EQ(0.1, pGps->throttle);
		EXPECT_EQ(4, pGps->gear);*/

	});
	while (flagSetV2XInfoUpdateCB) {}
	if (flagSetV2XInfoUpdateCB) {
		GTEST_FATAL_FAILURE_("not call back");
	}
}

TEST_F(GlobalTestNoSync, GetV2XInfoResult) {
	std::unique_ptr<SimOne_Data_V2XNFS> pDetections = std::make_unique<SimOne_Data_V2XNFS>();
	std::map<int, SimOne_Data_V2XNFS>pDetectionsMap;
	// assert APIData
	//EXPECT_TRUE(resultGetGps) << "GetGps Error" << std::endl;
	while (1) {
		bool resultGetGps = SimOneAPI::GetV2XInfo(0, "10", 0, pDetections.get());
		if (pDetections->frame <= 5) {
			pDetectionsMap.insert(map<int, SimOne_Data_V2XNFS>::value_type(pDetections->frame, *pDetections));
		}
		if (pDetections->frame > 5) {
			std::this_thread::sleep_for(std::chrono::milliseconds(1000));
			pDetectionsMap.insert(map<int, SimOne_Data_V2XNFS>::value_type(pDetections->frame, *pDetections));
		}
		if (SimOneAPI::GetCaseRunStatus() == SimOne_Case_Status::SimOne_Case_Status_Stop) {
			break;
		}
	}
	std::cout << pDetectionsMap.size() << std::endl;
	map<int, SimOne_Data_V2XNFS>::iterator iter;
	for (iter = pDetectionsMap.begin(); iter != pDetectionsMap.end(); iter++) {
		std::cout << "frame: " << iter->first << std::endl;
		std::cout << "V2XMsgFrameSize: " << iter->second.V2XMsgFrameSize << std::endl;
		for (int i = 0; i < iter->second.V2XMsgFrameSize; i++) {
			std::cout << "posX: " << iter->second.MsgFrameData[i] << std::endl;
		}
	}
}


int main(int argc, char* argv[]) {
	//testing::GTEST_FLAG(output) = "xml:";
	//testing::GTEST_FLAG(filter) = "GlobalSMTest.GetStreamingPointCloud";
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
