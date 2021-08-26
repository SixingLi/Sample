#include "SimOneSensorAPI.h"
#include "SimOneV2XAPI.h"
#include "SimOneServiceAPI.h"
#include <thread> 
#include <chrono>
#include <iostream>
using namespace std;
int main(int argc, char* argv[])
{
	bool isJoinTimeLoop = false;
	int MainVehicleId = 0;
	std::unique_ptr<SimOne_Data_Obstacle> pObstacle = std::make_unique<SimOne_Data_Obstacle>();
	SimOneAPI::InitSimOneAPI(MainVehicleId, isJoinTimeLoop);
	std::unique_ptr<SimOne_Data_V2XNFS> pDetections = std::make_unique<SimOne_Data_V2XNFS>();
	std::unique_ptr<SimOne_Data_Gps> pGps = std::make_unique<SimOne_Data_Gps>();

	int mainVehicleId = 0;
	//bool isJoinTimeLoop = false;

	while (1)
	{

		//1 GetGps
		if (SimOneAPI::GetGps(mainVehicleId, pGps.get())) {
			cout << "MainVehicle Pos.X: " << pGps.get()->posX << ", MainVehicle Pos.Y: " << pGps.get()->posY << endl;
		}
		else {
			cout << "GetGps fail " << endl;
		}


		////1 GetGroundTruth
		//SimOneAPI::GetGroundTruth(0, pObstacle.get());
		//if (pObstacle.get() != NULL) {
		//	std::cout << "obstacleSize: " << pObstacle->obstacleSize << " height:" << pObstacle->obstacle[0].height << std::endl;
		//}

		auto function = [](int mainVehicleId, const char* sensorId, SimOne_Data_V2XNFS *pDetections) {
			std::cout << pDetections->MsgFrameData << std::endl;
		};
		SimOneAPI::SetV2XInfoUpdateCB(function);

		if (SimOneAPI::GetV2XInfo(0, "v2x", MessageFrame_PR_bsmFrame, pDetections.get())) {
			std::cout << "strlen = " << strlen(pDetections->MsgFrameData) << "  " << pDetections->MsgFrameData << std::endl;
			std::this_thread::sleep_for(std::chrono::milliseconds(50));
		}
	}
	return 0;
}
