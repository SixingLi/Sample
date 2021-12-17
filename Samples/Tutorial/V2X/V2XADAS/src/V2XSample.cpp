#include "SimOneSensorAPI.h"
#include "SimOneV2XAPI.h"
#include "SimOneServiceAPI.h"

#include <thread> 
#include <chrono>
#include <iostream>
#include <cstring>

int main(int argc, char* argv[])
{
	bool isJoinTimeLoop = false;
	const char* MainVehicleId = "0";
	SimOneAPI::InitSimOneAPI(MainVehicleId, isJoinTimeLoop);
	//SimOneAPI::SetSensorObjectbasedDataEnable(false);
	std::unique_ptr<SimOne_Data_Obstacle> pObstacle = std::make_unique<SimOne_Data_Obstacle>();

#ifndef WITHOUT_HDMAP
	SimOneAPI::InitSimOneAPI(MainVehicleId, isJoinTimeLoop);
	std::unique_ptr<SimOne_Data_V2XNFS> pDetections = std::make_unique<SimOne_Data_V2XNFS>();
#endif
	while (1)
	{
		//Sensor Only Test
		//SimOneAPI::GetGroundTruth(0, pObstacle.get());
		//if (pObstacle.get() != NULL) {
		//	std::cout << "obstacleSize: " << pObstacle->obstacleSize << " height:" << pObstacle->obstacle[0].height << std::endl;
		//}

#ifndef WITHOUT_HDMAP
		auto function = [](const char* mainVehicleId, const char* sensorId, SimOne_Data_V2XNFS *pDetections) {
			std::cout << pDetections->MsgFrameData << std::endl;
		};
		SimOneAPI::SetV2XInfoUpdateCB(function);

		if (SimOneAPI::GetV2XInfo(0, "v2x", ESimOne_V2X_MessageFrame_PR_bsmFrame, pDetections.get())) {
			std::cout << "strlen = " << strlen(pDetections->MsgFrameData) << "  " << pDetections->MsgFrameData << std::endl;
			std::this_thread::sleep_for(std::chrono::milliseconds(50));
		}
#else

#endif // !WITHOUT_HDMAP
	}
	return 0;
}
