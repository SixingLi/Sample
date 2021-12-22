#include "test_bench.h"

int main(int argc, char* argv[])
{
	const char* mv_id = "0";
	tester t(mv_id);

	bool isJoinTimeLoop = false;
	const char* serverIP = "127.0.0.1";
	t. Test_InitSimOneAPI(isJoinTimeLoop, serverIP);

	// std::vector<std::string> apiNames = {"GetTrafficSignList","GetTrafficLightList","GetCrossHatchList","GetLaneLink"};
	// std::vector<std::string> apiNames = {"GetTrafficLightList"};
	// t.Test_HDMap_ALL(apiNames);

	t.Test_GetSensorConfigurations();
	t.Test_V2X(true);
	t.Test_UltrasonicRadars(false);
	t.Test_UltrasonicRadar();
	t.Test_SensorLaneInfo(false);
	t.Test_GPS(true);
	t.Test_SensorSensorDetection(true);
	t.Test_RadarDetection(false);
	t.Test_GetGroundTruth(false);
	// system("pause");

	t.Test_SetEnvironment();
	//std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	t.Test_GetEnvironment();

	while (1)
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(30));
	}
	return 0;
}
