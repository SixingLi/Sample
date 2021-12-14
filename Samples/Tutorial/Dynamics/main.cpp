#include "SimOneServiceAPI.h"
#include "SimOnePNCAPI.h"
#include "SimOneSensorAPI.h"
#include "TestSample.hpp"
#include <string.h>

using namespace std;

#define EXIT_FAILURE 1

int main(int argc, char* argv[])
{
	if (SimOneAPI::InitSimOneAPI("0", false, "10.66.9.111"))
	{
		cout << "InitSimOneAPI Done" << endl;
	}

	SimOneAPI::SetDriverName("0", "DynaTest");

	std::string csvPath = "";
	for (int i = 1; i < argc; ++i) {
		if (strcmp(argv[i], "-csvPath") == 0) {
			csvPath = argv[i + 1];
			++i;
		}
		else {
			csvPath = "resultsALL.csv";
		}
	}

	std::unique_ptr<SimOne_Data_Control> pControl = std::make_unique <SimOne_Data_Control>();
	std::unique_ptr<SimOne_Data_Gps> gpsPtr = std::make_unique<SimOne_Data_Gps>();
	while (1)
	{
		SimOneAPI::GetGps("0", gpsPtr.get());
		if (SimOneAPI::GetGps("0", gpsPtr.get()))
		{
			std::cout << "pGps->posX:" << gpsPtr->posX << ",pGps->posY" << gpsPtr->posY << endl;
		}
		else
		{
			std::cout << "Fetch GPS failed" << std::endl;
		}

		pControl->throttle = 1.0f;
		pControl->brake = 0.0f;
		pControl->steering = 0.0f;
		pControl->handbrake = false;
		pControl->isManualGear = false;
		pControl->gear = ESimOne_Gear_Mode_Drive;
		pControl->timestamp = gpsPtr->timestamp;
		bool testre = SimOneAPI::SetDrive("0", pControl.get());
		// std::this_thread::sleep_for(std::chrono::milliseconds(100));
		printf("----------------------------------%d\n", testre);

	}

	/* 1. full brake */
	TestSample test;
	test.FullBrakeTest("0", 1.0f, csvPath);

	/////* 2. accelerate to a certain speed */
	////test.DriveToSpeed(0, 100.f, 1.0f, csvPath);

	///* 3. dynamical steer */
	//test.SteerDynamicTest(0, 60.f, TestSample::SteerType::sineSteer, csvPath);
	return 0;
}