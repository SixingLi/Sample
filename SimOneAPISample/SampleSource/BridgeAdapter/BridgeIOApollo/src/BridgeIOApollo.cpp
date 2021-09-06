#include "SimOneNetAPI.h"
#include <iostream>
#include <vector>
#include <thread>
#include <fstream>
#include <csignal>
#include <chrono>
#include <math.h>
#include "CyberWriterGps.hpp"
#include "CyberWriterInsStat.hpp"
#include "CyberReaderControl.hpp"
#include "CyberWriterObstacles.hpp"
#include "CyberWriterChassis.hpp"
#include "CyberWriterImu.hpp"
#include "CyberWriterRoutingReq.hpp"

// std::string fileName = "BridgeIOApollo.txt";
// std::ofstream outfile(fileName, std::ios::app);
// int number = 0;
bool run = true; 

void OnShutdown(int sig) {
  (void)sig;
  run =false;
}

float calculateSpeed(float x, float y, float z) {
	float a = powf32(x,(float)2.0) + powf32(y,(float)2.0) + powf32(z,(float)2.0);
	return sqrtf32(a);
} 

std::unique_ptr<SimOne_Data_Gps> gpsPtr = std::make_unique<SimOne_Data_Gps>();
std::unique_ptr<SimOne_Data_Obstacle> obstaclePtr = std::make_unique<SimOne_Data_Obstacle>();

std::unique_ptr<SimOne_Data_Control> controlPtr = std::make_unique<SimOne_Data_Control>();
std::unique_ptr<SimOne_Data_WayPoints> waypointsPtr = std::make_unique<SimOne_Data_WayPoints>();
int main(int argc, char* argv[])
{
	std::signal(SIGINT, OnShutdown);
	bool isJoinTimeLoop = true;
	CyberWriterGps mCyberGps;
	CyberReaderControl mCyberControl;
	//CyberWriterObstacles mCyberObstacles;
	CyberWriterImu mCyberImu;
	CyberWriterChassis mCyberChassis;
	CyberWriterInsStat mCyberInsStat;
	CyberWriterRoutingReq mCyberRoutingReq;
	//
	SimOneAPI::StartSimOneNode(0, isJoinTimeLoop, 0, 0);
	SimOneAPI::SetFrameCallback(0, 0);
	while(run){
		int frame = SimOneAPI::Wait();
		SimOneAPI::GetSimOneGps(gpsPtr.get());
		std::signal(SIGINT, OnShutdown);
		SimOneSM::GetWayPoints(waypointsPtr.get());
		printf("routing request way point size is [%d].\n",waypointsPtr->wayPointsSize);
		if (waypointsPtr->wayPointsSize > 0 && waypointsPtr->wayPointsSize < 100){
			printf("receive routing request.\n");
			mCyberRoutingReq.mWayPointSize = waypointsPtr->wayPointsSize;
			for (int i = 0; i < waypointsPtr->wayPointsSize; ++i){
			SimOne_Data_WayPoints_Entry &mWayPoints = waypointsPtr->wayPoints[i];
			CyberWriterRoutingReq::WayPoint &pubWayPoints = mCyberRoutingReq.mWayPointList[i];
			pubWayPoints.x = mWayPoints.posX;
			pubWayPoints.y = mWayPoints.posY;
			pubWayPoints.accel = 0;
			pubWayPoints.headingw = 0;
			pubWayPoints.headingx = 0;
			pubWayPoints.headingy = 0;
			pubWayPoints.headingz = 0;
			pubWayPoints.speed = 0;
			pubWayPoints.time_interval = 0;
			}
			mCyberRoutingReq.publish(frame);
		}
		//std::chrono::system_clock::time_point today = std::chrono::system_clock::now();
		//need to get rotation:OriX,Y,Z then trans to quaternions.
		mCyberGps.publish(gpsPtr->posX, gpsPtr->posY, gpsPtr->posZ, gpsPtr->velX, gpsPtr->velY, gpsPtr->velZ, 0.0, 0.0, 0.0, 1.0);
		//GPS status, need a check.
		mCyberInsStat.publish();
		mCyberChassis.publish(calculateSpeed(gpsPtr->velX, gpsPtr->velY, gpsPtr->velZ), 3000, gpsPtr->throttle, gpsPtr->steering, gpsPtr->brake, /*COMPLETE_AUTO_DRIVE*/ 1);
		mCyberImu.publish(gpsPtr->angVelX, gpsPtr->angVelY, gpsPtr->angVelZ, gpsPtr->accelX, gpsPtr->accelY, gpsPtr->accelZ);
		// SimOneAPI::GetGroundTruth(obstaclePtr.get());
		// mCyberObstacles.mObstacleSize = obstaclePtr->obstacleSize;
		// for (int i = 0; i < obstaclePtr->obstacleSize; ++i){
		// 	SimOne_Data_Obstacle_Entry &mObs = obstaclePtr->obstacle[i];
		// 	CyberWriterObstacles::Obstacle &pubObs = mCyberObstacles.mObstacleList[i];
		// 	pubObs.id = mObs.id;
		// 	switch (mObs.type)
		// 	{
		// 	case ESimOne_Obstacle_Type_Car:
		// 	{	pubObs.type = 5;
		// 		break;
		// 	}
		// 	case ESimOne_Obstacle_Type_Pedestrian:
		// 	{	pubObs.type = 3;
		// 		break;
		// 	}
		// 	case ESimOne_Obstacle_Type_Bicycle:
		// 	{	pubObs.type = 4;
		// 		break;
		// 	}
		// 	case ESimOne_Obstacle_Type_Motorcycle:
		// 	{	pubObs.type = 4;
		// 		break;
		// 	}
		// 	case ESimOne_Obstacle_Type_Truck:
		// 	{	pubObs.type = 5;
		// 		break;
		// 	}
		// 	case ESimOne_Obstacle_Type_Bus:
		// 	{	pubObs.type = 5;
		// 		break;
		// 	}
		// 	case ESimOne_Obstacle_Type_Dynamic:
		// 	{	pubObs.type = 1;
		// 		break;
		// 	}
		// 	case ESimOne_Obstacle_Type_RoadObstacle:
		// 	{	pubObs.type = 2;
		// 		break;
		// 	}
		// 	default:
		// 		pubObs.type = 0;
		// 		break;
		// 	}
		// 	/* pubObs.type.
		// 	enum PerceptionObstacle_Type {
		// 	PerceptionObstacle_Type_UNKNOWN = 0,
		// 	PerceptionObstacle_Type_UNKNOWN_MOVABLE = 1,invalid operands to binary expression ('float' and 'float')
		// 	PerceptionObstacle_Type_UNKNOWN_UNMOVABLE = 2,
		// 	PerceptionObstacle_Type_PEDESTRIAN = 3,
		// 	PerceptionObstacle_Type_BICYCLE = 4,
		// 	PerceptionObstacle_Type_VEHICLE = 5
		// 	};
		// 	*/
		// 	pubObs.theta = mObs.theta;
		// 	pubObs.posX = mObs.posX;
		// 	pubObs.posY = mObs.posY;
		// 	pubObs.posZ = mObs.posZ;
		// 	pubObs.velX = mObs.velX;
		// 	pubObs.velY = mObs.velY;
		// 	pubObs.velZ = mObs.velZ;
		// 	pubObs.length = mObs.length;
		// 	pubObs.width = mObs.width;
		// 	pubObs.height = mObs.height;
		// 	pubObs.polygonPointSize = 0;
		// 	pubObs.confidence = 1.0;
		// 	pubObs.confidenceType = 1;
		// }
		// mCyberObstacles.publish(frame);
		mCyberControl.spinOnce();
		controlPtr->throttle = mCyberControl.mThrottle * 0.01;
		controlPtr->brake = mCyberControl.mBrake * 0.01;
		controlPtr->steering = mCyberControl.mSteering * 0.01;
		if (controlPtr->throttle > 5){
			controlPtr->gear = EGearMode::EGearMode_Drive;
		}
		controlPtr->handbrake = false;
		controlPtr->isManualGear = false;
		SimOneSM::SetDrive(0,controlPtr.get());
		SimOneAPI::NextFrame(frame);
	}
	return 0;
}