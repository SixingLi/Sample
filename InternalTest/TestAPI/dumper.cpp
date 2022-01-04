#include "dumper.h"

dumper::dumper(){}
dumper::~dumper(){}

void dumper::dump_gps(const char* mainVehicleId, SimOne_Data_Gps* pData)
{
	std::cout<<"------ mainVehicleId:"<< mainVehicleId << std::endl;
	std::cout<<"------ frame:"<< pData->frame << std::endl;
	std::cout << "posX/Y/Z: [" << pData->posX << ", " << pData->posY << ", " << pData->posZ << "]" << std::endl;
	std::cout << "oriX/Y/Z: [" << pData->oriX << ", " << pData->oriY << ", " << pData->oriZ << "]" << std::endl;
	std::cout << "velX/Y/Z: [" << pData->velX << ", " << pData->velY << ", " << pData->velZ << "]" << std::endl;
	std::cout << "throttle: " << pData->throttle << std::endl;
	std::cout << "brake: " << pData->brake << std::endl;
	std::cout << "steering: " << pData->steering << std::endl;
	std::cout << "gear: " << pData->gear << std::endl;
	std::cout << "accelX/Y/Z: " << pData->accelX << ", "  << pData->accelY << ", " << pData->accelZ << "]" << std::endl;
	std::cout << "angVelX/Y/Z: " << pData->angVelX << ", "  << pData->angVelY << ", " << pData->angVelZ << "]" << std::endl;
	std::cout << "wheelSpeedFL: " << pData->wheelSpeedFL << std::endl;
	std::cout << "wheelSpeedFR: " << pData->wheelSpeedFR << std::endl;
	std::cout << "wheelSpeedRL: " << pData->wheelSpeedRL << std::endl;
	std::cout << "wheelSpeedRR: " << pData->wheelSpeedRR << std::endl;
	std::cout << "engineRpm: " << pData->engineRpm << std::endl;
	std::cout << "odometer: " << pData->odometer << std::endl;
	std::cout << "extraStateSize: " << pData->extraStateSize << std::endl;
	for (int i=0; i<pData->extraStateSize; i++)
	{
		std::cout << "extraStates[" << i << "]: " << pData->extraStates[i] << std::endl;
	}
}

void dumper::dump_ground_truth(const char* mainVehicleId, SimOne_Data_Obstacle* pData)
{
	std::cout<<"------ mainVehicleId:"<< mainVehicleId << std::endl;
	std::cout<<"------ frame:"<< pData->frame << std::endl;
	std::cout << "obstacleSize: " << pData->obstacleSize << std::endl;
	for (int i=0; i<pData->obstacleSize ; i++)
	{
		std::cout << "obstacle[" << i << "].id: " << pData->obstacle[i].id << std::endl;
		std::cout << "obstacle[" << i << "].viewId: " << pData->obstacle[i].viewId << std::endl;
		std::cout << "obstacle[" << i << "].type: " << pData->obstacle[i].type << std::endl;
		std::cout << "obstacle[" << i << "].theta: " << pData->obstacle[i].theta << std::endl;
		std::cout << "obstacle[" << i << "].posX/Y/Z: [" << pData->obstacle[i].posX << ", " << pData->obstacle[i].posY << ", " << pData->obstacle[i].posZ << "]" << std::endl;
		std::cout << "obstacle[" << i << "].oriX/Y/Z: [" << pData->obstacle[i].oriX << ", " << pData->obstacle[i].oriY << ", " << pData->obstacle[i].oriZ << "]" << std::endl;
		std::cout << "obstacle[" << i << "].velX/Y/Z: [" << pData->obstacle[i].velX << ", " << pData->obstacle[i].velY << ", " << pData->obstacle[i].velZ << "]" << std::endl;
		std::cout << "obstacle[" << i << "].length: " << pData->obstacle[i].length << std::endl;
		std::cout << "obstacle[" << i << "].width: " << pData->obstacle[i].width << std::endl;
		std::cout << "obstacle[" << i << "].height: " << pData->obstacle[i].height << std::endl;
		std::cout << "obstacle[" << i << "].accelX/Y/Z: [" << pData->obstacle[i].accelX << ", " << pData->obstacle[i].accelY << ", " << pData->obstacle[i].accelZ << "]" << std::endl;
	}
}

void dumper::dump_radar_detection(const char* mainVehicleId, SimOne_Data_RadarDetection * pData)
{

	std::cout<<"------ mainVehicleId:"<< mainVehicleId << std::endl;
	std::cout<<"------ frame:"<< pData->frame << std::endl;
	std::cout<<"detectNum :"<< pData->detectNum << std::endl;
	for (int i = 0; i < pData->detectNum; i++)
	{
		std::cout << "detections[" << i << "].ip: " << pData->detections[i].id << std::endl;
		std::cout << "detections[" << i << "].subId: " << pData->detections[i].subId << std::endl;
		std::cout << "detections[" << i << "].type: " << pData->detections[i].type << std::endl;
		std::cout << "detections[" << i << "].posX/Y/Z: [" << pData->detections[i].posX << ", " << pData->detections[i].posY << ", " << pData->detections[i].posZ << "]"<< std::endl;
		std::cout << "detections[" << i << "].velX/Y/Z: [" << pData->detections[i].velX << ", " << pData->detections[i].velY << ", " << pData->detections[i].velZ << "]"<< std::endl;
		std::cout << "detections[" << i << "].accelX/Y/Z: [" << pData->detections[i].accelX << ", " << pData->detections[i].accelY << ", " << pData->detections[i].accelZ << "]"<< std::endl;
		std::cout << "detections[" << i << "].oriX/Y/Z: [" << pData->detections[i].oriX << ", " << pData->detections[i].oriY << ", " << pData->detections[i].oriZ << "]"<< std::endl;
		std::cout << "detections[" << i << "].length: " << pData->detections[i].length << std::endl;
		std::cout << "detections[" << i << "].width: " << pData->detections[i].width << std::endl;
		std::cout << "detections[" << i << "].height: " << pData->detections[i].height << std::endl;
		std::cout << "detections[" << i << "].range: " << pData->detections[i].range << std::endl;
		std::cout << "detections[" << i << "].rangeRate: " << pData->detections[i].rangeRate << std::endl;
		std::cout << "detections[" << i << "].azimuth: " << pData->detections[i].azimuth << std::endl;
		std::cout << "detections[" << i << "].vertical: " << pData->detections[i].vertical << std::endl;
		std::cout << "detections[" << i << "].snrdb: " << pData->detections[i].snrdb << std::endl;
		std::cout << "detections[" << i << "].rcsdb: " << pData->detections[i].rcsdb << std::endl;
		std::cout << "detections[" << i << "].probability: " << pData->detections[i].probability << std::endl;
	}
}
