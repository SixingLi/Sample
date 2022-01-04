#ifndef DUMPER_H
#define DUMPER_H

#include "Service/SimOneIOStruct.h"
#include <iostream>

class dumper
{
public:
	dumper();
	~dumper();

	void dump_gps(const char* mainVehicleId, SimOne_Data_Gps* pData);
	void dump_ground_truth(const char* mainVehicleId, SimOne_Data_Obstacle* pData);
	void dump_radar_detection(const char* mainVehicleId, SimOne_Data_RadarDetection * pData);


};

#endif