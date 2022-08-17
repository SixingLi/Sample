#ifndef TASKCYBERCHASSISIMU_HPP_
#define TASKCYBERCHASSISIMU_HPP_

#include "CyberWriterChassis.hpp"
#include "CyberWriterImu.hpp"
#include "SimOneSensorAPI.h"
#include <iostream>
#include <math.h>

typedef struct
{
    float speedMps;
    float engineRpm;
    int gear;
    float throttleP;
    float steeringP;
    float brakeP;
    int drivingMode;
} Chassis_T;

typedef struct
{
    float angVelX;
    float angVelY;
    float angVelZ;
    float linAccX;
    float linAccY;
    float linAccZ;
} Imu_T;

class CW_Chassis_Imu
{
    public:
        void set_chassis_imu(Chassis_T& data_chassis, Imu_T& data_imu);
        void task_chassis_imu();
    private:
        CyberWriterChassis mCyberChassis;
        CyberWriterImu mCyberImu;
};

#endif