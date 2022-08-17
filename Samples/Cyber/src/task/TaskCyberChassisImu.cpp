#include "TaskCyberChassisImu.hpp"

void CW_Chassis_Imu::set_chassis_imu(Chassis_T& data_chassis, Imu_T& data_imu)
{
    static int lastFrame = 0;
    std::unique_ptr<SimOne_Data_Gps> pGps = std::make_unique<SimOne_Data_Gps>();

    if (!SimOneAPI::GetGps(0, pGps.get()))
    {
        std::cout << "[CW_Chassis_Imu] GetGps Failed!" << std::endl;
        return;
    }
    if (pGps->frame == lastFrame)
    {
        return;
    }
    lastFrame = pGps->frame;

    data_chassis.speedMps = sqrt((pGps->velX * pGps->velX) + (pGps->velY * pGps->velY));;
    data_chassis.engineRpm = pGps->engineRpm;
    data_chassis.gear = pGps->gear;
    data_chassis.throttleP = pGps->throttle;
    data_chassis.steeringP = pGps->steering;
    data_chassis.brakeP = pGps->brake;
    // data_chassis.drivingMode = ;

    data_imu.angVelX = pGps->angVelX;
    data_imu.angVelY = pGps->angVelY;
    data_imu.angVelZ = pGps->angVelZ;
    data_imu.linAccX = pGps->accelX;
    data_imu.linAccY = pGps->accelY;
    data_imu.linAccZ = pGps->accelZ;
}

void CW_Chassis_Imu::task_chassis_imu()
{
    Chassis_T data_chassis;
    Imu_T data_imu;
    set_chassis_imu(data_chassis, data_imu);
    mCyberChassis.publish(data_chassis.speedMps, data_chassis.engineRpm, data_chassis.gear, data_chassis.throttleP, data_chassis.steeringP, data_chassis.brakeP, 1);    // 1: COMPLETE_AUTO_DRIVE
    mCyberImu.publish(data_imu.angVelX, data_imu.angVelY, data_imu.angVelZ, data_imu.linAccX, data_imu.linAccY, data_imu.linAccZ);
}
