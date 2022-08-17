#include "TaskCyberGps.hpp"

void CW_Gps::set_gps(Gps_T& data)
{
    static int lastFrame = 0;
    std::unique_ptr<SimOne_Data_Gps> pGps = std::make_unique<SimOne_Data_Gps>();

    if (!SimOneAPI::GetGps(0, pGps.get()))
    {
        std::cout << "[CW_Gps] GetGps Failed!" << std::endl;
        return;
    }
    if (pGps->frame == lastFrame)
    {
        return;
    }
    lastFrame = pGps->frame;

    cybertron::quat q(cybertron::vec3(pGps->oriX, pGps->oriY, pGps->oriZ - (M_PI/ 2)));

    data.posX = pGps->posX;
    data.posY = pGps->posY;
    data.posZ = pGps->posZ;
    data.mVelocityX = pGps->velX;
    data.mVelocityY = pGps->velY;
    data.mVelocityZ = pGps->velZ;
    data.oriX = q.x;
    data.oriY = q.y;
    data.oriZ = q.z;
    data.oriW = q.w;
}

void CW_Gps::task_gps()
{
    Gps_T data;
    set_gps(data);
    mCyberGps.publish(data.posX, data.posY, data.posZ, data.mVelocityX, data.mVelocityY, data.mVelocityZ, data.oriX, data.oriY, data.oriZ, data.oriW);
    mCyberInsStat.publish();
}
