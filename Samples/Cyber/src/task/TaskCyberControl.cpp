#include "TaskCyberControl.hpp"

void CW_Ctl::set_ctl(std::unique_ptr<SimOne_Data_Control>& pData)
{
	mCyberControl.spinOnce();
    pData->gear = (ESimOne_Gear_Mode)mCyberControl.mGear;
    pData->throttleMode = ESimOne_Throttle_Mode_Percent;
    pData->throttle = mCyberControl.mThrottle * 0.01;
    pData->steeringMode = ESimOne_Steering_Mode_Percent;
    pData->steering = -mCyberControl.mSteering *0.01;
    pData->brakeMode = ESimOne_Brake_Mode_Percent;
    pData->brake = mCyberControl.mBrake *0.01;

    std::cout << "------------ CW_Ctl ------------" << std::endl;
    std::cout << "gear = " << pData->gear << std::endl;
    std::cout << "throttleMode = " << pData->throttleMode << std::endl;
    std::cout << "throttle = " << pData->throttle << std::endl;
    std::cout << "steeringMode = " << pData->steeringMode << std::endl;
    std::cout << "steering = " << pData->steering << std::endl;
    std::cout << "brakeMode = " << pData->brakeMode << std::endl;
    std::cout << "brake = " << pData->brake << std::endl;
}

void CW_Ctl::task_ctl()
{
    std::unique_ptr<SimOne_Data_Control> pCtrl = std::make_unique<SimOne_Data_Control>();
    set_ctl(pCtrl);

    if (!SimOneAPI::SetDrive("0", pCtrl.get()))
    {
        std::cout << "[CW_Ctl] SetDrive Failed!" << std::endl;
    }
}