#include "TaskCyberTrajectory.hpp"

void CW_Trajectory::set_trajectory(std::unique_ptr<SimOne_Data_Control_Trajectory>& pData)
{
	mCyberTrajectory.spinOnce();

    pData->point_num = mCyberTrajectory.mTrajectorySize;
    for(int i = 0; i < pData->point_num; i++)
    {
        pData->points[i].posx = mCyberTrajectory.mTrajectoryList[i].x;
        pData->points[i].posy = mCyberTrajectory.mTrajectoryList[i].y;
        pData->points[i].speed = mCyberTrajectory.mTrajectoryList[i].v;
    }
}

void CW_Trajectory::task_trajectory()
{
    std::unique_ptr<SimOne_Data_Control_Trajectory> pControlTrajectory = std::make_unique<SimOne_Data_Control_Trajectory>();
    set_trajectory(pControlTrajectory);

    if (!SimOneAPI::SetDriveTrajectory("0", pControlTrajectory.get()));
    {
        std::cout << "SimOneAPI::SetDriveTrajectory Failed!" << std::endl;
    }
}
