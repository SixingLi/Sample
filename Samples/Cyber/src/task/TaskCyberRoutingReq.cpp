#include "TaskCyberRoutingReq.hpp"

void CW_RoutingReq::set_routing_req(unsigned& point_size, CyberWriterRoutingReq::WayPoint*point_list)
{
    // std::unique_ptr<SimOne_Data_Gps> pGps = std::make_unique<SimOne_Data_Gps>();
    // if (!SimOneAPI::GetGps(0, pGps.get()))
    // {
    //     std::cout << "[CW_RoutingReq] GetGps Failed!" << std::endl;
    // }

    std::unique_ptr<SimOne_Data_WayPoints> pWayPoints = std::make_unique<SimOne_Data_WayPoints>();
    if (!SimOneAPI::GetWayPoints("0", pWayPoints.get()))
    {
        std::cout << "[CW_RoutingReq] GetWayPoints Failed!" << std::endl;
        return;
    }

    point_size = pWayPoints->wayPointsSize;
    for (int i=0; i<point_size; i++)
    {
        SimOne_Data_WayPoints_Entry& point = pWayPoints->wayPoints[i];
        point_list[i].x = point.posX;
        point_list[i].y = point.posY;
        point_list[i].headingx = point.heading_x; // pGps->oriX; 
        point_list[i].headingy = point.heading_y; // pGps->oriY; 
        point_list[i].headingz = point.heading_z; // pGps->oriZ; 
        point_list[i].headingw = point.heading_w;

        // point_list[i].speed = std::sqrt(pGps->velX * pGps->velX + pGps->velY * pGps->velY);
        // point_list[i].accel = std::sqrt(pGps->accelX * pGps->accelX + pGps->accelY * pGps->accelY);
        // point_list[i].time_interval = 1;
    }
}

void CW_RoutingReq::task_routing_req()
{
    static uint64_t  frame = 0;

    mCyberRoutingReq.mWayPointSize = 0;
    memset(mCyberRoutingReq.mWayPointList, 0, sizeof(CyberWriterRoutingReq::WayPoint) * WAYPOINT_MAX_SIZE);

    set_routing_req(mCyberRoutingReq.mWayPointSize, mCyberRoutingReq.mWayPointList);
    // std::cout << "-------------- WayPointSize: " << mCyberRoutingReq.mWayPointSize << std::endl;
    // for (int i=0; i<mCyberRoutingReq.mWayPointSize; i++)
    // {
    //     std::cout << "mWayPointList[" << i << "].x: "<< mCyberRoutingReq.mWayPointList[i].x << std::endl;
    //     std::cout << "mWayPointList[" << i << "].y: "<< mCyberRoutingReq.mWayPointList[i].y << std::endl;
    //     std::cout << "mWayPointList[" << i << "].headingx: "<< mCyberRoutingReq.mWayPointList[i].headingx << std::endl;
    //     std::cout << "mWayPointList[" << i << "].headingy: "<< mCyberRoutingReq.mWayPointList[i].headingy << std::endl;
    //     std::cout << "mWayPointList[" << i << "].headingz: "<< mCyberRoutingReq.mWayPointList[i].headingz << std::endl;
    //     std::cout << "mWayPointList[" << i << "].headingw: "<< mCyberRoutingReq.mWayPointList[i].headingw << std::endl;
    // }
    mCyberRoutingReq.publish(frame);
}
