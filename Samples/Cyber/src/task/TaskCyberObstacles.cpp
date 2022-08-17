#include "TaskCyberObstacles.hpp"

void CW_Obstacles::set_obstacles(int& obj_size, CyberWriterObstacles::Obstacle* obj_list)
{
    static int lastFrame = 0;
    std::unique_ptr<SimOne_Data_Obstacle> pGroundTruth = std::make_unique<SimOne_Data_Obstacle>();

    if (!SimOneAPI::GetGroundTruth("0", pGroundTruth.get()))
    {
        std::cout << "[TaskCyberObstacles] GetGroundTruth Failed!" << std::endl;
        return;
    }
    if (pGroundTruth->frame == lastFrame)
    {
        return;
    }
    lastFrame = pGroundTruth->frame;

    obj_size = pGroundTruth->obstacleSize;
    for(int i=0; i< obj_size; i++)
    {
        SimOne_Data_Obstacle_Entry& obj = pGroundTruth->obstacle[i];
        obj_list[i].id = obj.id;

        // apollo/modules/perception/proto/perception_obstacle.proto
        switch(obj.type)
        {
            case ESimOne_Obstacle_Type_Truck:
            case ESimOne_Obstacle_Type_Bus:
            case ESimOne_Obstacle_Type_SpecialVehicle:
                obj_list[i].type = 5;
                break;
            case ESimOne_Obstacle_Type_Pedestrian:
                obj_list[i].type = 3;
                break;
            case ESimOne_Obstacle_Type_Bicycle:
                obj_list[i].type = 4;
                break;
            default:
                obj_list[i].type = 0;
        }

        obj_list[i].theta = obj.oriZ;
        obj_list[i].posX = obj.posX;
        obj_list[i].posY = obj.posY;
        obj_list[i].posZ = obj.posZ;
        obj_list[i].velX = obj.velX;
        obj_list[i].velY = obj.velY;
        obj_list[i].velZ = obj.velZ;
        obj_list[i].length = obj.length;
        obj_list[i].width = obj.width;
        obj_list[i].height = obj.height;
        obj_list[i].polygonPointSize = 0;
        obj_list[i].confidence = 1;
        obj_list[i].confidenceType = 1;
    }
}

void CW_Obstacles::task_obstacles()
{
    static uint64_t  frame = 0;
    mCyberObstacles.mObstacleSize = 0;
    memset(mCyberObstacles.mObstacleList, 0, sizeof(CyberWriterObstacles::Obstacle) * OBSTACLE_MAX_SIZE);

    set_obstacles(mCyberObstacles.mObstacleSize, mCyberObstacles.mObstacleList);
    mCyberObstacles.publish(frame);
}
