#ifndef TASKCYBEROBSTACLES_HPP_
#define TASKCYBEROBSTACLES_HPP_

#include <iostream>
#include <string.h>
#include "CyberWriterObstacles.hpp"
#include "SimOneSensorAPI.h"

class CW_Obstacles
{
    public:
        void set_obstacles(int& obj_size, CyberWriterObstacles::Obstacle* obj_list);
        void task_obstacles();
    private:
	    CyberWriterObstacles mCyberObstacles;
};

#endif
