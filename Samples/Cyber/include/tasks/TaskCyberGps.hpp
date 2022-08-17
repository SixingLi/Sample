#ifndef TASKCYBERGPS_HPP_
#define TASKCYBERGPS_HPP_

#include "CyberWriterGps.hpp"
#include "CyberWriterInsStat.hpp"
#include "SimOneSensorAPI.h"
#include "glm/quat.hpp"
#include "glm/vec3.hpp"
#include <iostream>
#include <math.h>

typedef struct
{
    float posX;
    float posY;
    float posZ;
    float mVelocityX;
    float mVelocityY;
    float mVelocityZ;
    float oriX;
    float oriY;
    float oriZ;
    float oriW;
} Gps_T;

class CW_Gps
{
    public:
        void set_gps(Gps_T& data);
        void task_gps();
    private:
	    CyberWriterGps mCyberGps;
	    CyberWriterInsStat mCyberInsStat;
};

#endif
