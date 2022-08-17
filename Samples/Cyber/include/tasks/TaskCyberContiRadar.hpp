#ifndef TASKCYBERCONTIRADAR_HPP_
#define TASKCYBERCONTIRADAR_HPP_

#include <iostream>
#include <memory>
#include <math.h>
#include <string.h>
#include "CyberWriterContiRadar.hpp"
#include "SimOneSensorAPI.h"

class CW_ContiRadar
{
public:
    void set_conti_radar(int& obj_size, CyberWriterContiRadar::Object *obj_list);
    void task_conti_radar();

private:
    CyberWriterContiRadar mCyberContiRadar;
};

#endif