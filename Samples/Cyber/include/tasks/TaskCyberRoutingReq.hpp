#ifndef TASKCYBERROUTINGREQ_HPP_
#define TASKCYBERROUTINGREQ_HPP_

#include <iostream>
#include <string.h>
#include <math.h>
#include "CyberWriterRoutingReq.hpp"
#include "SimOnePNCAPI.h"

#include "SimOneSensorAPI.h"

class CW_RoutingReq
{
public:
    void set_routing_req(unsigned &point_size, CyberWriterRoutingReq::WayPoint *point_list);
    void task_routing_req();

private:
    CyberWriterRoutingReq mCyberRoutingReq;
};

#endif
