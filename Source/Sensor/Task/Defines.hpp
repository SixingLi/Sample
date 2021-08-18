// 
// Created By: Qilin.Ren 2018/06/06
// 
#pragma once
#ifndef WITHOUT_SENSOR
#include "cybertron/core/Log.hpp"
#include "cybertron/network/SocketTcpServer.hpp"
#include "cybertron/network/SocketTcpClient.hpp"
#include "cybertron/network/SocketEventHandler.hpp"
#include "cybertron/network/Message.hpp"
#include "cybertron/network/MessageRouter.hpp"
#include "cybertron/node/StartupArguments.hpp"
#include "Node/Coordinator.pb.h"
#include "Node/Timer.pb.h"
#include "Node/HotArea.pb.h"
#include "Node/NodeCommon.pb.h"
#include "Node/Bridge.pb.h"
#include "Traffic/TrafficCommon.pb.h"
#include "Sensor/Vehicle.pb.h"
#include "Sensor/SensorCommon.pb.h"
#include "cybertron/core/ConcurrentQueue.hpp"
using namespace cybertron;
//class EventHandlerSimulationNode;
#endif // !WITHOUT_SENSOR



