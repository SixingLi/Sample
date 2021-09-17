#!/usr/bin/env python3 
# -*- coding: utf-8 -*-
from SimOneServiceAPI import *
from SimOneSensorAPI import *
from SimOnePNCAPI import *
from SimOneV2XAPI import *
import json

# This is an example of get result from perception algorithms by python
# start = time.clock()
Flag = False
import time

SimOne_Data_TrafficLights_Test_Sync = SimOne_Data_TrafficLights()
SimOne_Data_Obstacle_Test = SimOne_Data_Obstacle()


def ServiceAPITest():
    case_info = SimOne_Data_CaseInfo()
    SimOne_Data_MainVehicle_Info_Test = SimOne_Data_MainVehicle_Info()
    SimOne_Data_MainVehicle_Status_Test = SimOne_Data_MainVehicle_Status()

    while (1):
        print("SoAPIGetVersion:%s" % SoAPIGetVersion())
        print("case run status:%s" % SoGetCaseRunStatus())
        if SoAPIGetCaseInfo(case_info):
            print("GetCaseInfo caseName: %s" % case_info.caseName)
            print("GetCaseInfo caseId: %s" % case_info.caseId)
            print("GetCaseInfo taskId: %s" % case_info.taskId)

        if SoGetMainVehicleList(SimOne_Data_MainVehicle_Info_Test):
            Vehicle_size = SimOne_Data_MainVehicle_Info_Test.size
            print("MainVehicle size: %s" % Vehicle_size)
            print("MainVehicle type: %s" % SimOne_Data_MainVehicle_Info_Test.type_list[0].value)
            print("MainVehicle id: %s" % SimOne_Data_MainVehicle_Info_Test.id_list[0])

        if SoGetMainVehicleStatus(SimOne_Data_MainVehicle_Status_Test):
            print("mainVehicleStatus:%s" % SimOne_Data_MainVehicle_Status_Test.mainVehicleStatus)
            mainVehicleStatus = SimOne_Data_MainVehicle_Status_Test.mainVehicleStatus
            if mainVehicleStatus > 0:
                break
    SoStopSimOneNode()


def SimOnePNCAPI():
    ESimOne_Data_Vehicle_State_Test = ESimOne_Data_Vehicle_State()
    SimOne_Data_Pose_Control_Test = SimOne_Data_Pose_Control()
    SimOne_Data_Control_Test = SimOne_Data_Control()
    SimOne_Data_Signal_Lights_Test = SimOne_Data_Signal_Lights()
    SimOne_Data_Driver_Status_Test = SimOne_Data_Driver_Status()
    SimOne_Data_WayPoints_Test = SimOne_Data_WayPoints()
    SimOne_Data_V2XNFS_Test = SimOne_Data_V2XNFS()

    # def ScenarioEventTest(0, ):
    # SoAPISetScenarioEventCB
    # SoAPISetScenarioEventCB(0, ScenarioEventTest())

    while(1):
        # SoRegisterVehicleState()
        # SoSetPose
        if SoGetGps('0', SimOne_Data_Gps_Test):
            gps_result = list()
            gps_result.append({"timestamp": SimOne_Data_Gps_Test.timestamp})
            gps_result.append({"frame": SimOne_Data_Gps_Test.frame})
            gps_result.append({"posX": SimOne_Data_Gps_Test.posX})
            gps_result.append({"posY": SimOne_Data_Gps_Test.posY})
            gps_result.append({"posZ": SimOne_Data_Gps_Test.posZ})
            gps_result.append({"oriX": SimOne_Data_Gps_Test.oriX})
            gps_result.append({"oriY": SimOne_Data_Gps_Test.oriZ})
            gps_result.append({"velX": SimOne_Data_Gps_Test.velX})
            gps_result.append({"velY": SimOne_Data_Gps_Test.velY})
            gps_result.append({"velZ": SimOne_Data_Gps_Test.velZ})
            gps_result.append({"throttle": SimOne_Data_Gps_Test.throttle})
            gps_result.append({"brake": SimOne_Data_Gps_Test.brake})
            gps_result.append({"steering": SimOne_Data_Gps_Test.steering})
            gps_result.append({"gear": SimOne_Data_Gps_Test.gear})
            gps_result.append({"accelX": SimOne_Data_Gps_Test.accelX})
            gps_result.append({"accelY": SimOne_Data_Gps_Test.accelY})
            gps_result.append({"accelZ": SimOne_Data_Gps_Test.accelZ})
            gps_result.append({"angVelX": SimOne_Data_Gps_Test.angVelX})
            gps_result.append({"angVelY": SimOne_Data_Gps_Test.angVelY})
            gps_result.append({"angVelZ": SimOne_Data_Gps_Test.angVelZ})
            gps_result.append({"wheelSpeedFL": SimOne_Data_Gps_Test.wheelSpeedFL})
            gps_result.append({"wheelSpeedFR": SimOne_Data_Gps_Test.wheelSpeedFR})
            gps_result.append({"wheelSpeedRL": SimOne_Data_Gps_Test.wheelSpeedRL})
            gps_result.append({"wheelSpeedRR": SimOne_Data_Gps_Test.wheelSpeedRR})
            print(json.dumps({"SoGetGps": gps_result}, ensure_ascii=False))

        SimOne_Data_Pose_Control_Test.posX = SimOne_Data_Gps_Test.posX + 10
        SimOne_Data_Pose_Control_Test.posY = SimOne_Data_Gps_Test.posY + 10
        SimOne_Data_Pose_Control_Test.posZ = SimOne_Data_Gps_Test.posZ

        # if SoSetPose("0", SimOne_Data_Pose_Control_Test):
        #     print("setPose ok")

        # SoSetDrive
        SimOne_Data_Control_Test.throttle = 1
        SimOne_Data_Control_Test.steering = 0.0
        SimOne_Data_Control_Test.brake = 0
        SimOne_Data_Control_Test.gear = ESimOne_Gear_Mode.ESimOne_Gear_Mode_Drive

        # # SoSetDriverName
        # if SoSetDriverName("0", "APItest"):
        #     print("SoSetDriverName ok")
        #
        # # SoSetDrive
        # if SoSetDrive("0", SimOne_Data_Control_Test):
        #     print("SoSetDrive ok")

        # #SoSetSignalLights()
        # SimOne_Data_Signal_Lights_Test.signalLights |= ESimOne_Signal_Light.ESimOne_Signal_Light_LeftBlinker
        # if SoSetSignalLights("0", SimOne_Data_Signal_Lights_Test):
        #     print("SoSetSignalLights ok")

        # SoGetDriverStatus()
        if SoGetDriverStatus("0", SimOne_Data_Driver_Status_Test):
            status = SimOne_Data_Driver_Status_Test.driverStatus.value
            print("SoGetDriverStatus status:%s----------------"%status)

        # # SoGetDriverControl
        # DriverStatusTestResult = SoGetDriverControl(0, SimOne_Data_Driver_Status_Test)
        # if DriverStatusTestResult:
        #     print("SoGetDriverControl ok")

        # # SoGetWayPoints
        # if SoGetWayPoints(SimOne_Data_WayPoints_Test):
        #     print("SoGetWayPoints wayPointsSize:%s" % SimOne_Data_WayPoints_Test.wayPointsSize)
        #     for i in range(SimOne_Data_WayPoints_Test.wayPointsSize):
        #         print(SimOne_Data_WayPoints_Test.wayPoints[i].index, SimOne_Data_WayPoints_Test.wayPoints[i].posX, SimOne_Data_WayPoints_Test.wayPoints[i].posY)

        # # SoGetV2XInfo
        # if SoGetV2XInfo(0, 10, 0, SimOne_Data_V2XNFS_Test):
        #     print("V2XMsgFrameSize%s:" %SimOne_Data_V2XNFS_Test.V2XMsgFrameSize)


def SimOneSensorAPI():
    SimOne_Data_SensorDetections_Test = SimOne_Data_SensorDetections()
    SimOne_Data_SensorConfigurations_Test = SimOne_Data_SensorConfigurations ()
    SimOne_Data_RadarDetection_Test = SimOne_Data_RadarDetection()
    SimOne_Data_UltrasonicRadars_Test = SimOne_Data_UltrasonicRadars()
    SimOne_Data_UltrasonicRadar_Test = SimOne_Data_UltrasonicRadar()
    SimOne_Data_Environment_Test = SimOne_Data_Environment()
    SimOne_Data_LaneInfo_Test = SimOne_Data_LaneInfo()

    SoGetSensorConfigurations(SimOne_Data_SensorConfigurations_Test)
    print("sensorConfiguration size:%s" % SimOne_Data_SensorConfigurations_Test.dataSize)

    for i in range(SimOne_Data_SensorConfigurations_Test.dataSize):
        sensor_type = bytes.decode(SimOne_Data_SensorConfigurations_Test.data[i].sensorType)
        print("sensor_type:%s" % sensor_type)
        print("sensor_id:%s" % SimOne_Data_SensorConfigurations_Test.data[i].id)
        print("sensor_mainVehicle:%s" %SimOne_Data_SensorConfigurations_Test.data[i].mainVehicle)
        print("sensor_hz:%s" % SimOne_Data_SensorConfigurations_Test.data[i].hz)
        print("sensor_x:%s" % SimOne_Data_SensorConfigurations_Test.data[i].x)
        print("sensor_y:%s" % SimOne_Data_SensorConfigurations_Test.data[i].y)

        if sensor_type == "objectBasedRadar" or sensor_type == "objectBasedRadar":  # "radar":  # ESimOneNode_MMWRadar
            RadarDetectionDataTest = list()
            if (SoGetRadarDetections("0", "objectBasedRadar1", SimOne_Data_RadarDetection_Test)):
                RadarDetectionDataTest.append({"detectNum": SimOne_Data_RadarDetection_Test.detectNum})
                print("detectNum:%s" %SimOne_Data_RadarDetection_Test.detectNum)
                for i in range(SimOne_Data_RadarDetection_Test.detectNum):
                    tmp = dict()
                    tmp.update({"id": SimOne_Data_RadarDetection_Test.detections[i].id})
                    tmp.update({"subId": SimOne_Data_RadarDetection_Test.detections[i].subId})
                    tmp.update({"type": SimOne_Data_RadarDetection_Test.detections[i].type.value})
                    tmp.update({"posX": SimOne_Data_RadarDetection_Test.detections[i].posX})
                    tmp.update({"posY": SimOne_Data_RadarDetection_Test.detections[i].posY})
                    tmp.update({"posZ": SimOne_Data_RadarDetection_Test.detections[i].posZ})
                    tmp.update({"velX": SimOne_Data_RadarDetection_Test.detections[i].velX})
                    tmp.update({"velY": SimOne_Data_RadarDetection_Test.detections[i].velY})
                    tmp.update({"velZ": SimOne_Data_RadarDetection_Test.detections[i].velZ})
                    tmp.update({"range": SimOne_Data_RadarDetection_Test.detections[i].range})
                    tmp.update({"rangeRate": SimOne_Data_RadarDetection_Test.detections[i].rangeRate})
                    tmp.update({"azimuth": SimOne_Data_RadarDetection_Test.detections[i].azimuth})
                    tmp.update({"vertical": SimOne_Data_RadarDetection_Test.detections[i].vertical})
                    tmp.update({"snrdb": SimOne_Data_RadarDetection_Test.detections[i].snrdb})
                    tmp.update({"rcsdb": SimOne_Data_RadarDetection_Test.detections[i].rcsdb})
                    tmp.update({"probability": SimOne_Data_RadarDetection_Test.detections[i].probability})
                    RadarDetectionDataTest.append(tmp)
                print(json.dumps({"SoGetRadarDetections objectBasedRadar": RadarDetectionDataTest}, ensure_ascii=False))
                # break
            else:
                print("SoGetRadarDetections objectBasedRadar error---------------")
        # if sensor_type == "radar":  # "radar":  # ESimOneNode_MMWRadar
        #     RadarDetectionDataTest = list()
        #     if (SoGetRadarDetections("0", "radar1", SimOne_Data_RadarDetection_Test)):
        #         RadarDetectionDataTest.append({"detectNum": SimOne_Data_RadarDetection_Test.detectNum})
        #         print("detectNum:%s" %SimOne_Data_RadarDetection_Test.detectNum)
        #         for i in range(SimOne_Data_RadarDetection_Test.detectNum):
        #             tmp = dict()
        #             tmp.update({"id": SimOne_Data_RadarDetection_Test.detections[i].id})
        #             tmp.update({"subId": SimOne_Data_RadarDetection_Test.detections[i].subId})
        #             tmp.update({"type": SimOne_Data_RadarDetection_Test.detections[i].type.value})
        #             tmp.update({"posX": SimOne_Data_RadarDetection_Test.detections[i].posX})
        #             tmp.update({"posY": SimOne_Data_RadarDetection_Test.detections[i].posY})
        #             tmp.update({"posZ": SimOne_Data_RadarDetection_Test.detections[i].posZ})
        #             tmp.update({"velX": SimOne_Data_RadarDetection_Test.detections[i].velX})
        #             tmp.update({"velY": SimOne_Data_RadarDetection_Test.detections[i].velY})
        #             tmp.update({"velZ": SimOne_Data_RadarDetection_Test.detections[i].velZ})
        #             tmp.update({"range": SimOne_Data_RadarDetection_Test.detections[i].range})
        #             tmp.update({"rangeRate": SimOne_Data_RadarDetection_Test.detections[i].rangeRate})
        #             tmp.update({"azimuth": SimOne_Data_RadarDetection_Test.detections[i].azimuth})
        #             tmp.update({"vertical": SimOne_Data_RadarDetection_Test.detections[i].vertical})
        #             tmp.update({"snrdb": SimOne_Data_RadarDetection_Test.detections[i].snrdb})
        #             tmp.update({"rcsdb": SimOne_Data_RadarDetection_Test.detections[i].rcsdb})
        #             tmp.update({"probability": SimOne_Data_RadarDetection_Test.detections[i].probability})
        #             RadarDetectionDataTest.append(tmp)
        #         print(json.dumps({"SoGetRadarDetections radar": RadarDetectionDataTest}, ensure_ascii=False))
        #         break
        #     else:
        #         print("SoGetRadarDetections radar error---------------")
        #
        if sensor_type == "ultrasonic":  # ESimOneNode_AllUltrasonicRadar
            UltrasonicsRadarDataTest = list()
            if (SoGetUltrasonicRadars("0", SimOne_Data_UltrasonicRadars_Test)):
                UltrasonicsRadarDataTest.append({"detectNum": SimOne_Data_UltrasonicRadars_Test.UltrasonicRadarsNum})
                for i in range(SimOne_Data_UltrasonicRadars_Test.UltrasonicRadarsNum):
                    tmp = dict()
                    tmp.update({"id": SimOne_Data_UltrasonicRadars_Test.detections[i].id})
                    tmp.update({"obstacleNum": SimOne_Data_UltrasonicRadars_Test.detections[i].obstacleNum})
                    for j in range(SimOne_Data_UltrasonicRadars_Test.detections[i].obstacleNum):
                        tmp.update(
                            {"obstacleRanges": SimOne_Data_UltrasonicRadars_Test.detections[i].obstacledetections[j].obstacleRanges})
                        tmp.update({"x": SimOne_Data_UltrasonicRadars_Test.detections[i].obstacledetections[j].x})
                    UltrasonicsRadarDataTest.append({'tmp': tmp})
                print(json.dumps({"SoGetUltrasonicRadars": UltrasonicsRadarDataTest}, ensure_ascii=False))

            if SoGetUltrasonicRadar("0", "ultrasonic2", SimOne_Data_UltrasonicRadar_Test):
                obstacleNum = SimOne_Data_UltrasonicRadar_Test.obstacleNum
                print("SoGetUltrasonicRadar obstacleNum:%s" % obstacleNum)
                for i in range(obstacleNum):
                    print("obstacleRanges:%s" %SimOne_Data_UltrasonicRadar_Test.obstacleDetections[i].obstacleRanges)
                    print("x:%s" %SimOne_Data_UltrasonicRadar_Test.obstacleDetections[i].x)
                    print("y:%s" % SimOne_Data_UltrasonicRadar_Test.obstacleDetections[i].y)

    # SoGetGroundTruth
    # if SoGetGroundTruth("0", SimOne_Data_Obstacle_Test):
    #     obstacle_result = list()
    #     obstacle_result.append({"timestamp": SimOne_Data_Obstacle_Test.timestamp})
    #     obstacle_result.append({"frame": SimOne_Data_Obstacle_Test.frame})
    #     obstacle_result.append({"obstacleSize": SimOne_Data_Obstacle_Test.obstacleSize})
    #     print(SimOne_Data_Obstacle_Test.obstacleSize)
    #     for i in range(SimOne_Data_Obstacle_Test.obstacleSize):
    #         tmp = dict()
    #         tmp.update({"type": SimOne_Data_Obstacle_Test.obstacle[i].type.value})
    #         tmp.update({"id": SimOne_Data_Obstacle_Test.obstacle[i].id})
    #         tmp.update({"viewId": SimOne_Data_Obstacle_Test.obstacle[i].viewId})
    #         tmp.update({"theta": SimOne_Data_Obstacle_Test.obstacle[i].theta})
    #         tmp.update({"posX": SimOne_Data_Obstacle_Test.obstacle[i].posX})
    #         tmp.update({"posY": SimOne_Data_Obstacle_Test.obstacle[i].posY})
    #         tmp.update({"posZ": SimOne_Data_Obstacle_Test.obstacle[i].posZ})
    #         tmp.update({"oriZ": SimOne_Data_Obstacle_Test.obstacle[i].oriZ})
    #         tmp.update({"velX": SimOne_Data_Obstacle_Test.obstacle[i].velX})
    #         tmp.update({"velY": SimOne_Data_Obstacle_Test.obstacle[i].velY})
    #         tmp.update({"velZ": SimOne_Data_Obstacle_Test.obstacle[i].velZ})
    #         tmp.update({"length": SimOne_Data_Obstacle_Test.obstacle[i].length})
    #         tmp.update({"width": SimOne_Data_Obstacle_Test.obstacle[i].width})
    #         tmp.update({"height": SimOne_Data_Obstacle_Test.obstacle[i].height})
    #         obstacle_result.append(tmp)
    #     print(json.dumps({"SoGetGroundTruth": obstacle_result}, ensure_ascii=False))
    #
    # # SoGetSensorDetections
    # if SoGetSensorDetections(0, 1, SimOne_Data_SensorDetections_Test):
    #     sensorDetections_result = list()
    #     sensorDetections_result.append({"timestamp": SimOne_Data_SensorDetections_Test.timestamp})
    #     sensorDetections_result.append({"frame": SimOne_Data_SensorDetections_Test.frame})
    #     sensorDetections_result.append({"objectSize": SimOne_Data_SensorDetections_Test.objectSize})
    #     for i in range(0, SimOne_Data_SensorDetections_Test.objectSize):
    #         tmp = dict()
    #         tmp.update({"type": SimOne_Data_SensorDetections_Test.objects[i].type})
    #         tmp.update({"probability": SimOne_Data_SensorDetections_Test.objects[i].probability})
    #         tmp.update({"id": SimOne_Data_SensorDetections_Test.objects[i].id})
    #         tmp.update({"posX": SimOne_Data_SensorDetections_Test.objects[i].posX})
    #         tmp.update({"posY": SimOne_Data_SensorDetections_Test.objects[i].posY})
    #         tmp.update({"posZ": SimOne_Data_SensorDetections_Test.objects[i].posZ})
    #         tmp.update({"velX": SimOne_Data_SensorDetections_Test.objects[i].velX})
    #         tmp.update({"velY": SimOne_Data_SensorDetections_Test.objects[i].velY})
    #         tmp.update({"velZ": SimOne_Data_SensorDetections_Test.objects[i].velZ})
    #         tmp.update({"length": SimOne_Data_SensorDetections_Test.objects[i].length})
    #         tmp.update({"width": SimOne_Data_SensorDetections_Test.objects[i].width})
    #         tmp.update({"height": SimOne_Data_SensorDetections_Test.objects[i].height})
    #         tmp.update({"probability": SimOne_Data_SensorDetections_Test.objects[i].probability})
    #         tmp.update({"relativePosX": SimOne_Data_SensorDetections_Test.objects[i].relativePosX})
    #         tmp.update({"relativePosY": SimOne_Data_SensorDetections_Test.objects[i].relativePosY})
    #         tmp.update({"relativePosZ": SimOne_Data_SensorDetections_Test.objects[i].relativePosZ})
    #         tmp.update({"relativeVelX": SimOne_Data_SensorDetections_Test.objects[i].relativeVelX})
    #         tmp.update({"relativeVelY": SimOne_Data_SensorDetections_Test.objects[i].relativeVelY})
    #         tmp.update({"relativeVelZ": SimOne_Data_SensorDetections_Test.objects[i].relativeVelZ})
    #         sensorDetections_result.append(tmp)
    #     print(json.dumps({"SimOne_Data_SensorDetections": sensorDetections_result}, ensure_ascii=False))
    #
    # setEnvironment
    # getEnvironmentJson = dict()
    # if SoGetEnvironment(SimOne_Data_Environment_Test):
    #     getEnvironmentJson.update({'timeOfDay': SimOne_Data_Environment_Test.timeOfDay})
    #     getEnvironmentJson.update({'directionalLight': SimOne_Data_Environment_Test.directionalLight})
    #     getEnvironmentJson.update({'artificialLight': SimOne_Data_Environment_Test.artificialLight})
    #     getEnvironmentJson.update({'ambientLight': SimOne_Data_Environment_Test.ambientLight})
    #     getEnvironmentJson.update({'heightAngle': SimOne_Data_Environment_Test.heightAngle})
    #     getEnvironmentJson.update({'cloudDensity': SimOne_Data_Environment_Test.cloudDensity})
    #     getEnvironmentJson.update({'rainDensity': SimOne_Data_Environment_Test.rainDensity})
    #     getEnvironmentJson.update({'snowDensity': SimOne_Data_Environment_Test.snowDensity})
    #     getEnvironmentJson.update({'groundHumidityLevel': SimOne_Data_Environment_Test.groundHumidityLevel})
    #     getEnvironmentJson.update({'groundDirtyLevel': SimOne_Data_Environment_Test.groundDirtyLevel})
    # else:
    #     print("SoGetEnvironment Error")
    # print(json.dumps(getEnvironmentJson, ensure_ascii=False))
    #
    # SimOne_Data_Environment_Test.timeOfDay = 2300
    # SimOne_Data_Environment_Test.snowDensity = 1.0
    # SoSetEnvironment(SimOne_Data_Environment_Test)
    #
    # time.sleep(3)
    # # SoGetEnvironment
    # getEnvironmentJson = dict()
    # if SoGetEnvironment(SimOne_Data_Environment_Test):
    #     getEnvironmentJson.update({'timeOfDay': SimOne_Data_Environment_Test.timeOfDay})
    #     getEnvironmentJson.update({'directionalLight': SimOne_Data_Environment_Test.directionalLight})
    #     getEnvironmentJson.update({'artificialLight': SimOne_Data_Environment_Test.artificialLight})
    #     getEnvironmentJson.update({'ambientLight': SimOne_Data_Environment_Test.ambientLight})
    #     getEnvironmentJson.update({'heightAngle': SimOne_Data_Environment_Test.heightAngle})
    #     getEnvironmentJson.update({'cloudDensity': SimOne_Data_Environment_Test.cloudDensity})
    #     getEnvironmentJson.update({'rainDensity': SimOne_Data_Environment_Test.rainDensity})
    #     getEnvironmentJson.update({'snowDensity': SimOne_Data_Environment_Test.snowDensity})
    #     getEnvironmentJson.update({'groundHumidityLevel': SimOne_Data_Environment_Test.groundHumidityLevel})
    #     getEnvironmentJson.update({'groundDirtyLevel': SimOne_Data_Environment_Test.groundDirtyLevel})
    # else:
    #     print("SoGetEnvironment Error")
    # print(json.dumps(getEnvironmentJson, ensure_ascii=False))
    #
    #     if SoGetSensorLaneInfo(0, "sensorId", SimOne_Data_LaneInfo_Test):
    #         print("GetSensorLaneInfo laneType:%s"%SimOne_Data_LaneInfo_Test.laneType)
    #         print("GetSensorLaneInfo laneLeftID:%s" % SimOne_Data_LaneInfo_Test.laneLeftID)
    #         print("GetSensorLaneInfo laneRightID:%s" % SimOne_Data_LaneInfo_Test.laneRightID)
    #         print("GetSensorLaneInfo lanePredecessorID:%s" % SimOne_Data_LaneInfo_Test.lanePredecessorID)
    #         print("GetSensorLaneInfo l_Line.lineID:%s" % SimOne_Data_LaneInfo_Test.l_Line.lineID)
    #         print("GetSensorLaneInfo l_Line.lineType:%s" % SimOne_Data_LaneInfo_Test.l_Line.lineType)
    #         print("GetSensorLaneInfo l_Line.lineColor:%s" % SimOne_Data_LaneInfo_Test.l_Line.lineColor)

    # # SoAPIGetSimOneTrafficLight
    # if SoAPIGetSimOneTrafficLight(SimOne_Data_TrafficLights_Test_Sync):
    #     traffic_result = list()
    #     traffic_result.append({"timestamp": SimOne_Data_TrafficLights_Test_Sync.timestamp})
    #     traffic_result.append({"frame": SimOne_Data_TrafficLights_Test_Sync.frame})
    #     traffic_result.append({"trafficlightNum": SimOne_Data_TrafficLights_Test_Sync.trafficlightNum})
    #     for i in range(SimOne_Data_TrafficLights_Test_Sync.trafficlightNum):
    #         tmp = dict()
    #         tmp.update({"index": SimOne_Data_TrafficLights_Test_Sync.trafficlights[i].index})
    #         tmp.update({"opendriveLightId": SimOne_Data_TrafficLights_Test_Sync.trafficlights[i].opendriveLightId})
    #         tmp.update({"countDown": SimOne_Data_TrafficLights_Test_Sync.trafficlights[i].countDown})
    #         tmp.update({"status": SimOne_Data_TrafficLights_Test_Sync.trafficlights[i].status.value})
    #         traffic_result.append(tmp)
    #     print(json.dumps({"SoAPIGetSimOneTrafficLight": traffic_result}, ensure_ascii=False))


def Data_Gps_Test(mainVehicleId, data_gps):
    print("jinlaile")
    gps_result = list()
    if data_gps:
        gps_result.append({"timestamp": data_gps[0].timestamp})
        gps_result.append({"frame": data_gps[0].frame})
        gps_result.append({"posX": data_gps[0].posX})
        gps_result.append({"posY": data_gps[0].posY})
        gps_result.append({"posZ": data_gps[0].posZ})
        gps_result.append({"oriX": data_gps[0].oriX})
        gps_result.append({"oriY": data_gps[0].oriY})
        gps_result.append({"oriZ": data_gps[0].oriZ})
        gps_result.append({"velX": data_gps[0].velX})
        gps_result.append({"velY": data_gps[0].velY})
        gps_result.append({"velZ": data_gps[0].velZ})
        gps_result.append({"throttle": data_gps[0].throttle})
        gps_result.append({"brake": data_gps[0].brake})
        gps_result.append({"steering": data_gps[0].steering})
        gps_result.append({"gear": data_gps[0].gear})
        gps_result.append({"accelX": data_gps[0].accelX})
        gps_result.append({"accelY": data_gps[0].accelY})
        gps_result.append({"accelZ": data_gps[0].accelZ})
        gps_result.append({"angVelX": data_gps[0].angVelX})
        gps_result.append({"angVelY": data_gps[0].angVelY})
        gps_result.append({"angVelZ": data_gps[0].angVelZ})
        gps_result.append({"wheelSpeedFL": data_gps[0].wheelSpeedFL})
        gps_result.append({"wheelSpeedFR": data_gps[0].wheelSpeedFR})
        gps_result.append({"wheelSpeedRL": data_gps[0].wheelSpeedRL})
        gps_result.append({"wheelSpeedRR": data_gps[0].wheelSpeedRR})
    print(json.dumps({"SoAPISetSimOneGpsCB": gps_result}, ensure_ascii=False))


def SoApiSetSensorDetectionsUpdateCBTest(mainVehicleId, sensorId, data):
    obstacle_result = list()
    if data:
        obstacle_result.append({"timestamp": data[0].timestamp})
        obstacle_result.append({"frame": data[0].frame})
        obstacle_result.append({"obstacleSize": data[0].objectSize})
        print(data[0].objectSize)
        for i in range(data[0].objectSize):
            tmp = dict()
            # obstacle_result.update({"===":"=================================="})
            tmp.update({"id": data[0].objects[i].id})
            tmp.update({"type": data[0].objects[i].type})
            tmp.update({"posX": data[0].objects[i].posX})
            tmp.update({"posY": data[0].objects[i].posY})
            tmp.update({"posZ": data[0].objects[i].posZ})
            tmp.update({"velX": data[0].objects[i].velX})
            tmp.update({"velY": data[0].objects[i].velY})
            tmp.update({"velZ": data[0].objects[i].velZ})
            tmp.update({"oriX": data[0].objects[i].oriX})
            tmp.update({"oriY": data[0].objects[i].oriY})
            tmp.update({"oriZ": data[0].objects[i].oriZ})
            tmp.update({"length": data[0].objects[i].length})
            tmp.update({"width": data[0].objects[i].width})
            tmp.update({"height": data[0].objects[i].height})
            tmp.update({"probability": data[0].objects[i].probability})
            tmp.update({"relativePosX": data[0].objects[i].relativePosX})
            tmp.update({"relativePosY": data[0].objects[i].relativePosY})
            tmp.update({"relativePosZ": data[0].objects[i].relativePosZ})
            tmp.update({"relativeVelX": data[0].objects[i].relativeVelX})
            tmp.update({"relativeVelY": data[0].objects[i].relativeVelY})
            tmp.update({"relativePosZ": data[0].objects[i].relativeVelZ})
            obstacle_result.append(tmp)
    print(json.dumps({"SoApiSetSensorDetectionsUpdateCB": obstacle_result}, ensure_ascii=False))


def SoApiSetRadarDetectionsUpdateCBTest(mainVehicleId, sensorId, data):
    if data:
        RadarDetectionDataTest = list()
        RadarDetectionDataTest.append({"detectNum": data[0].detectNum})
        for i in range(data[0].detectNum):
            tmp = dict()
            tmp.update({"id": data[0].detections[i].id})
            tmp.update({"subId": data[0].detections[i].subId})
            tmp.update({"type": data[0].detections[i].type.value})
            tmp.update({"posX": data[0].detections[i].posX})
            tmp.update({"posY": data[0].detections[i].posY})
            tmp.update({"posZ": data[0].detections[i].posZ})
            tmp.update({"velX": data[0].detections[i].velX})
            tmp.update({"velY": data[0].detections[i].velY})
            tmp.update({"velZ": data[0].detections[i].velZ})
            tmp.update({"range": data[0].detections[i].range})
            tmp.update({"rangeRate": data[0].detections[i].rangeRate})
            tmp.update({"azimuth": data[0].detections[i].azimuth})
            tmp.update({"vertical": data[0].detections[i].vertical})
            tmp.update({"snrdb": data[0].detections[i].snrdb})
            tmp.update({"rcsdb": data[0].detections[i].rcsdb})
            tmp.update({"probability": data[0].detections[i].probability})
            RadarDetectionDataTest.append(tmp)
        print(json.dumps({"SoApiSetRadarDetectionsUpdateCB": RadarDetectionDataTest}, ensure_ascii=False))


def SoApiSetUltrasonicRadarsCBTest(mainVehicleId, data):
    print("------------------------------wwwww----------------------------------------------")
    if data:
        ultrasonicRadarNum = data[0].ultrasonicRadarNum
        print("SoApiSetUltrasonicRadarsCB ultrasonicRadarNum:%s" % ultrasonicRadarNum)
        for i in range(ultrasonicRadarNum):
            sensorId = bytes.decode(data[0].ultrasonicRadars[i].sensorId)
            print("SoApiSetUltrasonicRadarsCB number is %s, sensorId:%s" % (i, sensorId))
            obstacleNum = data[0].ultrasonicRadars[i].obstacleNum

            print("SoApiSetUltrasonicRadarsCB sensorId is %s obstacleNum:%s" %(sensorId, obstacleNum))

            for j in range(obstacleNum):
                print("SoApiSetUltrasonicRadarsCB obstacleDetections obstacleRanges:%s" % data[0].ultrasonicRadars[i].obstacleDetections[j].obstacleRanges)
                print("SoApiSetUltrasonicRadarsCB obstacleDetections obstacleRanges x:%s" % data[0].ultrasonicRadars[i].obstacleDetections[j].x)
                print("SoApiSetUltrasonicRadarsCB obstacleDetections obstacleRanges y:%s" % data[0].ultrasonicRadars[i].obstacleDetections[j].y)


def Data_Obstacle_Test(mainVehicleId, data_obstacle):
    obstacle_result = list()
    if data_obstacle:
        obstacle_result.append({"timestamp": data_obstacle[0].timestamp})
        obstacle_result.append({"frame": data_obstacle[0].frame})
        obstacle_result.append({"obstacleSize": data_obstacle[0].obstacleSize})
        for i in range(data_obstacle[0].obstacleSize):
            tmp = dict()
            # obstacle_result.update({"===":"=================================="})
            tmp.update({"id": data_obstacle[0].obstacle[i].type.id})
            tmp.update({"viewId": data_obstacle[0].obstacle[i].type.viewId})
            tmp.update({"type": data_obstacle[0].obstacle[i].type.value})
            tmp.update({"theta": data_obstacle[0].obstacle[i].theta})
            tmp.update({"posX": data_obstacle[0].obstacle[i].posX})
            tmp.update({"posY": data_obstacle[0].obstacle[i].posY})
            tmp.update({"posZ": data_obstacle[0].obstacle[i].posZ})
            tmp.update({"oriX": data_obstacle[0].obstacle[i].oriX})
            tmp.update({"oriY": data_obstacle[0].obstacle[i].oriY})
            tmp.update({"oriZ": data_obstacle[0].obstacle[i].oriZ})
            tmp.update({"velX": data_obstacle[0].obstacle[i].velX})
            tmp.update({"velY": data_obstacle[0].obstacle[i].velY})
            tmp.update({"velZ": data_obstacle[0].obstacle[i].velZ})
            tmp.update({"length": data_obstacle[0].obstacle[i].length})
            tmp.update({"width": data_obstacle[0].obstacle[i].width})
            tmp.update({"height": data_obstacle[0].obstacle[i].height})
            obstacle_result.append(tmp)
    print(json.dumps({"SoAPISetSimOneObstacleCB":obstacle_result}, ensure_ascii=False))


def SoApiSetStreamingImageCBTest(data):
    if data:
        print("SoApiSetStreamingImageCB imagedataSize:%s" % data[0].imagedataSize)
        print("SoApiSetStreamingImageCB width:%s" % data[0].width)
        print("SoApiSetStreamingImageCB height:%s" % data[0].height)


def SoApiSetStreamingPointCloudUpdateCBTest(data):
    if data:
        print("SoApiSetStreamingPointCloudUpdateCB pointCloudDataSize:%s" % data[0].pointCloudDataSize)
        print("SoApiSetStreamingPointCloudUpdateCB width:%s" % data[0].width)
        print("SoApiSetStreamingPointCloudUpdateCB height:%s" % data[0].height)
        print("SoApiSetStreamingPointCloudUpdateCB pointStep:%s" % data[0].pointStep)


def SimOneSensorAPICallBack():
    # SoAPISetFrameCB(0, 0)
    print("calle back")
    # SoApiSetGpsUpdateCB(Data_Gps_Test)

    # SoApiSetSensorDetectionsUpdateCB(SoApiSetSensorDetectionsUpdateCBTest)
    # SoApiSetRadarDetectionsUpdateCB(SoApiSetRadarDetectionsUpdateCBTest)
    SoApiSetUltrasonicRadarsCB(SoApiSetUltrasonicRadarsCBTest)
    SoApiSetGroundTruthUpdateCB(Data_Obstacle_Test)

    # SoApiSetStreamingPointCloudUpdateCB("127.0.0.1", 6699, 7788, SoApiSetStreamingPointCloudUpdateCBTest)
    # SoApiSetStreamingImageCB("127.0.0.1", 7890, SoApiSetStreamingImageCBTest)
    while (1):
        pass


if __name__ == '__main__':
    SimOne_Data_Gps_Test = SimOne_Data_Gps()
    while(1):
        try:
            if SoInitSimOneAPI(isFrameSync=0) == 1:
                print("################## API init success!!!")
                Flag = True
            else:
                print("################## API init fail!!!")
        except Exception as e:
            print(e)
            pass

        case_status = SoGetCaseRunStatus()
        print("case status:%s" % case_status)
        if case_status == 2:
            while(1):
                # SoGetGps
                # frame = SoAPIWait()
                # print("frame:%s"% frame)
                time.sleep(0.5)
                if SoGetGps('0', SimOne_Data_Gps_Test):
                    gps_result = list()
                    gps_result.append({"timestamp": SimOne_Data_Gps_Test.timestamp})
                    gps_result.append({"frame": SimOne_Data_Gps_Test.frame})
                    gps_result.append({"posX": SimOne_Data_Gps_Test.posX})
                    gps_result.append({"posY": SimOne_Data_Gps_Test.posY})
                    gps_result.append({"posZ": SimOne_Data_Gps_Test.posZ})
                    gps_result.append({"oriX": SimOne_Data_Gps_Test.oriX})
                    gps_result.append({"oriY": SimOne_Data_Gps_Test.oriZ})
                    gps_result.append({"velX": SimOne_Data_Gps_Test.velX})
                    gps_result.append({"velY": SimOne_Data_Gps_Test.velY})
                    gps_result.append({"velZ": SimOne_Data_Gps_Test.velZ})
                    gps_result.append({"throttle": SimOne_Data_Gps_Test.throttle})
                    gps_result.append({"brake": SimOne_Data_Gps_Test.brake})
                    gps_result.append({"steering": SimOne_Data_Gps_Test.steering})
                    gps_result.append({"gear": SimOne_Data_Gps_Test.gear})
                    gps_result.append({"accelX": SimOne_Data_Gps_Test.accelX})
                    gps_result.append({"accelY": SimOne_Data_Gps_Test.accelY})
                    gps_result.append({"accelZ": SimOne_Data_Gps_Test.accelZ})
                    gps_result.append({"angVelX": SimOne_Data_Gps_Test.angVelX})
                    gps_result.append({"angVelY": SimOne_Data_Gps_Test.angVelY})
                    gps_result.append({"angVelZ": SimOne_Data_Gps_Test.angVelZ})
                    gps_result.append({"wheelSpeedFL": SimOne_Data_Gps_Test.wheelSpeedFL})
                    gps_result.append({"wheelSpeedFR": SimOne_Data_Gps_Test.wheelSpeedFR})
                    gps_result.append({"wheelSpeedRL": SimOne_Data_Gps_Test.wheelSpeedRL})
                    gps_result.append({"wheelSpeedRR": SimOne_Data_Gps_Test.wheelSpeedRR})
                    print(json.dumps({"SoGetGps": gps_result}, ensure_ascii=False))
                    break
                else:
                    print("waiting simone started")
                # SoAPINextFrame(frame)
                # break

            ServiceAPITest()
            SimOnePNCAPI()
            SimOneSensorAPI()
            SimOneSensorAPICallBack()
        else:
            break





