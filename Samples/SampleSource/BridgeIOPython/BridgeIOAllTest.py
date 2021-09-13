#!/usr/bin/env python3 
# -*- coding: utf-8 -*-
from SimOneIOStruct import *
from osi3.osi_featuredata_pb2 import FeatureData, UltrasonicDetection
from osi3.osi_sensorview_pb2 import SensorView
from osi3.osi_sensordata_pb2 import SensorData
from osi3.osi_groundtruth_pb2 import GroundTruth
import time
import json
import numpy as np
import ctypes
import base64
import struct


# This is an example of get result from perception algorithms by python
case_info = SimOne_Data_CaseInfo()
osi_sensor_data = SensorData()
ground_truth = dict()
osi_groundtruth = SimOne_Data_OSI()
sensor_groundtruth = GroundTruth()
# start = time.clock()

SimOne_Data_Gps_Test_Sync = SimOne_Data_Gps()
SimOne_Data_MainVehicle_Info_Test = SimOne_Data_MainVehicle_Info()
SimOne_Data_MainVehicle_Status_Test = SimOne_Data_MainVehicle_Status()
SimOne_Data_TrafficLights_Test_Sync = SimOne_Data_TrafficLights()

sensorConfigurations = SimOneSensorConfigurations()
imageData = SimOne_Data_Image()
pointCloudData = SimOne_Data_Point_Cloud()
RadarDetectionData = SimOne_Data_RadarDetection()
UltrasonicsRadarData = SimOne_Data_UltrasonicRadars()
EnvironmentData = SimOne_Data_Environment()
# SensorDetectionsData = SimOne_Data_SensorDetections()

SimOne_Data_ObstacleTest_Sync = SimOne_Data_Obstacle()


def start_case():
    pass


def stop_case():
    pass


def Data_Gps_Test(mainVehicleId, data_gps):
    gps_result = list()
    if data_gps:
        gps_result.append({"timestamp": data_gps[0].timestamp})
        gps_result.append({"frame": data_gps[0].frame})
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


def SimOne_Data_Gps_Test(data_gps):
    gps_result = list()
    if data_gps:
        gps_result.append({"timestamp": data_gps[0].timestamp})
        gps_result.append({"frame": data_gps[0].frame})
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


def SimOne_Data_TrafficLights_Test(data_traffic):
    traffic_result = list()

    if data_traffic:
        traffic_result.append({"timestamp": data_traffic[0].timestamp})
        traffic_result.append({"frame": data_traffic[0].frame})
        traffic_result.append({"trafficlightNum": data_traffic[0].trafficlightNum})
        for i in range(data_traffic[0].trafficlightNum):
            tmp = dict()
            tmp.update({"index": data_traffic[0].trafficlights[i].index})
            tmp.update({"opendriveLightId": data_traffic[0].trafficlights[i].opendriveLightId})
            tmp.update({"countDown": data_traffic[0].trafficlights[i].countDown})
            tmp.update({"status": data_traffic[0].trafficlights[i].status.value})
            traffic_result.append(tmp)
    print(json.dumps({"SoAPISetSimOneTrafficLightCB": traffic_result}, ensure_ascii=False))


def SimOne_Data_Obstacle_Test(data_obstacle):
    obstacle_result = list()
    if data_obstacle:
        obstacle_result.append({"timestamp": data_obstacle[0].timestamp})
        obstacle_result.append({"frame": data_obstacle[0].frame})
        obstacle_result.append({"obstacleSize": data_obstacle[0].obstacleSize})
        for i in range(data_obstacle[0].obstacleSize):
            tmp = dict()
            # obstacle_result.update({"===":"=================================="})
            tmp.update({"type": data_obstacle[0].obstacle[i].type.value})
            tmp.update({"theta": data_obstacle[0].obstacle[i].theta})
            tmp.update({"posX": data_obstacle[0].obstacle[i].posX})
            tmp.update({"posY": data_obstacle[0].obstacle[i].posY})
            tmp.update({"posZ": data_obstacle[0].obstacle[i].posZ})
            tmp.update({"velX": data_obstacle[0].obstacle[i].velX})
            tmp.update({"velY": data_obstacle[0].obstacle[i].velY})
            tmp.update({"velZ": data_obstacle[0].obstacle[i].velZ})
            tmp.update({"length": data_obstacle[0].obstacle[i].length})
            tmp.update({"width": data_obstacle[0].obstacle[i].width})
            tmp.update({"height": data_obstacle[0].obstacle[i].height})
            obstacle_result.append(tmp)

    print(json.dumps({"SoAPISetSimOneObstacleCB":obstacle_result}, ensure_ascii=False))

def Data_Obstacle_Test(mainVehicleId, data_obstacle):
    obstacle_result = list()
    if data_obstacle:
        obstacle_result.append({"timestamp": data_obstacle[0].timestamp})
        obstacle_result.append({"frame": data_obstacle[0].frame})
        obstacle_result.append({"obstacleSize": data_obstacle[0].obstacleSize})
        for i in range(data_obstacle[0].obstacleSize):
            tmp = dict()
            # obstacle_result.update({"===":"=================================="})
            tmp.update({"type": data_obstacle[0].obstacle[i].type.value})
            tmp.update({"theta": data_obstacle[0].obstacle[i].theta})
            tmp.update({"posX": data_obstacle[0].obstacle[i].posX})
            tmp.update({"posY": data_obstacle[0].obstacle[i].posY})
            tmp.update({"posZ": data_obstacle[0].obstacle[i].posZ})
            tmp.update({"velX": data_obstacle[0].obstacle[i].velX})
            tmp.update({"velY": data_obstacle[0].obstacle[i].velY})
            tmp.update({"velZ": data_obstacle[0].obstacle[i].velZ})
            tmp.update({"length": data_obstacle[0].obstacle[i].length})
            tmp.update({"width": data_obstacle[0].obstacle[i].width})
            tmp.update({"height": data_obstacle[0].obstacle[i].height})
            obstacle_result.append(tmp)

    print(json.dumps({"SoAPISetSimOneObstacleCB":obstacle_result}, ensure_ascii=False))


def SetImageTest(mainVehicleId, sensorId, Data_Image):
    if Data_Image:
        print("SoApiSetImageUpdateCB imagedataSize:%s" % Data_Image[0].imagedataSize)
        print("SoApiSetImageUpdateCB width:%s" % Data_Image[0].width)
        print("SoApiSetImageUpdateCB height:%s" % Data_Image[0].height)


def SoApiSetPointCloudUpdateCBTest(mainVehicleId, sensorId, Data_Point):
    if Data_Point:
        print("SoApiSetPointCloudUpdateCB pointCloudDataSize:%s" % Data_Point[0].pointCloudDataSize)
        print("SoApiSetPointCloudUpdateCB width:%s" % Data_Point[0].width)
        print("SoApiSetPointCloudUpdateCB height:%s" % Data_Point[0].height)
        print("SoApiSetPointCloudUpdateCB pointStep:%s" % Data_Point[0].pointStep)


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
            tmp.update({"type": data[0].detections[i].type})
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
    if data:
        print("SoApiSetUltrasonicRadarsCB:%s" % data[0].UltrasonicRadarsNum)


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


def SMAPIStart():
    SoApiStart()
    while (1):
        if SoAPIGetMainVehicleStatus(SimOne_Data_MainVehicle_Status_Test):
            print("mainVehicleId:%s" % SimOne_Data_MainVehicle_Status_Test.mainVehicleId)
            mainVehicleStatus = SimOne_Data_MainVehicle_Status_Test.mainVehicleStatus
            if mainVehicleStatus > 0:
                break


def NewAPIAllStart(isJoinTimeLoop):
    SoAPIStartSimOneNode(0, 0)
    SoAPISimOneNodeReady()
    if SoAPIGetCaseInfo(case_info):
        print("GetCaseInfo caseName: %s" % case_info.caseName)
        print("GetCaseInfo caseId: %s" % case_info.caseId)
        print("GetCaseInfo taskId: %s" % case_info.taskId)
        print("GetCaseInfo sessionId: %s" % case_info.sessionId)

    print("SoAPIGetCaseRunStatus: %s" % SoAPIGetCaseRunStatus())

    version = SoAPIGetVersion()
    print("version:%s" %version)

    if SoAPIGetMainVehicleList(SimOne_Data_MainVehicle_Info_Test):
        print("MainVehicle size: %s" % SimOne_Data_MainVehicle_Info_Test.size)

    while (1):
        SoAPISubMainVehicle_result = SoAPISubMainVehicle(0, isJoinTimeLoop)
        print("SoAPISubMainVehicle_result:%s" % SoAPISubMainVehicle_result)
        if SoAPISubMainVehicle_result:
            break

    while(1):
        if SoAPIGetMainVehicleStatus(SimOne_Data_MainVehicle_Status_Test):
            print("mainVehicleId:%s" %SimOne_Data_MainVehicle_Status_Test.mainVehicleId)
            mainVehicleStatus = SimOne_Data_MainVehicle_Status_Test.mainVehicleStatus
            if mainVehicleStatus > 0:
                break


def NewAPISyncFrame():
    NewAPIAllStart(True)
    control = SimOne_Data_Control()
    while(1):
        frame = SoAPIWait()


        # SoAPIGetSimOneGps
        if SoAPIGetSimOneGps(SimOne_Data_Gps_Test_Sync):
            gps_result = list()
            gps_result.append({"timestamp": SimOne_Data_Gps_Test_Sync.timestamp})
            gps_result.append({"frame": SimOne_Data_Gps_Test_Sync.frame})
            gps_result.append({"frame": SimOne_Data_Gps_Test_Sync.frame})
            gps_result.append({"posX": SimOne_Data_Gps_Test_Sync.posX})
            gps_result.append({"posY": SimOne_Data_Gps_Test_Sync.posY})
            gps_result.append({"posZ": SimOne_Data_Gps_Test_Sync.posZ})
            gps_result.append({"oriX": SimOne_Data_Gps_Test_Sync.oriX})
            gps_result.append({"oriY": SimOne_Data_Gps_Test_Sync.oriY})
            gps_result.append({"oriZ": SimOne_Data_Gps_Test_Sync.oriZ})
            gps_result.append({"velX": SimOne_Data_Gps_Test_Sync.velX})
            gps_result.append({"velY": SimOne_Data_Gps_Test_Sync.velY})
            gps_result.append({"velZ": SimOne_Data_Gps_Test_Sync.velZ})
            gps_result.append({"throttle": SimOne_Data_Gps_Test_Sync.throttle})
            gps_result.append({"brake": SimOne_Data_Gps_Test_Sync.brake})
            gps_result.append({"steering": SimOne_Data_Gps_Test_Sync.steering})
            gps_result.append({"gear": SimOne_Data_Gps_Test_Sync.gear})
            gps_result.append({"accelX": SimOne_Data_Gps_Test_Sync.accelX})
            gps_result.append({"accelY": SimOne_Data_Gps_Test_Sync.accelY})
            gps_result.append({"accelZ": SimOne_Data_Gps_Test_Sync.accelZ})
            gps_result.append({"angVelX": SimOne_Data_Gps_Test_Sync.angVelX})
            gps_result.append({"angVelY": SimOne_Data_Gps_Test_Sync.angVelY})
            gps_result.append({"angVelZ": SimOne_Data_Gps_Test_Sync.angVelZ})
            gps_result.append({"wheelSpeedFL": SimOne_Data_Gps_Test_Sync.wheelSpeedFL})
            gps_result.append({"wheelSpeedFR": SimOne_Data_Gps_Test_Sync.wheelSpeedFR})
            gps_result.append({"wheelSpeedRL": SimOne_Data_Gps_Test_Sync.wheelSpeedRL})
            gps_result.append({"wheelSpeedRR": SimOne_Data_Gps_Test_Sync.wheelSpeedRR})
            print(json.dumps({"SoAPIGetSimOneGps": gps_result}, ensure_ascii=False))

            # SoApiSetDrive

            # control.timestamp = SimOne_Data_Gps_Test_Sync.timestamp
            control.throttle = 5
            control.steering = 0.0
            control.gear = EGearMode.EGearMode_Drive
            SoApiSetDrive(0, control)

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

        # # SoAPIGetSimOneObstacle
        # if SoAPIGetSimOneGroundTruth(SimOne_Data_ObstacleTest_Sync):
        #     obstacle_result = list()
        #     obstacle_result.append({"timestamp": SimOne_Data_ObstacleTest_Sync.timestamp})
        #     obstacle_result.append({"frame": SimOne_Data_ObstacleTest_Sync.frame})
        #     obstacle_result.append({"obstacleSize": SimOne_Data_ObstacleTest_Sync.obstacleSize})
        #     print(SimOne_Data_ObstacleTest_Sync.obstacleSize)
        #     for i in range(0, SimOne_Data_ObstacleTest_Sync.obstacleSize):
        #         tmp = dict()
        #         # obstacle_result.update({"===":"=================================="})
        #         tmp.update({"type": SimOne_Data_ObstacleTest_Sync.obstacle[i].type.value})
        #         tmp.update({"theta": SimOne_Data_ObstacleTest_Sync.obstacle[i].theta})
        #         tmp.update({"posX": SimOne_Data_ObstacleTest_Sync.obstacle[i].posX})
        #         tmp.update({"posY": SimOne_Data_ObstacleTest_Sync.obstacle[i].posY})
        #         tmp.update({"posZ": SimOne_Data_ObstacleTest_Sync.obstacle[i].posZ})
        #         tmp.update({"velX": SimOne_Data_ObstacleTest_Sync.obstacle[i].velX})
        #         tmp.update({"velY": SimOne_Data_ObstacleTest_Sync.obstacle[i].velY})
        #         tmp.update({"velZ": SimOne_Data_ObstacleTest_Sync.obstacle[i].velZ})
        #         tmp.update({"length": SimOne_Data_ObstacleTest_Sync.obstacle[i].length})
        #         tmp.update({"width": SimOne_Data_ObstacleTest_Sync.obstacle[i].width})
        #         tmp.update({"height": SimOne_Data_ObstacleTest_Sync.obstacle[i].height})
        #         obstacle_result.append(tmp)
        #     print(json.dumps({"SoAPIGetSimOneObstacle": obstacle_result}, ensure_ascii=False))

        # SoAPIGetSimOneObstacle
        SoAPINextFrame(frame)


def getGroundTruthCB(mainVehicleId, osi_groundtruth):
    if osi_groundtruth:
        osi_groundtruth_tmp = osi_groundtruth[0].data
        osi_groundtruth_result = base64.b64decode(osi_groundtruth_tmp)
        sensor_groundtruth.ParseFromString(osi_groundtruth_result)
        print("sensor_groundtruth:%s"%sensor_groundtruth)

def getSensorDataUpdateCB(mainVehicleId, sensro_id, simone_sensor_data):
    if simone_sensor_data:
        osi_sensor_data_tmp = simone_sensor_data[0].data
        osi_sensor_data_result = base64.b64decode(osi_sensor_data_tmp)
        osi_sensor_data.ParseFromString(osi_sensor_data_result)
        print("osi_sensor_data:%s"%osi_sensor_data)


def NewAPICallBack():
    NewAPIAllStart(False)

    if SoAPIGetMainVehicleStatus(SimOne_Data_MainVehicle_Status_Test):
        print("mainVehicleId:%s" % SimOne_Data_MainVehicle_Status_Test.mainVehicleId)
        print("mainVehicleStatus:%s" % SimOne_Data_MainVehicle_Status_Test.mainVehicleStatus)

    SoAPISetSimOneGpsCB(SimOne_Data_Gps_Test)
    SoAPISetSimOneGroundTruthCB(SimOne_Data_Obstacle_Test)
    while(1):
        pass


def BridgeSMCallBack():
    SMAPIStart()
    # NewAPIAllStart(False)

    SoAPISetSensorPhysicalbasedDataEnable(True)
    SoAPISetSensorObjectbasedDataEnable(True)
    SoAPISetSensorOSIDataEnable(True)

    # CallBack
    # SoApiSetGpsUpdateCB(Data_Gps_Test)
    # SoApiSetGroundTruthUpdateCB(Data_Obstacle_Test)

    # SoApiSetImageUpdateCB(SetImageTest)
    # SoApiSetPointCloudUpdateCB(SoApiSetPointCloudUpdateCBTest)

    # SoApiSetSensorDetectionsUpdateCB(SoApiSetSensorDetectionsUpdateCBTest)

    # SoApiSetRadarDetectionsUpdateCB(SoApiSetRadarDetectionsUpdateCBTest)
    # SoApiSetUltrasonicRadarsCB(SoApiSetUltrasonicRadarsCBTest)
    #
    # SoApiSetStreamingPointCloudUpdateCB("127.0.0.1", 6699, 7788, SoApiSetStreamingPointCloudUpdateCBTest)
    # SoApiSetStreamingImageCB("127.0.0.1", 7890, SoApiSetStreamingImageCBTest)

    # SoApiOSISetGroundTruthUpdateCB(getGroundTruthCB)
    # SoApiOSISetSensorDataUpdateCB(getSensorDataUpdateCB)

    while(1):
        pass


def BridgeSMGet():
    #GetMethod
    # SoAPISetServerInfo(server='10.2.35.237', port=23789)
    # SoApiStart()

    SMAPIStart()

    SoAPISetSensorPhysicalbasedDataEnable(True)
    SoAPISetSensorObjectbasedDataEnable(True)
    SoAPISetSensorOSIDataEnable(True)
    control = SimOne_Data_Control()
    sensorDetections = SimOne_Data_SensorDetections()
    while(1):
        SoGetSensorConfigurations(sensorConfigurations)
        print("sensorConfiguration size:%s" % sensorConfigurations.dataSize)

        for i in range(sensorConfigurations.dataSize):
            sensor_type = bytes.decode(sensorConfigurations.data[i].sensorType)

            if sensor_type == "camera":  # ESimOneNode_Camera
                SoGetImage(0, sensorConfigurations.data[i].sensorId, imageData)
                print("SoGetImage imagedataSize:%s" % imageData.imagedataSize)
                print("wSoGetImage width:%s" % imageData.width)
                print("wSoGetImage height:%s" % imageData.height)

            if sensor_type == "lidar":  # ESimOneNode_LiDAR
                SoGetPointCloud(0, sensorConfigurations.data[i].sensorId, pointCloudData)
                print("SoGetPointCloud pointCloudDataSize:%s" % pointCloudData.pointCloudDataSize)
                print("SoGetPointCloud width:%s" % pointCloudData.width)
                print("SoGetPointCloud height:%s" % pointCloudData.height)
                print("SoGetPointCloud pointStep:%s" % pointCloudData.pointStep)

            if sensor_type == "objectBasedRadar": # "radar":  # ESimOneNode_MMWRadar
                RadarDetectionDataTest = list()
                SoGetRadarDetections(0, sensorConfigurations.data[i].sensorId, RadarDetectionData)
                RadarDetectionDataTest.append({"detectNum": RadarDetectionData.detectNum})
                for i in range(RadarDetectionData.detectNum):
                    tmp = dict()
                    tmp.update({"id": RadarDetectionData.detections[i].id})
                    tmp.update({"subId": RadarDetectionData.detections[i].subId})
                    tmp.update({"type": RadarDetectionData.detections[i].type})
                    tmp.update({"posX": RadarDetectionData.detections[i].posX})
                    tmp.update({"posY": RadarDetectionData.detections[i].posY})
                    tmp.update({"posZ": RadarDetectionData.detections[i].posZ})
                    tmp.update({"velX": RadarDetectionData.detections[i].velX})
                    tmp.update({"velY": RadarDetectionData.detections[i].velY})
                    tmp.update({"velZ": RadarDetectionData.detections[i].velZ})
                    tmp.update({"range": RadarDetectionData.detections[i].range})
                    tmp.update({"rangeRate": RadarDetectionData.detections[i].rangeRate})
                    tmp.update({"azimuth": RadarDetectionData.detections[i].azimuth})
                    tmp.update({"vertical": RadarDetectionData.detections[i].vertical})
                    tmp.update({"snrdb": RadarDetectionData.detections[i].snrdb})
                    tmp.update({"rcsdb": RadarDetectionData.detections[i].rcsdb})
                    tmp.update({"probability": RadarDetectionData.detections[i].probability})
                    RadarDetectionDataTest.append(tmp)
                print(json.dumps({"SoGetRadarDetections": RadarDetectionDataTest}, ensure_ascii=False))

            if sensor_type == "ultrasonic":  # ESimOneNode_AllUltrasonicRadar
                UltrasonicsRadarDataTest = list()
                SoGetUltrasonicRadars(0, UltrasonicsRadarData)
                UltrasonicsRadarDataTest.append({"detectNum": UltrasonicsRadarData.UltrasonicRadarsNum})
                for i in range(UltrasonicsRadarData.UltrasonicRadarsNum):
                    tmp = dict()
                    tmp.update({"id": UltrasonicsRadarData.detections[i].id})
                    tmp.update({"obstacleNum": UltrasonicsRadarData.detections[i].obstacleNum})
                    for j in range(UltrasonicsRadarData.detections[i].obstacleNum):
                        tmp.update({"obstacleRanges": UltrasonicsRadarData.detections[i].obstacledetections[j].obstacleRanges})
                        tmp.update({"x": UltrasonicsRadarData.detections[i].obstacledetections[j].x})
                    UltrasonicsRadarDataTest.append({'tmp': tmp})
                print(json.dumps({"SoGetUltrasonicRadars": UltrasonicsRadarDataTest}, ensure_ascii=False))

        # SoGetGps
        if SoGetGps(0, SimOne_Data_Gps_Test_Sync):
            gps_result = list()
            gps_result.append({"timestamp": SimOne_Data_Gps_Test_Sync.timestamp})
            gps_result.append({"frame": SimOne_Data_Gps_Test_Sync.frame})
            gps_result.append({"posX": SimOne_Data_Gps_Test_Sync.posX})
            gps_result.append({"posY": SimOne_Data_Gps_Test_Sync.posY})
            gps_result.append({"posZ": SimOne_Data_Gps_Test_Sync.posZ})
            gps_result.append({"oriX": SimOne_Data_Gps_Test_Sync.oriX})
            gps_result.append({"oriY": SimOne_Data_Gps_Test_Sync.oriY})
            gps_result.append({"oriZ": SimOne_Data_Gps_Test_Sync.oriZ})
            gps_result.append({"velX": SimOne_Data_Gps_Test_Sync.velX})
            gps_result.append({"velY": SimOne_Data_Gps_Test_Sync.velY})
            gps_result.append({"velZ": SimOne_Data_Gps_Test_Sync.velZ})
            gps_result.append({"throttle": SimOne_Data_Gps_Test_Sync.throttle})
            gps_result.append({"brake": SimOne_Data_Gps_Test_Sync.brake})
            gps_result.append({"steering": SimOne_Data_Gps_Test_Sync.steering})
            gps_result.append({"gear": SimOne_Data_Gps_Test_Sync.gear})
            gps_result.append({"accelX": SimOne_Data_Gps_Test_Sync.accelX})
            gps_result.append({"accelY": SimOne_Data_Gps_Test_Sync.accelY})
            gps_result.append({"accelZ": SimOne_Data_Gps_Test_Sync.accelZ})
            gps_result.append({"angVelX": SimOne_Data_Gps_Test_Sync.angVelX})
            gps_result.append({"angVelY": SimOne_Data_Gps_Test_Sync.angVelY})
            gps_result.append({"angVelZ": SimOne_Data_Gps_Test_Sync.angVelZ})
            gps_result.append({"wheelSpeedFL": SimOne_Data_Gps_Test_Sync.wheelSpeedFL})
            gps_result.append({"wheelSpeedFR": SimOne_Data_Gps_Test_Sync.wheelSpeedFR})
            gps_result.append({"wheelSpeedRL": SimOne_Data_Gps_Test_Sync.wheelSpeedRL})
            gps_result.append({"wheelSpeedRR": SimOne_Data_Gps_Test_Sync.wheelSpeedRR})
            # print(json.dumps({"SoGetGps": gps_result}, ensure_ascii=False))
        #
        # # SoGetGroundTruth
        # if SoGetGroundTruth(0, SimOne_Data_ObstacleTest_Sync):
        #     obstacle_result = list()
        #     obstacle_result.append({"timestamp": SimOne_Data_ObstacleTest_Sync.timestamp})
        #     obstacle_result.append({"frame": SimOne_Data_ObstacleTest_Sync.frame})
        #     obstacle_result.append({"obstacleSize": SimOne_Data_ObstacleTest_Sync.obstacleSize})
        #     print(SimOne_Data_ObstacleTest_Sync.obstacleSize)
        #     for i in range(SimOne_Data_ObstacleTest_Sync.obstacleSize):
        #         tmp = dict()
        #         # obstacle_result.update({"===":"=================================="})
        #         tmp.update({"type": SimOne_Data_ObstacleTest_Sync.obstacle[i].type.value})
        #         tmp.update({"theta": SimOne_Data_ObstacleTest_Sync.obstacle[i].theta})
        #         tmp.update({"posX": SimOne_Data_ObstacleTest_Sync.obstacle[i].posX})
        #         tmp.update({"posY": SimOne_Data_ObstacleTest_Sync.obstacle[i].posY})
        #         tmp.update({"posZ": SimOne_Data_ObstacleTest_Sync.obstacle[i].posZ})
        #         tmp.update({"velX": SimOne_Data_ObstacleTest_Sync.obstacle[i].velX})
        #         tmp.update({"velY": SimOne_Data_ObstacleTest_Sync.obstacle[i].velY})
        #         tmp.update({"velZ": SimOne_Data_ObstacleTest_Sync.obstacle[i].velZ})
        #         tmp.update({"length": SimOne_Data_ObstacleTest_Sync.obstacle[i].length})
        #         tmp.update({"width": SimOne_Data_ObstacleTest_Sync.obstacle[i].width})
        #         tmp.update({"height": SimOne_Data_ObstacleTest_Sync.obstacle[i].height})
        #         obstacle_result.append(tmp)
        #
        #     print(json.dumps({"SoGetGroundTruth": obstacle_result}, ensure_ascii=False))
        #
        # SoGetSensorDetections

        if SoGetSensorDetections(0, 1, sensorDetections):
            sensorDetections_result = list()
            sensorDetections_result.append({"timestamp": sensorDetections.timestamp})
            sensorDetections_result.append({"frame": sensorDetections.frame})
            sensorDetections_result.append({"objectSize": sensorDetections.objectSize})
            for i in range(0, sensorDetections.objectSize):
                tmp = dict()
                tmp.update({"type": sensorDetections.objects[i].type})
                tmp.update({"probability": sensorDetections.objects[i].probability})
                tmp.update({"id": sensorDetections.objects[i].id})
                tmp.update({"posX": sensorDetections.objects[i].posX})
                tmp.update({"posY": sensorDetections.objects[i].posY})
                tmp.update({"posZ": sensorDetections.objects[i].posZ})
                tmp.update({"velX": sensorDetections.objects[i].velX})
                tmp.update({"velY": sensorDetections.objects[i].velY})
                tmp.update({"velZ": sensorDetections.objects[i].velZ})
                tmp.update({"length": sensorDetections.objects[i].length})
                tmp.update({"width": sensorDetections.objects[i].width})
                tmp.update({"height": sensorDetections.objects[i].height})
                tmp.update({"probability": sensorDetections.objects[i].probability})
                tmp.update({"relativePosX": sensorDetections.objects[i].relativePosX})
                tmp.update({"relativePosY": sensorDetections.objects[i].relativePosY})
                tmp.update({"relativePosZ": sensorDetections.objects[i].relativePosZ})
                tmp.update({"relativeVelX": sensorDetections.objects[i].relativeVelX})
                tmp.update({"relativeVelY": sensorDetections.objects[i].relativeVelY})
                tmp.update({"relativeVelZ": sensorDetections.objects[i].relativeVelZ})
                sensorDetections_result.append(tmp)

            print(json.dumps({"SimOne_Data_SensorDetections": sensorDetections_result}, ensure_ascii=False))

        # # SoApiSetPose
        # pose = SimOne_Data_Pose_Control()
        # pose.timestamp = SimOne_Data_Gps_Test_Sync.timestamp
        # pose.posX = SimOne_Data_Gps_Test_Sync.posX + 10
        # pose.posY = SimOne_Data_Gps_Test_Sync.posY + 10
        # pose.posZ = SimOne_Data_Gps_Test_Sync.posZ
        # SoApiSetPose(0, pose)

        # SoApiSetDrive

        # control.timestamp = SimOne_Data_Gps_Test_Sync.timestamp
        control.throttle = 5
        control.steering = 0.0
        control.brake = False
        # control.isManualGear = False
        control.gear = EGearMode.EGearMode_Drive
        SoApiSetDrive(0, control)

        #  # SoApiSetVehicleEvent
        # vehicleEvent = SimOne_Data_Vehicle_EventInfo()
        # vehicleEvent.timestamp = SimOne_Data_Gps_Test_Sync.timestamp
        # vehicleEvent.type = ESimone_Vehicle_EventInfo_Type.ESimOne_VehicleEventInfo_Forward_Collision
        # SoApiSetVehicleEvent(0, vehicleEvent)
        #
        # # SoGetEnvironment
        # getEnvironment = SimOne_Data_Environment()
        # getEnvironmentJson = dict()
        # if SoGetEnvironment(getEnvironment):
        #     getEnvironmentJson.update({'timeOfDay': getEnvironment.timeOfDay})
        #     getEnvironmentJson.update({'directionalLight': getEnvironment.directionalLight})
        #     getEnvironmentJson.update({'artificialLight': getEnvironment.artificialLight})
        #     getEnvironmentJson.update({'ambientLight': getEnvironment.ambientLight})
        #     getEnvironmentJson.update({'heightAngle': getEnvironment.heightAngle})
        #     getEnvironmentJson.update({'cloudDensity': getEnvironment.cloudDensity})
        #     getEnvironmentJson.update({'rainDensity': getEnvironment.rainDensity})
        #     getEnvironmentJson.update({'snowDensity': getEnvironment.snowDensity})
        #     getEnvironmentJson.update({'groundHumidityLevel': getEnvironment.groundHumidityLevel})
        #     getEnvironmentJson.update({'groundDirtyLevel': getEnvironment.groundDirtyLevel})
        # print(json.dumps(getEnvironmentJson, ensure_ascii=False))

        # # setEnvironment
        # setEnvironmentJson = dict()
        # setEnvironment = SimOne_Data_Environment()
        # setEnvironment.timeOfDay = 2300
        # setEnvironment.snowDensity = 1.0
        # SoSetEnvironment(setEnvironment)

        # # SoGetEnvironment
        # if SoGetEnvironment(setEnvironment):
        #     setEnvironmentJson.update({'timeOfDay': setEnvironment.timeOfDay})
        #     setEnvironmentJson.update({'snowDensity': setEnvironment.snowDensity})
        # print(json.dumps(setEnvironmentJson, ensure_ascii=False))

        # # SoSetSignalLights
        # signalLightsData = SimOne_Data_Signal_Lights()
        # signalLightsData.signalLights |= SimOne_Signal_Light.ESimOne_Signal_Light_LeftBlinker
        # SoSetSignalLights(0, signalLightsData)

        # # SoGetDriverStatus
        # DriverStatus = SimOne_Data_Driver_Status()
        # if SoGetDriverStatus(0, DriverStatus):
        #     status = DriverStatus.driverStatus.value
        #     print("status:%s"%status)
        #
        # # SoGetWayPoints
        # wayPointsTest = SimOne_Data_WayPoints()
        # if SoGetWayPoints(wayPointsTest):
        #     print("status:%s" % wayPointsTest.wayPointsSize)
        #     for i in range(wayPointsTest.wayPointsSize):
        #         print(wayPointsTest.wayPoints[i].index, wayPointsTest.wayPoints[i].posX, wayPointsTest.wayPoints[i].posY)

        # # SoGetStreamingImage
        # if SoGetStreamingImage("10.2.35.159", 7890, imageData):
        #     print("SoGetStreamingImage imagedataSize:%s" % imageData.imagedataSize)
        #     print("SoGetStreamingImage width:%s" % imageData.width)
        #     print("SoGetStreamingImage height:%s" % imageData.height)

        # # SoGetStreamingPointCloud
        # if SoGetStreamingPointCloud("10.2.35.159", 6699, 7788, pointCloudData):
        #     if pointCloudData:
        #         print("SoGetStreamingPointCloud pointCloudDataSize:%s" % pointCloudData.pointCloudDataSize)
        #         print("SoGetStreamingPointCloud width:%s" % pointCloudData.width)
        #         print("SoGetStreamingPointCloud height:%s" % pointCloudData.height)
        #         print("SoGetStreamingPointCloud pointStep:%s" % pointCloudData.pointStep)


if __name__ == '__main__':
    NewAPICallBack()
    NewAPISyncFrame()
    BridgeSMCallBack()
    BridgeSMGet()








