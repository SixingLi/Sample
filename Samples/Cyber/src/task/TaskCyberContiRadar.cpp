#include "TaskCyberContiRadar.hpp"

void CW_ContiRadar::set_conti_radar(int& obj_size, CyberWriterContiRadar::Object *obj_list)
{
    static int lastFrame = 0;
    std::unique_ptr<SimOne_Data_RadarDetection> pDetections = std::make_unique<SimOne_Data_RadarDetection>();

    if (!SimOneAPI::GetRadarDetections("0", "objectBasedRadar1", pDetections.get()))
    {
        std::cout << "[CW_ContiRadar] GetRadarDetections Failed!" << std::endl;
        return;
    }
    if (pDetections->frame == lastFrame)
    {
        return;
    }
    lastFrame = pDetections->frame;

    obj_size = pDetections->detectNum;
    for (int i=0; i<obj_size; i++)
    {
        obj_list[i].id = pDetections->detections[i].id;
        obj_list[i].longitudeDist = pDetections->detections[i].range * sin(pDetections->detections[i].azimuth);
        obj_list[i].lateralDist = pDetections->detections[i].range * cos(pDetections->detections[i].azimuth);
        obj_list[i].longitudeVel = pDetections->detections[i].velX;
        obj_list[i].lateralVel = pDetections->detections[i].velY;
        obj_list[i].rcs = pDetections->detections[i].snrdb;
        obj_list[i].dynprop = 4;
    }
}

void CW_ContiRadar::task_conti_radar()
{
    mCyberContiRadar.mObjectSize = 0;
    memset(mCyberContiRadar.mObjectList, 0, sizeof(CyberWriterContiRadar::Object) * OBJECT_MAX_SIZE);

    set_conti_radar(mCyberContiRadar.mObjectSize, mCyberContiRadar.mObjectList);
    mCyberContiRadar.publish();
}
