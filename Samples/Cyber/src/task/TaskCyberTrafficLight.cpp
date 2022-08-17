#include "TaskCyberTrafficLight.hpp"

void CW_TrafficLight::set_traffic_light(TrafficLight_T& data, int& size)
{
    std::unique_ptr<SimOne_Data_SensorConfigurations> pSensorConfigurations = std::make_unique<SimOne_Data_SensorConfigurations>();
    std::unique_ptr<SimOne_Data_SensorDetections> pSensorDetections = std::make_unique<SimOne_Data_SensorDetections>();
    std::vector<SimOne_Data_SensorDetections_Entry> SensorDetections_Trafficlights;

    if (!SimOneAPI::GetSensorConfigurations("0", pSensorConfigurations.get()))
    {
        std::cout << "[CW_TrafficLight] GetSensorConfigurations Failed!" << std::endl;
        return;
    }
    for (int i = 0; i < pSensorConfigurations->dataSize; i++)
    {
        // std::cout << "[CW_TrafficLight] SensorConfigurations data[" << j << "].sensorType: " << pSensorConfigurations->data[j].sensorType;
        if (strcmp(pSensorConfigurations->data[i].sensorType, "sensorFusion1") == 0)
        {
            if (!SimOneAPI::GetSensorDetections(0, pSensorConfigurations->data[i].sensorId, pSensorDetections.get()))
            {
                std::cout  << "[CW_TrafficLight] GetSensorDetections Failed!" << std::endl;
                return;
            }
            if (pSensorDetections->objectSize == 0)
            {
                std::cout << "[CW_TrafficLight] SensorDetections objectSize: " << pSensorDetections->objectSize << std::endl;
                return;
            }
            for (int j = 0; j < pSensorDetections->objectSize; ++j)
            {
                switch (pSensorDetections->objects[i].type)
                {
                case ESimOne_Obstacle_Type_TrafficLight:
                    SensorDetections_Trafficlights.push_back(pSensorDetections->objects[i]);
                    break;
                default:;
                }
            }
        }
    }
    
    size = SensorDetections_Trafficlights.size();
    for (int i = 0; i < size; i++)
    {
        // std::cout << "SensorDetections Trafficlights[" << i << "] ID: " << SensorDetections_Trafficlights[i].id;
        SimOne_Data_TrafficLight Single_TrafficLight;
        if (!SimOneAPI::GetTrafficLight(0, SensorDetections_Trafficlights[i].id, &Single_TrafficLight))
        {
            std::cout << "[CW_TrafficLight] GetTrafficLight Failed!" << std::endl;
        }
        else
        {
            memset(data.id[i], '\0', sizeof(data.id[i]));
            std::string id = std::to_string(Single_TrafficLight.opendriveLightId);
            strncpy(data.id[i], id.c_str(), id.size());
            data.confidence[i] = 1.0;
            // apollo/modules/perception/proto/traffic_light_detection.proto
            switch (Single_TrafficLight.status)
            {
            case ESimOne_TrafficLight_Status_Red:
                data.color[i] = 1;
                break;
            case ESimOne_TrafficLight_Status_Green:
                data.color[i] = 3;
                break;
            case ESimOne_TrafficLight_Status_Yellow:
                data.color[i] = 2;
                break;
            case ESimOne_TrafficLight_Status_Black:
                data.color[i] = 4;
                break;
            default:
                data.color[i] = 0;
            }
        }
    }
}

void CW_TrafficLight::task_traffic_light()
{
    static int frame = 0;
    int size = 0;
    TrafficLight_T data;
    set_traffic_light(data, size);
    char* ids[LIGHT_MAX_NUM];
    memset(ids, '\0', sizeof(ids));
    for (int i = 0; i < size; i++)
    {
        ids[i] = data.id[i];
    }
    mCyberTrafficLight.publish(frame, ids, data.confidence, data.color, size);
}
        