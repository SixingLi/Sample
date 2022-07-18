#include <SimOneStreamingAPI.h>
#include <opencv2/opencv.hpp>
#include "SimOneServiceAPI.h"
#include "SimOneSensorAPI.h"
#include <vector>
#include <chrono>
#include <thread>
#include <mutex>
#include <map>

std::string gIP = "127.0.0.1";
unsigned short gPort = 13956;
SimOne_Streaming_Image gDataImage;
std::mutex	gDataImageMutex;

void dataImageCallback(SimOne_Streaming_Image *pImage)
{
	std::lock_guard<std::mutex> lock(gDataImageMutex);
	gDataImage.frame = pImage->frame;
	gDataImage.timestamp = pImage->timestamp;
	gDataImage.height = pImage->height;
	gDataImage.width = pImage->width;
	gDataImage.imageDataSize = pImage->imageDataSize;
	memcpy(&gDataImage.imageData, &pImage->imageData, pImage->imageDataSize);
}

int main(int argc, char* argv[])
{
	if (argc >= 2)
	{
		gIP = argv[1];
		gPort = atoi(&*argv[2]);
	}
	printf("IP: %s; Port: %d\n", gIP.c_str(), gPort);
	std::map<int, SimOne_Data_SensorDetections> mGroundtruthMap;
	bool isinit = SimOneAPI::InitSimOneAPI();
	SimOneAPI::SetStreamingImageUpdateCB(gIP.c_str(), gPort, dataImageCallback);
	SimOne_Data_SensorDetections pGroundtruth;


	int lastFrame = 0;
	while (1)
	{
		{
			std::lock_guard<std::mutex> lock(gDataImageMutex);
			if (gDataImage.frame != lastFrame) {
				SimOneAPI::GetSensorDetections("0", "objectBasedCamera1", &pGroundtruth);
				cv::Mat img(gDataImage.height, gDataImage.width, CV_8UC3, gDataImage.imageData);

				mGroundtruthMap[(int)(pGroundtruth.frame*1000.0/60+0.5)] = pGroundtruth;
				printf("pGroundtruth.frame:%d, gDataImage.frame:%d\n", (int)((int)(pGroundtruth.frame*1000.0 / 60 + 0.5)), gDataImage.frame);
				auto iter = mGroundtruthMap.find(gDataImage.frame-16);
				auto iter1 = mGroundtruthMap.find(gDataImage.frame - 49);
				auto iter2 = mGroundtruthMap.find(gDataImage.frame -83);
				if (iter != mGroundtruthMap.end()) {
					for (int index = 0; index < iter->second.objectSize; index++) {
						cv::rectangle(img, cv::Point(iter->second.objects[index].bbox2dMinX, iter->second.objects[index].bbox2dMinY),
							cv::Point(iter->second.objects[index].bbox2dMaxX, iter->second.objects[index].bbox2dMaxY), cv::Scalar(0, 255, 0));
					}
					//mGroundtruthMap.erase(iter->first);
				}
				else if(iter1 != mGroundtruthMap.end()){
					for (int index = 0; index < iter1->second.objectSize; index++) {
						cv::rectangle(img, cv::Point(iter1->second.objects[index].bbox2dMinX, iter1->second.objects[index].bbox2dMinY),
							cv::Point(iter1->second.objects[index].bbox2dMaxX, iter1->second.objects[index].bbox2dMaxY), cv::Scalar(0, 255, 0));
					}
					cv::imshow("51Sim-One Camera Video Injection", img);
					//mGroundtruthMap.erase(iter1->first);
				}
				else if (iter2 != mGroundtruthMap.end()) {
					for (int index = 0; index < iter1->second.objectSize; index++) {
						cv::rectangle(img, cv::Point(iter2->second.objects[index].bbox2dMinX, iter2->second.objects[index].bbox2dMinY),
							cv::Point(iter2->second.objects[index].bbox2dMaxX, iter2->second.objects[index].bbox2dMaxY), cv::Scalar(0, 255, 0));
					}
					cv::imshow("51Sim-One Camera Video Injection", img);
					//mGroundtruthMap.erase(iter2->first);
				}
				//mGroundtruthMap[pGroundtruth.timestamp] = pGroundtruth;
				cv::imshow("51Sim-One Camera Video Injection", img);
				lastFrame = gDataImage.frame;
			}
		}
		if (cv::waitKey(1) == 27)
			break;
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return 0;
}

