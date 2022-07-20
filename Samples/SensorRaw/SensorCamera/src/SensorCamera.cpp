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
	std::map<int, SimOne_Data_RoadMarkInfo> mRoadMarkMap;
	bool isinit = SimOneAPI::InitSimOneAPI();
	SimOneAPI::SetStreamingImageUpdateCB(gIP.c_str(), gPort, dataImageCallback);
	SimOne_Data_RoadMarkInfo pRoadMark;


	int lastFrame = 0;
	while (1)
	{
		{
			std::lock_guard<std::mutex> lock(gDataImageMutex);
			if (gDataImage.frame != lastFrame) {
				cv::Mat img(gDataImage.height, gDataImage.width, CV_8UC3, gDataImage.imageData);
				if (SimOneAPI::GetSensorRoadMarkInfo("0", "objectBasedCamera1", &pRoadMark)) {
					for (int index = 0; index < pRoadMark.detectNum; index++) {
						for (int i =  0; i < pRoadMark.roadMarks[index].pointSize-1; i++) {
							if (pRoadMark.roadMarks[index].type == RoadMarkType_Graphics) {
								cv::line(img, cv::Point(pRoadMark.roadMarks[index].pixs2d[i].x, pRoadMark.roadMarks[index].pixs2d[i].y),
									cv::Point(pRoadMark.roadMarks[index].pixs2d[i+1].x, pRoadMark.roadMarks[index].pixs2d[i+1].y), cv::Scalar(255, 0, 0));
							}
 							else if (pRoadMark.roadMarks[index].type == RoadMarkType_StopLine) {
								cv::line(img, cv::Point(pRoadMark.roadMarks[index].pixs2d[i].x, pRoadMark.roadMarks[index].pixs2d[i].y),
									cv::Point(pRoadMark.roadMarks[index].pixs2d[i + 1].x, pRoadMark.roadMarks[index].pixs2d[i + 1].y), cv::Scalar  (0, 255, 0));
							}
							else if (pRoadMark.roadMarks[index].type == RoadMarkType_CrossWalk) {
								cv::line(img, cv::Point(pRoadMark.roadMarks[index].pixs2d[i].x, pRoadMark.roadMarks[index].pixs2d[i].y),
									cv::Point(pRoadMark.roadMarks[index].pixs2d[i + 1].x, pRoadMark.roadMarks[index].pixs2d[i + 1].y), cv::Scalar(0, 0, 255));
							}
						}
					}
				}
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

