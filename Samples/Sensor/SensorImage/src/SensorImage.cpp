#include <SimOneStreamingAPI.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include <chrono>
#include <thread>
#include <mutex>

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

	SimOneAPI::SetStreamingImageUpdateCB(gIP.c_str(), gPort, dataImageCallback);
	int lastFrame = 0;
	while (1)
	{
		{
			std::lock_guard<std::mutex> lock(gDataImageMutex);
			if (gDataImage.frame != lastFrame) {
				cv::Mat img(gDataImage.height, gDataImage.width, CV_8UC3, gDataImage.imageData);
				cv::imshow("51Sim-One Camera Video Injection", img);
				gDataImage.frame = lastFrame;
			}
		}
		if (cv::waitKey(1) == 27)
			break;
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return 0;
}

