#include <SimOneStreamingAPI.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include <chrono>
#include <thread>
#include <mutex>

typedef unsigned char BYTE;

#pragma pack(1)
typedef struct RLE_Header {
	BYTE flag;
	char rle[3];
	unsigned short height;
	unsigned short width;
}Horizon_RLE_Header;
#pragma pack()


std::string gIP = "127.0.0.1";
unsigned short gPort = 13956;
SimOne_Streaming_Image gDataImage;
std::mutex        gDataImageMutex;

void dataImageCallback(SimOne_Streaming_Image *pImage)
{
	std::lock_guard<std::mutex> lock(gDataImageMutex);
	gDataImage.frame = pImage->frame;
	gDataImage.timestamp = pImage->timestamp;
	gDataImage.height = pImage->height;
	gDataImage.width = pImage->width;
	gDataImage.imageDataSize = pImage->imageDataSize;
	gDataImage.format = pImage->format;
	memcpy(&gDataImage.imageData, &pImage->imageData, pImage->imageDataSize);
}

int Rle_Decode_Horizon(Horizon_RLE_Header &frameHeader, BYTE *inbuf, int inSize, BYTE *outbuf) {
	memcpy(&frameHeader, inbuf, sizeof(Horizon_RLE_Header));
	frameHeader.height = frameHeader.height >> 8 | frameHeader.height << 8;
	frameHeader.width = frameHeader.width >> 8 | frameHeader.width << 8;
	int outBuffSzie = frameHeader.width * frameHeader.height;

	BYTE *src = inbuf + sizeof(Horizon_RLE_Header);
	int i;
	int decSize = 0;
	while (src < (inbuf + inSize))
	{
		unsigned short count;
		memcpy(&count, src + 1, 2);

		if ((decSize + count) > outBuffSzie)
		{
			return -1;
		}
		BYTE value = *src;
		for (i = 0; i < count; i++)
		{
			outbuf[decSize++] = value;
		}
		src = src + 3;
	}
	return decSize;
}

int main(int argc, char* argv[])
{
	if (argc >= 2)
	{
		gIP = argv[1];
		gPort = atoi(&*argv[2]);
	}
	printf("IP: %s; Port: %d\n", gIP.c_str(), gPort);

	//SimOneAPI::SetStreamingImageUpdateCB(gIP.c_str(), gPort, dataImageCallback);
	int lastFrame = 0;
	while (1)
	{
		//std::lock_guard<std::mutex> lock(gDataImageMutex);
		//getchar();
		if(SimOneAPI::GetStreamingImage(gIP.c_str(), gPort, &gDataImage))
		{
			if (gDataImage.frame != lastFrame) {

				if (gDataImage.format == ESimOne_Streaming_Image_Format_RGB) {
					cv::Mat img(gDataImage.height, gDataImage.width, CV_8UC3, gDataImage.imageData);
					cv::imshow("51Sim-One Camera Video Injection", img);
				}
				else if (gDataImage.format == ESimOne_Streaming_Image_Format_RLESegmentation) {
					Horizon_RLE_Header frameHeader;
					char * ImageRleData = new char[SOSM_IMAGE_DATA_SIZE_MAX];
					Rle_Decode_Horizon(frameHeader, (BYTE*)(void*)&(gDataImage.imageData), gDataImage.imageDataSize, (BYTE*)(void*)ImageRleData);
					cv::Mat img(gDataImage.height, gDataImage.width, CV_8U, ImageRleData);
					cv::imshow("51Sim-One Camera Video Injection", img);
					delete ImageRleData;
				}
				
				gDataImage.frame = lastFrame;
			}
			
		}
		if (cv::waitKey(1) == 27)
			break;
		std::this_thread::sleep_for(std::chrono::milliseconds(30));
	}
	return 0;
}