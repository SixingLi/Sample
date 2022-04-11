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
unsigned short gPortRgb = 13956;
unsigned short gPortRle = 13966;
SimOne_Streaming_Image gDataImageRgb;
SimOne_Streaming_Image gDataImageRle;
std::mutex        gDataImageRgbMutex;
std::mutex        gDataImageRleMutex;

void dataImageRgbCallback(SimOne_Streaming_Image *pImage)
{
	std::lock_guard<std::mutex> lock(gDataImageRgbMutex);
	gDataImageRgb.frame = pImage->frame;
	gDataImageRgb.timestamp = pImage->timestamp;
	gDataImageRgb.height = pImage->height;
	gDataImageRgb.width = pImage->width;
	gDataImageRgb.imageDataSize = pImage->imageDataSize;
	gDataImageRgb.format = pImage->format;
	memcpy(&gDataImageRgb.imageData, &pImage->imageData, pImage->imageDataSize);
}

void dataImageRleCallback(SimOne_Streaming_Image *pImage)
{
	std::lock_guard<std::mutex> lock(gDataImageRleMutex);
	gDataImageRle.frame = pImage->frame;
	gDataImageRle.timestamp = pImage->timestamp;
	gDataImageRle.height = pImage->height;
	gDataImageRle.width = pImage->width;
	gDataImageRle.imageDataSize = pImage->imageDataSize;
	gDataImageRle.format = pImage->format;
	memcpy(&gDataImageRle.imageData, &pImage->imageData, pImage->imageDataSize);
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
	if (argc >= 3)
	{
		gIP = argv[1];
		gPortRgb = atoi(&*argv[2]);
		gPortRle = atoi(&*argv[3]);
	}
	printf("IP: %s; RgbPort: %d\n", gIP.c_str(), gPortRgb);
	printf("IP: %s; RlePort: %d\n", gIP.c_str(), gPortRle);

	SimOneAPI::SetStreamingImageUpdateCB(gIP.c_str(), gPortRgb, dataImageRgbCallback);
	SimOneAPI::SetStreamingImageUpdateCB(gIP.c_str(), gPortRle, dataImageRleCallback);
	std::thread RgbThread([&]() {
		int lastFrame = 0;
		while (1)
		{
			std::lock_guard<std::mutex> lock(gDataImageRgbMutex);
			{
				if (gDataImageRgb.frame != lastFrame) {

					if (gDataImageRgb.format == ESimOne_Streaming_Image_Format_RGB) {
						cv::Mat img(gDataImageRgb.height, gDataImageRgb.width, CV_8UC3, gDataImageRgb.imageData);
						cv::imshow("51Sim-One Camera Video Injection Rgb", img);
						std::cout << "111111111111111111111111111111111  " << gDataImageRgb.frame<< std::endl;
					}
					else if (gDataImageRgb.format == ESimOne_Streaming_Image_Format_RLESegmentation) {
						Horizon_RLE_Header frameHeader;
						char * ImageRleData = new char[SOSM_IMAGE_DATA_SIZE_MAX];
						Rle_Decode_Horizon(frameHeader, (BYTE*)(void*)&(gDataImageRgb.imageData), gDataImageRgb.imageDataSize, (BYTE*)(void*)ImageRleData);
						cv::Mat img(gDataImageRgb.height, gDataImageRgb.width, CV_8U, ImageRleData);
						cv::imshow("51Sim-One Camera Video Injection Rle", img);
						delete ImageRleData;
					}
					else if (gDataImageRgb.format == ESimOne_Streaming_Image_Format_JPEG) {
						cv::Mat imgbuf(gDataImageRgb.height, gDataImageRgb.width, CV_8UC3, gDataImageRgb.imageData);
						cv::Mat img = imdecode(imgbuf, CV_LOAD_IMAGE_COLOR);
						cv::imshow("51Sim-One Camera Video Injection Jpeg", img);
						std::cout << "this is jpeg channel" << std::endl;
					}

					gDataImageRgb.frame = lastFrame;
				}

			}
			if (cv::waitKey(1) == 27)
				break;
		}
	});

	std::thread RleThread([&]() {
		int lastFrame = 0;
		while (1)
		{
			std::lock_guard<std::mutex> lock(gDataImageRleMutex);
			{
				if (gDataImageRle.frame != lastFrame) {

					if (gDataImageRle.format == ESimOne_Streaming_Image_Format_RGB) {
						cv::Mat img(gDataImageRle.height, gDataImageRle.width, CV_8UC3, gDataImageRle.imageData);
						cv::imshow("51Sim-One Camera Video Injection Rgb", img);
					}
					else if (gDataImageRle.format == ESimOne_Streaming_Image_Format_RLESegmentation) {
						Horizon_RLE_Header frameHeader;
						char * ImageRleData = new char[SOSM_IMAGE_DATA_SIZE_MAX];
						Rle_Decode_Horizon(frameHeader, (BYTE*)(void*)&(gDataImageRle.imageData), gDataImageRle.imageDataSize, (BYTE*)(void*)ImageRleData);
						cv::Mat img(gDataImageRle.height, gDataImageRle.width, CV_8U, ImageRleData);
						cv::imshow("51Sim-One Camera Video Injection Rle", img);
						delete ImageRleData;
					}
					else if (gDataImageRle.format == ESimOne_Streaming_Image_Format_JPEG) {
						cv::Mat imgbuf(gDataImageRgb.height, gDataImageRgb.width, CV_8UC3, gDataImageRgb.imageData);
						cv::Mat img = imdecode(imgbuf, CV_LOAD_IMAGE_COLOR);
						cv::imshow("51Sim-One Camera Video Injection Jpeg", img);
						std::cout << "this is jpeg channel" << std::endl;
					}

					gDataImageRle.frame = lastFrame;
				}

			}
			if (cv::waitKey(1) == 27)
				break;
		}
	});

	RgbThread.detach();
	RleThread.detach();

	getchar();
	return 0;
}