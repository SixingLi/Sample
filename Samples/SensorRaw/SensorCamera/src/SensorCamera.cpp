#include <SimOneStreamingAPI.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include <chrono>
#include <thread>
#include <mutex>
#include <fstream>
#include <string>
#include <queue>

std::queue<SimOne_Streaming_Image> messgeQueue;

extern "C"
{
#include <libswscale/swscale.h>
#include <libavcodec/avcodec.h>
#include <libavutil/mathematics.h>
#include <libavformat/avformat.h>
#include <libavutil/opt.h>
}

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
std::mutex	gDataImageMutex;


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

std::string Buffer2Hex(unsigned char *buffer, int len) {
	std::string HEX = "0123456789ABCDEF";
	std::string finalHex = "";
	for (int i = 0; i < len; i++) {
		int high = buffer[i] >> 4;
		finalHex.push_back(HEX.at(high));
		int low = buffer[i] & 0x0F;
		finalHex.push_back(HEX.at(low));
	}
	return finalHex;
}
class H264Decoder {
public:
	const AVCodec *codec;
	AVCodecContext *c = nullptr;
	int frame_count;
	AVFrame *frame;
	AVFrame *pFrameBGR;

	int BGRsize;
	uint8_t *out_buffer = nullptr;

	struct SwsContext *img_convert_ctx;
	cv::Mat pCvMat;
	bool matReady;

public:
	void init() {
		matReady = false;

		avcodec_register_all();

		codec = avcodec_find_decoder(AV_CODEC_ID_H265);
		if (!codec) {
			fprintf(stderr, "Codec not found\n");
			exit(1);
		}
		c = avcodec_alloc_context3(codec);
		if (!c) {
			fprintf(stderr, "Could not allocate video codec context\n");
			exit(1);
		}

		if (avcodec_open2(c, codec, NULL) < 0) {
			fprintf(stderr, "Could not open codec\n");
			exit(1);
		}

		frame = av_frame_alloc();
		if (!frame) {
			fprintf(stderr, "Could not allocate video frame\n");
			exit(1);
		}

		frame_count = 0;
		//�洢�����ת����RGB����
		pFrameBGR = av_frame_alloc();
	}

	void decode(unsigned char *inputbuf, size_t size) {
		AVPacket avpkt;
		av_init_packet(&avpkt);
		if (avpkt.size == 0)
			return;
		memcpy(&avpkt.pts, inputbuf, 8);
		memcpy(&avpkt.dts, inputbuf + 8, 8);
		memcpy(&avpkt.stream_index, inputbuf+16, 8);

		avpkt.size = size-24;
		avpkt.data = inputbuf+24;
		//std::string temp_str = Buffer2Hex(avpkt.data, 12);
		//std::cout << temp_str << std::endl;
		//if (temp_str == "0000000140010C01FFFF0160") {
		//	std::cout << "11111111111111111111111111111111111111111" << std::endl;
		//}
		//else {
		//	std::cout << "00000000000000000000000000000000000000000" << std::endl;
		//}
		int len, got_frame;


		len = avcodec_decode_video2(c, frame, &got_frame, &avpkt);
		av_packet_unref(&avpkt);
		if (len < 0) {
			matReady = false;
			fprintf(stderr, "Error while decoding frame %d\n", frame_count);
			frame_count++;

			return;
		}
		if (out_buffer == nullptr) {
			BGRsize = avpicture_get_size(AV_PIX_FMT_BGR24, c->width,
				c->height);
			out_buffer = (uint8_t *)av_malloc(BGRsize);
			avpicture_fill((AVPicture *)pFrameBGR, out_buffer, AV_PIX_FMT_BGR24,
				c->width, c->height);

			img_convert_ctx =
				sws_getContext(c->width, c->height, c->pix_fmt,
					c->width, c->height, AV_PIX_FMT_BGR24, SWS_BICUBIC, NULL, NULL,
					NULL);
			pCvMat.create(cv::Size(c->width, c->height), CV_8UC3);

		}
		if (got_frame) {
			matReady = true;
			sws_scale(img_convert_ctx, (const uint8_t *const *)frame->data,
				frame->linesize, 0, c->height, pFrameBGR->data, pFrameBGR->linesize);

			memcpy(pCvMat.data, out_buffer, BGRsize);

			//        printf("decoding frame: %d\n",frame_count);
			frame_count++;
			av_frame_unref(frame);
		}
		else {
			matReady = false;
		}
	}

	void play() {
		if (matReady) {
			cv::imshow("51Sim-One Camera Video Injection H265", pCvMat);
			cv::waitKey(1);
		}
	}

	H264Decoder() {
		init();
	}

	cv::Mat getMat() {
		if (matReady) {
			return pCvMat;
		}
		else {
			return cv::Mat();
		}
	}
};

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
	messgeQueue.push(gDataImage);
	//messgeQueue.pop();
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

	std::thread mImageProcess([&]() {
		int lastFrame = 0;
		H264Decoder decoder;
		while (1)
		{
			if (!messgeQueue.empty())
			{
				std::lock_guard<std::mutex> lock(gDataImageMutex);
				SimOne_Streaming_Image *gDataImage = &messgeQueue.front();
				if (gDataImage->frame != lastFrame) {

					if (gDataImage->format == ESimOne_Streaming_Image_Format_RGB) {
						cv::Mat img(gDataImage->height, gDataImage->width, CV_8UC3, gDataImage->imageData);
						cv::imshow("51Sim-One Camera Video Injection Rgb", img);
					}
					else if (gDataImage->format == ESimOne_Streaming_Image_Format_RLESegmentation) {
						Horizon_RLE_Header frameHeader;
						char * ImageRleData = new char[SOSM_IMAGE_DATA_SIZE_MAX];
						Rle_Decode_Horizon(frameHeader, (BYTE*)(void*)&(gDataImage->imageData), gDataImage->imageDataSize, (BYTE*)(void*)ImageRleData);
						cv::Mat img(gDataImage->height, gDataImage->width, CV_8U, ImageRleData);
						cv::imshow("51Sim-One Camera Video Injection Rle", img);
						delete ImageRleData;
					}
					else if (gDataImage->format == ESimOne_Streaming_Image_Format_JPEG) {
						cv::Mat imgbuf(gDataImage->height, gDataImage->width, CV_8UC3, gDataImage->imageData);
						cv::Mat img = imdecode(imgbuf, CV_LOAD_IMAGE_COLOR);
						cv::imshow("51Sim-One Camera Video Injection Jpeg", img);
					}
					else if (gDataImage->format == ESimOne_Streaming_Image_Format_H265) {
						decoder.decode((unsigned char *)gDataImage->imageData, gDataImage->imageDataSize);
						decoder.play();
					}
				}
				messgeQueue.pop();
			}
			if (cv::waitKey(1) == 27)
				break;
		}
	
	});
	mImageProcess.detach();
	while (1) {
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
	}
	return 0;
}
