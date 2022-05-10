#include <SimOneStreamingAPI.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include <chrono>
#include <thread>
#include <mutex>
#include <fstream>
#include <string>
#include <queue>
#include <atomic>

std::queue<SimOne_Streaming_Image> messgeQueue;
std::atomic<bool> flip;

extern "C"
{
#include "libswscale/swscale.h"
#include "libavcodec/avcodec.h"
#include "libavutil/mathematics.h"
#include "libavformat/avformat.h"
#include "libavutil/opt.h"
}
#define WIDTH 1920
#define HEIGHT 1080
typedef unsigned char BYTE;

#pragma pack(1)
typedef struct RLE_Header {
	BYTE flag;
	char rle[3];
	unsigned short height;
	unsigned short width;
}Horizon_RLE_Header;
#pragma pack()

// std::string gIP = "127.0.0.1";
std::string gIP = "10.66.9.244";
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
class HEVCDecoder {
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
	AVPacket avpkt;

public:
	void init() {
		av_init_packet(&avpkt);
		matReady = false;
		avcodec_register_all();
		codec = avcodec_find_decoder(AV_CODEC_ID_HEVC);
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
		memcpy(&avpkt.pts, inputbuf, 8);
		memcpy(&avpkt.dts, inputbuf + 8, 8);
		memcpy(&avpkt.stream_index, inputbuf+16, 8);
		avpkt.size = size-24;
		avpkt.data = inputbuf+24;
		// std::string temp_str = Buffer2Hex(avpkt.data, 12);
		// std::cout << temp_str << std::endl;
		// if (temp_str == "0000000140010C01FFFF0160") {
		// 	std::cout << "11111111111111111111111111111111111111111" << std::endl;
		// }
		// else {
		// 	std::cout << "00000000000000000000000000000000000000000" << std::endl;
		// }
		int len, got_frame;


		len = avcodec_decode_video2(c, frame, &got_frame, &avpkt);
		// av_packet_unref(&avpkt);
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
		}
	}

	HEVCDecoder() {
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

class HEVCEncoder{
private:
    bool mInited;
    int mWidth;
    int mHeight;
	AVCodecContext *mpFFCodecCtx = nullptr;
	AVCodecContext *mpFFCodec2H265 = nullptr;
	AVFormatContext* fc = nullptr;
	AVStream* stream = nullptr;
    AVFrame *mpFFFrameYuv = nullptr;
    AVPacket *mpFFPackage = nullptr;
    AVPacket pkt;
    SwsContext *mpFFConvertCtx = nullptr;

public:
bool initFFmpeg2H265(int width,int height) {

	// Setting up the codec.
	// jpeg ---------------------------yuv 420 p
	av_register_all();
	avcodec_register_all();

	AVCodec *pCodec = avcodec_find_decoder(AV_CODEC_ID_MJPEG);
	if (!pCodec)
	{
		printf("avcodec_find_decoder failed");
		return false;
	}
	mpFFCodecCtx = avcodec_alloc_context3(pCodec);
	if (!mpFFCodecCtx)
	{
		printf("avcodec_alloc_context3 failed");
		return false;
	}

	if (avcodec_open2(mpFFCodecCtx, pCodec, NULL) < 0)
	{
		printf("avcodec_open2 failed");
		return false;
	}

	mpFFFrameYuv = av_frame_alloc();
	if (!mpFFFrameYuv)
	{
		printf("av_frame_alloc mpFFFrameYuv failed");
		return false;
	}
	mpFFPackage = new AVPacket();

	// yuv420p-------->h265
	int fps = 30;
	int bit_rate_ = 8192000;
	AVCodec *codec = avcodec_find_encoder(AV_CODEC_ID_HEVC);
	if(!codec){
			printf("###########can't find hevc encoder\n");
	}else{
			printf("########### find hevc encoder\n");
	}

	AVDictionary *param = NULL;
	av_dict_set(&param, "preset", "ultrafast", 0);
	av_dict_set(&param, "tune", "zero-latency", 0);

	mpFFCodec2H265 = avcodec_alloc_context3(codec);
	mpFFCodec2H265->width = width;
	mpFFCodec2H265->height = height;
	mpFFCodec2H265->pix_fmt = AV_PIX_FMT_YUV420P;
	mpFFCodec2H265->time_base = AVRational{ 1, fps };
	mpFFCodec2H265->bit_rate = bit_rate_;//8192000
	mpFFCodec2H265->keyint_min = fps;
	mpFFCodec2H265->max_b_frames = 0;
	mpFFCodec2H265->gop_size = fps;

	if(&param){
		printf("param set success!!!!\n");
	}else{
		printf("param set fail!!!!\n");
	}

	int ret = avcodec_open2(mpFFCodec2H265, codec,&param);
	if (ret<0)
	{
		printf("avcodec_open2 failed\n");
		av_dict_free(&param);
		return false;
	}
	av_dict_free(&param);
	return true;
}
int encodeToH265Image(int width, int height, int inSize, const char *inImage, char *&outImage)
{
	if (!mInited || width != mWidth || height != mHeight)
	{
		mWidth = width;
		mHeight = height;
		printf("Init image decoder with size(%d, %d)\n", mWidth, mHeight);
		freeFFmpeg();
		mInited = initFFmpeg2H265(width,height);
	}
	// JPEG to YUV420
	mpFFPackage->size = inSize;
	mpFFPackage->data = (uint8_t *)inImage;
	int re = 0;
	int got_frame = 0;
	re = avcodec_decode_video2(mpFFCodecCtx, mpFFFrameYuv, &got_frame, mpFFPackage);
	if (got_frame != 1)
	{
		printf("avcodec_decode_video2 failed\n");
		return -1;
	}
	av_init_packet(&pkt);
	pkt.data = NULL;
	pkt.size = 0;
	int got_output;
     int ret = avcodec_encode_video2(mpFFCodec2H265, &pkt, mpFFFrameYuv, &got_output);
	 if (got_output != 1) {
 		printf("avcodec_decode_video2 failed\n");
	}	

	int64_t pts = pkt.pts;
	int64_t dts =  pkt.dts;
	int64_t stream_index =  pkt.stream_index;

	unsigned char * avpacket_current = (uint8_t *)malloc(sizeof(unsigned char)*(24 +  pkt.size));
	memset(avpacket_current, 0, sizeof(uint8_t) * 3 +  pkt.size);
	memcpy(avpacket_current, &pts, sizeof(int64_t));
	memcpy(avpacket_current + sizeof(int64_t), &dts, sizeof(int64_t));
	memcpy(avpacket_current + 2 * sizeof(int64_t), &stream_index, sizeof(int64_t));
	memcpy(avpacket_current + 3 * sizeof(int64_t), pkt.data,  pkt.size);

	size_t nStreamSize =  pkt.size + 3 * sizeof(int64_t);
	outImage = (char *)(avpacket_current);
	return nStreamSize;
}

void freeFFmpeg()
{
	if (mpFFCodecCtx)
	{
		avcodec_close(mpFFCodecCtx);
		mpFFCodecCtx = nullptr;
	}
	if (mpFFCodec2H265) {
		avcodec_close(mpFFCodec2H265);
		mpFFCodecCtx = nullptr;
	}
	if (mpFFConvertCtx)
	{
		sws_freeContext(mpFFConvertCtx);
		mpFFConvertCtx = nullptr;
	}
	if (mpFFFrameYuv)
	{
		av_frame_free(&mpFFFrameYuv);
		mpFFFrameYuv = nullptr;
	}
	if (mpFFPackage)
	{
		av_free_packet(mpFFPackage);
		mpFFPackage = nullptr;
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

    flip.store(false);

	std::thread mImageProcess([&]() {
		int lastFrame = 0;
		HEVCEncoder encoder;
		HEVCDecoder decoder;
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
						char *imageTemp = nullptr;
						int sizeofhevc = encoder.encodeToH265Image(WIDTH,HEIGHT,gDataImage->imageDataSize,gDataImage->imageData,imageTemp);
						// if(sizeofhevc>0){
						// 	printf("messgeQueue's size = %d ,encode success and size = %d\n",messgeQueue.size(),sizeofhevc);
						// }else{
						// 	printf("fail !!!!!!!!!!!!!!\n");
						// }
						// decoder.decode((unsigned char *)imageTemp, sizeofhevc);
						// decoder.play();
					}
				}
				messgeQueue.pop();
			}
			// std::this_thread::sleep_for(std::chrono::milliseconds(1));
			std::this_thread::sleep_for(std::chrono::nanoseconds(1));
			// if (cv::waitKey(1) == 27)
			// 	break;
		}
	
	});
	mImageProcess.detach();
	while (1) {
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
	}
	return 0;
}
