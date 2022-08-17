#include "TaskCyberImage.hpp"

CyberWriterImage* CW_Image::pCyberImage = nullptr;

void CW_Image::set_params(Img_Param_T& param)
{
    mParam.ip = param.ip;
    mParam.port = param.port;
}

void CW_Image::set_image(SimOne_Streaming_Image *pImage)
{
	// pImage->frame;
	// pImage->timestamp;
	// pImage->imageDataSize;
    pCyberImage->publish(pImage->height, pImage->width, pImage->imageData);
}

void CW_Image::task_image()
{
    pCyberImage = &mCyberImage;
	SimOneAPI::SetStreamingImageUpdateCB(mParam.ip.c_str(), mParam.port, set_image);
}
