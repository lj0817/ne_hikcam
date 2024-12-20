#ifndef HIKCAMERAIO
#define HIKCAMERAIO

#include <opencv2/opencv.hpp>
#include <string>
#include <cmath>
#include <chrono>

#include "MvCameraControl.h"

namespace ne_io
{
class HikCam
{
public:

	HikCam();
	~HikCam();
	void start();
	void shutdown();
	void reNewExposureStart(double a);
	void reNewGainStart(double b);
	void reConnectStart();
	void getImg();
	bool toConnect();


	unsigned char *pData;
	void *handle;
	float newexposuretime;
	float newgain;
	MVCC_INTVALUE stParam;
	MV_FRAME_OUT stImageInfo;
	
private:
	int WidthValue;
    int HeightValue;

    float ExposureTimeValue;
    float GainValue;
	int nRet;
	
	MVCC_INTVALUE stIntVal;
	//MVCC_FLOATVALUE stFloatVal;
	MV_CC_DEVICE_INFO_LIST stDeviceList;
};
}

#endif