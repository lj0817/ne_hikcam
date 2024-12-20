#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <chrono>
#include <iostream>

#include "MvCameraControl.h"

#include "hik_camera_io.hpp"
namespace ne_io
{
HikCam::HikCam()
    : handle(NULL),
      nRet(MV_OK),
      pData(NULL),
      WidthValue(1280),
      HeightValue(1024),
      ExposureTimeValue(3000.0),
      GainValue(8.0)
{
}

HikCam::~HikCam()
{
    shutdown();
}

void HikCam::shutdown()
{
    if (handle != NULL)
    {
        MV_CC_DestroyHandle(handle);
        handle = NULL;
    }
    if (pData)
    {
        free(pData);
        pData = NULL;
    }

    printf("exit\n");
    return;
}

void HikCam::start()
{
    memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
    
    MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
    
    if (stDeviceList.nDeviceNum > 0)
    {
        for (int i = 0; i < stDeviceList.nDeviceNum; i++)
        {
            MV_CC_DEVICE_INFO *pDeviceInfo = stDeviceList.pDeviceInfo[i];
            if (NULL == pDeviceInfo)
            {
                shutdown();
                return;
            }
        }
    }
    else
    {
        printf("Find No Devices!\n");
        shutdown();
        return;
    }
    unsigned int nIndex = 0;
    MV_CC_CreateHandle(&handle, stDeviceList.pDeviceInfo[nIndex]);
    MV_CC_OpenDevice(handle);
    MV_CC_SetPixelFormat(handle,PixelType_Gvsp_BayerGR8);
    MV_CC_SetIntValueEx(handle, "Height", HeightValue);
    MV_CC_SetIntValueEx(handle, "Width", WidthValue);
    MV_CC_SetFloatValue(handle, "ExposureTime", ExposureTimeValue);
    MV_CC_SetFloatValue(handle, "Gain", GainValue);

    MV_CC_SetEnumValue(handle, "TriggerMode", 0);
    memset(&stParam, 0, sizeof(MVCC_INTVALUE));
    MV_CC_GetIntValue(handle, "PayloadSize", &stParam);

    MV_CC_StartGrabbing(handle);

    stImageInfo = {0};
    memset(&stImageInfo, 0, sizeof(MV_FRAME_OUT));
    pData = (unsigned char *)malloc(sizeof(unsigned char) * stParam.nCurValue);
}

void HikCam::reNewExposureStart(double a)
{
    MV_CC_StopGrabbing(handle);
    MV_CC_CloseDevice(handle);
    MV_CC_DestroyHandle(handle);

    memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
    MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
    if (stDeviceList.nDeviceNum > 0)
    {
        for (int i = 0; i < stDeviceList.nDeviceNum; i++)
        {
            //printf("[device %d]:\n", i);
            MV_CC_DEVICE_INFO *pDeviceInfo = stDeviceList.pDeviceInfo[i];
            if (NULL == pDeviceInfo)
            {
                shutdown();
                return;
            }
        }
    }
    else
    {
        printf("Find No Devices!\n");
        shutdown();
        return;
    }
    unsigned int nIndex = 0;
    MV_CC_CreateHandle(&handle, stDeviceList.pDeviceInfo[nIndex]);
    MV_CC_OpenDevice(handle);
    MV_CC_SetPixelFormat(handle,PixelType_Gvsp_BayerGR8);
    MV_CC_SetIntValueEx(handle, "Height", HeightValue);
    MV_CC_SetIntValueEx(handle, "Width", WidthValue);
    MV_CC_SetFloatValue(handle, "ExposureTime", a);
    MV_CC_SetFloatValue(handle, "Gain", GainValue);
    MV_CC_SetEnumValue(handle, "TriggerMode", 0);
    memset(&stParam, 0, sizeof(MVCC_INTVALUE));
    MV_CC_GetIntValue(handle, "PayloadSize", &stParam);
    MV_CC_StartGrabbing(handle);
    ExposureTimeValue = a;

    stImageInfo = {0};
    memset(&stImageInfo, 0, sizeof(MV_FRAME_OUT));
    pData = (unsigned char *)malloc(sizeof(unsigned char) * stParam.nCurValue);
}

void HikCam::reNewGainStart(double b)
{
    MV_CC_StopGrabbing(handle);
    MV_CC_CloseDevice(handle);
    MV_CC_DestroyHandle(handle);

    memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
    MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
    if (stDeviceList.nDeviceNum > 0)
    {
        for (int i = 0; i < stDeviceList.nDeviceNum; i++)
        {
            //printf("[device %d]:\n", i);
            MV_CC_DEVICE_INFO *pDeviceInfo = stDeviceList.pDeviceInfo[i];
            if (NULL == pDeviceInfo)
            {
                shutdown();
                return;
            }
        }
    }
    else
    {
        printf("Find No Devices!\n");
        shutdown();
        return;
    }
    unsigned int nIndex = 0;
    MV_CC_CreateHandle(&handle, stDeviceList.pDeviceInfo[nIndex]);
    MV_CC_OpenDevice(handle);
    MV_CC_SetPixelFormat(handle,PixelType_Gvsp_BayerGR8);
    MV_CC_SetIntValueEx(handle, "Height", HeightValue);
    MV_CC_SetIntValueEx(handle, "Width", WidthValue);
    MV_CC_SetFloatValue(handle, "ExposureTime", ExposureTimeValue);
    MV_CC_SetFloatValue(handle, "Gain", b);
    MV_CC_SetEnumValue(handle, "TriggerMode", 0);
    memset(&stParam, 0, sizeof(MVCC_INTVALUE));
    MV_CC_GetIntValue(handle, "PayloadSize", &stParam);
    MV_CC_StartGrabbing(handle);
    GainValue = b;


    stImageInfo = {0};
    memset(&stImageInfo, 0, sizeof(MV_FRAME_OUT));
    pData = (unsigned char *)malloc(sizeof(unsigned char) * stParam.nCurValue);
}

void HikCam::getImg(){
    MV_CC_FreeImageBuffer(handle, &stImageInfo);
    MV_CC_GetImageBuffer(handle, &stImageInfo, 1000);
    //printf("Get Image Buffer: Width[%d], Height[%d], FrameNum[%d]\n", stImageInfo.stFrameInfo.nWidth, stImageInfo.stFrameInfo.nHeight, stImageInfo.stFrameInfo.nFrameNum);
}

void HikCam::reConnectStart()
{
    MV_CC_StopGrabbing(handle);
    MV_CC_CloseDevice(handle);
    MV_CC_DestroyHandle(handle);

    toConnect();

    unsigned int nIndex = 0;
    MV_CC_CreateHandle(&handle, stDeviceList.pDeviceInfo[nIndex]);
    MV_CC_OpenDevice(handle);
    MV_CC_SetPixelFormat(handle,PixelType_Gvsp_BayerGR8);
    MV_CC_SetIntValueEx(handle, "Height", HeightValue);
    MV_CC_SetIntValueEx(handle, "Width", WidthValue);
    MV_CC_SetFloatValue(handle, "ExposureTime",ExposureTimeValue );
    MV_CC_SetFloatValue(handle, "Gain", GainValue);
    MV_CC_SetEnumValue(handle, "TriggerMode", 0);
    memset(&stParam, 0, sizeof(MVCC_INTVALUE));
    MV_CC_GetIntValue(handle, "PayloadSize", &stParam);
    MV_CC_StartGrabbing(handle);

    stImageInfo = {0};
    memset(&stImageInfo, 0, sizeof(MV_FRAME_OUT));
    pData = (unsigned char *)malloc(sizeof(unsigned char) * stParam.nCurValue);
}

bool HikCam::toConnect(){
    while(1){

        unsigned int nIndex = -1;
        MV_CC_DEVICE_INFO_LIST stDeviceList = {0};
        memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
        MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);

        if (stDeviceList.nDeviceNum > 0)
        {
            for (int i = 0; i < stDeviceList.nDeviceNum; i++)
            {
                MV_CC_DEVICE_INFO* pDeviceInfo = stDeviceList.pDeviceInfo[i];
                if (NULL == pDeviceInfo)
                {
                    std::cout<<"111"<<std::endl;
                    continue;
                } 
                
                else 
                {
                    std::cout<<"reconnect successfully"<<std::endl;
                    return true;
                }
            }  
        } 
        else
        {   std::cout<<"connecting, please wait..."<<std::endl;
            continue;
        }
        
    }
}

}
