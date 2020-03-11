#include "CameraApi.h" //相机SDK的API头文件

#include <ros/ros.h>  
#include <opencv2/core/core.hpp>
#include <stdio.h>
#include <iostream>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

using namespace cv;
using namespace std;

unsigned char           * g_pRgbBuffer;     //处理后数据缓存区

int main(int argc, char**argv)
{

    int                     iCameraCounts = 1;
    int                     iStatus=-1;
    tSdkCameraDevInfo       tCameraEnumList;
    int                     hCamera;
    tSdkCameraCapbility     tCapability;      //设备描述信息
    tSdkFrameHead           sFrameInfo;
    BYTE*                   pbyBuffer;
    int                     iDisplayFrames = 1000;
    IplImage *iplImage = NULL;
    int                     channel=3;
    tSdkImageResolution     psCurVideoSize;
    

    CameraSdkInit(1);

    //枚举设备，并建立设备列表
    iStatus = CameraEnumerateDevice(&tCameraEnumList,&iCameraCounts);
    printf("state = %d\n", iStatus);

    printf("count = %d\n", iCameraCounts);
    //没有连接设备
    if(iCameraCounts==0){
        return -1;
    }

    //相机初始化。初始化成功后，才能调用任何其他相机相关的操作接口
    iStatus = CameraInit(&tCameraEnumList,-1,-1,&hCamera);

    //初始化失败
    printf("state = %d\n", iStatus);
    if(iStatus!=CAMERA_STATUS_SUCCESS){
        return -1;
    }

    //获得相机的特性描述结构体。该结构体中包含了相机可设置的各种参数的范围信息。决定了相关函数的参数
    CameraGetCapability(hCamera,&tCapability);

    //
    g_pRgbBuffer = (unsigned char*)malloc(tCapability.sResolutionRange.iHeightMax*tCapability.sResolutionRange.iWidthMax*3);
    //g_readBuf = (unsigned char*)malloc(tCapability.sResolutionRange.iHeightMax*tCapability.sResolutionRange.iWidthMax*3);

    /*让SDK进入工作模式，开始接收来自相机发送的图像
    数据。如果当前相机是触发模式，则需要接收到
    触发帧以后才会更新图像。    */
    CameraPlay(hCamera);

    /*其他的相机参数设置
    例如 CameraSetExposureTime   CameraGetExposureTime  设置/读取曝光时间
         CameraSetImageResolution  CameraGetImageResolution 设置/读取分辨率
         CameraSetGamma、CameraSetConrast、CameraSetGain等设置图像伽马、对比度、RGB数字增益等等。
         更多的参数的设置方法，，清参考MindVision_Demo。本例程只是为了演示如何将SDK中获取的图像，转成OpenCV的图像格式,以便调用OpenCV的图像处理函数进行后续开发
    */
    CameraGetImageResolution(hCamera,&psCurVideoSize);
    cout << " width is " << psCurVideoSize.iWidth << endl;
    cout << " height is " << psCurVideoSize.iHeight << endl;


    if(tCapability.sIspCapacity.bMonoSensor){
        channel=1;
        CameraSetIspOutFormat(hCamera,CAMERA_MEDIA_TYPE_MONO8);
    }else{
        channel=3;
        CameraSetIspOutFormat(hCamera,CAMERA_MEDIA_TYPE_BGR8);
    }

    ros::init(argc, argv, "image_publisher");  
    ros::NodeHandle nh;  
    image_transport::ImageTransport it(nh);  
    image_transport::Publisher pub = it.advertise("camera/image", 1);
    sensor_msgs::ImagePtr msg;

    //ros::Rate loop_rate(5);  
    //while (nh.ok())
    //循环显示1000帧图像
    //while(iDisplayFrames--)
    string imgfile,str;
    int count = 1385030400;
    ros::Rate loop_rate(50); 
    while (nh.ok())
    {
        if(CameraGetImageBuffer(hCamera,&sFrameInfo,&pbyBuffer,1000) == CAMERA_STATUS_SUCCESS)
        {
            CameraImageProcess(hCamera, pbyBuffer, g_pRgbBuffer,&sFrameInfo);
            
            cv::Mat matImage(
                    cvSize(sFrameInfo.iWidth,sFrameInfo.iHeight), 
                    sFrameInfo.uiMediaType == CAMERA_MEDIA_TYPE_MONO8 ? CV_8UC1 : CV_8UC3,
                    g_pRgbBuffer
                    );
            //imshow("Opencv Demo", matImage);
            msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", matImage).toImageMsg();  
            pub.publish(msg);
            //waitKey(1);
            namedWindow("matImage",CV_WINDOW_NORMAL);
            imshow("matImage",matImage);
            //在成功调用CameraGetImageBuffer后，必须调用CameraReleaseImageBuffer来释放获得的buffer。
            //否则再次调用CameraGetImageBuffer时，程序将被挂起一直阻塞，直到其他线程中调用CameraReleaseImageBuffer来释放了buffer
            CameraReleaseImageBuffer(hCamera,pbyBuffer);
            char key = static_cast<char>(waitKey(1));
            if(key == 27) break;
            if(key=='w'||key=='W')
            {   
                stringstream ss;
                ss << count;
                ss >> str;
                imgfile="/home/reid/Desktop/tem_folder/mv_cali_image_20191210/cam0/"+str+"000000000.png";
                imwrite(imgfile,matImage);
                count++;
                cout << "write image_"+str+" done" << endl;     
            }   

        }
        ros::spinOnce();  
        loop_rate.sleep();  
    }

    CameraUnInit(hCamera);
    //注意，现反初始化后再free
    free(g_pRgbBuffer);


    return 0;
}

