#define _CRT_SECURE_NO_WARNINGS
#include <string.h>
#include <iostream>
#include <thread>
#include <mutex>
#include <signal.h>
#include <unistd.h>
#include "./include/TCPClient.h"
#include <csignal>
#include <ctime>
#include <sys/time.h>
#include <opencv2/core/core.hpp>  
#include <opencv2/highgui/highgui.hpp>  
#include <opencv2/imgproc/imgproc.hpp> 
#include <stdio.h>
#include <vector>
#include <fstream>
#include <istream>
#include <sstream>
#include "./include/AXonLink.h"
#include "./include/OpenNI.h"
#include "./include/test_m5_openni2_class.h"

using namespace std;
using namespace cv;
using namespace openni;



//double test_number = 100;
TCPClient tcp;
double length;

void sig_exit(int s)
{
	tcp.exit();
	exit(0);   // exit program
}

int main(int argc, char *argv[])
{
	if(argc != 3) {
		cerr << "Usage: ./client ip port " << endl;
		return 0;
	}
	openni::Status rc = openni::STATUS_OK;
	openni::Device device;
	openni::VideoStream depth, color;

    openni::VideoFrameRef  depthFrame;

	const char* deviceURI = openni::ANY_DEVICE;
	//printf("%0x\n",deviceURI);
	std::cout << "deviceURI is " << static_cast<const void*>(deviceURI) << std::endl;
	
	//if (argc > 1)
	//{
	//	deviceURI = argv[1];
	//}

	rc = openni::OpenNI::initialize();

	printf("After initialization:\n%s\n", openni::OpenNI::getExtendedError());

	rc = device.open(deviceURI);
	if (rc != openni::STATUS_OK)
	{
		printf("SimpleViewer: Device open failed:\n%s\n", openni::OpenNI::getExtendedError());
		openni::OpenNI::shutdown();
		return 1;
	}

	rc = depth.create(device, openni::SENSOR_DEPTH);
	if (rc == openni::STATUS_OK)
	{
		rc = depth.start();
		if (rc != openni::STATUS_OK)
		{
			printf("SimpleViewer: Couldn't start depth stream:\n%s\n", openni::OpenNI::getExtendedError());
			depth.destroy();
		}
	}
	else
	{
		printf("SimpleViewer: Couldn't find depth stream:\n%s\n", openni::OpenNI::getExtendedError());
	}

	rc = color.create(device, openni::SENSOR_COLOR);
	if (rc == openni::STATUS_OK)
	{
		rc = color.start();
	 	if (rc != openni::STATUS_OK)
	 	{
	 		printf("SimpleViewer: Couldn't start color stream:\n%s\n", openni::OpenNI::getExtendedError());
	 		color.destroy();
	 	}
	 }
	 else
	 {
	 	printf("SimpleViewer: Couldn't find color stream:\n%s\n", openni::OpenNI::getExtendedError());
	 }

	if (!depth.isValid() || !color.isValid())
	{
		printf("SimpleViewer: No valid streams. Exiting\n");
		openni::OpenNI::shutdown();
		return 2;
	}
	
	device.setDepthColorSyncEnabled(true);
	//device.setImageRegistrationMode(IMAGE_REGISTRATION_COLOR_TO_DEPTH);

	
	AXonCropping depthAC ;
	depthAC.originX = 0;
	depthAC.originY = 0;
	depthAC.width = 640;
	depthAC.height = 480;
	depthAC.gx = 0;
	depthAC.gy = 0;
	depth.setProperty(AXONLINK_STREAM_PROPERTY_CROPPING,depthAC);
	
    SampleViewer sampleViewer("AXon Depth Viewer", device, depth,color);

    rc = sampleViewer.init();
	if (rc != openni::STATUS_OK)
	{
		
		openni::OpenNI::shutdown();
		return 3;
	}



    

    signal(SIGINT, sig_exit);    // catch SIGINT signal then deal in sig_exit()

    tcp.setup(argv[1],atoi(argv[2]));      // //  tranfer stirng to int number  //set   socket 

	while(true)
	{
		if (depth.isValid() && color.isValid())
		{
			length = sampleViewer.display();
			char key = static_cast<char>(waitKey(1));
			if(key == 'q' || key == 'Q') break;
		
		}
        std::stringstream tem_ss;
        string tem_str;
        tem_ss << length;
        tem_ss >> tem_str;
        tcp.Send(tem_str); 
		string rec = tcp.receive();    //receive server message

		// jerry: add kill message to stop process
		if( rec.compare("kill") != 0)
		{
			cout << rec << endl;
		} 
		else{
			cout << "killed" << endl;
			break;
		}

		//if( rec != "" )
		//{
		//	cout << rec << endl;
		//}
		//test_number = test_number + 0.1 ;
		//sleep(0.5);
		//usleep(500);   
		// jerry: sleep for 5 second
		usleep(1000 * 1000);   
	}

	//system("pause");
	return 0;
}


/*

				std::fstream outfile;
					outfile.open("data.txt",ios::out|ios::app);
					outfile<<"行数为: " << i  << " ,货物边界列数为:  " << j+4 <<std::endl;
					for (int k = 0; k < 350; k++)
					{
						outfile << "ptr[ " << k << "]: " << ptr[k] <<std::endl;  
					}

					time_t now = time(0);
					tm *ltm = localtime(&now);
					//cout << "时间: "<< ltm->tm_hour << ":";
   					//	cout << ltm->tm_min << ":";
   					//	cout << ltm->tm_sec << endl;
					outfile << "时间: "<< ltm->tm_hour << ":" << ltm->tm_min << ":" << ltm->tm_sec << endl;
   						
					outfile << "******************************" << std::endl;
					outfile.close();

*/

