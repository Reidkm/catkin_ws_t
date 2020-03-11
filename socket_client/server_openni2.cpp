#include <iostream>
#include <csignal>
#include <ctime>
#include "socket_openni2/TCPServer.h"
#include <sys/time.h>
#include <opencv2/core/core.hpp>  
#include <opencv2/highgui/highgui.hpp>  
#include <opencv2/imgproc/imgproc.hpp> 
#include <stdio.h>
#include <vector>
#include <fstream>
#include <istream>
#include <sstream>
#include "socket_openni2/AXonLink.h"
#include "socket_openni2/OpenNI.h"
#include "socket_openni2/test_m5_openni2_class.h"

using namespace std;
using namespace cv;
using namespace openni;

TCPServer tcp;
double length;
pthread_t msg1[MAX_CLIENT];
int num_message = 0;
int time_send   = 2700;
openni::Device device;
openni::VideoStream depth, color;
SampleViewer sampleViewer("AXon Depth Viewer", device, depth,color);

void close_app(int s) {
	tcp.closed();
	exit(0);
}

void * send_client(void * m) {
        struct descript_socket *desc = (struct descript_socket*) m;

	while(1) {
		if(!tcp.is_online() && tcp.get_last_closed_sockets() == desc->id) {
			cerr << "Connessione chiusa: stop send_clients( id:" << desc->id << " ip:" << desc->ip << " )"<< endl;
			break;
		}
		struct timeval tv;
		//tv.tv_sec

		//std::time_t t = std::time(0);
		tv.tv_sec = std::time(0);
		//std::tm* now = std::localtime(&t);
		gettimeofday(&tv, NULL);
		std::tm* now = std::localtime(&tv.tv_sec);
		int hour = now->tm_hour;
		int min  = now->tm_min;
		int sec  = now->tm_sec;

		std::string date = 
			    to_string(now->tm_year + 1900) + "-" +
			    to_string(now->tm_mon + 1)     + "-" +
			    to_string(now->tm_mday)        + " " +
			    to_string(hour)                + ":" +
			    to_string(min)                 + ":" +
			    to_string(sec)                 + "." +
				to_string(tv.tv_usec/1000)          + "\r\n";
				
				//to_string(sec)                 + "\r\n";
		cerr << date << endl;
		tcp.Send(date, desc->id);  // 
		sleep(time_send);
		//usleep(20000);
	}
	pthread_exit(NULL);
	return 0;
}

void * received(void * m)
{
        pthread_detach(pthread_self());    //release thread resource 
	vector<descript_socket*> desc;
	while(1)
	{
		desc = tcp.getMessage();     // lock thread  return message
		if (depth.isValid() && color.isValid())
		{
			length = sampleViewer.display();
			char key = static_cast<char>(waitKey(1));
			if(key == 'q' || key == 'Q') break;
		
		}

		for(unsigned int i = 0; i < desc.size(); i++) {
			if( desc[i]->message != "" )
			{
				if(!desc[i]->enable_message_runtime) 
				{
					desc[i]->enable_message_runtime = true;
			                if( pthread_create(&msg1[num_message], NULL, send_client, (void *) desc[i]) == 0) {   // start send thread
						cerr << "ATTIVA THREAD INVIO MESSAGGI" << endl;
					}
					num_message++;
					// start message background thread
				}
				cout << "id:      " << desc[i]->id      << endl
				     << "ip:      " << desc[i]->ip      << endl
				     << "message: " << desc[i]->message << endl
				     << "socket:  " << desc[i]->socket  << endl
				     << "enable:  " << desc[i]->enable_message_runtime << endl;
				tcp.clean(i);
			}
		}
		usleep(1000);
	}

	depth.stop();
	depth.destroy();
	device.close();
	OpenNI::shutdown();
	return 0;
}

int main(int argc, char **argv)
{
	openni::Status rc = openni::STATUS_OK;

	
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
	
    
    rc = sampleViewer.init();
	if (rc != openni::STATUS_OK)
	{
		
		openni::OpenNI::shutdown();
		return 3;
	}


	if(argc < 2) {
		cerr << "Usage: ./server port (opt)time-send" << endl;
		return 0;
	}
	if(argc == 3)
		time_send = atoi(argv[2]);   //  tranfer stirng to int number 
	std::signal(SIGINT, close_app);    //  catch SIGINT, deal in close_app

	pthread_t msg;    // thread ID 
        vector<int> opts = { SO_REUSEPORT, SO_REUSEADDR };  //mutil thread binding the same port 

	if( tcp.setup(atoi(argv[1]),opts) == 0) {         // socket found, bind ,listen 
		if( pthread_create(&msg, NULL, received, (void *)0) == 0)    // thread  found  0:success
		{
			while(1) {
				tcp.accepted();
				cerr << "Accepted" << endl;
			}
		}
	}
	else
		cerr << "Errore apertura socket" << endl;
	return 0;
}
