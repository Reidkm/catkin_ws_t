#define _CRT_SECURE_NO_WARNINGS
#include <string.h>
#include <iostream>
#include <vector>
#include <thread>
#include <mutex>
#include <ctime>
#include <signal.h>
#include <unistd.h>
#include <sstream>
#include "socket_openni2/TCPClient.h"


using namespace std;

double test_number = 100;
TCPClient tcp;

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

    signal(SIGINT, sig_exit);    // catch SIGINT signal then deal in sig_exit()

    tcp.setup(argv[1],atoi(argv[2]));      // //  tranfer stirng to int number  //set   socket 

	while(true)
	{
        std::stringstream tem_ss;
        string tem_str;
        tem_ss << test_number;
        tem_ss >> tem_str;
        tcp.Send(tem_str); 
		string rec = tcp.receive();    //receive server message
		if( rec != "" )
		{
			cout << rec << endl;
		}
		test_number = test_number + 0.1 ;
		//sleep(0.5);
		usleep(500);   
	}

	//system("pause");
	return 0;
}








