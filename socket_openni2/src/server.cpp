#include <iostream>
#include <csignal>
#include <ctime>
#include "socket_openni2/TCPServer.h"
#include <sys/time.h>

TCPServer tcp;
pthread_t msg1[MAX_CLIENT];
int num_message = 0;
int time_send   = 2700;

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
		//std::cout << "/* message */" << std::endl;
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
	return 0;
}

int main(int argc, char **argv)
{
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
