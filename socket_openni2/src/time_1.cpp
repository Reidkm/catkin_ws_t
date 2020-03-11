#include <stdio.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>
#include <iostream>
/*
取当前时间，精确到微秒 ；
*/
//using namespace 
int main()
{
    struct timeval tv;
	struct tm *     time_ptr;
    memset(&tv, 0, sizeof(timeval));
    gettimeofday(&tv, NULL);
    time_ptr = localtime(&tv.tv_sec);
    std::cout <<time_ptr->tm_year + 1900 
        << "-"<< time_ptr->tm_mon + 1 << "-"<< time_ptr->tm_mday<< " "
        << time_ptr->tm_hour<<":"
        <<time_ptr->tm_min << ":"
        <<time_ptr->tm_sec<<":"
        <<tv.tv_usec << std::endl; 
    /*
     printf("%d-%02d-%02d %02d:%02d:%02d.%.04d\n",
            time_ptr->tm_year + 1900,
            time_ptr->tm_mon + 1,
            time_ptr->tm_mday,
            time_ptr->tm_hour,
            time_ptr->tm_min,
            time_ptr->tm_sec,
			tv.tv_usec);
            */
	getchar();
    return 0;
}
