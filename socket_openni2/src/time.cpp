#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <time.h>
#include <iostream>
#include <string>
#include <cstring>
#include <sstream>
 
using namespace std;
//由struct timeval结构体数据（由gettimeofday获取到的）转换成可显示的时间字符串
static char * get_local_time(char *time_str, int len, struct timeval *tv)
{
    struct tm* ptm;
    char time_string[40];
    long milliseconds;
    
    ptm = localtime (&(tv->tv_sec));
 
    /* 格式化日期和时间，精确到秒为单位。*/
    //strftime (time_string, sizeof(time_string), "%Y/%m/%d %H:%M:%S", ptm); //输出格式为: 2018/12/09 10:48:31.391
    //strftime (time_string, sizeof(time_string), "%Y|%m|%d %H-%M-%S", ptm); //输出格式为: 2018|12|09 10-52-28.302
    strftime (time_string, sizeof(time_string), "%Y-%m-%d %H:%M:%S", ptm); //输出格式为: 2018-12-09 10:52:57.200
    //strftime (time_string, sizeof(time_string), "%Y\\%m\\%d %H-%M-%S", ptm); //输出格式为: 2018\12\09 10-52-28.302
 
    /* 从微秒计算毫秒。*/
    milliseconds = tv->tv_usec / 1000;
    stringstream tem_ss;
    string tem_str_int ;
    tem_ss << milliseconds ;
    tem_ss >> tem_str_int;
    //tem_ss << time_string;//<< "." << milliseconds;
    //tem_ss << "." ;
    //tem_ss >> tem_str;
    const char *tem_char = ".";
    string tem_str = time_string ;//+ (char)milliseconds;
    tem_str+=tem_char;
    tem_str+=tem_str_int;
    std::cout << tem_str<< std::endl;
   // std::cout << time_string<< std::endl;
    /* 以秒为单位打印格式化后的时间日期，小数点后为毫秒。*/
    snprintf (time_str, len, "%s.%03ld", time_string, milliseconds);
 
    return time_str;
}
 
int main(int argc, const char **argv)
{
    char local_time_str[128];
    char *p = NULL;
    struct timeval tv;
 
    gettimeofday(&tv, NULL);
    p = get_local_time(local_time_str, sizeof(local_time_str), &tv);
    printf("Get local time: \n%s\n", p);
 
    return 0;
}