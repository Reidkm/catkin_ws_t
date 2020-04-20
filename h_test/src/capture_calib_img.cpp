#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <opencv2/calib3d.hpp>
#include <sstream>
//#include <string>

using namespace cv;
using namespace std;
int main(int argc,char **argv)
{
	//VideoCapture cap("rtsp://admin:kiktech2016@192.168.1.64:554/out.h264");
	
	VideoCapture cap("rtsp://admin:kiktech2016@192.168.1.64:554/h264/ch1/main/av_stream");
	if(!cap.isOpened())
	{
		cout<< "could not open the camera"<<endl;
		return -1;
	}
	Mat img;
	string imgfile;
	int image_index = 138503021 ;
	//int count =1;
	for(;;)
	{	
		stringstream ss;
		string str;
		ss << image_index;
		ss >> str;
		cap>>img;
		if(img.empty()) break;
		namedWindow("img", WINDOW_NORMAL );
		//namedWindow("disparity", CV_WINDOW_AUTOSIZE);
		imshow("img",img);
		char key = static_cast<char>(waitKey(1));
		if(key == 27) break;
		if(key=='w'||key=='W')
		{
			imgfile="/home/reid/Desktop/tem_folder/webcamera_undistort/cali_img_283/"+str+"0000000000.png";
			imwrite(imgfile,img);
			image_index++;
			cout << "image saved " << image_index <<endl;		
		}
	}
	cout<<"finish writing"<<endl;
	return 0;
}
