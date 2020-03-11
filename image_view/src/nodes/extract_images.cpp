/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#include <opencv2/highgui/highgui.hpp>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <boost/thread.hpp>
#include <boost/format.hpp>

#include <fstream>
#include<iostream>
#include"opencv2/imgproc/imgproc.hpp"
#include"opencv2/highgui/highgui.hpp"

using namespace std;
using namespace cv;
class ExtractImages
{
private:
  image_transport::Subscriber sub_;

  sensor_msgs::ImageConstPtr last_msg_;
  boost::mutex image_mutex_;

  std::string window_name_;
  boost::format filename_format_;
  int count_;
  double _time;
  double sec_per_frame_;

#if defined(_VIDEO)
  CvVideoWriter* video_writer;
#endif //_VIDEO

public:
  ExtractImages(const ros::NodeHandle& nh, const std::string& transport)
    : filename_format_(""), count_(0), _time(ros::Time::now().toSec())
  {
    std::string topic = nh.resolveName("image");
    ros::NodeHandle local_nh("~");

    std::string format_string;
    local_nh.param("filename_format", format_string, std::string("frame%04i.png"));
    filename_format_.parse(format_string);

    local_nh.param("sec_per_frame", sec_per_frame_, 0.1);

    image_transport::ImageTransport it(nh);
    sub_ = it.subscribe(topic, 1, &ExtractImages::image_cb, this, transport);

#if defined(_VIDEO)
    video_writer = 0;
#endif

    ROS_INFO("Initialized sec per frame to %f", sec_per_frame_);
  }

  ~ExtractImages()
  {
  }

  void image_cb(const sensor_msgs::ImageConstPtr& msg)
  {
    boost::lock_guard<boost::mutex> guard(image_mutex_);
	
	
    // Hang on to message pointer for sake of mouse_cb
    last_msg_ = msg;

	cv_bridge::CvImagePtr cv_ptr;
    //Convert from the ROS image message to a CvImage suitable for working with OpenCV for processing
    try
    {
        //Always copy, returning a mutable CvImage
        //OpenCV expects color images to use BGR channel order.
        cv_ptr = cv_bridge::toCvCopy(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        //if there is an error during conversion, display it
        ROS_ERROR("tutorialROSOpenCV::main.cpp::cv_bridge exception: %s", e.what());
        return;
    }

    // May want to view raw bayer data
    // NB: This is hacky, but should be OK since we have only one image CB.
	/*
    if (msg->encoding.find("bayer") != std::string::npos)
      boost::const_pointer_cast<sensor_msgs::Image>(msg)->encoding = "mono16";
	*/
    cv::Mat image(424,512,CV_16UC1);
	cv::Mat image_normal;
	//Mat M0(3,3,CV_32F)
	//cv::Mat image_normal;
    try
    {
	normalize(cv_ptr->image, image_normal, 0, 65535, NORM_MINMAX,CV_16UC1);
	image = image_normal;
	//image = cv_bridge::toCvShare(msg, "bgr8")->image;
      //image = cv_bridge::toCvShare(msg,sensor_msgs::image_encodings::TYPE_16UC1)->image;
	//normalize(image, image_normal, 0, 65535, NORM_MINMAX,CV_16UC1);
	//image = cv_bridge::toCvShare(msg,sensor_msgs::image_encodings::TYPE_32FC1)->image;
      cout << "image depth is " << image.depth() <<endl;
	cout << "image  type is " << image.type() <<endl;
	cout << "image_normal depth is " << image_normal.depth() <<endl;
	cout << "image_normal  type is " << image_normal.type() <<endl;
	cout << "cv_ptr->encoding.c_str()  type is " << cv_ptr->encoding.c_str() <<endl;
	
    } catch(cv_bridge::Exception)
    {
      ROS_ERROR("Unable to convert %s image to bgr8", msg->encoding.c_str());
    }

    double delay = ros::Time::now().toSec()-_time;
    if(delay >= sec_per_frame_)
    {
      _time = ros::Time::now().toSec();

      if (!image.empty()) {
        std::string filename = (filename_format_ % count_).str();

#if !defined(_VIDEO)
        // Save raw image if the defined file extension is ".raw", otherwise use OpenCV
        std::string file_extension = filename.substr(filename.length() - 4, 4);
        if (filename.length() >= 4 && file_extension == ".raw")
        {
          std::ofstream raw_file;
          raw_file.open(filename.c_str());
          if (raw_file.is_open() == false)
          {
            ROS_WARN_STREAM("Failed to open file " << filename);
          }
          else
          {
            raw_file.write((char*)(msg->data.data()), msg->data.size());
            raw_file.close();
          }
        }
        else
        {
	  cout << "image depth is " << image.depth() <<endl;
	  cout << "image  type is " << image.type() <<endl;
          if (cv::imwrite(filename, image) == false)
          {
            ROS_WARN_STREAM("Failed to save image " << filename);
          }
        }
#else
        if(!video_writer)
        {
            video_writer = cvCreateVideoWriter("video.avi", CV_FOURCC('M','J','P','G'),
                int(1.0/sec_per_frame_), cvSize(image->width, image->height));
        }

        cvWriteFrame(video_writer, image);
#endif // _VIDEO
	cout << "image depth is " << image.depth() <<endl;
	cout << "image  type is " << image.type() <<endl;
        ROS_INFO("Saved image %s", filename.c_str());
        count_++;
      } else {
        ROS_WARN("Couldn't save image, no data!");
      }
    }
  }
};

int main(int argc, char **argv)
{	
	cout << "start" << endl;
  ros::init(argc, argv, "extract_images", ros::init_options::AnonymousName);
  ros::NodeHandle n;
  if (n.resolveName("image") == "/image") {
    ROS_WARN("extract_images: image has not been remapped! Typical command-line usage:\n"
             "\t$ ./extract_images image:=<image topic> [transport]");
  }

  ExtractImages view(n, (argc > 1) ? argv[1] : "raw");

  ros::spin();

  return 0;
}
