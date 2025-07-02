#include <iostream>
#include <string>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PointStamped.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include "grasp_helper/CamToReal.h"

using namespace cv;
using namespace std;

class ImageConverter
{
private:
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_depth; //接收深度图像
	// image_transport::Subscriber image_sub_color; //接收彩色图像

	ros::Subscriber camera_info_sub_; //接收深度图像对应的相机参数话题

	ros::ServiceServer CamtoReal;
	double dist, depth;
	int window_size;
	int mid_pos[2];

	sensor_msgs::CameraInfo camera_info;
	geometry_msgs::PointStamped output_point;

	/* Mat depthImage,colorImage; */
	// Mat colorImage;
	Mat depthImage = Mat::zeros(480, 640, CV_16UC1); // camera_info

public:
	ImageConverter() : it_(nh_)
	{
		try
		{
			ros::NodeHandle private_nh("~");
			private_nh.param("window_size", window_size, 20);
			image_sub_depth = it_.subscribe("/camera/aligned_depth_to_color/image_raw",
											1, &ImageConverter::imageDepthCb, this);
			camera_info_sub_ = nh_.subscribe("/camera/aligned_depth_to_color/camera_info", 1,
											 &ImageConverter::cameraInfoCb, this);
			CamtoReal = nh_.advertiseService("/cam_to_real", &ImageConverter::CamtoRealCallback, this);
		}
		catch (std::exception &e)
		{
			ROS_WARN_STREAM(e.what());
		}
	}

	~ImageConverter()
	{
	}

	void cameraInfoCb(const sensor_msgs::CameraInfo &msg)
	{
		camera_info = msg;
	}

	void imageDepthCb(const sensor_msgs::ImageConstPtr &msg)
	{
		cv_bridge::CvImagePtr cv_ptr;

		try
		{
			cv_ptr =
				cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
			depthImage = cv_ptr->image;
		}
		catch (cv_bridge::Exception &e)
		{
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}
	}

	bool CamtoRealCallback(grasp_helper::CamToReal::Request &req,
						   grasp_helper::CamToReal::Response &res)
	{
		std::cout <<"x" <<req.pixel_x << std::endl;
		std::cout << "y"<<req.pixel_y << std::endl;
		if(req.pixel_x > 639 || req.pixel_y > 479) return false;

		mid_pos[0] = req.pixel_x;
		mid_pos[1] = req.pixel_y;

		int get_distance = 0;

		for (int i = 0; i < window_size && get_distance != 1; i++)
		{
			for (int j = mid_pos[0] - i; j <= mid_pos[0] + i && get_distance != 1; j++)
			{

					dist = depthImage.at<u_int16_t>(mid_pos[1], j);
					if (dist != 0)
					{
						depth = 0.001 * dist;
						mid_pos[0] = j;
						mid_pos[1] = mid_pos[1];
						get_distance = 1;
						break;
					}

			}
		}
		if (get_distance == 1)
		{
			float real_z = depth;
			float real_x =
				(mid_pos[0] - camera_info.K.at(2)) / camera_info.K.at(0) * real_z;
			float real_y =
				(mid_pos[1] - camera_info.K.at(5)) / camera_info.K.at(4) * real_z;
			res.obj_x = real_x;
			res.obj_y = real_y;
			res.obj_z = real_z;
		}

		dist = 0;
	}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "cam_to_real");
	ImageConverter imageconverter;
	ros::AsyncSpinner spinner(1);
	spinner.start();
	ros::waitForShutdown();
}
