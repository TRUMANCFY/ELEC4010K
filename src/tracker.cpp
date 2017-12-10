#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "geometry_msgs/Point.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>

#include <iostream>
#include <vector>

using namespace cv;

ros::Publisher target_pub;
/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void image_callback(const sensor_msgs::Image::ConstPtr& msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

  Mat image_th,image_c;
  copyMakeBorder(cv_ptr->image, image_c, 0, 200, 150, 150, BORDER_CONSTANT, Scalar(0,0,0));
  inRange(image_c,Scalar(0,100,100), Scalar(40,255,255),image_th);

  std::vector<std::vector<Point> > contours;
  findContours(image_th, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE );

  Point2f center,center_m;
  float radius,radius_m = 0;
  for (int i = 0; i < contours.size(); i++)
  {
      minEnclosingCircle(contours[i], center, radius);
      if(radius > radius_m)
      {
        radius_m = radius;
        center_m = center;
      }
  }
  circle(image_c, center_m, radius_m, Scalar(0,255,0), 10);

  geometry_msgs::Point pub;
  if(radius_m > 15)
  {
    pub.x = center_m.x;
    pub.y = center_m.y;
    pub.z = radius_m;
  }
  else
    pub.z = -1;

  target_pub.publish(pub);

  imshow("ImageC",image_c);
  waitKey(10);
}

int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "ball tracker");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/vrep/image", 1000, image_callback);
  target_pub = n.advertise<geometry_msgs::Point>("/tracker/target", 1000);

  namedWindow( "ImageC", WINDOW_AUTOSIZE );// Create a window for display.

  ros::spin();

  return 0;
}
