#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
ros::Publisher laser_pub;

void laser_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  laser_pub.publish(msg);
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
  ros::init(argc, argv, "laser");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;
  laser_pub = n.advertise<sensor_msgs::LaserScan>("scan", 1000);
  ros::Subscriber sub = n.subscribe("/vrep/scan", 1000, laser_callback);

  ros::spin();

  return 0;
}
