#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
using namespace ros;

class Autodriver {

public:
    Autodriver(NodeHandle &n, const double min, const double nodetect): min_distance(min), nodetect_distance(nodetect)
    {
      sub = n.subscribe("/vrep/scan", 1000, &Autodriver::scan_callback, this);
    }

    void scan_callback(const sensor_msgs::LaserScan::ConstPtr&);

private:
    Subscriber sub;

    double detect[50] = { 0.0 };
    const double min_distance;
    const double nodetect_distance;

    const double braitenbergL[50];
    const double braitenbergR[50];
    const double WEIGHT_SUM = -7.2;
};
/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void Autodriver::scan_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{

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
  init(argc, argv, "autodriver");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  NodeHandle n;
  double min,nodetect;

  Autodriver autodriver(n);

  spin();

  return 0;
}
