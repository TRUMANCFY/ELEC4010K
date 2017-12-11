#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
using namespace ros;
#define LASER_POINT_NUM 900

class Autodriver {

public:
    Autodriver(NodeHandle &n, const double min, const double nodetect, const double weight):
    min_distance(min), nodetect_distance(nodetect), WEIGHT_SUM(weight)
    {
      sub = n.subscribe("/vrep/scan", 1000, &Autodriver::scan_callback, this);
      pub = n.advertise<geometry_msgs::Twist>("/vrep/cmd_vel", 1000);

      double mean = WEIGHT_SUM / LASER_POINT_NUM;
      double delta = mean / (LASER_POINT_NUM/2 + 0.5);

      double L,R;
      L = mean + delta * (LASER_POINT_NUM/2);
      R = mean - delta * (LASER_POINT_NUM/2);

      for(int i = 0; i < LASER_POINT_NUM; i++)
      {
        braitenbergL[i] = L - delta;
        braitenbergR[i] = R;
        L -= delta;
        R += delta;
      }

      double derivative = 1/(min-nodetect);
      _k = -derivative*min*min;
      _n = 1 - _k/min;
    }

    void scan_callback(const sensor_msgs::LaserScan::ConstPtr&);
    void drive(void);

    double v_linear;
    double v_angular;
private:
    Subscriber sub;
    Publisher pub;

    const double min_distance;
    const double nodetect_distance;
    double _k,_n;

    double braitenbergL[LASER_POINT_NUM];
    double braitenbergR[LASER_POINT_NUM];
    const double WEIGHT_SUM;

    void scaleOutput(const double linear, const double angular)
    {
      if(linear > 0.8)
        v_linear = 0.8;
      else if(v_linear < -0.5)
        v_linear = -0.5;
      else
        v_linear = linear;

      if(v_angular > 1)
        v_angular = 1;
      else if(v_angular < -1)
        v_angular = -1;
      else
        v_angular = angular;
    }
};
/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void Autodriver::scan_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  double dist,detect, vleft = 3, vright = 3;
  for (int i = 0; i < LASER_POINT_NUM; i++)
  {
    dist = msg->ranges[i];
    if(dist > nodetect_distance)
      dist = nodetect_distance;

    //Penalty function
    if(dist < min_distance)
      detect = _k/dist + _n;
    else
      detect = 1-((dist - min_distance)/(nodetect_distance - min_distance));

    vleft  += detect * braitenbergR[i];
    vright += detect * braitenbergL[i]; //R and L reversed QwQ, too lazy to switch back
  }

  double linear = (vleft + vright) / 12;
  double angular = (vright - vleft)*5/6;

  scaleOutput(linear, angular);
}

void Autodriver::drive(void)
{
  geometry_msgs::Twist msg;
  msg.linear.x = v_linear;
  msg.linear.y = 0;
  msg.linear.z = 0;
  msg.angular.x = 0;
  msg.angular.y = 0;
  msg.angular.z = v_angular;

  pub.publish(msg);
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
  double min,nodetect,weight;
  ros::param::get("/elec4010_autodriver/min", min);
  ros::param::get("/elec4010_autodriver/nodetect", nodetect);
  ros::param::get("/elec4010_autodriver/weight", weight);

  Autodriver autodriver(n, min, nodetect, weight);

  Rate r(100);
  while (ok()) {
      printf("%f,%f\n",autodriver.v_linear, autodriver.v_angular);

      autodriver.drive();
      r.sleep();
      spinOnce();
  }

  return 0;
}
