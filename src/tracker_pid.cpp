#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Twist.h"

class ball_tracker{
public:
  const double DISTANCE_PSC = -10;
  const double ANGLE_PSC = -1;
  double _curr_d;
  double _curr_a;

  ball_tracker(): _on(false), _integrator_d(0.0),_integrator_a(0.0) {}

  bool isOn(void){return _on;}
  void setParams(const double kp_d, const double ki_d, const double int_max_d,
                 const double kp_a, const double ki_a, const double int_max_a);
  void switchOn(bool on) {_on = on;}
  void setDistance(const double target) { _target_d = target;}
  void output_zero(void)
  {
    _curr_d = _target_d;
    _curr_a = 400;
    _integrator_d = 0.0;
    _integrator_a = 0.0;
  }

  double pid_control_d(void);
  double pid_control_a(void);

private:
  bool  _on;
  double _target_d;
  double _kp_d;
  double _ki_d;
  double _id_max;
  double _integrator_d;

  double _kp_a;
  double _ki_a;
  double _ia_max;
  double _integrator_a;

};

void ball_tracker::setParams(const double kp_d, const double ki_d, const double int_max_d,
                             const double kp_a, const double ki_a, const double int_max_a)
{
  _kp_d = kp_d;
  _ki_d = ki_d;
  _id_max = int_max_d;
  _kp_a = kp_a;
  _ki_a = ki_a;
  _ia_max = int_max_a;
}

double ball_tracker::pid_control_d(void)
{
  double error = (_target_d - _curr_d) * DISTANCE_PSC;
  _integrator_d += error * _ki_d;

  if(_integrator_d > _id_max)
    _integrator_d = _id_max;
  else if(_integrator_d < -_id_max)
    _integrator_d = -_id_max;

//  printf("d:%f\r\n",_target_d - _curr_d);
  return error*_kp_d + _integrator_d;
}

double ball_tracker::pid_control_a(void)
{
  double error = (400 -_curr_a) * ANGLE_PSC;
  _integrator_a += error * _ki_a;

  if(_integrator_a > _ia_max)
    _integrator_a = _ia_max;
  else if(_integrator_a < -_ia_max)
    _integrator_a = -_ia_max;

//  printf("a:%f\r\n",400 -_curr_a);
  return error *_kp_a + _integrator_a;
}

static ball_tracker tracker;
ros::Publisher pid_pub;

void switch_callback(const std_msgs::Bool::ConstPtr& msg)
{
  tracker.switchOn(msg->data);
}

void distance_callback(const std_msgs::Float64::ConstPtr& msg)
{
  tracker.setDistance(msg->data);
}

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void tracker_callback(const geometry_msgs::Point::ConstPtr& msg)
{
  if(msg->z > 0)
  {
    tracker._curr_d = 1/msg->z;
    tracker._curr_a = msg->x;
  }
  else if(tracker.isOn())
    tracker.output_zero();
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
  ros::init(argc, argv, "ball tracker pid");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  ros::Subscriber sub_switch = n.subscribe("/tracker/switch", 1000, switch_callback);
  ros::Subscriber sub_dist =   n.subscribe("/tracker/dist", 1000, distance_callback);
  ros::Subscriber sub_image =  n.subscribe("/tracker/target", 1000, tracker_callback);

  pid_pub = n.advertise<geometry_msgs::Twist>("/vrep/cmd_vel", 1000);

  double kp_d,ki_d,int_max_d,kp_a,ki_a,int_max_a, dist;
  ros::param::get("/elec4010_tracker_pid/kp_d", kp_d);
  ros::param::get("/elec4010_tracker_pid/ki_d", ki_d);
  ros::param::get("/elec4010_tracker_pid/int_max_d", int_max_d);
  ros::param::get("/elec4010_tracker_pid/kp_a", kp_a);
  ros::param::get("/elec4010_tracker_pid/ki_a", ki_a);
  ros::param::get("/elec4010_tracker_pid/int_max_a", int_max_a);

  ros::param::get("/elec4010_tracker_pid/dist_sp", dist);

  tracker.setParams(kp_d,ki_d,int_max_d,kp_a,ki_a,int_max_a);
  tracker.setDistance(dist);

  ros::Rate r(1000);

  geometry_msgs::Twist msg;

  msg.linear.x = 0.0;
  msg.linear.y = 0.0;
  msg.linear.z = 0.0;
  msg.angular.x = 0.0;
  msg.angular.y = 0.0;
  msg.angular.z = 0.0;

  while (ros::ok()) {
      if(tracker.isOn())
      {
        msg.linear.x = tracker.pid_control_d();
        msg.angular.z = tracker.pid_control_a();

        if(msg.linear.x > 0.8)
          msg.linear.x = 0.8;
        else if(msg.linear.x < -0.5)
          msg.linear.x = -0.5;

        if(msg.angular.z > 1)
          msg.angular.z = 1;
        else if(msg.angular.z < -1)
          msg.angular.z = -1;

        //printf("d:%f\r\na:%f\r\n",msg.linear.x,msg.angular.z);
        pid_pub.publish(msg);
      }
      r.sleep();
      ros::spinOnce();
  }

  return 0;
}
