// %Tag(FULL)%
// %Tag(INCLUDE)%
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <joy/Joy.h>
// %EndTag(INCLUDE)%
// %Tag(CLASSDEF)%
class TeleopYouBot
{
public:
  TeleopYouBot();

private:
  void joyCallback(const joy::Joy::ConstPtr& joy);
  
  ros::NodeHandle nh_;

  int linear_x, linear_y;
  int angular_z;
  double l_scale_, a_scale_;
  ros::Publisher vel_pub_;
  ros::Subscriber joy_sub_;
  
};
// %EndTag(CLASSDEF)%
// %Tag(PARAMS)%
TeleopYouBot::TeleopYouBot():
  linear_x(1),linear_y(0),
  angular_z(2), l_scale_(1), a_scale_(1)
{

  nh_.param("axis_linear_x", linear_x, linear_x);
  nh_.param("axis_linear_y", linear_y, linear_y);
  nh_.param("axis_angular_z", angular_z, angular_z);

  nh_.param("scale_angular", a_scale_, a_scale_);
  nh_.param("scale_linear", l_scale_, l_scale_);
  // %EndTag(PARAMS)%
  // %Tag(PUB)%
  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  // %EndTag(PUB)%
  // %Tag(SUB)%
  joy_sub_ = nh_.subscribe<joy::Joy>("joy", 10, &TeleopYouBot::joyCallback, this);
  // %EndTag(SUB)%
}
// %Tag(CALLBACK)%
void TeleopYouBot::joyCallback(const joy::Joy::ConstPtr& joy)
{
  geometry_msgs::Twist vel;
  vel.linear.x = l_scale_ * joy->axes[linear_x];
  vel.linear.y = l_scale_ * joy->axes[linear_y];
  vel.angular.z = a_scale_ * joy->axes[angular_z];
  vel_pub_.publish(vel);
}
// %EndTag(CALLBACK)%
// %Tag(MAIN)%
int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_youbot");
  TeleopYouBot teleop_youbot;

  ros::spin();
}
// %EndTag(MAIN)%
// %EndTag(FULL)%
