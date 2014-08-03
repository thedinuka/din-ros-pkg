#include <ros/ros.h>
#include <joy/Joy.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Empty.h>


class TeleopTurtle
{
public:
  TeleopTurtle();

private:
  void joyCallback(const joy::Joy::ConstPtr& joy);
  
  ros::NodeHandle nh_;

  int linear_, angular_;
  int state;
  double l_scale_, a_scale_;
  ros::Publisher vel_pub_;
  ros::Subscriber joy_sub_;
  ros::Publisher takeoff_pub_;
  ros::Publisher land_pub_;
  ros::Publisher reset_pub_;
  bool manual;
};

TeleopTurtle::TeleopTurtle():
  linear_(1),
  angular_(2)
{

  nh_.param("axis_linear", linear_, linear_);
  nh_.param("axis_angular", angular_, angular_);
  nh_.param("scale_angular", a_scale_, a_scale_);
  nh_.param("scale_linear", l_scale_, l_scale_);

  state = 0;
  manual = true;		//manual control
  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("joy_cmd_vel", 1);
  takeoff_pub_ = nh_.advertise<std_msgs::Empty>("ardrone/takeoff",1);
  land_pub_ = nh_.advertise<std_msgs::Empty>("ardrone/land",1);
  reset_pub_ = nh_.advertise<std_msgs::Empty>("ardrone/reset",5);

  joy_sub_ = nh_.subscribe<joy::Joy>("joy", 10, &TeleopTurtle::joyCallback, this);
}

void TeleopTurtle::joyCallback(const joy::Joy::ConstPtr& joy)
{
  geometry_msgs::Twist vel;
  vel.linear.x = 0.25*joy->axes[1];	//1
  vel.linear.y = 0.25*joy->axes[0];	//0
  vel.linear.z = 0.5*joy->axes[3];	//3
  vel.angular.z = 0.25*joy->axes[2];	//2
  vel_pub_.publish(vel);

  std_msgs::Empty emp;

  if (joy->buttons[0] == 1) {   //0   
	reset_pub_.publish(emp);
  }
  if (joy->buttons[15] == 1 && state == 0) {   //15
	takeoff_pub_.publish(emp);
	state =1;
  }
  if (joy->buttons[13] == 1) {   //13
	land_pub_.publish(emp);
	state = 0;
  }
  if (joy->buttons[12] == 1) { //12
	manual = true;
	nh_.setParam("manual",manual);
  }
  if (joy->buttons[14] == 1) {   //14
	manual = false;
	nh_.setParam("manual",manual);
  }

}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "joy_teleop");
  TeleopTurtle teleop_turtle;

  ros::spin();
}

