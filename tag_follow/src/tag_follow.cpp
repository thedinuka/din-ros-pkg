#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Empty.h>
#include <ar_recog/Tag.h>
#include <ar_recog/Tags.h>

#include <iostream>

//#include "stdafx.h"
#include <fstream>
#include <sys/time.h>
#include <iomanip>

using namespace std;


class Tagger
{
public:
  Tagger();
  ~Tagger();

private:
  void tagsCallback(const ar_recog::Tags::ConstPtr& tags);
  void cmdVelCallback(const geometry_msgs::TwistConstPtr &msg);
  double y_lp_filter(double val);
  double x_lp_filter(double val);
  
  ros::NodeHandle nh_;

  double l_scale_, a_scale_;
  ros::Publisher vel_pub_;
  ros::Subscriber tags_sub_;
  ros::Subscriber cmd_sub_;  
  ros::Publisher takeoff_pub_;
  ros::Publisher land_pub_;
  ros::Publisher reset_pub_;
  bool manual;
  int vis_count;

  double y_err, y_p_err, y_d_err, y_i_err;
  double x_err, x_p_err, x_d_err, x_i_err;
  double yaw_err, yaw_p_err,yaw_d_err;
  double z_err;
  double y_val_1, y_val_2, y_val_3, y_val_4;
  double x_val_1, x_val_2, x_val_3, x_val_4;
  double f_coef_0, f_coef_1, f_coef_2, f_coef_3, f_coef_4;
  ofstream outputFile;
  ros::Time begin;

  geometry_msgs::Twist vel;
  struct timeval sys_time;

  double ctrl_vel_x;
  double ctrl_vel_y;
  double ctrl_vel_z;
  double ctrl_yaw;

};

Tagger::Tagger()
{


  outputFile.open("ctrldata.txt");
  begin = ros::Time::now();

  manual = true; //manual control
  vis_count = 0;
  y_err = y_p_err = y_d_err = y_i_err = 0.0;
  x_err = x_p_err = x_d_err = x_i_err = 0.0;
  y_val_1 = y_val_2 = y_val_3 = y_val_4 = 0.0;
  x_val_1 = x_val_2 = x_val_3 = x_val_4 = 0.0;

  yaw_err = yaw_p_err = yaw_d_err = 0.0;
  z_err = 0;

  f_coef_0 = 0.2;//0.05467108787;
  f_coef_1 = 0.2;//0.25;
  f_coef_2 = 0.2;//0.39065782426;
  f_coef_3 = 0.2;//0.25;
  f_coef_4 = 0.2;//0.05467108787;


  ctrl_vel_x = 0;
  ctrl_vel_y = 0;
  ctrl_vel_z = 0;
  ctrl_yaw = 0;  

  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  takeoff_pub_ = nh_.advertise<std_msgs::Empty>("ardrone/takeoff",1);
  land_pub_ = nh_.advertise<std_msgs::Empty>("ardrone/land",1);
  reset_pub_ = nh_.advertise<std_msgs::Empty>("ardrone/reset",5);

  tags_sub_ = nh_.subscribe<ar_recog::Tags>("tags", 10, &Tagger::tagsCallback, this);
  cmd_sub_ = nh_.subscribe<geometry_msgs::Twist>("joy_cmd_vel", 10, &Tagger::cmdVelCallback, this);

}


Tagger::~Tagger()	{
  outputFile.close();
}


void Tagger::cmdVelCallback(const geometry_msgs::TwistConstPtr &msg)
{

	ctrl_vel_x  = msg->linear.x;
	ctrl_vel_y  = msg->linear.y;
	ctrl_vel_z  = msg->linear.z;
	ctrl_yaw = msg->angular.z;

	if(manual)	{
	
		vel.linear.x = msg->linear.x;
		vel.linear.y = msg->linear.y;
		vel.linear.z = msg->linear.z;
		vel.angular.z = msg->angular.z;
		vel_pub_.publish(vel);
	}
}



void Tagger::tagsCallback(const ar_recog::Tags::ConstPtr& tag_list)
{


  cout << manual <<"\n";

  gettimeofday(&sys_time, NULL);
  double time = double(sys_time.tv_sec + double(sys_time.tv_usec)/1000000) - 1318898000;
//  printf("%lf \n", time);


  nh_.getParam("manual",manual);

  if (tag_list->tag_count != 0) {
	if(!manual)	{
		vis_count++;
		//y_err = (tag_list->tags[0].yMetric);
		y_err = y_lp_filter(tag_list->tags[0].yMetric) ;
		y_d_err = y_err - y_p_err;
		y_p_err = y_err;
		y_i_err = y_i_err + (0.6/30)*y_err + 0.0*ctrl_vel_y ;
		vel.linear.y = 1.0*(0.36*y_err + 15.4/2*y_d_err + 0.006*2*y_i_err) ;
		//vel.linear.y = (0.4*y_err);


		//x_err = (tag_list->tags[0].xMetric) -1;
		x_err = x_lp_filter(tag_list->tags[0].xMetric) - 1.8;
		x_d_err = x_err - x_p_err;
		x_p_err = x_err;
		x_i_err = x_i_err + (0.6/30)*x_err;
		vel.linear.x = (0.09*x_err + 1.5/2*x_d_err + 0.002*2*x_i_err);

		yaw_err = 0 - tag_list->tags[0].yRot;
		yaw_d_err = yaw_err - yaw_p_err;
		yaw_p_err = yaw_err;
		vel.angular.z = 0.4*yaw_err + 0.5*y_d_err;    //look towards target if its moving

		z_err = 0 - tag_list->tags[0].zMetric ;
		vel.linear.z = -1.5*z_err;
		
 	}


outputFile << fixed << setprecision(6) << time << "\t" << tag_list->tags[0].cwCorners[0] << "\t" << tag_list->tags[0].cwCorners[1]<< "\t" << tag_list->tags[0].cwCorners[2] <<"\t" << tag_list->tags[0].cwCorners[3] <<"\t" << tag_list->tags[0].cwCorners[4] <<"\t" << tag_list->tags[0].cwCorners[5] <<"\t" << tag_list->tags[0].cwCorners[6] <<"\t" << tag_list->tags[0].cwCorners[7] << "\t" << tag_list->tags[0].xMetric << "\t" << tag_list->tags[0].yMetric << "\t" << tag_list->tags[0].zMetric << endl;


  }
else if ((tag_list->tag_count == 0) && !manual) 	{

		vel.linear.x = 0.1*ctrl_vel_x;
		vel.linear.y = 0.1*ctrl_vel_y;
		//vel.linear.z = msg->linear.z;
		//vel.angular.z = msg->angular.z;

}		

if (manual) { y_i_err =0; x_i_err = 0;}
cout << vel.linear.y << endl;
if(!manual)  vel_pub_.publish(vel);


//outputFile << ros::Time::now() - begin << "\t" <<  << "\t" << vel.linear.y<< endl;
//outputFile << ros::Time::now()  << "\t" << y_err << "\t" << x_err<< endl;

}





double Tagger::y_lp_filter(double new_val)	{

	double output;
		
	output = f_coef_0*new_val + f_coef_1*y_val_1 + f_coef_2*y_val_2 + f_coef_3*y_val_3 + f_coef_4*y_val_4;
	y_val_4 = y_val_3;
	y_val_3 = y_val_2;
	y_val_2 = y_val_1;
	y_val_1 = new_val;
	return(output);
}

double Tagger::x_lp_filter(double new_val)	{

	double output;

	output = f_coef_0*new_val + f_coef_1*x_val_1 + f_coef_2*x_val_2 + f_coef_3*x_val_3 + f_coef_4*x_val_4;
	x_val_4 = x_val_3;
	x_val_3 = x_val_2;
	x_val_2 = x_val_1;
	x_val_1 = new_val;
	return(output);
}



int main(int argc, char** argv)
{
  ros::init(argc, argv, "tag_follow");
  Tagger tag_follow;

  ros::spin();
}

