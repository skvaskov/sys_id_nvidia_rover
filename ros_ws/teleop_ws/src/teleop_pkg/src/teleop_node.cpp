#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <ros_pololu_servo/MotorCommand.h>


using namespace ros_pololu_servo;


double now_angular_vel = 0;
double now_linear_vel = 0;
int linear_, angular_;
double l_scale_ = 1.0, a_scale_ = 1.0;
bool joy_processing = false;
bool reset_flag = false;

void joyCallback(const sensor_msgs::Joy::ConstPtr& joy_msg)
{   
  joy_processing = true;
  now_angular_vel = double(joy_msg->axes[angular_]);
  now_linear_vel = double(joy_msg->axes[linear_]);
  int stop_button_0 = int(joy_msg->buttons[0]);
  int stop_button_1 = int(joy_msg->buttons[1]);
  int stop_button_2 = int(joy_msg->buttons[2]);
  int stop_button_3 = int(joy_msg->buttons[3]);
  int stop_button_5 = int(joy_msg->buttons[5]);
  double stop_axis_4 = double(joy_msg->axes[4]);
  if ((stop_button_0 == 1 || stop_button_1 == 1 || 
       stop_button_2 == 1 || stop_button_3 == 1)
       && now_linear_vel>0.05) {  // publish a drastic backward command to make the motor stops faster ehwn its going forward
    now_linear_vel = -5/l_scale_;
  }
  if (stop_axis_4 < 0){
    reset_flag = true; 
  }
//  if ((stop_button_0 == 1 || stop_button_1 == 1 || 
//       stop_button_2 == 1 || stop_button_3 == 1)
//       && now_linear_vel<-0.05) {  // publish a zero command to make the motor stops when its going backward
//    now_linear_vel = 0;
//  }

  ROS_INFO("From joypad, now_angular_vel is : %f; now_linear_vel is : %f", now_angular_vel, now_linear_vel);
  joy_processing = false;

}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_pkg");

  ros::NodeHandle np("~");
  int publish_rate;
  np.param("axis_linear", linear_, 1);
  np.param("axis_angular", angular_, 0);
  np.param("scale_angular", a_scale_, 1.0);
  np.param("scale_linear", l_scale_, 1.0);
  np.param("publish_rate", publish_rate, 10);

  ros::NodeHandle nh_;

  ros::Publisher vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1);
  ros::Publisher servo_pub = nh_.advertise<ros_pololu_servo::MotorCommand>("/pololu/command", 2);
  ros::Publisher reset_pub = nh_.advertise<geometry_msgs::Twist>("/rover/reset_twist", 1);
  ros::Subscriber joy_sub_ = nh_.subscribe("/joy", 0, joyCallback);

  geometry_msgs::Twist twist;
  ros_pololu_servo::MotorCommand motorcmd_msg;  

  ros::Rate loop_rate(2*publish_rate);
  while(ros::ok() && !joy_processing){
    if (reset_flag){
        geometry_msgs::Twist twist_reset;
        reset_pub.publish(twist_reset);
        reset_flag = false;
    }



    motorcmd_msg.joint_name = "1";     // For steering_wheel
    motorcmd_msg.position = a_scale_*now_angular_vel;
    motorcmd_msg.speed = 0.0;          //speed should between 0 -- 1
    motorcmd_msg.acceleration = 0.5;
    ROS_INFO("now_angular_vel is : %f", now_angular_vel);
    servo_pub.publish(motorcmd_msg);

    loop_rate.sleep();

    motorcmd_msg.joint_name = "0";     // For traction_motor
    motorcmd_msg.position = -l_scale_*now_linear_vel;
    motorcmd_msg.speed = 0.0;          //speed should between 0 -- 1
    motorcmd_msg.acceleration = 0.0;
    ROS_INFO("now_linear_vel is : %f", now_linear_vel);
    servo_pub.publish(motorcmd_msg);
	
    twist.angular.z = now_angular_vel;
    twist.linear.x = now_linear_vel;
    vel_pub_.publish(twist);
    ros::spinOnce();
    loop_rate.sleep();
  }

  ros::spin();
  ros::shutdown();
  return 0;
}
