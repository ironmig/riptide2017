#include <ros/ros.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>
#include <rosfrc/Encoder.h>
#include <std_msgs/Float64.h>
#include <string>

class MyRobot : public hardware_interface::RobotHW
{
public:
  MyRobot(ros::NodeHandle& nh) :
  left_goal_msg(new std_msgs::Float64),
  right_goal_msg(new std_msgs::Float64)
 { 
   // connect and register the joint state interface
   hardware_interface::JointStateHandle state_handle_a("left_wheels", &pos[0], &vel[0], &eff[0]);
   jnt_state_interface.registerHandle(state_handle_a);

   hardware_interface::JointStateHandle state_handle_b("right_wheels", &pos[1], &vel[1], &eff[1]);
   jnt_state_interface.registerHandle(state_handle_b);

   registerInterface(&jnt_state_interface);

   // connect and register the joint position interface
   //~ hardware_interface::JointHandle pos_handle_a(jnt_state_interface.getHandle("left_wheels"), &cmd[0]);
   //~ jnt_pos_interface.registerHandle(pos_handle_a);
//~ 
   //~ hardware_interface::JointHandle pos_handle_b(jnt_state_interface.getHandle("right_wheels"), &cmd[1]);
   //~ jnt_pos_interface.registerHandle(pos_handle_b);
//~ 
   //~ registerInterface(&jnt_pos_interface);
   
   hardware_interface::JointHandle pos_handle_vel_left(jnt_state_interface.getHandle("left_wheels"), &vel_cmd[0]);
   hardware_interface::JointHandle pos_handle_vel_right(jnt_state_interface.getHandle("right_wheels"), &vel_cmd[1]);
   jnt_velocity_interface.registerHandle(pos_handle_vel_left);
   jnt_velocity_interface.registerHandle(pos_handle_vel_right);
   registerInterface(&jnt_velocity_interface);
   
   
	left_wheel_encoder_sub = nh.subscribe("/encoder_left", 1, &MyRobot::left_wheel_encoder_cb, this);
	right_wheel_encoder_sub = nh.subscribe("/encoder_right", 1, &MyRobot::right_wheel_encoder_cb, this);
	left_wheel_effort_pub = nh.advertise<std_msgs::Float64>("/left_wheels_vel", 5);
	right_wheel_effort_pub = nh.advertise<std_msgs::Float64>("/right_wheels_vel", 5);
  }
  void left_wheel_encoder_cb(const rosfrc::EncoderConstPtr& msg)
  {
	vel_tmp[0] = msg->rate;
	pos_tmp[0] = msg->distance;
  }
  void right_wheel_encoder_cb(const rosfrc::EncoderConstPtr& msg)
  {
	vel_tmp[1] = msg->rate;
	pos_tmp[1] = msg->distance;
  }
  void read()
  {
	vel[0] = vel_tmp[0] / WHEEL_RADIUS;
	vel[1] = vel_tmp[1] / WHEEL_RADIUS;
	pos[0] = pos_tmp[0] / WHEEL_RADIUS;
	pos[1] = pos_tmp[1] / WHEEL_RADIUS;
	eff[0] = cmd[0];
	eff[1] = cmd[1];
  }
  void write()
  {
	left_goal_msg->data = vel_cmd[0] * WHEEL_RADIUS;
	right_goal_msg->data = vel_cmd[1] * WHEEL_RADIUS;
	left_wheel_effort_pub.publish(left_goal_msg);
	right_wheel_effort_pub.publish(right_goal_msg);
  }
private:
  std_msgs::Float64Ptr left_goal_msg;
  std_msgs::Float64Ptr right_goal_msg;
  ros::Subscriber left_wheel_encoder_sub;
  ros::Subscriber right_wheel_encoder_sub;
  ros::Publisher left_wheel_effort_pub;
  ros::Publisher right_wheel_effort_pub;
  hardware_interface::JointStateInterface  jnt_state_interface;
  hardware_interface::EffortJointInterface jnt_pos_interface;
  hardware_interface::VelocityJointInterface jnt_velocity_interface;
  const double WHEEL_RADIUS = 0.0762;
  double vel_tmp[2];
  double pos_tmp[2];
  double cmd[2];
  double vel_cmd[2];
  double pos[2];
  double vel[2];
  double eff[2];
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "my_robot");
  ros::NodeHandle nh;
  MyRobot robot(nh);
  controller_manager::ControllerManager cm(&robot);
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::Time prev_time = ros::Time::now();
  ros::Rate rate(10.0);
  while (ros::ok())
  {
	ros::Time now = ros::Time::now();
	ros::Duration period = now - prev_time;
	prev_time = now;
    robot.read();
    cm.update(now, period);
    robot.write();
    rate.sleep();
  }
}
