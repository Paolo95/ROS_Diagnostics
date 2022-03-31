#include <diagnostic_updater/diagnostic_updater.h>
#include <std_msgs/Bool.h>
#include <diagnostic_updater/publisher.h>
#include <control_msgs/PidState.h>
#include <sstream>

float left_wheel_joint_values = 0.0;
float left_wheel_joint_values_threshold = 0.0;

void LeftJointCallback(const control_msgs::PidState::ConstPtr& msg)
{
  
    left_wheel_joint_values = msg->output;
  //left_wheel_joint_values[0] = msg->position;//pose.pose.position.x;
  //ROS_INFO("left joint values: %f", left_wheel_joint_values);
}

void LeftJointDiagostic(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  if (left_wheel_joint_values > left_wheel_joint_values_threshold)
    
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "%f.", left_wheel_joint_values);
  else
    
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK, "Il valore rientra nella norma, left_wheel_joint_values: %f", left_wheel_joint_values);

  
  stat.add("Diagnostica left_wheel_joint", "Valore");
  std::ostringstream left_wheel_joint_values_string;
  left_wheel_joint_values_string << left_wheel_joint_values;
  stat.addf("x Pose Position", left_wheel_joint_values_string.str().c_str());
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "left_wheel_joint_diagnostic");
  ros::NodeHandle nh;
  diagnostic_updater::Updater updater;
  updater.setHardwareID("wheel_joint/left");

  nh.getParam("/left_joint_wheel_params/left_joint_wheel_thresholds/left_wheel_joint_values_threshold", left_wheel_joint_values_threshold);

  ros::Subscriber sub = nh.subscribe("/gazebo_ros_control/pid_gains/left_wheel_joint/state", 1000, LeftJointCallback);

  updater.add("Diagnostica Joint left wheel: ", LeftJointDiagostic);

  while (nh.ok())
  {
    std_msgs::Bool msg;
    ros::Duration(0.1).sleep();
    ros::spinOnce();
   
    msg.data = false;
    
    updater.update();
  }

  return 0; 
}