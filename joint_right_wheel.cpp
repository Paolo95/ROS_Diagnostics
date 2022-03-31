#include <diagnostic_updater/diagnostic_updater.h>
#include <std_msgs/Bool.h>
#include <diagnostic_updater/publisher.h>
#include <control_msgs/PidState.h>
#include <sstream>

float right_wheel_joint_values = 0.0;
float right_wheel_joint_values_threshold = 0.0;


void RightJointCallback(const control_msgs::PidState::ConstPtr& msg)
{
  
    right_wheel_joint_values = msg->output;
}

void RightJointDiagostic(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  if (right_wheel_joint_values > right_wheel_joint_values_threshold)
    
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "%f.", right_wheel_joint_values);
  else
    
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK, "Il valore rientra nella norma, right_wheel_joint_values: %f", right_wheel_joint_values);

  
  stat.add("Diagnostica right_wheel_joint", "Valore");
  std::ostringstream right_wheel_joint_values_string;
  right_wheel_joint_values_string << right_wheel_joint_values;
  stat.addf("Right Wheel output", right_wheel_joint_values_string.str().c_str());
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "right_wheel_joint_diagnostic");
  ros::NodeHandle nh;
  diagnostic_updater::Updater updater;
  updater.setHardwareID("wheel_joint/right");

  nh.getParam("/right_joint_wheel_params/right_joint_wheel_thresholds/right_wheel_joint_values_threshold", right_wheel_joint_values_threshold);

  ros::Subscriber sub = nh.subscribe("/gazebo_ros_control/pid_gains/right_wheel_joint/state", 1000, RightJointCallback);

  updater.add("Diagnostica Joint right wheel: ", RightJointDiagostic);

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