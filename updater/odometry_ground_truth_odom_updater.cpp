#include <diagnostic_updater/diagnostic_updater.h>
#include <std_msgs/Bool.h>
#include <diagnostic_updater/publisher.h>
#include <nav_msgs/Odometry.h>
#include <sstream>

float x_gt_pose_position = 0.0;
float x_gt_pose_position_threshold = 0.0;
float y_gt_pose_position = 0.0;
float y_gt_pose_position_threshold = 0.0;
float z_gt_pose_position = 0.0;
float z_gt_pose_position_threshold = 0.0;

float x_gt_pose_orientation = 0.0;
float x_gt_pose_orientation_threshold = 0.0;
float y_gt_pose_orientation = 0.0;
float y_gt_pose_orientation_threshold = 0.0;
float z_gt_pose_orientation = 0.0;
float z_gt_pose_orientation_threshold = 0.0;
float w_gt_pose_orientation = 0.0;
float w_gt_pose_orientation_threshold = 0.0;

float x_gt_twist_linear = 0.0;
float x_gt_twist_linear_threshold = 0.0;
float y_gt_twist_linear = 0.0;
float y_gt_twist_linear_threshold = 0.0;
float z_gt_twist_linear = 0.0;
float z_gt_twist_linear_threshold = 0.0;

float x_gt_twist_angular = 0.0;
float x_gt_twist_angular_threshold = 0.0;
float y_gt_twist_angular = 0.0;
float y_gt_twist_angular_threshold = 0.0;
float z_gt_twist_angular = 0.0;
float z_gt_twist_angular_threshold = 0.0;


void gtPosePositionCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  x_gt_pose_position = msg->pose.pose.position.x;
  y_gt_pose_position = msg->pose.pose.position.y;
  z_gt_pose_position = msg->pose.pose.position.z;

  x_gt_pose_orientation = msg->pose.pose.orientation.x;
  y_gt_pose_orientation = msg->pose.pose.orientation.y;
  z_gt_pose_orientation = msg->pose.pose.orientation.z;
  w_gt_pose_orientation = msg->pose.pose.orientation.w;

  x_gt_twist_linear = msg->twist.twist.linear.x;
  y_gt_twist_linear = msg->twist.twist.linear.y;
  z_gt_twist_linear = msg->twist.twist.linear.z;

  x_gt_twist_angular = msg->twist.twist.angular.x;
  y_gt_twist_angular = msg->twist.twist.angular.y;
  z_gt_twist_angular = msg->twist.twist.angular.z;

}

void xGtPosePositionDiagostic(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  if (x_gt_pose_position > x_gt_pose_position_threshold)
    
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "Il valore x_gt_pose_position e' fuori soglia! Valore limite: %f", x_gt_pose_position_threshold);
  else
    
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK, "Il valore rientra nella norma, x_gt_pose_position: %f", x_gt_pose_position);

  
  stat.add("Diagnostica Ground Truth Pose Position", "Valore");
  std::ostringstream x_gt_pose_position_string;
  x_gt_pose_position_string << x_gt_pose_position;
  stat.addf("x Ground Truth Pose Position", x_gt_pose_position_string.str().c_str());
}

void yGtPosePositionDiagostic(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  if (y_gt_pose_position > y_gt_pose_position_threshold)
    
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "Il valore y_gt_pose_position e' fuori soglia! Valore limite: %f", y_gt_pose_position_threshold);
  else
    
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK, "Il valore rientra nella norma, y_gt_pose_position: %f", y_gt_pose_position);

  
  stat.add("Diagnostica Ground Truth Position", "Valore");
  std::ostringstream y_gt_pose_position_string;
  y_gt_pose_position_string << y_gt_pose_position;
  stat.addf("y Ground Truth Pose Position", y_gt_pose_position_string.str().c_str());
}

void zGtPosePositionDiagostic(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  if (z_gt_pose_position > z_gt_pose_position_threshold)
    
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "Il valore z_gt_pose_position e' fuori soglia! Valore limite: %f", z_gt_pose_position_threshold);
  else
    
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK, "Il valore rientra nella norma, z_gt_pose_position: %f", z_gt_pose_position);

  
  stat.add("Diagnostica Ground Truth Pose Position", "Valore");
  std::ostringstream z_gt_pose_position_string;
  z_gt_pose_position_string << z_gt_pose_position;
  stat.addf("z Ground Truth Pose Position", z_gt_pose_position_string.str().c_str());
}

void xGtPoseOrientationDiagostic(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  if (x_gt_pose_orientation > x_gt_pose_orientation_threshold)
    
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "Il valore x_gt_pose_orientation e' fuori soglia! Valore limite: %f", x_gt_pose_orientation_threshold);
  else
    
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK, "Il valore rientra nella norma, x_gt_pose_orientation: %f", x_gt_pose_orientation);

  
  stat.add("Diagnostica Ground Truth Pose Orientation", "Valore");
  std::ostringstream x_gt_pose_orientation_string;
  x_gt_pose_orientation_string << x_gt_pose_orientation;
  stat.addf("x Ground Truth Pose Orientation", x_gt_pose_orientation_string.str().c_str());
}

void yGtPoseOrientationDiagostic(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  if (y_gt_pose_orientation > y_gt_pose_orientation_threshold)
    
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "Il valore y_gt_pose_orientation e' fuori soglia! Valore limite: %f", y_gt_pose_orientation_threshold);
  else
    
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK, "Il valore rientra nella norma, y_gt_pose_orientation: %f", y_gt_pose_orientation);

  
  stat.add("Diagnostica Ground Truth Pose Orientation", "Valore");
  std::ostringstream y_gt_pose_orientation_string;
  y_gt_pose_orientation_string << y_gt_pose_orientation;
  stat.addf("y Ground Truth Pose Orientation", y_gt_pose_orientation_string.str().c_str());
}

void zGtPoseOrientationDiagostic(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  if (z_gt_pose_orientation > z_gt_pose_orientation_threshold)
    
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "Il valore z_gt_pose_orientation e' fuori soglia! Valore limite: %f", z_gt_pose_orientation_threshold);
  else
    
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK, "Il valore rientra nella norma, z_gt_pose_orientation: %f", z_gt_pose_orientation);

  
  stat.add("Diagnostica Ground Truth Pose Orientation", "Valore");
  std::ostringstream z_gt_pose_orientation_string;
  z_gt_pose_orientation_string << z_gt_pose_orientation;
  stat.addf("z Ground Truth Pose Orientation", z_gt_pose_orientation_string.str().c_str());
}

void wGtPoseOrientationDiagostic(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  if (w_gt_pose_orientation > w_gt_pose_orientation_threshold)
    
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "Il valore w_gt_pose_orientation e' fuori soglia! Valore limite: %f", w_gt_pose_orientation_threshold);
  else
    
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK, "Il valore rientra nella norma, w_gt_pose_orientation: %f", w_gt_pose_orientation);

  
  stat.add("Diagnostica Ground Truth Pose Orientation", "Valore");
  std::ostringstream w_gt_pose_orientation_string;
  w_gt_pose_orientation_string << w_gt_pose_orientation;
  stat.addf("w Ground Truth Pose Orientation", w_gt_pose_orientation_string.str().c_str());
}

void xGtTwistLinearDiagostic(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  if (x_gt_twist_linear > x_gt_twist_linear_threshold)
    
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "Il valore x_gt_twist_linear e' fuori soglia! Valore limite: %f", x_gt_twist_linear_threshold);
  else
    
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK, "Il valore rientra nella norma, x_gt_twist_linear: %f", x_gt_twist_linear);

  
  stat.add("Diagnostica Ground Truth Twist Linear", "Valore");
  std::ostringstream x_gt_twist_linear_string;
  x_gt_twist_linear_string << x_gt_twist_linear;
  stat.addf("x Ground Truth Twist Linear", x_gt_twist_linear_string.str().c_str());
}

void yGtTwistLinearDiagostic(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  if (y_gt_twist_linear > y_gt_twist_linear_threshold)
    
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "Il valore y_gt_twist_linear e' fuori soglia! Valore limite: %f", y_gt_twist_linear_threshold);
  else
    
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK, "Il valore rientra nella norma, y_gt_twist_linear: %f", y_gt_twist_linear);

  
  stat.add("Diagnostica Ground Truth Twist Linear", "Valore");
  std::ostringstream y_gt_twist_linear_string;
  y_gt_twist_linear_string << y_gt_twist_linear;
  stat.addf("y Ground Truth Twist Linear", y_gt_twist_linear_string.str().c_str());
}

void zGtTwistLinearDiagostic(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  if (z_gt_twist_linear > y_gt_twist_linear_threshold)
    
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "Il valore z_gt_twist_linear e' fuori soglia! Valore limite: %f", z_gt_twist_linear_threshold);
  else
    
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK, "Il valore rientra nella norma, z_gt_twist_linear: %f", z_gt_twist_linear);

  
  stat.add("Diagnostica Ground Truth Twist Linear", "Valore");
  std::ostringstream z_gt_twist_linear_string;
  z_gt_twist_linear_string << z_gt_twist_linear;
  stat.addf("z Ground Truth Twist Linear", z_gt_twist_linear_string.str().c_str());
}

void xGtTwistAngularDiagostic(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  if (x_gt_twist_angular > x_gt_twist_angular_threshold)
    
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "Il valore x_gt_twist_angular e' fuori soglia! Valore limite: %f", x_gt_twist_angular_threshold);
  else
    
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK, "Il valore rientra nella norma, x_gt_twist_angular: %f", x_gt_twist_angular);

  
  stat.add("Diagnostica Ground Truth Twist Angular", "Valore");
  std::ostringstream x_gt_twist_angular_string;
  x_gt_twist_angular_string << x_gt_twist_angular;
  stat.addf("x Ground Truth Twist Angular", x_gt_twist_angular_string.str().c_str());
}

void yGtTwistAngularDiagostic(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  if (y_gt_twist_angular > y_gt_twist_angular_threshold)
    
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "Il valore y_gt_twist_angular e' fuori soglia! Valore limite: %f", y_gt_twist_angular_threshold);
  else
    
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK, "Il valore rientra nella norma, y_gt_twist_angular: %f", y_gt_twist_angular);

  
  stat.add("Diagnostica Ground Truth Twist Angular", "Valore");
  std::ostringstream y_gt_twist_angular_string;
  y_gt_twist_angular_string << y_gt_twist_angular;
  stat.addf("y Ground Truth Twist Angular", y_gt_twist_angular_string.str().c_str());
}

void zGtTwistAngularDiagostic(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  if (z_gt_twist_angular > z_gt_twist_angular_threshold)
    
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "Il valore z_gt_twist_angular e' fuori soglia! Valore limite: %f", z_gt_twist_angular_threshold);
  else
    
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK, "Il valore rientra nella norma, z_gt_twist_angular: %f", z_gt_twist_angular);

  
  stat.add("Diagnostica Ground Truth Twist Angular", "Valore");
  std::ostringstream z_gt_twist_angular_string;
  z_gt_twist_angular_string << z_gt_twist_angular;
  stat.addf("z Ground Truth Twist Angular", z_gt_twist_angular_string.str().c_str());
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "odometry_ground_truth_diagnostic");
  ros::NodeHandle ground_truth_nh;
  
  diagnostic_updater::Updater ground_truth_updater;
  ground_truth_updater.setHardwareID("odometry/ground_truth_odom");

  ground_truth_nh.getParam("/ground_truth_params/ground_truth_pose_position/x_threshold", x_gt_pose_position_threshold);
  ground_truth_nh.getParam("/ground_truth_params/ground_truth_pose_position/y_threshold", y_gt_pose_position_threshold);
  ground_truth_nh.getParam("/ground_truth_params/ground_truth_pose_position/z_threshold", z_gt_pose_position_threshold);

  ground_truth_nh.getParam("/ground_truth_params/ground_truth_pose_orientation/x_threshold", x_gt_pose_orientation_threshold);
  ground_truth_nh.getParam("/ground_truth_params/ground_truth_pose_orientation/y_threshold", y_gt_pose_orientation_threshold);
  ground_truth_nh.getParam("/ground_truth_params/ground_truth_pose_orientation/z_threshold", z_gt_pose_orientation_threshold);
  ground_truth_nh.getParam("/ground_truth_params/ground_truth_pose_orientation/w_threshold", w_gt_pose_orientation_threshold);

  ground_truth_nh.getParam("/ground_truth_params/ground_truth_twist_linear/x_threshold", x_gt_twist_linear_threshold);
  ground_truth_nh.getParam("/ground_truth_params/ground_truth_twist_linear/y_threshold", y_gt_twist_linear_threshold);
  ground_truth_nh.getParam("/ground_truth_params/ground_truth_twist_linear/z_threshold", z_gt_twist_linear_threshold);

  ground_truth_nh.getParam("/ground_truth_params/ground_truth_twist_angular/x_threshold", x_gt_twist_angular_threshold);
  ground_truth_nh.getParam("/ground_truth_params/ground_truth_twist_angular/y_threshold", y_gt_twist_angular_threshold);
  ground_truth_nh.getParam("/ground_truth_params/ground_truth_twist_angular/z_threshold", z_gt_twist_angular_threshold);
    
  ros::Subscriber sub = ground_truth_nh.subscribe("odometry/ground_truth_odom", 1000, gtPosePositionCallback);
  
  ground_truth_updater.add("Funzione di diagnostica della x ground truth pose position", xGtPosePositionDiagostic);
  ground_truth_updater.add("Funzione di diagnostica della y ground truth pose position", yGtPosePositionDiagostic);
  ground_truth_updater.add("Funzione di diagnostica della z ground truth pose position", zGtPosePositionDiagostic);

  ground_truth_updater.add("Funzione di diagnostica della x ground truth pose orientation", xGtPoseOrientationDiagostic);
  ground_truth_updater.add("Funzione di diagnostica della y ground truth pose orientation", yGtPoseOrientationDiagostic);
  ground_truth_updater.add("Funzione di diagnostica della z ground truth pose orientation", zGtPoseOrientationDiagostic);
  ground_truth_updater.add("Funzione di diagnostica della w ground truth pose orientation", wGtPoseOrientationDiagostic);

  ground_truth_updater.add("Funzione di diagnostica della x ground truth twist linear", xGtTwistLinearDiagostic);
  ground_truth_updater.add("Funzione di diagnostica della y ground truth twist linear", yGtTwistLinearDiagostic);
  ground_truth_updater.add("Funzione di diagnostica della z ground truth twist linear", zGtTwistLinearDiagostic);

  ground_truth_updater.add("Funzione di diagnostica della x ground truth twist angular", xGtTwistAngularDiagostic);
  ground_truth_updater.add("Funzione di diagnostica della y ground truth twist angular", yGtTwistAngularDiagostic);
  ground_truth_updater.add("Funzione di diagnostica della z ground truth twist angular", zGtTwistAngularDiagostic);
  

  while (ground_truth_nh.ok())
  {
    std_msgs::Bool msg;
    ros::Duration(0.1).sleep();
    ros::spinOnce();
   
    msg.data = false;
    
    ground_truth_updater.update();
  }

  return 0; 
}