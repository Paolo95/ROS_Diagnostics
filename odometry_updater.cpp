#include <diagnostic_updater/diagnostic_updater.h>
#include <std_msgs/Bool.h>
#include <diagnostic_updater/publisher.h>
#include <nav_msgs/Odometry.h>
#include <sstream>

float x = 0.0;
float x_threshold = 0.0;
float y = 0.0;
float y_threshold = 0.0;
float z = 0.0;
float z_threshold = 0.0;

void positionCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  x = msg->pose.pose.position.x;
  y = msg->pose.pose.position.y;
  z = msg->pose.pose.position.z;
}

void x_diagnostic(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  if (x > x_threshold)
    
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "Il valore x ha superato %f", x_threshold);
  else
    
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK, "Va tutto bene x: %f", x);

  
  stat.add("Diagnostica di esempio", "Controllo del valore della x");
  std::ostringstream x_string;
  x_string << x;
  stat.addf("Posizione x:", x_string.str().c_str());
}

void y_diagnostic(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  if (y > y_threshold)
    
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "Il valore y ha superato %f", y_threshold);
  else
    
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK, "Va tutto bene y: %f", y);
  
   
  stat.add("Diagnostica di esempio", "Controllo del valore della y");
  std::ostringstream y_string;
  y_string << y;
  stat.addf("Posizione y:", y_string.str().c_str());
}

void z_diagnostic(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  if (z > z_threshold)
    
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "Il valore z ha superato %f", z_threshold);
  else
    
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK, "Va tutto bene z: %f", z);
  
   
  stat.add("Diagnostica di esempio", "Controllo del valore della z");
  std::ostringstream z_string;
  z_string << z;
  stat.addf("Posizione z:", z_string.str().c_str());
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "odometry_diagnostic");
  ros::NodeHandle nh;
  
  diagnostic_updater::Updater updater;
  updater.setHardwareID("diff_drive_controller/odom");
  nh.getParam("/odometry_diff_drive_params/odom_diff_drive_thresholds/x_threshold", x_threshold);
  nh.getParam("/odometry_diff_drive_params/odom_diff_drive_thresholds/y_threshold", y_threshold);
  nh.getParam("/odometry_diff_drive_params/odom_diff_drive_thresholds/z_threshold", z_threshold);
  ros::Subscriber sub = nh.subscribe("diff_drive_controller/odom", 1000, positionCallback);
  
  updater.add("Funzione di diagnostica della x", x_diagnostic);
  updater.add("Funzione di diagnostica della y", y_diagnostic);
  updater.add("Funzione di diagnostica della z", z_diagnostic);

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
