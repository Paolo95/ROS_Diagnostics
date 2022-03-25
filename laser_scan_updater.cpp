#include <diagnostic_updater/diagnostic_updater.h>
#include <std_msgs/Bool.h>
#include <diagnostic_updater/publisher.h>
#include <sstream>
#include <nav_msgs/Odometry.h>
#include "sensor_msgs/LaserScan.h"


float angle_min = 0.0;
float angle_min_threshold = 0.0;


void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{

  angle_min = msg->angle_min;
  ROS_INFO("angle_min: %f", angle_min);

}


// DIAGNOSTICA SCANNER ====================================================================================

void angle_min_diagnostic(diagnostic_updater::DiagnosticStatusWrapper &stat){

  if(angle_min > angle_min_threshold) stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "angle_min ERROR! val: %f, threshold: %f", angle_min, angle_min_threshold);
  else stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK, "angle_min OK val: %f, threshold: %f", angle_min, angle_min_threshold);


  stat.add("Diagnostica di esempio", "Controllo del valore della angle_min del laser");
  std::ostringstream string;
  string << angle_min;
  stat.addf("angle_min:", string.str().c_str());

}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "laser_scan_diagnostic");

  ros::NodeHandle nh_scan;
  diagnostic_updater::Updater scan_updater;
  scan_updater.setHardwareID("/scan");

  nh_scan.getParam("/scan_params/scan_thresholds/angle_min_threshold", angle_min_threshold);

  ros::Subscriber sub = nh_scan.subscribe("/scan", 1000, scanCallback);

  scan_updater.add("Funzione di diagnostica di angle_min", angle_min_diagnostic);


  while (nh_scan.ok())
  {
    std_msgs::Bool msg;
    ros::Duration(0.1).sleep();
    ros::spinOnce();
   
    msg.data = false;
    
    scan_updater.update();
  }

  return 0; 
}
