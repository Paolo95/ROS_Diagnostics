#include <diagnostic_updater/diagnostic_updater.h>
#include <std_msgs/Bool.h>
#include <diagnostic_updater/publisher.h>
#include <sstream>
#include <nav_msgs/Odometry.h>
#include "sensor_msgs/LaserScan.h"
#include <stdio.h>
#include <stdlib.h>
#include <vector>


// TODO ==========================================================================================================
/*
* data types e mini guida per hokuyo: http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/LaserScan.html
*
* aggiusta ranges, che sul yaml è una stringa e qua è un array di float
* vedi intensities che non so se viene pubblicato dal sensore
* 
*
*
*/


// valori attuali dei sensori
float angle_min = 0.0;
float angle_max = 0.0;
float angle_increment = 0.0;
float scan_time = 0.0;
float range_min = 0.0;
float range_max = 0.0;
//float ranges[256] = {0.0};
std::vector<float> ranges;
//float[] intensities;


// valori delle soglie e inizializzazione
float angle_min_threshold = 0.0;
float angle_max_threshold = 0.0;
float angle_increment_threshold = 0.0;
float scan_time_threshold = 0.0;
float range_min_threshold = 0.0;
float range_max_threshold = 0.0;


void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{

  angle_min = msg->angle_min;
  angle_max = msg->angle_max;
  angle_increment = msg->angle_increment;
  scan_time = msg->scan_time;
  range_min = msg->range_min;
  range_max = msg->range_max;
  ranges = msg->ranges;


  //ROS_INFO("ranges: %d", ranges.size());

}


// DIAGNOSTICA SCANNER ====================================================================================

void angle_min_diagnostic(diagnostic_updater::DiagnosticStatusWrapper &stat){

  if(angle_min > angle_min_threshold) stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "angle_min ERROR! val: %f, threshold: %f", angle_min, angle_min_threshold);
  else stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK, "angle_min OK val: %f, threshold: %f", angle_min, angle_min_threshold);


  stat.add("Diagnostica di esempio", "Controllo del valore di angle_min del laser");
  std::ostringstream string;
  string << angle_min;
  stat.addf("angle_min:", string.str().c_str());

}

void angle_max_diagnostic(diagnostic_updater::DiagnosticStatusWrapper &stat){

  if(angle_max > angle_max_threshold) stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "angle_max ERROR! val: %f, threshold: %f", angle_max, angle_max_threshold);
  else stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK, "angle_max OK val: %f, threshold: %f", angle_min, angle_min_threshold);


  stat.add("Diagnostica di esempio", "Controllo del valore di angle_max del laser");
  std::ostringstream string;
  string << angle_max;
  stat.addf("angle_max:", string.str().c_str());

}

void angle_increment_diagnostic(diagnostic_updater::DiagnosticStatusWrapper &stat){

  if(angle_increment > angle_increment_threshold) stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "angle_increment ERROR! val: %f, threshold: %f", angle_increment, angle_increment_threshold);
  else stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK, "angle_increment OK val: %f, threshold: %f", angle_increment, angle_increment_threshold);


  stat.add("Diagnostica di esempio", "Controllo del valore di angle_increment del laser");
  std::ostringstream string;
  string << angle_increment;
  stat.addf("angle_increment:", string.str().c_str());

}

void scan_time_diagnostic(diagnostic_updater::DiagnosticStatusWrapper &stat){

  if(scan_time > scan_time_threshold) stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "scan_time ERROR! val: %f, threshold: %f", scan_time, scan_time_threshold);
  else stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK, "scan_time OK val: %f, threshold: %f", scan_time, scan_time_threshold);


  stat.add("Diagnostica di esempio", "Controllo del valore di scan_time del laser");
  std::ostringstream string;
  string << scan_time;
  stat.addf("scan_time:", string.str().c_str());

}

void range_min_diagnostic(diagnostic_updater::DiagnosticStatusWrapper &stat){

  if(range_min > range_min_threshold) stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "range_min ERROR! val: %f, threshold: %f", range_min, range_min_threshold);
  else stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK, "range_min OK val: %f, threshold: %f", range_min, range_min_threshold);


  stat.add("Diagnostica di esempio", "Controllo del valore di range_min del laser");
  std::ostringstream string;
  string << range_min;
  stat.addf("range_min:", string.str().c_str());

}

void range_max_diagnostic(diagnostic_updater::DiagnosticStatusWrapper &stat){

  if(range_max > range_max_threshold) stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "range_max ERROR! val: %f, threshold: %f", range_max, range_max_threshold);
  else stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK, "range_max OK val: %f, threshold: %f", range_max, range_max_threshold);


  stat.add("Diagnostica di esempio", "Controllo del valore di range_max del laser");
  std::ostringstream string;
  string << range_max;
  stat.addf("range_max:", string.str().c_str());

}



void ranges_diagnostic(diagnostic_updater::DiagnosticStatusWrapper &stat){


/*
  int rangesSize = sizeof(ranges)/sizeof(ranges[0]);

  for(int i = 0; i < rangesSize; i++){

    if(ranges[i] < range_min) stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "range ERROR! val: %f, < threshold: %f", ranges[i], range_min);
    if(ranges[i] > range_max) stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "range ERROR! val: %f, > threshold: %f", ranges[i], range_max);
    if(ranges[i] > range_min && ranges[i] < range_max) stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK, "range OK val: %f, > %f && < %f", ranges[i], range_min, range_max);

  }

*/



  //for(std::size_t i = 0; i < ranges.size(); ++i) {
  for(int i = 0; i < ranges.size(); ++i) {
    //ROS_INFO("size: %d", ranges.size());

}



  stat.add("Diagnostica di esempio", "Controllo dei valori dell'array ranges del laser");
  //std::ostringstream string;
  //string << i;
  //stat.addf("ranges:", string.str().c_str());

}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "laser_scan_diagnostic");

  ros::NodeHandle nh_scan;
  diagnostic_updater::Updater scan_updater;
  scan_updater.setHardwareID("/scan");

  nh_scan.getParam("/scan_params/scan_thresholds/angle_min_threshold", angle_min_threshold);
  nh_scan.getParam("/scan_params/scan_thresholds/angle_max_threshold", angle_max_threshold);
  nh_scan.getParam("/scan_params/scan_thresholds/angle_increment_threshold", angle_increment_threshold);
  nh_scan.getParam("/scan_params/scan_thresholds/scan_time_threshold", scan_time_threshold);
  nh_scan.getParam("/scan_params/scan_thresholds/range_min_threshold", range_min_threshold);
  nh_scan.getParam("/scan_params/scan_thresholds/range_max_threshold", range_max_threshold);
  //nh_scan.getParam("/scan_params/scan_thresholds/ranges", ranges_threshold);
  //nh_scan.getParam("/scan_params/scan_thresholds/angle_min_threshold", angle_min_threshold);

  ros::Subscriber sub = nh_scan.subscribe("/scan", 1000, scanCallback);

  scan_updater.add("Funzione di diagnostica di angle_min", angle_min_diagnostic);
  scan_updater.add("Funzione di diagnostica di angle_max", angle_max_diagnostic);
  scan_updater.add("Funzione di diagnostica di angle_increment", angle_increment_diagnostic);
  scan_updater.add("Funzione di diagnostica di scan_time", scan_time_diagnostic);
  scan_updater.add("Funzione di diagnostica di range_min", range_min_diagnostic);
  scan_updater.add("Funzione di diagnostica di range_max", range_max_diagnostic);
  scan_updater.add("Funzione di diagnostica di ranges", ranges_diagnostic);


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
