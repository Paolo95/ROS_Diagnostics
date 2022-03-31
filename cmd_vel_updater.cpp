#include <diagnostic_updater/diagnostic_updater.h>
#include <std_msgs/Bool.h>
#include <diagnostic_updater/publisher.h>
#include <sstream>
#include <geometry_msgs/Twist.h>

float x_cmd_vel_linear = 0.0;
float x_cmd_vel_linear_threshold = 0.0;
float y_cmd_vel_linear = 0.0;
float y_cmd_vel_linear_threshold = 0.0;
float z_cmd_vel_linear = 0.0;
float z_cmd_vel_linear_threshold = 0.0;

float x_cmd_vel_angular = 0.0;
float x_cmd_vel_angular_threshold = 0.0;
float y_cmd_vel_angular = 0.0;
float y_cmd_vel_angular_threshold = 0.0;
float z_cmd_vel_angular = 0.0;
float z_cmd_vel_angular_threshold = 0.0;


void cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& msg)
{
  x_cmd_vel_linear = msg->linear.x;
  y_cmd_vel_linear = msg->linear.y;
  z_cmd_vel_linear = msg->linear.z;

  x_cmd_vel_angular = msg->angular.x;
  y_cmd_vel_angular = msg->angular.y;
  z_cmd_vel_angular = msg->angular.z;
}


void x_cmd_vel_linear_diagnostic(diagnostic_updater::DiagnosticStatusWrapper &stat){

  if(x_cmd_vel_linear > x_cmd_vel_linear_threshold) 
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "Il valore x_cmd_vel_linear e' fuori soglia! Valore limite: %f", x_cmd_vel_linear_threshold);
  else 
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK, "Il valore rientra nella norma, x_cmd_vel_linear: %f", x_cmd_vel_linear);


  stat.add("Diagnostica x_cmd_vel_linear", "Valore");
  std::ostringstream x_cmd_vel_linear_string;
  x_cmd_vel_linear_string << x_cmd_vel_linear;
  stat.addf("x_cmd_vel_linear", x_cmd_vel_linear_string.str().c_str());

}

void y_cmd_vel_linear_diagnostic(diagnostic_updater::DiagnosticStatusWrapper &stat){

  if(y_cmd_vel_linear > y_cmd_vel_linear_threshold) 
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "Il valore y_cmd_vel_linear e' fuori soglia! Valore limite: %f", y_cmd_vel_linear_threshold);
  else 
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK, "Il valore rientra nella norma, y_cmd_vel_linear: %f", y_cmd_vel_linear);


  stat.add("Diagnostica y_cmd_vel_linear", "Valore");
  std::ostringstream y_cmd_vel_linear_string;
  y_cmd_vel_linear_string << y_cmd_vel_linear;
  stat.addf("y_cmd_vel_linear", y_cmd_vel_linear_string.str().c_str());

}

void z_cmd_vel_linear_diagnostic(diagnostic_updater::DiagnosticStatusWrapper &stat){

  if(z_cmd_vel_linear > z_cmd_vel_linear_threshold) 
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "Il valore z_cmd_vel_linear e' fuori soglia! Valore limite: %f", z_cmd_vel_linear_threshold);
  else 
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK, "Il valore rientra nella norma, z_cmd_vel_linear: %f", z_cmd_vel_linear);


  stat.add("Diagnostica z_cmd_vel_linear", "Valore");
  std::ostringstream z_cmd_vel_linear_string;
  z_cmd_vel_linear_string << z_cmd_vel_linear;
  stat.addf("z_cmd_vel_linear", z_cmd_vel_linear_string.str().c_str());

}

void x_cmd_vel_angular_diagnostic(diagnostic_updater::DiagnosticStatusWrapper &stat){

  if(x_cmd_vel_angular > x_cmd_vel_angular_threshold) 
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "Il valore x_cmd_vel_angular e' fuori soglia! Valore limite: %f", x_cmd_vel_angular_threshold);
  else 
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK, "Il valore rientra nella norma, x_cmd_vel_angular: %f", x_cmd_vel_angular);


  stat.add("Diagnostica x_cmd_vel_angular", "Valore");
  std::ostringstream x_cmd_vel_angular_string;
  x_cmd_vel_angular_string << x_cmd_vel_angular;
  stat.addf("x_cmd_vel_angular", x_cmd_vel_angular_string.str().c_str());

}

void y_cmd_vel_angular_diagnostic(diagnostic_updater::DiagnosticStatusWrapper &stat){

  if(y_cmd_vel_angular > y_cmd_vel_angular_threshold) 
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "Il valore y_cmd_vel_angular e' fuori soglia! Valore limite: %f", y_cmd_vel_angular_threshold);
  else 
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK, "Il valore rientra nella norma, y_cmd_vel_angular: %f", y_cmd_vel_angular);


  stat.add("Diagnostica y_cmd_vel_angular", "Valore");
  std::ostringstream y_cmd_vel_angular_string;
  y_cmd_vel_angular_string << y_cmd_vel_angular;
  stat.addf("y_cmd_vel_angular", y_cmd_vel_angular_string.str().c_str());

}

void z_cmd_vel_angular_diagnostic(diagnostic_updater::DiagnosticStatusWrapper &stat){

  if(z_cmd_vel_angular > z_cmd_vel_angular_threshold) 
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "Il valore z_cmd_vel_angular e' fuori soglia! Valore limite: %f", z_cmd_vel_angular_threshold);
  else 
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK, "Il valore rientra nella norma, z_cmd_vel_angular: %f", z_cmd_vel_angular);


  stat.add("Diagnostica z_cmd_vel_angular", "Valore");
  std::ostringstream z_cmd_vel_angular_string;
  z_cmd_vel_angular_string << z_cmd_vel_angular;
  stat.addf("z_cmd_vel_angular", z_cmd_vel_angular_string.str().c_str());

}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "cmd_vel_diagnostic");

  ros::NodeHandle nh_vel_upd;
  diagnostic_updater::Updater vel_upd_updater;
  vel_upd_updater.setHardwareID("/diff_drive_controller/cmd_vel");

  nh_vel_upd.getParam("/cmd_vel_thresholds/linear/x_threshold", x_cmd_vel_linear_threshold);
  nh_vel_upd.getParam("/cmd_vel_thresholds/linear/y_threshold", y_cmd_vel_linear_threshold);
  nh_vel_upd.getParam("/cmd_vel_thresholds/linear/z_threshold", z_cmd_vel_linear_threshold);
  
  nh_vel_upd.getParam("/cmd_vel_thresholds/angular/x_threshold", x_cmd_vel_angular_threshold);
  nh_vel_upd.getParam("/cmd_vel_thresholds/angular/y_threshold", y_cmd_vel_angular_threshold);
  nh_vel_upd.getParam("/cmd_vel_thresholds/angular/z_threshold", z_cmd_vel_angular_threshold);

  ros::Subscriber sub = nh_vel_upd.subscribe("/diff_drive_controller/cmd_vel", 1000, cmd_vel_callback);

  vel_upd_updater.add("Funzione di diagnostica di x_cmd_vel_linear", x_cmd_vel_linear_diagnostic);
  vel_upd_updater.add("Funzione di diagnostica di y_cmd_vel_linear", y_cmd_vel_linear_diagnostic);
  vel_upd_updater.add("Funzione di diagnostica di z_cmd_vel_linear", z_cmd_vel_linear_diagnostic);

  vel_upd_updater.add("Funzione di diagnostica di x_cmd_vel_angular", x_cmd_vel_angular_diagnostic);
  vel_upd_updater.add("Funzione di diagnostica di y_cmd_vel_angular", y_cmd_vel_angular_diagnostic);
  vel_upd_updater.add("Funzione di diagnostica di z_cmd_vel_angular", z_cmd_vel_angular_diagnostic);

  while (nh_vel_upd.ok())
  {
    std_msgs::Bool msg;
    ros::Duration(0.1).sleep();
    ros::spinOnce();
   
    msg.data = false;
    
    vel_upd_updater.update();
  }

  return 0; 
}
