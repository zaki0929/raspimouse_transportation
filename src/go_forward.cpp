#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"

#define RANGE_MAX 5.6

class GoForward{
public:
  GoForward(){
    scan_sub = n.subscribe("scan", 1000, &GoForward::scanCallback, this);
    vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);
  }

  double null_check(double target){
    if(!(target > 0)){
      target = (double)RANGE_MAX;
      ROS_WARN("RANGE OVER");
    }
    return target;
  }
  
  void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg){

    double center_number = (-msg->angle_min)/msg->angle_increment;
    double center = msg->ranges[center_number];
    double left = msg->ranges[center_number+255];
    double right = msg->ranges[center_number-255];

    center = null_check(center);
    left = null_check(left);
    right = null_check(right);

    //ROS_INFO("center: [%lf], left: [%lf], right: [%lf]", center, left, right);
    //ROS_INFO("center number: [%lf]", (-msg->angle_min)/msg->angle_increment);

    if(center < 0.5){
      ROS_WARN("center warning!!");
      cmd_vel.linear.x = 0.0;
      cmd_vel.linear.y = 0.0;
      cmd_vel.angular.z = 1.0;
    }
    if(left < 0.4){
      ROS_WARN("left warning!!");
      cmd_vel.linear.x = 0.0;
      cmd_vel.linear.y = 0.0;
      cmd_vel.angular.z = -1.0;
    }
    if(right < 0.4){
      ROS_WARN("right warning!!");
      cmd_vel.linear.x = 0.0;
      cmd_vel.linear.y = 0.0;
      cmd_vel.angular.z = 1.0;
    }
    if(center >=0.5 && left >= 0.4 && right >= 0.4){
      cmd_vel.linear.x = 0.2;
      cmd_vel.linear.y = 0.0;
      cmd_vel.angular.z = 0.0;
    }

    ROS_INFO("x: %lf, y: %lf, z: %lf", cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z);
    vel_pub.publish(cmd_vel);
  }

private:
  ros::NodeHandle n;
  ros::Subscriber scan_sub;
  ros::Publisher vel_pub;
  geometry_msgs::Twist cmd_vel;
};


int main(int argc, char **argv){
  ros::init(argc, argv, "go_forward");

  GoForward GFobj;

  ros::spin();
  return 0;
}

