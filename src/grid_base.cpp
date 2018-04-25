#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"

#define RANGE_MAX 5.6
#define SCALE 0.3

class GoForward{
public:
  GoForward(){
    scan_sub = n.subscribe("scan", 1000, &GoForward::scanCallback, this);
    vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);
  }

  double null_check(double target){
    if(!(target > 0)){
      target = (double)RANGE_MAX;
    }
    return target;
  }
  
  void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg){

    double center_number = (-msg->angle_min)/msg->angle_increment;
    
    double center = null_check(msg->ranges[center_number]);
    double left = null_check(msg->ranges[center_number+256]);
    double right = null_check(msg->ranges[center_number-256]);
    double fore_left = null_check(msg->ranges[center_number+128]);
    double fore_right = null_check(msg->ranges[center_number-128]);
    double back_left = null_check(msg->ranges[center_number+340]);
    double back_right = null_check(msg->ranges[center_number-340]);
    
    double grid[7] = {back_left/2*1.73, left, fore_left/1.41, center, fore_right/1.41, right, back_right/2*1.73}; 
    int state_now;

    // init
    state_now = 1;

    // 1, left state
    if(
      grid[0] > SCALE && grid[0] < SCALE*2
      && grid[1] > SCALE && grid[1] < SCALE*2
      && grid[2] > SCALE && grid[2] < SCALE*2
      && grid[3] > SCALE*2
      && grid[4] > SCALE*2
      && grid[5] > SCALE*2
      && grid[6] > SCALE*2
    ){
      state_now = 1;
      ROS_INFO("left state");
    }

    // 2, forward state
    if(
      grid[0] > SCALE*2
      && grid[1] > SCALE*2
      && grid[2] > SCALE && grid[2] < SCALE*2
      && grid[3] > SCALE && grid[3] < SCALE*2
      && grid[4] > SCALE && grid[4] < SCALE*2
      && grid[5] > SCALE*2
      && grid[6] > SCALE*2
    ){
      state_now = 2;
      ROS_INFO("forward state");
    }

    // 3, backward state
    if(
      grid[0] > SCALE && grid[0] < SCALE*2
      && grid[1] > SCALE*2
      && grid[2] > SCALE*2
      && grid[3] > SCALE*2
      && grid[4] > SCALE*2
      && grid[5] > SCALE*2
      && grid[6] > SCALE && grid[6] < SCALE*2
    ){
      state_now = 3;
      ROS_INFO("backward state");
    }

    // 4, surrounded state 
    if(
      grid[0] > SCALE && grid[0] < SCALE*2
      && grid[1] > SCALE*2
      && grid[2] > SCALE*2
      && grid[3] > SCALE*2
      && grid[4] > SCALE*2
      && grid[5] > SCALE*2
      && grid[6] > SCALE && grid[6] < SCALE*2
    ){
      state_now = 4;
      ROS_INFO("surrounded state");
    }

    // 5, free state 
    if(
      grid[0] > SCALE*2
      && grid[1] > SCALE*2
      && grid[2] > SCALE*2
      && grid[3] > SCALE*2
      && grid[4] > SCALE*2
      && grid[5] > SCALE*2
      && grid[6] > SCALE*2
    ){
      state_now = 5;
      ROS_INFO("free state");
    }

    // 6, front hole state 
    if(
      grid[0] > SCALE && grid[0] < SCALE*2
      && grid[1] > SCALE && grid[1] < SCALE*2
      && grid[2] > SCALE && grid[2] < SCALE*2
      && grid[3] > SCALE*2
      && grid[4] > SCALE && grid[4] < SCALE*2
      && grid[5] > SCALE && grid[5] < SCALE*2
      && grid[6] > SCALE && grid[6] < SCALE*2
    ){
      state_now = 6;
      ROS_INFO("front hole state");
    }

    // 7, back left state 
    if(
      grid[0] > SCALE && grid[0] < SCALE*2
      && grid[1] > SCALE*2
      && grid[2] > SCALE*2
      && grid[3] > SCALE*2
      && grid[4] > SCALE*2
      && grid[5] > SCALE*2
      && grid[6] > SCALE*2
    ){
      state_now = 7;
      ROS_INFO("back left state");
    }

    // 8, fore left state 
    if(
      grid[0] > SCALE*2
      && grid[1] > SCALE*2
      && grid[2] > SCALE && grid[2] < SCALE*2
      && grid[3] > SCALE*2
      && grid[4] > SCALE*2
      && grid[5] > SCALE*2
      && grid[6] > SCALE*2
    ){
      state_now = 8;
      ROS_INFO("fore left state");
    }

    // 9, recovery state 
    if(
      grid[0] <= SCALE
      || grid[1] <= SCALE
      || grid[2] <= SCALE
      || grid[3] <= SCALE
      || grid[4] <= SCALE
      || grid[5] <= SCALE
      || grid[6] <= SCALE
    ){
      if(
        grid[2] > SCALE
        && grid[3] > SCALE
        && grid[4] > SCALE
      ){
        state_now = 9;
        ROS_INFO("recovery state - safe");
      }else{
        state_now = 10;
        ROS_INFO("recovery state - out");
      }
    }
    
    // 10, back mid left state
    if(
      grid[0] > SCALE && grid[0] < SCALE*2
      && grid[1] > SCALE && grid[1] < SCALE*2
      && grid[2] > SCALE*2
      && grid[3] > SCALE*2
      && grid[4] > SCALE*2
      && grid[5] > SCALE*2
      && grid[6] > SCALE*2
    ){
      state_now = 11;
      ROS_INFO("back mid left state");
    }

    switch(state_now){
      // stop
      case 0:  
        cmd_vel.linear.x = 0.0;
        cmd_vel.linear.y = 0.0;
        cmd_vel.angular.z = 0.0;
        break;

      // go forward 
      case 1:  // left state
      case 5:  // free state
      case 6:  // front hole state
      case 8:  // fore left state
      case 9:  // recovery state - safe
        cmd_vel.linear.x = 0.2;
        cmd_vel.linear.y = 0.0;
        cmd_vel.angular.z = 0.0;
        ROS_INFO("-> go forward");
        break;

      // turn right
      case 2:  // forward state
      case 4:  // surrounded state
      case 10: // recovery state - out
        cmd_vel.linear.x = 0.1;
        cmd_vel.linear.y = 0.0;
        cmd_vel.angular.z = -1.0;
        ROS_INFO("-> turn right");
        break;

      // turn left
      case 3:  // backward state
      case 7:  // back left state
      case 11: // back mid left state
        cmd_vel.linear.x = 0.1;
        cmd_vel.linear.y = 0.0;
        cmd_vel.angular.z = 1.0;
        ROS_INFO("-> turn left");
        break;
    }

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

