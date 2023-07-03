#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include "std_msgs/Float32.h"
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <cmath>
#include <iostream>
#include <math.h>
#include <stdio.h>
#include <utils/Odometry.h>
#include <utils/Position_avoid_obstacles.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <utils/mode_indoor.h>

using namespace std;

float width = 0.5 ;
float hight = 0.75;

float CurrX_outdoor, CurrY_outdoor ;
float CurrX_indoor = 0.0 ;
float CurrY_indoor;
float CurrX, CurrY ;

float CurrHeading_indoor_x;
float CurrHeading_indoor_y;
float CurrHeading_indoor_z;
float CurrHeading_indoor_w;
float CurrOrient_indoor ;

float min_front  ;
float min_front_left  ;
float min_front_right ;
float _min ;

float range_front[1000] ;
float range_front_left[1000] ;
float range_front_right[1000];

int location_obstacle ;
int min_front_element;
float angle_min_front;
float angle_min_front_left;
float angle_min_front_right;

int min_front_left_element;
int min_front_right_element;

float angle_avoid;
float angle_add ;
float angle_new_waypoint;
float distance_avoid;

float pos_x, pos_y ;
float x_add, y_add ;
bool _trueObs  ;

float new_waypointX, new_waypointY;

void CallbackCurrPosition(const utils::Odometry &pos ){
  CurrX_outdoor = pos.position.x;
  CurrY_outdoor = pos.position.y ;
}

void CallbackCurrPositionIndoor(const geometry_msgs::PoseWithCovarianceStamped &pos_in){
  CurrX_indoor = pos_in.pose.pose.position.x;
  CurrY_indoor = pos_in.pose.pose.position.y;
  
    tf::Quaternion q(
  CurrHeading_indoor_x = pos_in.pose.pose.orientation.x,
  CurrHeading_indoor_y = pos_in.pose.pose.orientation.y,
  CurrHeading_indoor_z = pos_in.pose.pose.orientation.z,
  CurrHeading_indoor_w = pos_in.pose.pose.orientation.w);

    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    CurrOrient_indoor = - yaw ;
  //  ROS_INFO_STREAM("Orient_z = " << CurrOrient_indoor);
}

void CallrangeLaser(const sensor_msgs::LaserScan &laser){

    min_front = 30;
    min_front_left =  30;
    min_front_right =  30;

  for (int  i = 462 ; i < 616 ; i++)
  {
    range_front[i] = laser.ranges[i];
        if ( min_front > range_front[i])
        {
             min_front = range_front[i];
             min_front_element = i ;             
        }
  }

  for (int j = 617 ; j < 770 ; j++)
  {
    range_front_left[j] = laser.ranges[j];
        if( min_front_left > range_front_left[j])
        {
              min_front_left = range_front_left[j];
              min_front_left_element = j;
        }
  
  }
  for (int k = 308 ; k < 461 ; k++)
  {
    range_front_right[k] = laser.ranges[k];
        if( min_front_right > range_front_right[k])
        {
              min_front_right = range_front_right[k];
              min_front_right_element = k ;
        }
  }

  if(min_front < min_front_left && min_front < min_front_right && min_front <= 2*hight)
  {
      location_obstacle = 1 ;
      _min = min_front;
      _trueObs = true;
  }
  else if(min_front_left < min_front && min_front_left < min_front_right && min_front_left <= 2*hight)
  {
      location_obstacle = 2 ;
      _min = min_front_left ;
      _trueObs = true;
  }
  else if (min_front_right < min_front && min_front_right < min_front_left && min_front_right <= 2*hight)
  { 
      location_obstacle = 3 ;
      _min = min_front_right ;
      _trueObs = true;
  }
  else
  {
      _trueObs = false ;
      }
  if(CurrX_indoor !=0){
  if(location_obstacle == 1 )
  {
      angle_min_front = (abs(min_front_element-540)*0.25)*M_PI/180;
      angle_add = abs(atan(1.5*width/min_front));
      angle_avoid = angle_min_front + angle_add;
      distance_avoid = sqrt(pow(min_front,2)+ pow(1.5*width,2));
      if(min_front_element > 540){
      ROS_INFO("Obstacle in front_left -> Avoid in Right");
      angle_new_waypoint = abs(90-CurrOrient_indoor-angle_avoid);
        x_add = -distance_avoid*cos(angle_avoid);
        y_add = distance_avoid*sin(angle_avoid); 
      }
      if(min_front_element < 540){
      angle_new_waypoint = abs(CurrOrient_indoor-angle_avoid);
      ROS_INFO("Obstacle in front_right -> Avoid in Left");
        x_add = distance_avoid*cos(angle_avoid);
        y_add = distance_avoid*sin(angle_avoid); 
      }
  }
  else if (location_obstacle == 2 )
  {
      ROS_INFO("Obstacle in left -> Avoid in right");
      angle_min_front_left = (abs(min_front_left_element-540)*0.25)*M_PI/180;
      angle_add = abs(atan(1.5*width/min_front_left));
      angle_avoid = abs(angle_add - angle_min_front_left) ;
      distance_avoid = sqrt(pow(min_front_left,2)+ pow(1.5*width,2));
      angle_new_waypoint = abs(90-CurrOrient_indoor-angle_avoid);
      x_add = -distance_avoid*cos(angle_avoid);
      y_add = distance_avoid*sin(angle_avoid); 
     
  }
  else if (location_obstacle == 3)
  {  
      ROS_INFO("Obstacle in right -> Avoid in Left");
      angle_min_front_right = (abs(min_front_right_element-540)*0.25)*M_PI/180;
      angle_add = abs(atan(1.5*width/min_front_right));
      angle_avoid = abs(angle_add - angle_min_front_right);
      distance_avoid = sqrt(pow(min_front_right,2)+ pow(1.5*width,2));
      angle_new_waypoint = abs(CurrOrient_indoor-angle_avoid);
      x_add = distance_avoid*cos(angle_avoid);
      y_add = distance_avoid*sin(angle_avoid); 
      }
        new_waypointX = CurrX_indoor + x_add ;
        new_waypointY = CurrY_indoor + y_add ;
  }
  else 
  {
    if(location_obstacle == 1 ){
        angle_min_front = (abs(min_front_element-540)*0.25)*M_PI/180;
        angle_add = abs(atan(1.5*width/min_front));
        angle_avoid = angle_min_front + angle_add;
        distance_avoid = sqrt(pow(min_front,2)+ pow(1.5*width,2));

          x_add = distance_avoid*cos(angle_avoid);
          y_add = distance_avoid*sin(angle_avoid); 
        new_waypointX = CurrX_outdoor - x_add ;
        new_waypointY = CurrY_outdoor - y_add ;
        //  ROS_INFO("%.1f %.1f\n " ,new_waypointX_indoor , new_waypointY_indoor);    
        }
  }

}

//..................................................................................
int main(int argc, char** argv){
  ros::init(argc, argv, "test_detect_obstacle");

  ros::NodeHandle n;
	ros::Subscriber scan = n.subscribe("/horizontal_laser_2d", 1, CallrangeLaser);
  ros::Subscriber sub_odom = n.subscribe("/odom_1", 1, CallbackCurrPosition);
  ros::Publisher pub_pos_avoid =n.advertise<utils::Position_avoid_obstacles>("/position_avoid", 50);
  ros::Subscriber sub_odom_indoor = n.subscribe("/amcl_pose" ,  1, CallbackCurrPositionIndoor);

  ros::Rate r(20.0);
  while(n.ok()){
    utils::Position_avoid_obstacles _pose ;
    if(CurrX_indoor !=0 ){
    _pose.position_avoid_x = new_waypointX ;
    _pose.position_avoid_y = -new_waypointY ;
    }
    else {
    _pose.position_avoid_x = new_waypointX ;
    _pose.position_avoid_y = new_waypointY ;
    }
    _pose.obstacles = _trueObs ;

    pub_pos_avoid.publish(_pose);

    utils::mode_indoor _mod ;
    _mod.abc = 1.0 ;
    

    ros::spinOnce();   
    r.sleep();
  }
  return 0 ;
}
