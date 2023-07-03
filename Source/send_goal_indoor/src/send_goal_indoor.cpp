#include <ros/ros.h>
#include <utils/goal_indoor.h>
#include <utils/WaypointList.h>

using namespace std;

float set_goal_indoor ;
float lat_end_waypoint;
float lon_end_waypoint;
float goal_in;

float waypoint_x_0, waypoint_x_1, waypoint_x_2 ;
float waypoint_y_0, waypoint_y_1, waypoint_y_2 ;


float waypoint_C1_CDIO_x[] = {0.3, 0.589607715607, 27.3166999817 };
float waypoint_C1_CDIO_y[] = {0.0, 10.7453956604 , 10.245511055 };

float waypoint_C1_newroom_x[] = {0.3, 0.589607715607, 0.334822654724 };
float waypoint_C1_newroom_y[] = {0.0, 10.7453956604 , 16.5067253113 };

float waypoint_CDIO_C1_x[] = { 27.3166999817, 0.589607715607, 0.3 };
float waypoint_CDIO_C1_y[] = { 10.245511055, 10.7453956604, 0.0 };

float waypoint_newroom_C1_x[] = { 0.334822654724 , 0.589607715607, 0.3};
float waypoint_newroom_C1_y[] = { 16.5067253113, 10.7453956604, 0.0 };

float waypoint_CDIO_newroom_x[] = { 27.3166999817, 0.589607715607, 0.334822654724 };
float waypoint_CDIO_newroom_y[] = { 10.245511055, 10.7453956604,  16.5067253113 };

float waypoint_newroom_CDIO_x[] = { 0.334822654724 , 0.589607715607, 27.3166999817};
float waypoint_newroom_CDIO_y[] = { 16.5067253113, 10.7453956604, 10.245511055 };

// float waypoint_C1_CDIO_x[] = {0.01, 6.09483494129, 8.75942210115 };
// float waypoint_C1_CDIO_y[] = {0.1, 0.494473309231 , -21.7755431121 };

// float waypoint_C1_newroom_x[] = {0.01, 6.09483494129, 30.1908539723 };
// float waypoint_C1_newroom_y[] = {0.1, 0.494473309231 , 4.16770254799 };

// float waypoint_CDIO_C1_x[] = { 8.75942210115 ,6.09483494129, 0.01 };
// float waypoint_CDIO_C1_y[] = { -21.7755431121 , 0.494473309231, 0.1 };

// float waypoint_newroom_C1_x[] = { 30.1908539723 , 6.09483494129, 0.01};
// float waypoint_newroom_C1_y[] = { 4.16770254799, 0.494473309231, 0.1 };

// float waypoint_CDIO_newroom_x[] = { 8.75942210115, 6.09483494129, 30.1908539723};
// float waypoint_CDIO_newroom_y[] = { -21.7755431121, 0.494473309231,  4.16770254799 };

// float waypoint_newroom_CDIO_x[] = { 30.1908539723 ,6.09483494129, 8.75942210115 };
// float waypoint_newroom_CDIO_y[] = { 4.16770254799, 0.494473309231 , -21.7755431121 };

void CallbackWaypoint(const utils::WaypointList &waypoint){
  
    lat_end_waypoint = waypoint.waypoints.back().x_lat;
    lon_end_waypoint = waypoint.waypoints.back().y_long;
    set_goal_indoor = waypoint.waypoints.back().z_alt;

    ROS_INFO("lat = %.9f", lat_end_waypoint);
    ROS_INFO("lon = %.9f", lon_end_waypoint);
    ROS_INFO("goal_in = %f", set_goal_indoor);

}
//..................................................................................
int main(int argc, char** argv){
  ros::init(argc, argv, "send_goal_indoor");

  ros::NodeHandle n;

    ros::Subscriber waypoint_ = n.subscribe("/mission/item_list", 1, CallbackWaypoint);
    ros::Publisher pub_goal_in = n.advertise<utils::goal_indoor>("/goal_indoor", 50);

  ros::Rate r(20.0);
  while(n.ok()){
    ros::spinOnce();      
    // pub_waypoint();
    if(set_goal_indoor != 0.0f){
      if(set_goal_indoor == 1.2f){
  //  ROS_INFO("Goal_indoor_set is : C1 -> CDIO");
          waypoint_x_0 = waypoint_C1_CDIO_x[0];
          waypoint_x_1 = waypoint_C1_CDIO_x[1];
          waypoint_x_2 = waypoint_C1_CDIO_x[2];

          waypoint_y_0 = waypoint_C1_CDIO_y[0];
          waypoint_y_1 = waypoint_C1_CDIO_y[1];
          waypoint_y_2 = waypoint_C1_CDIO_y[2];

      }

      else if(set_goal_indoor == 2.1f){
  //  ROS_INFO("Goal_indoor_set is : CDIO -> C1");
          waypoint_x_0 = waypoint_CDIO_C1_x[0];
          waypoint_x_1 = waypoint_CDIO_C1_x[1];
          waypoint_x_2 = waypoint_CDIO_C1_x[2];

          waypoint_y_0 = waypoint_CDIO_C1_y[0];
          waypoint_y_1 = waypoint_CDIO_C1_y[1];
          waypoint_y_2 = waypoint_CDIO_C1_y[2];
      }

      else if(set_goal_indoor == 1.3f){
  //  ROS_INFO("Goal_indoor_set is : C1 -> Newroom");
          waypoint_x_0 = waypoint_C1_newroom_x[0];
          waypoint_x_1 = waypoint_C1_newroom_x[1];
          waypoint_x_2 = waypoint_C1_newroom_x[2];

          waypoint_y_0 = waypoint_C1_newroom_y[0];
          waypoint_y_1 = waypoint_C1_newroom_y[1];
          waypoint_y_2 = waypoint_C1_newroom_y[2];
      }

      else if(set_goal_indoor == 3.1f){
   // ROS_INFO("Goal_indoor_set is : Newroom -> C1");
          waypoint_x_0 = waypoint_newroom_C1_x[0];
          waypoint_x_1 = waypoint_newroom_C1_x[1];
          waypoint_x_2 = waypoint_newroom_C1_x[2];

          waypoint_y_0 = waypoint_newroom_C1_y[0];
          waypoint_y_1 = waypoint_newroom_C1_y[1];
          waypoint_y_2 = waypoint_newroom_C1_y[2];
      }

      else if(set_goal_indoor == 2.3f){
   // ROS_INFO("Goal_indoor_set is : CDIO -> Newroom");
          waypoint_x_0 = waypoint_CDIO_newroom_x[0];
          waypoint_x_1 = waypoint_CDIO_newroom_x[1];
          waypoint_x_2 = waypoint_CDIO_newroom_x[2];

          waypoint_y_0 = waypoint_CDIO_newroom_y[0];
          waypoint_y_1 = waypoint_CDIO_newroom_y[1];
          waypoint_y_2 = waypoint_CDIO_newroom_y[2];
      }

      else if(set_goal_indoor == 3.2f){
   // ROS_INFO("Goal_indoor_set is : Newroom -> CDIO");
          waypoint_x_0 = waypoint_newroom_CDIO_x[0];
          waypoint_x_1 = waypoint_newroom_CDIO_x[1];
          waypoint_x_2 = waypoint_newroom_CDIO_x[2];

          waypoint_y_0 = waypoint_newroom_CDIO_y[0];
          waypoint_y_1 = waypoint_newroom_CDIO_y[1];
          waypoint_y_2 = waypoint_newroom_CDIO_y[2];
      }

    utils::goal_indoor _goal_in;
    _goal_in.goal_indoor = set_goal_indoor ;

    _goal_in.way_x_0 = waypoint_x_0 ;
    _goal_in.way_x_1 = waypoint_x_1 ;
    _goal_in.way_x_2 = waypoint_x_2 ;

    _goal_in.way_y_0 = waypoint_y_0 ;
    _goal_in.way_y_1 = waypoint_y_1 ;
    _goal_in.way_y_2 = waypoint_y_2 ;
    pub_goal_in.publish(_goal_in);

    set_goal_indoor = 0.0 ;
  }
  r.sleep();
  }
}
