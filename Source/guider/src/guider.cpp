#include "guider.h"

Guider::Guider()
{
  ros::NodeHandle private_nh("~");
  private_nh.getParam("guiding_period", guiding_period);
  private_nh.getParam("set_speed", desiredSpeed);
  private_nh.getParam("set_heading", desiredHeading);
  private_nh.getParam("LOS_radius", BaseLOS::radius);
  private_nh.getParam("LOS_min_delta", BaseLOS::minDelta);
  private_nh.getParam("LOS_max_delta", BaseLOS::maxDelta);
  private_nh.getParam("LOS_beta", BaseLOS::beta);

  ros::NodeHandle nh;
  nh.getParam("ned_lat", ned_lat);
  nh.getParam("ned_lon", ned_lon);

  pubSetpoint = nh.advertise<Setpoint>("setpoint", 1);
  pubError = nh.advertise<Error>("error", 1);
  subOdom = nh.subscribe("odom_1", 10, &Guider::onOdomCallBack, this);
  subItemList = nh.subscribe("mission/item_list", 1, &Guider::onItemListCallBack, this);
  loopGuidance = nh.createTimer(ros::Duration(guiding_period), &Guider::onGuidanceLoop, this);
  loopAddwaypointAvoid = nh.createTimer(ros::Duration(guiding_period), &Guider::addwaypoint, this);
  loopAvoid = nh.createTimer(ros::Duration(guiding_period), &Guider::Avoid, this);

  sub_mode = nh.subscribe("/mode_indoor", 10 , &Guider::onModeCallback, this);
  subPose = nh.subscribe("/amcl_pose", 10, &Guider::onCallbackPose, this );
  pub_complete = nh.advertise<Complete>("/Complete", 1);
  pubheadingplot = nh.advertise<heading_plot>("/heading_plot", 1);
  sub_avoid = nh.subscribe("/position_avoid", 10, &Guider::onCallbackPosAvoid, this);
  sub_ID = nh.subscribe("/pointID", 10, &Guider::onCallbackpointID, this);
  sub_goal_indoor_set = nh.subscribe("/goal_indoor", 10, &Guider::onCallbackGoalindoor, this);

  resStartMission = nh.advertiseService("command/start_mission", &Guider::onStartMissionCallBack, this);
  resSetMode = nh.advertiseService("command/set_mode", &Guider::onSetModeCallBack, this);
  resSetSetpointParams =
      nh.advertiseService("parameter/set_setpoint_params", &Guider::onSetSetpointParamsCallBack, this);
  resGetSetpointParams =
      nh.advertiseService("parameter/get_setpoint_params", &Guider::onGetSetpointParamsCallBack, this);
  resSetLOSParams = nh.advertiseService("parameter/set_LOS_params", &Guider::onSetLOSParamsCallBack, this);
  resGetLOSParams = nh.advertiseService("parameter/get_LOS_params", &Guider::onGetLOSParamsCallBack, this);
  mode_in = 0 ;
}

Guider::~Guider()
{
  if (ros::isStarted())
  {
    ros::shutdown();
    ros::waitForShutdown();
  }
}

void Guider::onCallbackGoalindoor(const goal_indoor::ConstPtr& goal_in){
  goal_indoor_set = goal_in->goal_indoor;

  waypoint_x_0 = goal_in->way_x_0 ;
  waypoint_x_1 = goal_in->way_x_1 ;
  waypoint_x_2 = goal_in->way_x_2 ;

  waypoint_y_0 = goal_in->way_y_0 ;
  waypoint_y_1 = goal_in->way_y_1 ;
  waypoint_y_2 = goal_in->way_y_2 ;

 // ROS_INFO_STREAM("Goal_indoor = " << goal_indoor_set);

  complete_indoor = waypoint_x_2 ;

}

void Guider::onModeCallback(const mode_indoor::ConstPtr& mode){
  mode_in = mode->mode_indoor ;
    if(mode_in == 1 && complete_indoor != complete_indoor_set){
        isMissionStarted = true ;
      straightLOSGuider.resetLOS();
  float waypoint_indoor_x[] = {waypoint_x_0, waypoint_x_1, waypoint_x_2 };
  float waypoint_indoor_y[] = {waypoint_y_0, waypoint_y_1 , waypoint_y_2 };
        int   size = sizeof(waypoint_indoor_x)/sizeof(waypoint_indoor_x[0]);
        float _x , _y;
        for (int i = 0; i < size ; i++)
        {
          _x = waypoint_indoor_x[i];
          _y = waypoint_indoor_y[i];
            ROS_INFO_STREAM("waypoint_indoor: x = " << _x << ", y = " << _y);   
            BaseLOS::waypoints.push_back(BaseLOS::Point(_x, _y));
          }
              sleep(5);
      straightLOSGuider.setupLOS(); 

    }  
}

void Guider::onCallbackpointID(const pointID::ConstPtr& id){
  pointId = id->pointID;

}

void Guider::onOdomCallBack(const Odometry::ConstPtr& msg)
{
  currX_outdoor = msg->position.x;
  currY_outdoor = msg->position.y;
  currHeading = msg->orientation.z;
}

void Guider::onCallbackPose(const PoseWithCovarianceStamped::ConstPtr& pos) {

    currX_indoor =  pos->pose.pose.position.x ;
    currY_indoor =  - pos->pose.pose.position.y ;
    CurrX_indoor_plot = currX_indoor ;

  // ROS_INFO_STREAM(currX_indoor);


  if (currX_indoor != 0.0){
      indoor = true ;
   //   ROS_INFO_STREAM("Robot stay indoor !!!");
  }

  // ROS_INFO("odomX = %f odomY = %f" , currX_indoor,currY_indoor);
}

void Guider::onCallbackPosAvoid(const Position_avoid_obstacles::ConstPtr& pos_avoid){
    x_avoid = pos_avoid->position_avoid_x;
    y_avoid = pos_avoid->position_avoid_y;
    obstacles = pos_avoid->obstacles ; 

}

void Guider::addwaypoint(const ros::TimerEvent& ){

    if(obstacles == true){
        if( indoor == true){
           
                ROS_INFO_STREAM("Obstacles !!!!!");
                isMissionStarted = true ;
              float waypoint_avoid_x[] = {currX_indoor, x_avoid, x_avoid};
              float waypoint_avoid_y[] = {currY_indoor, y_avoid , y_avoid + float(0.5)};
                straightLOSGuider.resetLOS();
              int   size = sizeof(waypoint_avoid_x)/sizeof(waypoint_avoid_x[0]);
              float _x , _y;
                  for (int i = 0; i < size ; i++)
                  {
                    _x = waypoint_avoid_x[i];
                    _y = waypoint_avoid_y[i];
                  ROS_INFO_STREAM("Waypoint_avoid_indoor: x = " << _x << ", y = " << _y);   
                  BaseLOS::waypoints.push_back(BaseLOS::Point(_x, _y));
                  }
                  straightLOSGuider.setupLOS();
                  com_avoid = true ;
                //  break;
         }
        else {
            
            //    ROS_INFO_STREAM("Obstacles !!!!!");
                isMissionStarted = true ;
              float waypoint_avoid_x[] = {currX_outdoor, x_avoid};
              float waypoint_avoid_y[] = {currY_outdoor, y_avoid};
                straightLOSGuider.resetLOS();
             int   size = sizeof(waypoint_avoid_x)/sizeof(waypoint_avoid_x[0]);
              float _x , _y;
                  for (int i = 0; i < size ; i++)
                  {
                    _x = waypoint_avoid_x[i];
                    _y = waypoint_avoid_y[i];
                  ROS_INFO_STREAM("Waypoint_avoid_outdoor: x = " << _x << ", y = " << _y);   
                  BaseLOS::waypoints.push_back(BaseLOS::Point(_x, _y));
                  }
                  straightLOSGuider.setupLOS();
                  com_avoid = true ;         

      //  }
  }
      }
}

void Guider::Avoid(const ros::TimerEvent&){
//      if(complete_Avoid == true){
//    ROS_INFO_STREAM("dfsafsdf");
//        isMissionStarted = true ;
//        straightLOSGuider.resetLOS();
//           int size = BaseLOS::waypoints.size();
//          // ROS_INFO_STREAM(size);
//          float x , y ;
//          for(int i = 0 ; i < size ; i++){
//            x = BaseLOS::waypoints[i].x;
//            y = BaseLOS::waypoints[i].y;
//            ROS_INFO_STREAM("waypoint_indoor: x = " << x << ", y = " << y);   
//           BaseLOS::waypoints.push_back(BaseLOS::Point(x,y));
//           break;
//          }
//        //      ROS_INFO("way_x = %f\n  way_y = %f" , x, y);
//          straightLOSGuider.setupLOS();
//          complete_Avoid = false;
//          com_avoid = false;
//  }
  if(AAA == 1.0){
    ROS_INFO_STREAM("ASASAS");
           isMissionStarted = true ;
        //  straightLOSGuider.resetLOS();
          int _size = BaseLOS::waypoints.size();
         // ROS_INFO_STREAM(size);
         float x , y ;
         for(int i = 3 ; i < _size ; i++){
           x = BaseLOS::waypoints[i].x;
           y = BaseLOS::waypoints[i].y;
           ROS_INFO_STREAM("waypoint_next: x = " << x << ", y = " << y);   
          BaseLOS::waypoints.push_back(BaseLOS::Point(x,y));
         }
         straightLOSGuider.setupLOS();
  }

  if( currX_indoor != 0.0){
    float waypoint_indoor_x[] = {waypoint_x_0, waypoint_x_1, waypoint_x_2 };
    float waypoint_indoor_y[] = {waypoint_y_0, waypoint_y_1 , waypoint_y_2 };

     if(complete_Avoid == true){
        isMissionStarted = true ;
      straightLOSGuider.resetLOS();
        int   size = sizeof(waypoint_indoor_x)/sizeof(waypoint_indoor_x[0]);
        float _x , _y;
        for (int i = 0; i < size ; i++)
        {
          _x = waypoint_indoor_x[i];
          _y = waypoint_indoor_y[i];
            ROS_INFO_STREAM("waypoint_indoor: x = " << _x << ", y = " << _y);   
            BaseLOS::waypoints.push_back(BaseLOS::Point(_x, _y));
          }
      straightLOSGuider.setupLOS(); 
      complete_Avoid = false ;
      com_avoid = false ;
    }
  }
}

void Guider::onItemListCallBack(const WaypointList::ConstPtr& msg)
{  
    lat_end_waypoint = msg->waypoints.back().x_lat ;

  float waypoint_indoor_x[] = {waypoint_x_0, waypoint_x_1, waypoint_x_2 };
  float waypoint_indoor_y[] = {waypoint_y_0, waypoint_y_1 , waypoint_y_2 };
  if(mode_in == 1){

          straightLOSGuider.resetLOS();
        int   size = sizeof(waypoint_indoor_x)/sizeof(waypoint_indoor_x[0]);
        float _x , _y;
        for (int i = 0; i < size ; i++)
        {
          _x = waypoint_indoor_x[i];
          _y = waypoint_indoor_y[i];
            ROS_INFO_STREAM("waypoint_indoor: x = " << _x << ", y = " << _y);   
            BaseLOS::waypoints.push_back(BaseLOS::Point(_x, _y));
          }
     straightLOSGuider.setupLOS();

      // straightLOSGuider.resetLOS();

for (auto it = msg->waypoints.begin(); it != msg->waypoints.end(); it++)
        {
          switch (it->command)
          {
          case 16: // MAV_CMD_NAV_WAYPOINT
          {
          //  BaseLOS::waypoints.clear();
            double x, y;
            convert_global_to_local_coords(it->x_lat * 1e-7, it->y_long * 1e-7, ned_lat, ned_lon, x, y);

            ROS_INFO_STREAM("Waypoint___: x = " << x << ", y = " << y);
          BaseLOS::waypoints.push_back(BaseLOS::Point(x, y));
            break;
          }
          }
        }
        }
        
  else {

    straightLOSGuider.resetLOS();

  for (auto it = msg->waypoints.begin(); it != msg->waypoints.end(); it++)
  {
    switch (it->command)
    {
    case 16: // MAV_CMD_NAV_WAYPOINT
    {
      double x, y;
      convert_global_to_local_coords(it->x_lat * 1e-7, it->y_long * 1e-7, ned_lat, ned_lon, x, y);
      ROS_INFO_STREAM("Waypoint: x = " << x << ", y = " << y);
      BaseLOS::waypoints.push_back(BaseLOS::Point(x, y));

      break;
    }

    case 20: // MAV_CMD_NAV_RETURN_TO_LAUNCH
    {
      BaseLOS::waypoints.push_back(BaseLOS::waypoints[0]);
      break;
    }
    }
  }
        straightLOSGuider.setupLOS();
  }
 }
void Guider::onGuidanceLoop(const ros::TimerEvent& /*event*/)
{

  if (!isMissionStarted)
    return;

  if (customMode == "1") // AUTO_HEADING
    publishSetpoint(desiredSpeed, desiredHeading);

  if (customMode == "2") // LOS_STRAIGHT
  {

    // // test indoor LOS
    //   currX = currX_indoor ;
    //   currY =  currY_indoor ;

    if(mode_in == 1){
      currX = currX_indoor ;
      currY =  currY_indoor ;
    }
    else {
      currX = currX_outdoor ;
      currY =  currY_outdoor ;
    }
    if (straightLOSGuider.runLOS(currX,currY))
    {
       // ROS_INFO("currX = %f currY = %f" , currX_indoor,currY_indoor);
      publishSetpoint(desiredSpeed, straightLOSGuider.desiredHeading);
      publishError(straightLOSGuider.alongTrackError, straightLOSGuider.crossTrackError);
    }
    else
    {
      isMissionStarted = false;

      if (complete_indoor == complete_indoor_set){
        ROS_INFO_STREAM("Escape indoor !! \n");
        system("rosnode kill rviz");
        complete_indoor_set = 0.0 ;
   float waypoint_a[] = {-95.4052, -111.951, -127.296 ,-134.379 };
    float waypoint_b[] = {-18.4606, -49.6799 ,-39.3463,-37.2818 };

        isMissionStarted = true ;
      straightLOSGuider.resetLOS();
        int   size = sizeof(waypoint_a)/sizeof(waypoint_a[0]);
        float _x , _y;
        for (int i = 0; i < size ; i++)
        {
          _x = waypoint_a[i];
          _y = waypoint_b[i];
            ROS_INFO_STREAM("waypoint_ab: x = " << _x << ", y = " << _y);   
            BaseLOS::waypoints.push_back(BaseLOS::Point(_x, _y));
          }
      straightLOSGuider.setupLOS();

      }
      if(lat_end_waypoint != lat_end_waypoint_set){
          utils::Complete _complete ;
        _complete.complete = true ;
        _complete.complete_indoor = false ;
        pub_complete.publish(_complete);
        ROS_INFO("Complete Goal !!!");
      }
      if(lat_end_waypoint == lat_end_waypoint_set){
        if(com_avoid == set_com_avoid){
          complete_Avoid = true ;
          ROS_INFO_STREAM("complete_avoid");
        }
              utils::Complete _complete ;
        _complete.complete_indoor = true ;
        pub_complete.publish(_complete);
       //  _complete.complete = true ;
        ROS_INFO("Complete Goal Indoor !!!");

      }

    }
    }


}

bool Guider::onStartMissionCallBack(CommandLongRequest& /*req*/, CommandLongResponse& res)
{
  isMissionStarted = true;
  res.result = 0; // MAV_RESULT_ACCEPTED
  ROS_INFO("Mission started");

  return true;
}

bool Guider::onSetModeCallBack(SetModeRequest& req, SetModeResponse& res)
{
  customMode = req.custom_mode;
  res.mode_sent = 0;

  return true;
}

bool Guider::onSetSetpointParamsCallBack(ParamSetRequest& req, ParamSetResponse& res)
{
  res.success = false;

  if (compare_string(req.param_id.data(), "set_speed"))
  {
    desiredSpeed = req.value.real;
    res.success = true;
  }
  if (compare_string(req.param_id.data(), "set_heading"))
  {
    desiredHeading = req.value.real;
    res.success = true;
  }

  res.value.real = req.value.real;
  return res.success;
}

bool Guider::onGetSetpointParamsCallBack(ParamGetRequest& req, ParamGetResponse& res)
{
  res.success = false;

  if (compare_string(req.param_id.data(), "set_speed"))
  {
    res.value.real = desiredSpeed;
    res.success = true;
  }
  if (compare_string(req.param_id.data(), "set_heading"))
  {
    res.value.real = desiredHeading;
    res.success = true;
  }

  return res.success;
}

bool Guider::onSetLOSParamsCallBack(ParamSetRequest& req, ParamSetResponse& res)
{
  res.success = false;

  if (compare_string(req.param_id.data(), "LOS_radius"))
  {
    BaseLOS::radius = req.value.real;
    res.success = true;
  }
  if (compare_string(req.param_id.data(), "LOS_min_delta"))
  {
    BaseLOS::minDelta = req.value.real;
    res.success = true;
  }
  if (compare_string(req.param_id.data(), "LOS_max_delta"))
  {
    BaseLOS::maxDelta = req.value.real;
    res.success = true;
  }
  if (compare_string(req.param_id.data(), "LOS_beta"))
  {
    BaseLOS::beta = req.value.real;
    res.success = true;
  }

  res.value.real = req.value.real;
  return res.success;
}

bool Guider::onGetLOSParamsCallBack(ParamGetRequest& req, ParamGetResponse& res)
{
  res.success = false;

  if (compare_string(req.param_id.data(), "LOS_radius"))
  {
    res.value.real = BaseLOS::radius;
    res.success = true;
  }
  if (compare_string(req.param_id.data(), "LOS_min_delta"))
  {
    res.value.real = BaseLOS::minDelta;
    res.success = true;
  }
  if (compare_string(req.param_id.data(), "LOS_max_delta"))
  {
    res.value.real = BaseLOS::maxDelta;
    res.success = true;
  }
  if (compare_string(req.param_id.data(), "LOS_beta"))
  {
    res.value.real = BaseLOS::beta;
    res.success = true;
  }

  return res.success;
}

void Guider::publishSetpoint(const double& speed, const double& heading)
{
  Setpoint setpointMsg;
  setpointMsg.header.stamp = ros::Time::now();
  setpointMsg.linear_velocity.x = speed;
  setpointMsg.orientation.z = heading;
  pubSetpoint.publish(setpointMsg);
}

void Guider::publishError(const double& xe, const double& ye)
{
  Error errorMsg;
  errorMsg.header.stamp = ros::Time::now();
  errorMsg.along_track = xe;
  errorMsg.cross_track = ye;
  pubError.publish(errorMsg);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "guider");
  ros::NodeHandle n ;
  Guider guide;
  ros::spin();
}
