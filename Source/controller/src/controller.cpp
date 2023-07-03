#include "controller.h"

Controller::Controller()
{
  ros::NodeHandle private_nh("~");
  private_nh.getParam("controlling_period", controlling_period);
  private_nh.getParam("diff_rpm", diff_rpm);
  private_nh.getParam("heading_Kp", headingPID.Kp);
  private_nh.getParam("heading_Ki", headingPID.Ki);
  private_nh.getParam("heading_Kd", headingPID.Kd);

  ros::NodeHandle nh;
  nh.getParam("wheel_radius", wheel_radius);

  pubDiffVel = nh.advertise<DiffVel>("diff_motor/vel", 10);
  subOdom = nh.subscribe("odom_1", 10, &Controller::onOdomCallBack, this);
  subSetpoint = nh.subscribe("setpoint", 10, &Controller::onSetpointCallBack, this);
  loopControl = nh.createTimer(ros::Duration(controlling_period), &Controller::onControlLoop, this);
  subRange = nh.subscribe("horizontal_laser_2d", 10, &Controller::onLidarCallback, this);
  subPose = nh.subscribe("amcl_pose", 10, &Controller::onPoseCallback, this);
  sub_mode = nh.subscribe("/mode_indoor", 10 , &Controller::onModeCallback, this);


  resSetHeadingPID = nh.advertiseService("parameter/set_heading_PID", &Controller::onSetHeadingPIDCallBack, this);
  resGetHeadingPID = nh.advertiseService("parameter/get_heading_PID", &Controller::onGetHeadingPIDCallBack, this);

  lastControlUpdateTime = lastSetpointTime = ros::Time::now();
  headingPID.threshold = diff_rpm;
}

Controller::~Controller()
{
  if (ros::isStarted())
  {
    ros::shutdown();
    ros::waitForShutdown();
  }
}

void Controller::onModeCallback(const mode_indoor::ConstPtr& mode){
  mode_in = mode->mode_indoor ;
  if(mode_in == 1){}
    //  system("roslaunch cartographer_ros show_tf.launch ");
} 

void Controller::onLidarCallback(const LaserScan::ConstPtr& laser)
{

    min_front = laser -> ranges[462];
    min_front_left =  laser -> ranges[616];
    min_front_right =  laser -> ranges[308];

    range_right = laser -> ranges[154];
    range_left = laser -> ranges[924];

  for ( int i = 462 ; i < 616 ; i++)
  {
    range_front[i] = laser -> ranges[i];
        if ( min_front > range_front[i])
              min_front = range_front[i];
  }

  for (int j = 617 ; j < 770 ; j++)
  {
    range_front_left[j] = laser -> ranges[j];
        if( min_front_left > range_front_left[j])
              min_front_left = range_front_left[j];

  }
  for (int k = 308 ; k < 461 ; k++)
  {
    range_front_right[k] = laser -> ranges[k];
        if( min_front_right > range_front_right[k])
              min_front_right = range_front_right[k];

  }
}


void Controller::onOdomCallBack(const Odometry::ConstPtr& msg)
{
  currHeading = msg->orientation.z;
}

void Controller::onPoseCallback(const PoseWithCovarianceStamped::ConstPtr& pos)
{
  tf::Quaternion q(
    pos->pose.pose.orientation.x,
    pos->pose.pose.orientation.y,
    pos->pose.pose.orientation.z,
    pos->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    
  currHeading_indoor = - yaw;
  // ROS_INFO ("CurrHeading_indoor = %f" ,currHeading_indoor);
}

void Controller::onSetpointCallBack(const Setpoint::ConstPtr& msg)
{
  lastSetpointTime = msg->header.stamp;

  desiredSpeed = msg->linear_velocity.x;
  desiredHeading = msg->orientation.z;
}

void Controller::onControlLoop(const ros::TimerEvent& event)
{
  double dtc = (event.current_real - lastSetpointTime).toSec();
  if (dtc > 0.5)
  {
    DiffVel diffVelMsg;
    diffVelMsg.header.stamp = event.current_real;
    diffVelMsg.left_vel = 0;
    diffVelMsg.right_vel = 0;
   
    pubDiffVel.publish(diffVelMsg);
  

    return;
  }

  double dt = event.current_real.toSec() - lastControlUpdateTime.toSec();

  // // test indoor LOS 
  //      double deltaHeading = desiredHeading - currHeading_indoor;
  //      headingPID.error = atan2(sin(deltaHeading), cos(deltaHeading));
  //      headingPID.Ts = dt;
  //      headingPID.runPID();

  if(mode_in == 1){
      double deltaHeading = desiredHeading - currHeading_indoor;
      headingPID.error = atan2(sin(deltaHeading), cos(deltaHeading));
      headingPID.Ts = dt;
      headingPID.runPID();
  }
  else {
      double deltaHeading = desiredHeading - currHeading;
      headingPID.error = atan2(sin(deltaHeading), cos(deltaHeading));
      headingPID.Ts = dt;
      headingPID.runPID();
  } 

  DiffVel diffVelMsg;
  diffVelMsg.header.stamp = event.current_real;

  diffVelMsg.left_vel = desiredSpeed / wheel_radius + headingPID.output * M_PI / 30;
  diffVelMsg.right_vel = desiredSpeed / wheel_radius - headingPID.output * M_PI / 30;
   
  pubDiffVel.publish(diffVelMsg);

  lastControlUpdateTime = event.current_real;
}

bool Controller::onSetHeadingPIDCallBack(ParamSetRequest& req, ParamSetResponse& res)
{
  res.success = false;

  if (compare_string(req.param_id.data(), "heading_Kp"))
  {
    headingPID.Kp = req.value.real;
    res.success = true;
  }
  if (compare_string(req.param_id.data(), "heading_Ki"))
  {
    headingPID.Ki = req.value.real;
    res.success = true;
  }
  if (compare_string(req.param_id.data(), "heading_Kd"))
  {
    headingPID.Kd = req.value.real;
    res.success = true;
  }

  res.value.real = req.value.real;
  return res.success;
}

bool Controller::onGetHeadingPIDCallBack(ParamGetRequest& req, ParamGetResponse& res)
{
  res.success = false;

  if (compare_string(req.param_id.data(), "heading_Kp"))
  {
    res.value.real = headingPID.Kp;
    res.success = true;
  }
  if (compare_string(req.param_id.data(), "heading_Ki"))
  {
    res.value.real = headingPID.Ki;
    res.success = true;
  }
  if (compare_string(req.param_id.data(), "heading_Kd"))
  {
    res.value.real = headingPID.Kd;
    res.success = true;
  }

  return res.success;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "controller");
  Controller control;
  ros::spin();
}
