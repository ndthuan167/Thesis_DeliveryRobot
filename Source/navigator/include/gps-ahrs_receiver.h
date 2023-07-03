#ifndef GPSAHRS_RECEIVER_H
#define GPSAHRS_RECEIVER_H

#include <ros/ros.h>
#include <utils/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <iostream>
#include <QCoreApplication>
#include <QDebug>
#include <QObject>
#include <QSerialPort>
#include <QThread>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_broadcaster.h>

#include <boost/algorithm/string.hpp>

#include "utils/geo.h"
#include <utils/mode_indoor.h>
#include <utils/heading_plot.h>

#define BUFFER_SIZE 57

using namespace geometry_msgs;
using namespace utils;
using namespace std;
const unsigned char UBX_HEADER[] = {0xB5,0x62};
class GPSAHRSReceiverNode : public QThread
{
  Q_OBJECT

public:
  GPSAHRSReceiverNode();
  ~GPSAHRSReceiverNode();

  double ned_lat;
  double ned_lon;
  bool gps_enabled;
  bool ahrs_enabled;
  string gps_port;
  string ahrs_port;
  int gps_baudrate;
  int ahrs_baudrate;


  ros::Publisher pubOdom;
  ros::Publisher pubIMU;


  sensor_msgs::Imu imu_msg;
  Odometry odomMsg;
    

  
  void run();
};

class GPSAHRSReceiver : public QObject
{
  Q_OBJECT

public:
  GPSAHRSReceiver();
  ~GPSAHRSReceiver();
  float CurrX_indoor = 0.0 ;
  float CurrY_indoor = 0.0 ;
  float CurrHeading_indoor_x;
  float CurrHeading_indoor_y;
  float CurrHeading_indoor_z;
  float CurrHeading_indoor_w;
  float CurrOrient_indoor ;
  bool exit_indoor ;
  uint8_t mode_in = 0.0 ;
  
  GPSAHRSReceiverNode node;
  QSerialPort gpsDevice;
  QSerialPort ahrsDevice;
  ros::Subscriber sub_pose ;
  ros::Subscriber sub_hed;
  ros::Subscriber sub_mode ;
  void onCallbackPose(const PoseWithCovarianceStamped::ConstPtr& pos) ;
  void onCallbackhead(const heading_plot::ConstPtr& hed) ;
  void onCallbackmode(const mode_indoor::ConstPtr& mod);
  char ahrsBuffer[BUFFER_SIZE];
  int ahrsBytesReceived = 0;
  bool isAhrsFirst = true;

  struct NAV_POSLLH {
    unsigned char cls;
    unsigned char id;
    unsigned short len;
    unsigned long iTOW;
    long lon;
    long lat;
    char hlon;
    char hlat;
    long height;
    long hMSL;
    unsigned long hAcc;
    unsigned long vAcc;
  };
  inline double CalLat2Deg(double Lat)
  {
    static uint8_t Deg = 0;
    static double Min = 0, Result = 0;
    Deg = Lat / 100;
    Min = Lat - Deg * 100;
    Result = Deg + Min / 60;
    return Result;
  }
  inline double CalLong2Deg(double Long)
  {
    static uint16_t Deg = 0;
    static double Min = 0, Result = 0;
    Deg = Long / 100;
    Min = Long - Deg * 100;
    Result = Deg + Min / 60;
    return Result;
  }
  bool calcChecksum(unsigned char* CK, QByteArray buffer) {
  memset(CK, 0, 2);
  
  for (int i = 0; i < buffer.size(); i++) {
    CK[0] += buffer[i];
    CK[1] += CK[0];
  }
}

public slots:
  void processGPSFrame();
  void processAHRSFrame();
};

#endif // GPSAHRS_RECEIVER_H
