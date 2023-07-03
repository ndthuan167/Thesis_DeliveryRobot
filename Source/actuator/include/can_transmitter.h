#ifndef GPSINSDVL_RECEIVER_H
#define GPSINSDVL_RECEIVER_H

#include <ros/ros.h>

#include <utils/CommandLong.h>
#include <utils/DiffVel.h>

#include <QObject>
#include <QThread>
#include <QCanBus>
#include <QCoreApplication>
#include <QtDebug>

using namespace utils;
using namespace std;

class CANTransmitterNode : public QThread
{
  Q_OBJECT

public:
  CANTransmitterNode();
  ~CANTransmitterNode();

  bool enabled;
  string port;

  ros::Subscriber subDiffVel;
  ros::ServiceServer resSetArming;

  bool motorLocked = true;
  float vel_left;
  float vel_right;

  void run();

  void onDiffVelCallBack(const DiffVel::ConstPtr& msg);
  bool onSetArmingCallBack(CommandLongRequest& req, CommandLongResponse& res);

signals:
  void frameReceived(QByteArray payload);
};

class CANTransmitter : public QObject
{
  Q_OBJECT

public:
  CANTransmitter();
  ~CANTransmitter();

  CANTransmitterNode node;
  QCanBusDevice* device;

public slots:
  void transmitFrame(const QByteArray& payload);
};

#endif // GPSINSDVL_RECEIVER_H
