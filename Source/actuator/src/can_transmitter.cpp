#include "can_transmitter.h"

CANTransmitterNode::CANTransmitterNode()
{
  ros::NodeHandle private_nh("~");
  private_nh.getParam("enabled", enabled);
  private_nh.getParam("port", port);

  ros::NodeHandle nh;
  subDiffVel = nh.subscribe("diff_motor/vel", 10, &CANTransmitterNode::onDiffVelCallBack, this);
  resSetArming = nh.advertiseService("command/set_arming", &CANTransmitterNode::onSetArmingCallBack, this);
}

CANTransmitterNode::~CANTransmitterNode()
{
  if (ros::isStarted())
  {
    ros::shutdown();
    ros::waitForShutdown();
  }
}

void CANTransmitterNode::run()
{
  ros::AsyncSpinner spinner(0);
  spinner.start();
  ros::waitForShutdown();
}

void CANTransmitterNode::onDiffVelCallBack(const DiffVel::ConstPtr& msg)
{
  QByteArray payload;
  payload.resize(8);
  if (!motorLocked)
  {
    vel_left = float(msg->left_vel);
    vel_right = float(msg->right_vel);
  }
  else
  {
    vel_left = 0;
    vel_right = 0;
  }

  memcpy(payload.data(), &vel_left, 4);
  memcpy(payload.data() + 4, &vel_right, 4);
  emit frameReceived(payload);
}

bool CANTransmitterNode::onSetArmingCallBack(utils::CommandLongRequest& req, utils::CommandLongResponse& res)
{
  if (req.param1 == 1.0f)
  {
    motorLocked = false;
    ROS_INFO("Motor unlocked.");
  }
  else if (req.param1 == 0.0f)
  {
    motorLocked = true;
    ROS_INFO("Motor locked.");
  }
  res.result = 0; // MAV_RESULT_ACCEPTED

  return true;
}

CANTransmitter::CANTransmitter()
{
  if (node.enabled)
  {
    device = QCanBus::instance()->createDevice("socketcan", node.port.data());
    device->connectDevice();
    connect(&node, &CANTransmitterNode::frameReceived, this, &CANTransmitter::transmitFrame);
  }

  node.start();
}

CANTransmitter::~CANTransmitter() { device->disconnectDevice(); }

void CANTransmitter::transmitFrame(const QByteArray& payload)
{
  QCanBusFrame frame(0x111, payload);
  device->writeFrame(frame);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "can_transmitter");
  QCoreApplication a(argc, argv);
  CANTransmitter transmitter;
  return a.exec();
}
