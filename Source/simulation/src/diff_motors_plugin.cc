#include "diff_motors_plugin.hh"

DiffMotor::DiffMotor(DiffMotors* parent)
{
  this->plugin = parent;
  this->desiredVel.fill(0.0);
  this->currVel.fill(0.0);

  auto seed = std::chrono::system_clock::now().time_since_epoch().count();
  rndGen = std::default_random_engine(seed);
  encNoiseModel = std::normal_distribution<double>(0.0, encStdDev);
}

void DiffMotor::onVelCallBack(const DiffVel::ConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(this->plugin->mutex);

  if (keyboardEnabled)
    return;

  this->desiredVel[LEFT] = boost::algorithm::clamp(msg->left_vel, -this->maxVel, this->maxVel);
  this->desiredVel[RIGHT] = boost::algorithm::clamp(msg->right_vel, -this->maxVel, this->maxVel);
}

void DiffMotor::onKeyboardCommandCallBack(const KeyboardCommand::ConstPtr &msg)
{
  std::lock_guard<std::mutex> lock(this->plugin->mutex);

  keyboardEnabled = msg->mode;
  if (!keyboardEnabled)
    return;

  this->desiredVel[LEFT] = boost::algorithm::clamp(msg->left_vel, -this->maxVel, this->maxVel);
  this->desiredVel[RIGHT] = boost::algorithm::clamp(msg->right_vel, -this->maxVel, this->maxVel);
}

void DiffMotor::rotateMotors(common::Time /*stepTime*/)
{
  std::lock_guard<std::mutex> lock(this->plugin->mutex);

  for (size_t i = 0; i < 2; i++)
  {
    if (fabs(maxEffort - joint[i]->GetParam("fmax", 0)) > 1e-6)
      joint[i]->SetParam("fmax", 0, maxEffort);
    joint[i]->SetParam("vel", 0, desiredVel[i]);
  }
}

void DiffMotor::readEncoder()
{
  std::lock_guard<std::mutex> lock(this->plugin->mutex);

  for (unsigned i = 0; i < 2; i++)
    currVel[i] = joint[i]->GetVelocity(0) + encNoiseModel(rndGen);
}

void DiffMotors::Load(physics::ModelPtr parent, sdf::ElementPtr sdf)
{
  this->model = parent;
  this->world = this->model->GetWorld();

  sdf::ElementPtr motorSDF = sdf->GetElement("diffMotor");
  while (motorSDF)
  {
    DiffMotor diffMotor(this);

    std::string linkName[2], jointName[2];
    linkName[LEFT] = motorSDF->Get<std::string>("leftLink");
    linkName[RIGHT] = motorSDF->Get<std::string>("rightLink");
    jointName[LEFT] = motorSDF->GetElement("leftJoint")->Get<std::string>();
    jointName[RIGHT] = motorSDF->GetElement("rightJoint")->Get<std::string>();
    diffMotor.velTopic = motorSDF->Get<std::string>("velTopic");
    diffMotor.keyVelTopic = motorSDF->Get<std::string>("keyVelTopic");
    diffMotor.maxVel = motorSDF->Get<double>("maxVel");
    diffMotor.maxEffort = motorSDF->Get<double>("maxEffort");
    diffMotor.encStdDev = motorSDF->Get<double>("encStdDev");
    diffMotor.wheelRadius = motorSDF->Get<double>("wheelRadius");
    diffMotor.wheelSeparation = motorSDF->Get<double>("wheelSeparation");

    for (size_t i = 0; i < 2; i++)
    {
      diffMotor.link[i] = this->model->GetLink(linkName[i]);
      diffMotor.joint[i] = this->model->GetJoint(jointName[i]);
      diffMotor.joint[i]->SetVelocityLimit(0, diffMotor.maxVel);
      diffMotor.joint[i]->SetEffortLimit(0, diffMotor.maxEffort);
    }

    this->diffMotors.push_back(diffMotor);
    motorSDF = motorSDF->GetNextElement("diffMotor");
  }

  encTopic = sdf->Get<std::string>("encTopic");
  encPeriod = sdf->Get<double>("encPeriod");
  controlPeriod = sdf->Get<double>("controlPeriod");

  std::string nodeNamespace = sdf->Get<std::string>("robotNamespace") + "/";
  this->rosnode.reset(new ros::NodeHandle(nodeNamespace));
  pubEnc = this->rosnode->advertise<Encoder>(encTopic, 1);
  resSetArming = this->rosnode->advertiseService("command/set_arming", &DiffMotors::onSetArmingCallBack, this);
  loopEnc = this->rosnode->createTimer(ros::Duration(encPeriod), &DiffMotors::onEncLoop, this);

  for (size_t i = 0; i < this->diffMotors.size(); ++i)
  {
    this->diffMotors[i].subVel =
        this->rosnode->subscribe(this->diffMotors[i].velTopic, 10, &DiffMotor::onVelCallBack, &this->diffMotors[i]);
    this->diffMotors[i].subKeyCmd =
        this->rosnode->subscribe(this->diffMotors[i].keyVelTopic, 10, &DiffMotor::onKeyboardCommandCallBack, &this->diffMotors[i]);
  }

  this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&DiffMotors::Update, this));

  lastEncUpdateTime = ros::Time::now();
  lastVelUpdateTime = this->world->SimTime();
}

void DiffMotors::Update()
{
  common::Time now = this->world->SimTime();
  common::Time stepTime = now - lastVelUpdateTime;

  if (stepTime.Double() < controlPeriod)
  {
  }
  else
  {
    for (auto it = diffMotors.begin(); it != diffMotors.end(); it++)
    {
      if (motorLocked)
      {
        it->desiredVel[0] = 0;
        it->desiredVel[1] = 0;
      }
      it->rotateMotors(stepTime);
    }
    lastVelUpdateTime += stepTime;
  }
}

void DiffMotors::onEncLoop(const ros::TimerEvent& event)
{
  Encoder encMsg;
  encMsg.header.stamp = event.current_real;
  auto ptr = this->diffMotors[1];
  ptr.readEncoder();
  encMsg.left_twist = ptr.currVel[LEFT];
  encMsg.right_twist = ptr.currVel[RIGHT];
  pubEnc.publish(encMsg);
}

bool DiffMotors::onSetArmingCallBack(CommandLongRequest& req, CommandLongResponse& res)
{
  if (req.param1 == 1.0f)
    motorLocked = false;
  else if (req.param1 == 0.0f)
    motorLocked = true;
  res.result = 0; // MAV_RESULT_ACCEPTED

  return true;
}

GZ_REGISTER_MODEL_PLUGIN(DiffMotors)
