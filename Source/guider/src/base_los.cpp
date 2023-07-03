#include "base_los.h"

std::vector<BaseLOS::Point> BaseLOS::waypoints;
double BaseLOS::radius;
double BaseLOS::minDelta;
double BaseLOS::maxDelta;
double BaseLOS::beta;

BaseLOS::BaseLOS() {}

BaseLOS::~BaseLOS() {}

void BaseLOS::setupLOS() {}

void BaseLOS::resetLOS() {}

bool BaseLOS::runLOS(const double& /*odomX*/, const double& /*odomY*/) { return true; }
