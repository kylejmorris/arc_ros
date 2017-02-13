#include "ArcBase.h"

using namespace arc_behaviour;

ArcBase::ArcBase(ros::NodeHandle *nh) {
    this->nh = nh;
}

bool ArcBase::setupSchemas() {
    ROS_INFO("Setting up motor schemas.");
    return true;
}

void ArcBase::disableAll() {

}

void ArcBase::setup() {
    ROS_INFO("Initializing arc_base...");
    this->setupSchemas();
}


bool ArcBase::toggleSchema(std::string type) {
    return false;
}

geometry_msgs::Twist ArcBase::getActionVector() {
    return geometry_msgs::Twist();
}
