#include "ros/ros.h"
#include "../include/CommunicationManager.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "communication_manager_node");

    CommunicationManager manager;
    manager.run();

    return 0;
}