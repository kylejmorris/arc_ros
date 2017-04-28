#include "MinimalRecruitmentManager.h"
#include <ros/ros.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "minimal_recruitment_manager_node");
    MinimalRecruitmentManager manager;
    manager.process();

    return 0;
}