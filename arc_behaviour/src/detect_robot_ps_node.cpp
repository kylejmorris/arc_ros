#include "ros/ros.h"
#include "../include/DetectRobotPS.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "detect_robot_ps_node");
    arc_behaviour::DetectRobotPS detector;

    detector.run();

    return 0;
}