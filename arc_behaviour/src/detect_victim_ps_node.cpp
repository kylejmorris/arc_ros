#include "ros/ros.h"
#include "../include/DetectVictimPS.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "detect_robot_ps_node");
    arc_behaviour::DetectVictimPS detector;

    detector.run();

    return 0;
}