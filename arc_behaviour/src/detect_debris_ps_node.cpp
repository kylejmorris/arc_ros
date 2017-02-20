#include "ros/ros.h"
#include "../include/DetectDebrisPS.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "detect_debris_ps_node");

    arc_behaviour::DetectDebrisPS detector;
    detector.run();

    return 0;
}