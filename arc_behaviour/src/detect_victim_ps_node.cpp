#include "ros/ros.h"
#include "AdvancedDetectVictimPS.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "detect_robot_ps_node");
    arc_behaviour::AdvancedDetectVictimPS detector;

    detector.run();

    return 0;
}