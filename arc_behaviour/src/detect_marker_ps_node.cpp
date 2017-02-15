#include "ros/ros.h"
#include "DetectMarkerPS.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "detect_marker_ps_node");

    arc_behaviour::DetectMarkerPS detector;
    detector.run();

    return 0;
}