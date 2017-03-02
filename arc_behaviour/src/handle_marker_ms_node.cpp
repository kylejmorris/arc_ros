#include "ros/ros.h"
#include "HandleMarkerMS.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "handle_marker_ms_node");

    arc_behaviour::HandleMarkerMS handle_marker;
    handle_marker.run();
    return 0;
}