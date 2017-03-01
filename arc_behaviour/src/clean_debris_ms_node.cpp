#include "ros/ros.h"
#include "CleanDebrisMS.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "clean_debris_ms_node");
    arc_behaviour::CleanDebrisMS clean_debris;
    clean_debris.run();
    return 0;
}