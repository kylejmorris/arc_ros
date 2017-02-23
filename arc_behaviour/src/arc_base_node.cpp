#include "ros/ros.h"
#include "../include/ArcBase.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "arc_base");

    ros::NodeHandle nh("arc_base");

    arc_behaviour::ArcBase arc_base(&nh);

    //activating schemas
    arc_base.setup();
    arc_base.run();

    return 0;
}
