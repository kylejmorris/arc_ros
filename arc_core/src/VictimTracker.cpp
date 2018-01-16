#include "VictimTracker.h"

VictimTracker::VictimTracker() {
    ros::NodeHandle global_handle;
    ros::NodeHandle local_handle("victim_tracker");
    this->global = global_handle;
    this->local = local_handle;

    this->incoming_victims_sub = global_handle.subscribe("detect_victim_ps/found_victims", MAX_QUEUE_SIZE, &VictimTracker::incoming_victims_cb, this);
}

void VictimTracker::start() {
    ros::Rate r(10);

    while(ros::ok()) {

        r.sleep();
        ros::spinOnce();
    }
}

void VictimTracker::incoming_victims_cb(const arc_msgs::DetectedVictims &msg) {
    ROS_INFO("Received victim info.");
}

int main(int argc, char **argv)  {
    ros::init(argc, argv, "victim_tracker");

    VictimTracker tracker;
    tracker.start();

    return 0;
}
