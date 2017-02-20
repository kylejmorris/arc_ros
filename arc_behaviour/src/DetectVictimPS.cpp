#include <std_msgs/Bool.h>
#include "../include/DetectVictimPS.h"
#include "arc_msgs/DetectedVictims.h"

#define DEFAULT_MAX_RANGE 10
#define MAX_QUEUE_SIZE 1000
#define PUBLISH_RATE 10
using namespace arc_behaviour;

void DetectVictimPS::process_detect_victim_cb(const marker_msgs::MarkerDetection marker_info) {
    this->detected_markers = marker_info;
}

DetectVictimPS::DetectVictimPS() {
    ros::NodeHandle nh;
    this->global_handle = nh;
    ros::NodeHandle local_handle("detect_victim_ps");
    this->local_handle = local_handle;
    ROS_INFO("Setting up victim detection perceptual schema.");
    this->found_victims_pub = local_handle.advertise<arc_msgs::DetectedVictims>("found_victims", MAX_QUEUE_SIZE);
    this->victim_detector_sub =  this->getNodeHandle().subscribe("victim_detector", MAX_QUEUE_SIZE, &DetectVictimPS::process_detect_victim_cb, this); //TODO: Check if publisher exists
    ROS_INFO("detect_marker_ps subscribed to marker_detector.");

    int max_range;
    local_handle.getParam("max_range", max_range);
    //TODO: Stage has it's own max_range. Make stage use the max_range from parameter server, so they are both in sync. This way in stage the marker_detection will use the same max range as the behaviour module does.
    this->setMaxRange(max_range);
}

void DetectVictimPS::ProcessStageFiducial() {
    marker_msgs::MarkerDetection pruned;
    arc_msgs::DetectedVictims victims_found;
    //iterate through each marker
    for(std::vector<marker_msgs::Marker>::iterator it = this->detected_markers.markers.begin(); it != this->detected_markers.markers.end(); ++it) {
        //calculate distance to marker
        marker_msgs::Marker curr = *it;
        arc_msgs::DetectedVictim curr_victim;
        double distance_away = sqrt(pow(curr.pose.position.x, 2) + pow(curr.pose.position.y, 2));

        //is range within valid parameters?
        if (distance_away <= this->max_range) {
            curr_victim.pose = curr.pose;
            if (curr.ids.size()>0) {
                curr_victim.status = curr.ids.at(0); //use id of victim to determine status.
            } else {
                curr_victim.status = 0; //unknown //TODO: remove magic number
            }

            victims_found.victims.push_back(curr_victim);
        }
    }

    assert(pruned.markers.size() <= ( this->detected_markers.markers.size()));
//    assert(victims_found.victims.size() <= ( this->detected_markers.markers.size()));
    this->found_victims = victims_found;

    //now simplify marker info into victim messages
    ROS_DEBUG("Found %d markers within range.", pruned.markers.size());
}

void DetectVictimPS::setMaxRange(int new_range) {
    if(new_range<=0) {
        ROS_WARN("Unable to set parameter: max_range. Value must be > 0. Using default.");
        this->max_range = DEFAULT_MAX_RANGE;
    } else {
        this->max_range = new_range;
    }
}

ros::NodeHandle DetectVictimPS::getNodeHandle() {
    return this->global_handle;
}

void DetectVictimPS::run() {
    ros::Rate r(PUBLISH_RATE);

    while(ros::ok()) {
        ProcessStageFiducial(); //pruning to markers within range.

        this->found_victims_pub.publish(this->found_victims);

        ros::spinOnce();
        r.sleep();
    }
}
