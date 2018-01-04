#include <std_msgs/Bool.h>
#include "AdvancedDetectVictimPS.h"
#include "arc_msgs/DetectedVictims.h"

#define DEFAULT_MAX_RANGE 10
#define MAX_QUEUE_SIZE 1000
#define PUBLISH_RATE 10
using namespace arc_behaviour;

void AdvancedDetectVictimPS::process_detect_victim_cb(const marker_msgs::MarkerDetection marker_info) {
    this->detected_markers = marker_info;
}

AdvancedDetectVictimPS::AdvancedDetectVictimPS() {
    ros::NodeHandle nh;
    this->global_handle = nh;
    ros::NodeHandle local_handle("detect_victim_ps");
    this->local_handle = local_handle;
    ROS_INFO("Setting up victim detection perceptual schema.");
    this->found_victims_pub = local_handle.advertise<arc_msgs::DetectedVictims>("found_victims", MAX_QUEUE_SIZE);
    this->victim_detector_sub =  this->getNodeHandle().subscribe("victim_detector", MAX_QUEUE_SIZE, &AdvancedDetectVictimPS::process_detect_victim_cb, this); //TODO: Check if publisher exists
    ROS_INFO("detect_marker_ps subscribed to marker_detector.");

    int max_range;
    local_handle.getParam("max_range", max_range);
    local_handle.getParam("max_victim_detect_range", maxVictimIdentificationDistance);
    //TODO: Stage has it's own sensingRange. Make stage use the sensingRange from parameter server, so they are both in sync. This way in stage the marker_detection will use the same max range as the behaviour module does.
    this->setMaxRange(max_range);
}

void AdvancedDetectVictimPS::ProcessStageFiducial() {
    marker_msgs::MarkerDetection pruned;
    arc_msgs::DetectedVictims victims_found;

    //iterate through each marker
    for(std::vector<marker_msgs::Marker>::iterator it = this->detected_markers.markers.begin(); it != this->detected_markers.markers.end(); ++it) {
        //calculate distance to marker
        marker_msgs::Marker curr = *it;
        arc_msgs::DetectedVictim curr_victim;
        double distance_away = sqrt(pow(curr.pose.position.x, 2) + pow(curr.pose.position.y, 2));

        if (distance_away <= this->sensingRange) {
            curr_victim.pose = curr.pose;
            if (curr.ids.size()>0) {
                curr_victim.status = curr.ids.at(0); //use id of victim to determine status.
            } else {
                curr_victim.status = 0; //unknown //TODO: remove magic number
            }

            //if we are out of range of being able to see the victim correctly, forcefully label it as potential victim.
            if((curr_victim.status==3||curr_victim.status==2) && (distance_away >= maxVictimIdentificationDistance)) {
                curr_victim.status = 1;
            }

            victims_found.victims.push_back(curr_victim);
        }
    }

    assert(pruned.markers.size() <= ( this->detected_markers.markers.size()));

    //assert(victims_found.victims.size() <= ( this->detected_markers.markers.size()));
    this->found_victims = victims_found;

    //now simplify marker info into victim messages
    ROS_DEBUG("Found %d markers within range.", pruned.markers.size());
}

void AdvancedDetectVictimPS::setMaxRange(int new_range) {
    if(new_range<=0) {
        ROS_WARN("Unable to set parameter: sensingRange. Value must be > 0. Using default.");
        this->sensingRange = DEFAULT_MAX_RANGE;
    } else {
        this->sensingRange = new_range;
    }
}

ros::NodeHandle AdvancedDetectVictimPS::getNodeHandle() {
    return this->global_handle;
}

void AdvancedDetectVictimPS::run() {
    ros::Rate r(PUBLISH_RATE);

    while(ros::ok()) {
        ProcessStageFiducial(); //pruning to markers within range.

        this->found_victims_pub.publish(this->found_victims);

        ros::spinOnce();
        r.sleep();
    }
}
