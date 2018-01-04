#include <rosconsole/macros_generated.h>
#include <ros/ros.h>
#include "../include/DetectDebrisPS.h"
#include "arc_msgs/DetectedDebris.h"
#include "arc_msgs/Debris.h"
#include "marker_msgs/MarkerDetection.h"
#include "std_msgs/Bool.h"

#define DEFAULT_MAX_RANGE 10
#define MAX_QUEUE_SIZE 1000
#define PUBLISH_RATE 10

//TODO: Test this overall schema. Setup a specific world and test to make sure it works properly on that world.
using namespace arc_behaviour;

DetectDebrisPS::DetectDebrisPS() {
    ros::NodeHandle nh;
    this->global_handle = &nh;
    ros::NodeHandle local_handle("detect_debris_ps");
    this->local_handle = local_handle;
    ROS_INFO("Setting up debris detection perceptual schema.");
    this->debris_location_publisher = local_handle.advertise<arc_msgs::DetectedDebris>("debris_locations", MAX_QUEUE_SIZE);
    this->debris_detector_sub =  this->getNodeHandle()->subscribe("debris_detector", MAX_QUEUE_SIZE, &DetectDebrisPS::process_detect_debris_cb, this); //TODO: Check if publisher exists
    ROS_INFO("detect_debris_ps subscribed to debris_detector.");

    int max_range;
    local_handle.getParam("max_range", max_range);
    this->setMaxRange(max_range);
}

void DetectDebrisPS::process_detect_debris_cb(const marker_msgs::MarkerDetection marker_info) {
    ROS_DEBUG("process detect marker cb called.");
    this->found_markers = marker_info;
}

ros::NodeHandle *DetectDebrisPS::getNodeHandle() {
    return this->global_handle;
}

void DetectDebrisPS::setMaxRange(int new_range) {
    if(new_range<=0) {
        ROS_WARN("Unable to set parameter: sensingRange. Value must be > 0. Using default.");
        this->max_range = DEFAULT_MAX_RANGE;
    } else {
        this->max_range = new_range;
    }
}

void DetectDebrisPS::run() {
    ros::Rate r(PUBLISH_RATE);

    while(ros::ok()) {
        arc_msgs::DetectedDebris nearby;
        ProcessStageFiducial(); //pruning to Debriss within range.
        nearby = this->getDebris();

        this->debris_location_publisher.publish(nearby);

        ros::spinOnce();
        r.sleep();
    }
}

arc_msgs::DetectedDebris DetectDebrisPS::getDebris() {
    return this->debris_list;
}

void DetectDebrisPS::ProcessStageFiducial() {
    arc_msgs::DetectedDebris pruned;

    //iterate through each marker
    for(std::vector<marker_msgs::Marker>::iterator it = this->found_markers.markers.begin(); it != this->found_markers.markers.end(); ++it) {
        //calculate distance to marker
        arc_msgs::Debris curr_debris;
        marker_msgs::Marker curr = *it;
        double distance_away = sqrt(pow(curr.pose.position.x,2) + pow(curr.pose.position.y, 2));
//is range within valid parameters?
        if(distance_away <= this->max_range) {
            curr_debris.pose = curr.pose;

            if(it->ids.size()>0) {
                curr_debris.debris_id = it->ids.at(0);
                pruned.debris.insert(pruned.debris.begin(),curr_debris);
            }

        }
    }

    assert(pruned.debris.size() <= ( this->found_markers.markers.size()));
    this->debris_list = pruned;
    ROS_DEBUG("Found %d markers within range.", pruned.debris.size());
}
