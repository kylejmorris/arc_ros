#include <rosconsole/macros_generated.h>
#include <ros/ros.h>
#include "../include/DetectMarkerPS.h"
#include "marker_msgs/MarkerDetection.h"
#include "arc_msgs/DetectedMarkers.h"
#include "std_msgs/Bool.h"

#define DEFAULT_MAX_RANGE 10
#define MAX_QUEUE_SIZE 1000
#define PUBLISH_RATE 10

//TODO: Test this overall schema. Setup a specific world and test to make sure it works properly on that world.
using namespace arc_behaviour;

DetectMarkerPS::DetectMarkerPS() {
    ros::NodeHandle nh;
    this->global_handle = &nh;
    ros::NodeHandle local_handle("detect_marker_ps");
    this->local_handle = local_handle; //TODO: be consistent with pointers/nonpointer
    ROS_INFO("Setting up Marker detection perceptual schema.");
    this->marker_status_publisher = local_handle.advertise<arc_msgs::DetectedMarkers>("marker_status", MAX_QUEUE_SIZE);
    this->marker_detector_sub =  this->getNodeHandle()->subscribe("marker_detector", MAX_QUEUE_SIZE, &DetectMarkerPS::process_detect_marker_cb, this); //TODO: Check if publisher exists
    ROS_INFO("detect_marker_ps subscribed to marker_detector.");

    int max_range;
    local_handle.getParam("max_range", max_range);
    //TODO: Stage has it's own sensingRange. Make stage use the sensingRange from parameter server, so they are both in sync. This way in stage the marker_detection will use the same max range as the behaviour module does.
    this->setMaxRange(max_range);
}

void DetectMarkerPS::process_detect_marker_cb(const marker_msgs::MarkerDetection &marker_info)
{
    ROS_DEBUG("process detect marker cb called.");
    this->found_markers = marker_info;
}

ros::NodeHandle *DetectMarkerPS::getNodeHandle() {
    return this->global_handle;
}

void DetectMarkerPS::setMaxRange(int new_range) {
    if(new_range<=0) {
        ROS_WARN("Unable to set parameter: sensingRange. Value must be > 0. Using default.");
        this->max_range = DEFAULT_MAX_RANGE;
    } else {
        this->max_range = new_range;
    }
}

void DetectMarkerPS::run() {
    ros::Rate r(PUBLISH_RATE);

    while(ros::ok()) {

        ProcessStageFiducial(); //pruning to markers within range.
        arc_msgs::DetectedMarkers found;
        for(std::vector<marker_msgs::Marker>::iterator it = this->found_markers.markers.begin(); it != this->found_markers.markers.end(); ++it) {
            arc_msgs::DetectedMarker marker_found;

            //make sure marker has an id
            if(it->ids.size()>0) {
                marker_found.marker_id = it->ids.at(0);
            }

            marker_found.pose = it->pose;
            found.markers.push_back(marker_found);
        }

        this->marker_status_publisher.publish(found);
        ros::spinOnce();
        r.sleep();
    }
}

void DetectMarkerPS::ProcessStageFiducial() {
    marker_msgs::MarkerDetection pruned;

    //iterate through each marker
    for(std::vector<marker_msgs::Marker>::iterator it = this->found_markers.markers.begin(); it != this->found_markers.markers.end(); ++it) {
        //calculate distance to marker
        marker_msgs::Marker curr = *it;
        double distance_away = sqrt(pow(curr.pose.position.x,2) + pow(curr.pose.position.y, 2));
//is range within valid parameters?
        if(distance_away <= this->max_range) {
            pruned.markers.insert(pruned.markers.begin(),curr);
        }
    }

    assert(pruned.markers.size() <= ( this->found_markers.markers.size()));
    this->found_markers = pruned;
    ROS_DEBUG("Found %d markers within range.", pruned.markers.size());
}

bool DetectMarkerPS::areMarkersNearby() {
    return this->found_markers.markers.size()>0 ? true : false;
}
