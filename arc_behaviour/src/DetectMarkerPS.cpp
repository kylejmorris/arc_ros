#include <rosconsole/macros_generated.h>
#include <ros/ros.h>
#include "DetectMarkerPS.h"
#include "marker_msgs/MarkerDetection.h"

#define DEFAULT_MAX_RANGE 10

using namespace arc_behaviour;

DetectMarkerPS::DetectMarkerPS() {
    ros::NodeHandle nh("detect_marker_ps");
    this->handle = &nh;
    ROS_INFO("Setting up Marker detection perceptual schema.");

    int max_range;
    ros::param::get("/max_range", max_range);
    this->setMaxRange(max_range);
}

ros::NodeHandle *DetectMarkerPS::getNodeHandle() {
    return this->handle;
}

void DetectMarkerPS::setMaxRange(int new_range) {
    if(new_range<=0) {
        ROS_WARN("Unable to set parameter: max_range. Value must be > 0. Using default.");
        this->max_range = DEFAULT_MAX_RANGE;
    } else {
        this->max_range = new_range;
    }
}

void DetectMarkerPS::ProcessStageFiducial() {
}

bool DetectMarkerPS::areMarkersNearby() {
    return false;
}
