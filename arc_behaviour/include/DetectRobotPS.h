/**
* CLASS: DetectRobotPS
* DATE: 14/02/17
* AUTHOR: Kyle Morris
* DESCRIPTION: Determine if robots are nearby.
*/

#ifndef ARC_BEHAVIOUR_DETECTROBOTPS_H
#define ARC_BEHAVIOUR_DETECTROBOTPS_H

#include "marker_msgs/MarkerDetection.h"
#include "ros/ros.h"
#include "arc_msgs/DetectedRobot.h"
#include "arc_msgs/DetectedRobots.h"

namespace arc_behaviour {

class DetectRobotPS {
public:
    /*
     * Collection of the markers found in given perceptual instance.
     */
    marker_msgs::MarkerDetection detected_markers;

    /**
     * List of actual robots found after done pruning markers
     */
    arc_msgs::DetectedRobots found_robots;
    /**
     * Maximum range that we can detect markers from robots position.
     */
    int max_range;

    /**
     * Publish marker information, ie a simple boolean if markers are nearby.
     */
    ros::Publisher found_robots_pub;
    ros::Subscriber robot_detector_sub;

    //for public interface with ros
    ros::NodeHandle global_handle; //the handler for this node.

    //for private node material
    ros::NodeHandle local_handle;

    //callbacks

    //update us with most recent stage information
    void process_detect_robot_cb(const marker_msgs::MarkerDetection marker_info);

public:
    /**
     * Setup the detector with all of the ros topic publishers.
     * uses default global/local node handles
     */
    DetectRobotPS();

    /**
     * Check output from stage and prune markers to ones only within our maximum range.
     */
    void ProcessStageFiducial();

    void setMaxRange(int new_range);

    ros::NodeHandle getNodeHandle();

    /**
     * Main loop. Publishes robot status information.
     */
    void run();
};
}


#endif //ARC_BEHAVIOUR_DETECTROBOTPS_H
//TODO: Change stage to not publish marker information, ie for every single marker on a topic. Just let robot detect it in map, and publish info when it finds them. We have to mod stage to move markers using a service, not by having cmd_vel publishing nonsense. ROS can't handle 100's of markers all publishing odom info, they aren't robots.