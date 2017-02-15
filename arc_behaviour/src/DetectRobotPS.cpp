#include <std_msgs/Bool.h>
#include "DetectRobotPS.h"
#include "arc_msgs/DetectedRobots.h"
#include "arc_msgs/DetectedRobot.h"

#define DEFAULT_MAX_RANGE 10
#define MAX_QUEUE_SIZE 1000
#define PUBLISH_RATE 10
using namespace arc_behaviour;

void arc_behaviour::DetectRobotPS::process_detect_robot_cb(const marker_msgs::MarkerDetection marker_info) {
    this->detected_markers = marker_info;
}

DetectRobotPS::DetectRobotPS() {
    ros::NodeHandle nh;
    this->global_handle = nh;
    ros::NodeHandle local_handle("detect_robot_ps");
    this->local_handle = local_handle;
    ROS_INFO("Setting up Robot detection perceptual schema.");
    this->found_robots_pub = local_handle.advertise<arc_msgs::DetectedRobots>("found_robots", MAX_QUEUE_SIZE);
    this->robot_detector_sub =  this->getNodeHandle().subscribe("robot_detector", MAX_QUEUE_SIZE, &DetectRobotPS::process_detect_robot_cb, this); //TODO: Check if publisher exists
    ROS_INFO("detect_marker_ps subscribed to marker_detector.");

    int max_range;
    local_handle.getParam("max_range", max_range);
    //TODO: Stage has it's own max_range. Make stage use the max_range from parameter server, so they are both in sync. This way in stage the marker_detection will use the same max range as the behaviour module does.
    this->setMaxRange(max_range);
}

void DetectRobotPS::ProcessStageFiducial() {
    marker_msgs::MarkerDetection pruned;
    this->found_robots
    arc_msgs::DetectedRobots robots_found;

    //iterate through each marker
    for(std::vector<marker_msgs::Marker>::iterator it = this->detected_markers.markers.begin(); it != this->detected_markers.markers.end(); ++it) {
        //calculate distance to marker
        marker_msgs::Marker curr = *it;
        arc_msgs::DetectedRobot curr_robot;
        double distance_away = sqrt(pow(curr.pose.position.x,2) + pow(curr.pose.position.y, 2));

        //is range within valid parameters?
        if(distance_away <= this->max_range) {
            curr_robot.robot_id = curr.ids.at(0);
            curr_robot.pose = curr.pose;

            robots_found.robots.insert(robots_found.robots.begin(),curr_robot);
        }
    }

    assert(pruned.markers.size() <= ( this->detected_markers.markers.size()));
    assert(robots_found.robots.size() <= ( this->detected_markers.markers.size()));
    this->found_robots = robots_found;

    //now simplify marker info into robot messages
    ROS_DEBUG("Found %d markers within range.", pruned.markers.size());
}

void DetectRobotPS::setMaxRange(int new_range) {
    if(new_range<=0) {
        ROS_WARN("Unable to set parameter: max_range. Value must be > 0. Using default.");
        this->max_range = DEFAULT_MAX_RANGE;
    } else {
        this->max_range = new_range;
    }
}

ros::NodeHandle DetectRobotPS::getNodeHandle() {
    return this->global_handle;
}

void DetectRobotPS::run() {
    ros::Rate r(PUBLISH_RATE);

    while(ros::ok()) {
        ProcessStageFiducial(); //pruning to markers within range.

        this->found_robots_pub.publish(this->found_robots);

        ros::spinOnce();
        r.sleep();
    }
}
