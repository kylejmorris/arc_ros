/**
* CLASS: WifiHandler
* DATE: 20/03/17
* AUTHOR: Kyle Morris
* DESCRIPTION: All robots in ARC communicate over the global /arc/wifi namespace.
* Each robot will have it’s own wifi_handler node, that handles all incoming
* and outgoing communication signals.
* In a simulation this node just filters out messages sent from robots
* that are too far away. On a real robot, this node may be connected
* with actual wireless hardware for signaling.
*/
#ifndef ARC_CORE_WIFIHANDLER_H
#define ARC_CORE_WIFIHANDLER_H
#include "ros/ros.h"
#include "arc_msgs/WirelessAnnouncement.h"
#include "arc_msgs/WirelessRequest.h"
#include "arc_msgs/WirelessResponse.h"
#include "nav_msgs/Odometry.h"

class WifiHandler {
private:

    /**
     * Get updates on our recent location so we can calculate how far away signals are from us.
     */
    ros::Subscriber base_pose_sub;

    /**
     * Subscribing to incoming signals.
     */
    ros::Subscriber announcements_sub;
    ros::Subscriber requests_sub;
    ros::Subscriber responses_sub;

    /**
     * Subscribing to signals from our robot to the outside world.
    */
    ros::Subscriber outgoing_announcements_sub;
    ros::Subscriber outgoing_requests_sub;
    ros::Subscriber outgoing_responses_sub;

    /**
     * Publishing messages signals from this robot to the world
     */
    ros::Publisher announcements_pub;
    ros::Publisher requests_pub;
    ros::Publisher responses_pub;

    /**
     * Publishing broadcasts this robot received, onto it's internal communication lines.
     * this allows communication manager + other nodes to focus on broadcasts only pertaining to this robot.
     */
    ros::Publisher incoming_announcements_pub;
    ros::Publisher incoming_requests_pub;
    ros::Publisher incoming_responses_pub;

    ros::NodeHandle local_handle;
    ros::NodeHandle global_handle;

private:
    const int DEFAULT_MAX_SIGNAL_RANGE=10;

     /**
     * Maximum distance (in meters) we can detect a signal from.
     */
    int max_signal_range;

    /**
     * recent pose we found the robot in. Used for calculating distance away from signals.
     */
    nav_msgs::Odometry recent_position;

    /**
     * Check if a given signal is within our range.
     * @param x,y: The x and y location that the signal is coming from.
     * @return True if within range. False otherwise
     */
    bool isSignalWithinRange(int x, int y);
public:
    WifiHandler();

    /**
     * Main loop
     */
    void run();

    /**
    * Callbacks for all of the signals.
    */
    void process_announcements_cb(arc_msgs::WirelessAnnouncement announcement);
    void process_requests_cb(arc_msgs::WirelessRequest request);
    void process_responses_cb(arc_msgs::WirelessResponse response);

    void process_outgoing_announcements_cb(arc_msgs::WirelessAnnouncement announcement);
    void process_outgoing_requests_cb(arc_msgs::WirelessRequest request);
    void process_outgoing_responses_cb(arc_msgs::WirelessResponse response);

    void process_base_pose_cb(nav_msgs::Odometry odom);
};

#endif //ARC_CORE_WIFIHANDLER_H
