#include "WifiHandler.h"

#define DEFAULT_ROS_RATE 10
#define MAX_QUEUE_SIZE 1000

WifiHandler::WifiHandler() {
    ros::NodeHandle global_handle;
    ros::NodeHandle local_handle("wifi_handler");
    this->global_handle = global_handle;
    this->local_handle = local_handle;

    //reading parameters
    ROS_INFO("Setting up Wifi Handler");
    local_handle.param("max_signal_range", this->max_signal_range, this->DEFAULT_MAX_SIGNAL_RANGE);
    ROS_INFO("set parameter: max_signal_range=%d", this->max_signal_range);

    this->base_pose_sub = global_handle.subscribe("base_pose_ground_truth", MAX_QUEUE_SIZE, &WifiHandler::process_base_pose_cb, this);
    //setting up subscribers
    this->announcements_sub = global_handle.subscribe("/arc/wifi/announcements", MAX_QUEUE_SIZE, &WifiHandler::process_announcements_cb, this);
    this->requests_sub = global_handle.subscribe("/arc/wifi/requests", MAX_QUEUE_SIZE, &WifiHandler::process_requests_cb, this);
    this->responses_sub = global_handle.subscribe("/arc/wifi/responses", MAX_QUEUE_SIZE, &WifiHandler::process_responses_cb, this);

    this->outgoing_announcements_sub = local_handle.subscribe("outgoing_announcements", MAX_QUEUE_SIZE, &WifiHandler::process_outgoing_announcements_cb, this);
    this->outgoing_requests_sub = local_handle.subscribe("outgoing_requests", MAX_QUEUE_SIZE, &WifiHandler::process_outgoing_requests_cb, this);
    this->outgoing_responses_sub = local_handle.subscribe("outgoing_responses", MAX_QUEUE_SIZE, &WifiHandler::process_outgoing_responses_cb, this);

    //setting up publishers
    this->announcements_pub = global_handle.advertise<arc_msgs::WirelessAnnouncement>("/arc/wifi/announcements", MAX_QUEUE_SIZE);
    this->requests_pub = global_handle.advertise<arc_msgs::WirelessRequest>("/arc/wifi/requests", MAX_QUEUE_SIZE);
    this->responses_pub = global_handle.advertise<arc_msgs::WirelessResponse>("/arc/wifi/responses", MAX_QUEUE_SIZE);

    this->incoming_announcements_pub = local_handle.advertise<arc_msgs::WirelessAnnouncement>("incoming_announcements", MAX_QUEUE_SIZE);
    this->incoming_requests_pub = local_handle.advertise<arc_msgs::WirelessRequest>("incoming_requests", MAX_QUEUE_SIZE);
    this->incoming_responses_pub = local_handle.advertise<arc_msgs::WirelessResponse>("incoming_responses", MAX_QUEUE_SIZE);

    ROS_ASSERT(this->max_signal_range>=0);
    ROS_ASSERT(this->recent_position.pose.pose.position.x>=0);
    ROS_ASSERT(this->recent_position.pose.pose.position.y>=0);
}

void WifiHandler::process_outgoing_announcements_cb(arc_msgs::WirelessAnnouncement announcement) {
    ROS_DEBUG("Received announcement from inside world");
    this->announcements_pub.publish(announcement);
}


void WifiHandler::process_outgoing_requests_cb(arc_msgs::WirelessRequest request) {
    ROS_DEBUG("Received request from inside world");
    this->requests_pub.publish(request);
}

void WifiHandler::process_outgoing_responses_cb(arc_msgs::WirelessResponse response) {
    ROS_DEBUG("Received response from inside world");
    this->responses_pub.publish(response);
}

void WifiHandler::run() {
    ros::Rate rate(DEFAULT_ROS_RATE);

    while(ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }
}

void WifiHandler::process_announcements_cb(arc_msgs::WirelessAnnouncement announcement) {
    ROS_DEBUG("Received announcement from outside world");
    int signal_x = announcement.sender_location.position.x;
    int signal_y = announcement.sender_location.position.y;

    if(this->isSignalWithinRange(signal_x, signal_y)) {
        ROS_INFO("Found an announcement signal from (%d, %d)",signal_x, signal_y);
        this->incoming_announcements_pub.publish(announcement);
    }
}

void WifiHandler::process_base_pose_cb(nav_msgs::Odometry odom) {
    this->recent_position = odom;
}

void WifiHandler::process_requests_cb(arc_msgs::WirelessRequest request) {
    ROS_DEBUG("Received request from outside world");
    int signal_x = request.sender_location.position.x;
    int signal_y = request.sender_location.position.y;

    if(this->isSignalWithinRange(signal_x, signal_y)) {
        ROS_INFO("Found a request signal from (%d, %d)",signal_x, signal_y);
        this->incoming_requests_pub.publish(request);
    }
}

void WifiHandler::process_responses_cb(arc_msgs::WirelessResponse response) {
    ROS_DEBUG("Received response from outside world");
    int signal_x = response.sender_location.position.x;
    int signal_y = response.sender_location.position.y;

    if(this->isSignalWithinRange(signal_x, signal_y)) {
        ROS_INFO("Found a response signal from (%d, %d)",signal_x, signal_y);
        this->incoming_responses_pub.publish(response);
    }
}

bool WifiHandler::isSignalWithinRange(int x, int y) {
    //TODO: Fix this approximation. There may be slight inaccuracy due to floating point truncation to int
    int curr_x = this->recent_position.pose.pose.position.x;
    int curr_y = this->recent_position.pose.pose.position.y;

    double distance_away = sqrt(pow(curr_x - x,2) + pow(curr_y - y,2));

    if(distance_away <= this->max_signal_range) {
        return true;
    }  else {
        return false;
    }
}
