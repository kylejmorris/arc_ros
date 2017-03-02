#include <std_srvs/Empty.h>
#include "arc_msgs/Debris.h"
#include "arc_msgs/DetectedDebris.h"
#include "arc_msgs/MoveAlterableObject.h"
#include "marker_msgs/MarkerDetection.h"
#include "HandleMarkerMS.h"
#include "arc_msgs/DetectedMarkers.h"
#include "std_srvs/Empty.h"

#define ROS_RATE 10 //hz
#define MAX_QUEUE_SIZE 1000 //max number of messages to keep on queue before flushing

using namespace arc_behaviour;

HandleMarkerMS::HandleMarkerMS() {
    ros::NodeHandle global_handle;
    ros::NodeHandle local_handle("handle_marker_ms");
    this->global_handle = global_handle;
    this->local_handle = local_handle;
    ROS_INFO("Setting up handle marker ms");
    local_handle.param("priority", this->priority, this->DEFAULT_PRIORITY);
    local_handle.param("max_pickup_range", this->max_pickup_range, this->DEFAULT_MAX_PICKUP_RANGE);
    this->toggle_server = this->local_handle.advertiseService("toggle", &HandleMarkerMS::toggle_cb, this);
    this->drop_marker_server = this->local_handle.advertiseService("drop_marker", &HandleMarkerMS::drop_marker_cb, this);
    this->pickup_marker_server = this->local_handle.advertiseService("pickup_marker", &HandleMarkerMS::pickup_marker_cb, this);
    this->move_alterable_object_client = this->global_handle.serviceClient<arc_msgs::MoveAlterableObject>("/arc/move_alterable_object");
    this->markers_sub = this->global_handle.subscribe("detect_marker_ps/marker_status", MAX_QUEUE_SIZE, &HandleMarkerMS::process_marker_cb, this);
    this->base_pose_sub = this->global_handle.subscribe("base_pose_ground_truth", MAX_QUEUE_SIZE, &HandleMarkerMS::process_base_pose_cb, this);
    this->priority = local_handle.getParam("priority", this->DEFAULT_PRIORITY);
    this->requested_pickup = false;


    ROS_INFO("HandleMarkerMS Parameter max_pickup_range for cleaning set: %d", this->max_pickup_range);
    ROS_INFO("HandleMarkerMS priority set: %d", this->priority);

    //starts off disabled.
    this->toggle(false);

    ROS_ASSERT(this->max_pickup_range>0);
    ROS_ASSERT(this->recently_seen.size()==0);
    ROS_ASSERT(this->priority > 0);
    ROS_ASSERT(!this->requested_pickup);
}

void HandleMarkerMS::process_base_pose_cb(nav_msgs::Odometry odom) {
    this->recent_position = odom;
}

bool HandleMarkerMS::markerWithinRange(int x, int y) {
    double distance_away = sqrt(pow(x,2) + pow(y, 2));

    if(distance_away <= this->max_pickup_range) {
        return true;
    } else {
        return false;
    }
}

bool HandleMarkerMS::drop_marker_cb(arc_msgs::DropMarker::Request &req, arc_msgs::DropMarker::Response &res) {
    if(this->marker_inventory.size()>0) {
        int marker_id = this->marker_inventory.at(0);
        int position_x = this->recent_position.pose.pose.position.x+2;
        int position_y = this->recent_position.pose.pose.position.y+2;
        this->dropMarker(marker_id, 2, 2);
    } else {
        ROS_WARN("cannot drop a marker. No markers in inventory.");
    }
}

bool HandleMarkerMS::pickup_marker_cb(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
    this->requested_pickup = true;
}

void HandleMarkerMS::setMaxPickupRange(double max_range) {
    if(max_range<=0) {
        ROS_WARN("Unable to set parameter: max_pickup_range. Value must be > 0. Using default.");
        this->max_pickup_range = this->DEFAULT_MAX_PICKUP_RANGE;
    } else {
        this->max_pickup_range = max_range;
    }
}

void HandleMarkerMS::run() {
    ros::Rate r(ROS_RATE);

    while(ros::ok()) {
        ros::spinOnce();
        r.sleep();
    }

}

void HandleMarkerMS::process_marker_cb(arc_msgs::DetectedMarkers markers) {
    ROS_INFO("Process marker cb");
    if(this->enabled) {
        this->recently_seen.erase(this->recently_seen.begin(), this->recently_seen.end());
        for(std::vector<arc_msgs::DetectedMarker>::iterator it = markers.markers.begin(); it != markers.markers.end(); ++it) {
            this->recently_seen.push_back(it->marker_id);

            if(this->requested_pickup) {
                if(this->markerWithinRange(it->pose.position.x, it->pose.position.y)) {
                    this->pickupMarker(it->marker_id, it->pose.position.x, it->pose.position.y);
                } else {
                    ROS_WARN("Marker is not within in range to pickup. You need to get closer.");
                }
            }
        }
    }
}

void HandleMarkerMS::pickupMarker(int id, double x, double y) {
    ROS_ASSERT(!this->inventoryContainsMarker(id));
    ROS_ASSERT(this->markerWithinRange(x,y));

    const int MARKER_GRAVEYARD_X = -5;
    const int MARKER_GRAVEYARD_Y = -5;

    this->marker_inventory.push_back(id);
    ROS_INFO("Picked up marker %d", id);

    arc_msgs::MoveAlterableObject req;
    req.request.fiducial_return = id;
    req.request.pose.x = MARKER_GRAVEYARD_X;
    req.request.pose.y = MARKER_GRAVEYARD_Y;
    this->move_alterable_object_client.call(req);

    this->requested_pickup = false;

    ROS_ASSERT(this->inventoryContainsMarker(id));
}

bool HandleMarkerMS::inventoryContainsMarker(int id) {
    bool result = false;

    int index = 0;
    while(index < this->marker_inventory.size()) {
        if(this->marker_inventory.at(index)==id) {
            result = true;
            break;
        }
        index++;
    }

    return result;
}

void HandleMarkerMS::dropMarker(int id, double x, double y) {
    ROS_ASSERT(this->inventoryContainsMarker(id));

    arc_msgs::MoveAlterableObject req;
    req.request.fiducial_return = id;
    req.request.pose.x = x;
    req.request.pose.y = y;
    this->move_alterable_object_client.call(req);
    std::vector<int>::iterator position = std::find(this->marker_inventory.begin(), this->marker_inventory.end(), id);
    this->marker_inventory.erase(position);

    ROS_ASSERT(!this->inventoryContainsMarker(id));
}

bool HandleMarkerMS::toggle_cb(std_srvs::SetBoolRequest &req, std_srvs::SetBoolResponse &res) {
    this->toggle(req.data);
    return true;
}

void HandleMarkerMS::toggle(bool state) {
    this->enabled = state;
    this->requested_pickup = false;

    if(this->enabled) {
        ROS_INFO("HandleMarkerMS has been enabled.");
    } else {
        ROS_INFO("HandleMarkerMS has been disabled.");
    }
}
