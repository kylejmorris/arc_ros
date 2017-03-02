/**
* CLASS: HandleMarkerMS
* DATE: March 1, 2017
* AUTHOR: Kyle Morris
* DESCRIPTION: This schema will manage fiducial markers.
* This may involve picking one up off the ground, or dropping new ones (if it has any).
*/
#ifndef ARC_BEHAVIOUR_HANDLEMARKERMS_H
#define ARC_BEHAVIOUR_HANDLEMARKERMS_H
#include "ros/ros.h"
#include "MotorSchema.h"
#include "std_srvs/SetBool.h"
#include "marker_msgs/MarkerDetection.h"
#include "arc_msgs/DetectedMarkers.h"
#include "std_srvs/Empty.h"
#include "arc_msgs/DropMarker.h"
#include "nav_msgs/Odometry.h"

namespace arc_behaviour {
    class HandleMarkerMS : public MotorSchema {
    private:
        int DEFAULT_PRIORITY = 2;
        int DEFAULT_MAX_PICKUP_RANGE = 2;
        int GRAVEYARD_X = -5;
        int GRAVEYARD_Y = -5;

        /**
         * how much priority this behaviour has. Will determine if it is accepted by navigation adapter.
         */
        int priority;

        int max_pickup_range;

        //Is the schema currently active?
        bool enabled;

        /**
         * When pickup marker service is called, we will set this.
         * This means on any time we process recently found markers, we will pick one of them up.
         * If we just pick up a marker from our recently_seen list, we may end up picking a marker up
         * in the distant future, which may not exist on the ground anymore.
         */
        bool requested_pickup;

        /**
         * Keeping track of most recent position, so we can drop markers relative to our position;
         */
        nav_msgs::Odometry recent_position;
        /**
         * List of id's of the markers we currently have in store.
         */
        std::vector<int> marker_inventory;

        /**
         * The markers we most recently saw in the environment.
         */
        std::vector<int> recently_seen;

    private:
        /**
         * Service allowing enabling/disabling of this schema.
         */
        ros::ServiceServer toggle_server;

        /**
         * Allow dropping of a marker on the ground.
         */
        ros::ServiceServer drop_marker_server;
        /**
         * Allow robot to pick up a marker from the ground.
         */
        ros::ServiceServer pickup_marker_server;

        /**
         * Send requests to stage to move the markers we are handling.
         */
        ros::ServiceClient move_alterable_object_client;

        ros::NodeHandle global_handle;
        ros::NodeHandle local_handle;

        /**
         * Subscriber for markers in environment.
         */
        ros::Subscriber markers_sub;

        /*
         * allow robot to know of it's position on map
         */
        ros::Subscriber base_pose_sub;

    private:
        /**
         * Determine if marker is close enough to clean.
         * @param x: x coordinate of debris
         * @param y: y coordinate of debris
         * @return: True if within range, false otherwise.
         */
        bool markerWithinRange(int x, int y);

        /**
         * Pick up marker with given id off the ground.
         * This will send the marker into our inventory.
         * PRE:
         *      the marker with id exists and is within range to pickup
         *      !markerInInventory()
         * POST:
         *      Inventory now contains marker with "id"
         * @param id: Id of the marker to pick up
         * @param x,y: coordinates of marker relative to robot
         */
        void pickupMarker(int id, double x, double y);

        /**
         * Drop a marker with given id.
         * @param id the id of marker to drop
         * @param x, y: The coordinates to drop marker, relative to robot
         */
        void dropMarker(int id, double x, double y);

        /**
         * Check if given inventory contains marker
         * @param id: id of marker to check
         * @return bool: true if marker is in inventory, false otherwise
         */
        bool inventoryContainsMarker(int id);

    public:
        HandleMarkerMS();
        void process_marker_cb(arc_msgs::DetectedMarkers markers);
        void process_base_pose_cb(nav_msgs::Odometry odom);
        bool toggle_cb(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
        bool drop_marker_cb(arc_msgs::DropMarker::Request &req, arc_msgs::DropMarker::Response &res);
        bool pickup_marker_cb(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
        void setMaxPickupRange(double max_range);
        void toggle(bool state);

        /**
         * Main loop
         */
        void run();
    };
}

#endif //ARC_BEHAVIOUR_RANDOMWANDERMS_H
