/**
* CLASS: VictimTracker
* DATE: 04/01/18
* AUTHOR: Kyle Morris
* DESCRIPTION: Manage detected victim information in the environment.
*/

#ifndef ARC_CORE_VICTIMTRACKER_H
#define ARC_CORE_VICTIMTRACKER_H
#include <ros/ros.h>
#include <mutex>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <arc_msgs/DetectedVictims.h>


class VictimTracker {
private:
    ros::NodeHandle local;
    ros::NodeHandle global;

    ros::Subscriber incoming_victims_sub;

    /**
     * Broadcast when we need to confirm victims in environment.
     */
    ros::Publisher confirm_victim_task_request_pub;

    tf2_ros::Buffer *victim_buffer;
    tf2_ros::TransformListener *victim_listener;

    static const int MAX_QUEUE_SIZE = 100;

    std::string tf_prefix;

    constexpr static const double MAX_DISPLACEMENT_THRESHOLD = 2.0;

    /**
     * The victims we have already seen already.
     */
    std::vector<arc_msgs::DetectedVictim> confirmedVictims;

    /**
     * Victims that have not yet been confirmed but we broadcasted them and are waiting for someone to help us out.
     */
    std::vector<arc_msgs::DetectedVictim> awaitingConfirmation;

    /**
     * Victims we just found. Not yet confirmed.
     */
    std::vector<arc_msgs::DetectedVictim> potentialVictims;

    std::mutex potentialVictimsMutex;
    std::mutex awaitingConfirmationMutex;
    std::mutex confirmedVictimMutex;

    bool alreadyDetectedVictim(const arc_msgs::DetectedVictim &victim);


    template<typename T>
    double dist(const T &first, const T &second);

    void evaluatePotentialVictims();

    void broadcastConfirmVictimTask(const arc_msgs::DetectedVictims &victims);
public:
    VictimTracker(const std::string &customNamespace);

    void start();

    void incoming_victims_cb(const arc_msgs::DetectedVictims &msg);
};


#endif //ARC_CORE_VICTIMTRACKER_H
