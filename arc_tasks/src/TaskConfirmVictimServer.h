/**
* CLASS: TaskConfirmVictimServer
* DATE: 11/01/18
* AUTHOR: Kyle Morris
* DESCRIPTION: Robot goes to a victim location to confirm if a victim is actually there or not. Will broadcast a status response afterwards.
*/

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include "arc_msgs/ArcTaskAction.h"
#include "arc_msgs/DetectedDebris.h"
#include "arc_msgs/NavigationRequest.h"
#include "std_srvs/Trigger.h"
#include <actionlib/server/simple_action_server.h>
#include <arc_msgs/DetectedVictims.h>
#include "nav_msgs/Odometry.h"
#include "TaskServer.h"

#ifndef ARC_TASKS_TaskConfirmVictimServer_H
#define ARC_TASKS_TaskConfirmVictimServer_H
typedef actionlib::SimpleActionServer<arc_msgs::ArcTaskAction> ActionServer;

class TaskConfirmVictimServer : public TaskServer {
private:
    ros::NodeHandle global_handle;
    ros::NodeHandle local_handle;

    ActionServer server;

    /**
     * Timer to keep track of how much longer we explore the environment.
     * Once started, it will continuously cycle through a specified countdown and then call the
     * explore_timer_cb
     */
    ros::Timer explore_timer;

    /**
     * Subscribes to list of debris that robot has found in it's view.
     */
    ros::Subscriber victim_sub;

    /**
     * Subscribe to incoming announcements from robots that are accepting a confirm_victim task themselves.
     */
    ros::Subscriber confirm_victim_announcement_sub;

    /**
     * Publish results about if you found victim or not at a given location.
     */
    ros::Publisher victim_status_pub;

    /**
     * Subscribes to our current position
     */
    //TODO: Instead of tracking position using base_pose an such, just use the TF support. You can do this in quite a few places in the framework now instead of this gibberish.
    ros::Subscriber base_pos_sub;
    tf::TransformListener pose_listener;

    tf2_ros::Buffer *victim_buffer;
    tf2_ros::TransformListener *victim_listener;

    /**
     * List of the debris we most recently detected. It is always being updated
     */
    arc_msgs::DetectedVictims victims_found_nearby;

    /**
     * A list of the potential victims we were told to navigate to and confirm.
     */
    arc_msgs::DetectedVictims victim_list;

    /**
     * Keep tracking of most recent position of the robot so we can identify our relative goal towards debris.
     */
    nav_msgs::Odometry recent_pose;

    /**
     * Current potential victim we are navigating to.
     */
    arc_msgs::DetectedVictim target_victim;

    /**
     * List of the victims we have handled already. Don't recheck them now... this is to prevent us from detecting the victim in front of us repeatedly, and getting stuck in
     * infinite loop as we confirm their status.
     */
    arc_msgs::DetectedVictims checkedVictims;

    /**
     * After getting the target debris location relative to us, we set it's position as navigation goal
     */
    geometry_msgs::Pose target_pose;

    /**
     * used to call the arc_base node and toggle the schema of our choice
     */
    ros::ServiceClient arc_base_client;

    /**
     * Used to send movement request to this node, so we can navigate to debris
     */
    ros::ServiceClient move_to_debris_client;

    /**
     * Used to tell navigation adapter to stop, so we remain close to debris to clean.
     */
    ros::ServiceClient abort_all_goals_client;

    //control the state of the debris exploration/removal
    typedef enum {
        STATE_SelectVictimTarget, //select next potential victim to navigate to.
        STATE_SeekingVictimLocation, //Heading to where the victim is
        STATE_DetectingVictim
    } State;
    State state;

    /**
     * All state variables that only last for the length of the individual task instance.
     * We want an easy way to reset things after.
     */
    struct task_instance_state {
        /**
        * Track if we are currently moving towards a debris object
        */
        bool currently_seeking_debris = false;

        /**
         * When we are seeking debris, keep track when we see it. If we don't, then we assume it's not there
         * and we can just move on to next debris in list.
         */
        bool found_debris_target = false;
    };
    typedef task_instance_state InstanceState;
    InstanceState instance_state;

    /**
     * The results of running this task. Nothing required for the explore task.
     */
    arc_msgs::ArcTaskResult result;

    enum VICTIM_STATUS {
        UNKNOWN,
        POTENTIAL,
        NEGATIVE,
        POSITIVE
    };

    std::string robotName;

    //These variables are only here to ensure we don't repeatedly call ROS services, we only have to do it once.
    /**
     * Whether or not the task is currently active.
     */
    bool active = false;

    /**
     * Keeping track of amount of debris cleaned
     */
    int victim_success_count = 0;


    /**
     * How many debris objects were requested to be cleaned this task instance.
     */
    int victim_count = 0;

    /**
     * If we were told a victim was at some (x,y) location, we will only consider that we found the correct victim if we go there and find a victim within this
     * distance of the desired location.
     */
    constexpr static double MAX_VICTIM_DISPLACMENT_THRESHOLD = 0.5;

//PARAMETERS and their default values;
    double stopping_distance_from_victim = 0.5;

    /**
     * Simplifies process of storing parameters in this object.
     * Check a given string parameter and store it in the designated variable, if no such name is valid,
     * then we don't register this parameter.
     * @param name The name of the string parameter to decode
     * @param value Value of the parameter
     * @return bool: True if this paramter was decoded and stored in our object. False if failed (invalid name, invalid value...)
     */
    bool decodeStringParameter(std::string name, std::string value);

    /**
     * Given string consisting info on where debris is, return the list of debris objects.
     * @param input: The string containing info on where debris is. Something like "(id,x1,y1)|(id2,x2,y2)"..
     * @return list of debris found in input string
     */
    arc_msgs::DetectedVictims parseVictimList(std::string input);

    /**
     * Check if we already visited this victim yet or not.
     */
    bool alreadyCheckedVictim(const arc_msgs::DetectedVictim &victim);

    /**
     * Record that we are done confirming a given victim. This will ensure we don't look for it again.
     * @param victim: The victim to confirm
     */
    void completeConfirmingVictim(const arc_msgs::DetectedVictim &victim);

    template <typename T>
    double dist(const T &first, const T &second );

    bool victimSeenAlready(const arc_msgs::DetectedVictim &victim);

public:
    /**
     * @param robotName
     * @param customNamespace  In case of debugging mode, we can specify a custom namespace so we can redirect topics to a debugger.
     */
    TaskConfirmVictimServer(const std::string &robotName, const std::string &customNamespace);

    /**
     * Perform any routine startup procedures when this task instance is started.
     * Load request parameters, check for existence of nodes needed for this task.
     */
    virtual void startup(const arc_msgs::ArcTaskGoalConstPtr &goal);

    /**
     * the main state machine loop for the task
     */
    virtual void process();

    /**
     * Ensure after this task instance is no longer of use, we've shut down everything that was required for it.
     */
    virtual void shutdown();

//STATES
    /**
     * Perform the shutdown for a task, disabling anything that was originally set to the complete it.
     */
    void StateSelectVictimTarget();

    /**
     * Disable the random wander and head to debris location
     */
    void StateSeekingVictimLocation();

    /**
     * Clean debris once we are there.
     */
    void StateDetectingVictim();

    std::string stateToString(State state);

//CALLBACKS
    /**
    * Callback after the explore timer gets set off
    * @param event some data about how long we explored.
    */
    void explore_timer_cb(const ros::TimerEvent &event);
    void found_victims_cb(const arc_msgs::DetectedVictims &victims);
    void base_pose_cb(const nav_msgs::Odometry &odom);

    /**
     * Cleanup the variables and state information set during a task so we may call it again.
     */
    void cleanup();
};

#endif //ARC_TASKS_TASKCONFIRMVICTIMSERVER_H
