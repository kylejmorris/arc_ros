/**
* CLASS: CleanDebrisMS
* DATE: 28/02/17
* AUTHOR: Kyle Morris
* DESCRIPTION: Clean debris that is within range.
 * Right now the debris cleaning is trivial, that is we just approach debris,
 * and then have a timer that counts down until it is teleported away.
 * Debris that is nearby is kept in a map. If we go out of range of that debris, it will be forgotten and need
 * to be recleaned.
*/
#ifndef ARC_BEHAVIOUR_CLEANDEBRISMS_H
#define ARC_BEHAVIOUR_CLEANDEBRISMS_H
#include "ros/ros.h"
#include "MotorSchema.h"
#include "std_srvs/SetBool.h"
#include "arc_msgs/DetectedDebris.h"
#include "arc_msgs/Debris.h"

namespace arc_behaviour {
    class CleanDebrisMS : public MotorSchema {
    private:
        /**
         * Map the id of debris to the time it was found.
         */
        std::map<int, int> debris_found;

        int DEFAULT_PRIORITY = 1;
        double DEFAULT_MAX_CLEANING_RANGE = 10.0;
        int DEFAULT_CLEANING_TIME = 5; //5 seconds to clean debris

        /**
         * Service allowing enabling/disabling of this schema.
         */
        ros::ServiceServer toggle_server;

        /**
         * Sends requests to stage to delete debris from map.
         */
        ros::ServiceClient clean_debris_client;

        ros::NodeHandle global_handle;
        ros::NodeHandle local_handle;

        /**
         * Subscriber for debris detection ps
         */
        ros::Subscriber debris_sub;

        /**
         * how much priority this behaviour has. Will determine if it is accepted by navigation adapter.
         */
        int priority;

        /**
         * Maximum distance in meters, that a goal can be. Relative to current location.
         */
        double max_cleaning_range;

        /**
         * how long (in seconds) it takes to clean debris. We compare this with the time it was found
         * in a map to determine when we clean it.
         */
        int cleaning_time;
    private:


        /**
         * The holy grail of debris cleaning. The big method behind it all.
         * It's not really all that. This just sends a request to stage to get rid of a
         * debris object with some id.
         * PRE: Debris with id is readyToClean()
         */
        void cleanDebris(int id);

        /**
         * Check the record of this debris, and determine if we have been "cleaning" it long enough.
         * @param id
         * PRE: Debris with this id exists in records.
         * @return bool: True if ready, false otherwise.
         */
        bool readyToClean(int id);

        /**
         * Record a new debris object found. This will timestamp it and store in a map This will timestamp it and store in a map
         * @param id : the id of debris object found.
         */
        void recordDebris(int id);

        /**
         * Determine if debris is close enough to clean.
         * @param x: x coordinate of debris
         * @param y: y coordinate of debris
         * @return: True if within range, false otherwise.
         */
        bool debrisWithinRange(int x, int y);

        /**
         * Check if debris with given id is in our records.
         * @param id: id of debris to check
         * @return: bool: True if in records, false otherwise.
         */
        bool debrisInRecord(int id);
    public:
        CleanDebrisMS();
        void process_debris_cb(const arc_msgs::DetectedDebris debris);
        bool toggle_cb(std_srvs::SetBoolRequest &req, std_srvs::SetBoolResponse &res);
        void setMaxRange(double max_range);
        void toggle(bool state);

        /**
         * Main loop
         */
        void run();
    };
}

#endif //ARC_BEHAVIOUR_RANDOMWANDERMS_H
