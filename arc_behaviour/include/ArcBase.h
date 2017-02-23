/**
* CLASS: ArcBase
* DATE: 13/02/17
* AUTHOR: Kyle Morris
* DESCRIPTION: Central entry point for behaviour in arc_ros.
*/
#include "ros/ros.h"
#include "geometry_msgs/TwistStamped.h"
#include <map>
#include "arc_msgs/ToggleSchema.h"

#ifndef ARC_BEHAVIOUR_ARCBASE_H
#define ARC_BEHAVIOUR_ARCBASE_H

namespace arc_behaviour {

class ArcBase {
private:
    /*
     * Node that this object is working on.
     */
    ros::NodeHandle *nh;

    ros::NodeHandle global_handle;
    ros::ServiceServer toggle_server;

    /**
     * Setup all of the motor schemas.
     */
    bool setupSchemas();

    /**
     * list of clients that will allow for toggling motor schemas.
     */
    std::map<std::string, ros::ServiceClient> motor_clients;
public:
    ArcBase(ros::NodeHandle *nh);

    /**
     * Connect to various schemas.
     */
    void setup();

    inline void setNodeHandle(ros::NodeHandle *nh) { this->nh = nh; }

    inline ros::NodeHandle *getNodeHandle() { return this->nh; }

    //SERVICES /** * Turn off all motor schemas. */
    void disableAll();

    /**
     * Turn on a specified motor schema.
     * @param type: Name of the schema to toggle, as specified in ros message request.
     * @param state: True or false, depending on if you want to enable/disable schema.
     * @return bool: True if the schema was toggled, false if otherwise.
     */
    bool toggleSchema(std::string type, bool state);

    /**
     * Does calculation of final action vector by taking sum of all intermediate
     * motor schema action vectors.
     * @return Twist message representing final vector.
     */
    geometry_msgs::Twist getActionVector();

    bool toggle_schema_cb(arc_msgs::ToggleSchema::Request &req, arc_msgs::ToggleSchema::Response &res);

    void run();
};
};

#endif //ARC_BEHAVIOUR_ARCBASE_H
