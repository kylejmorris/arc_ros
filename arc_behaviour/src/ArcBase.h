/**
* CLASS: ArcBase
* DATE: 13/02/17
* AUTHOR: Kyle Morris
* DESCRIPTION: Central entry point for behaviour in arc_ros.
 * Manages motor/perceptual schemas and receives motor schema activation vectors and adds them together,
 * then publishes final result on cmd_vel.
*/
#include "ros/ros.h"
#include "geometry_msgs/TwistStamped.h"

#ifndef ARC_BEHAVIOUR_ARCBASE_H
#define ARC_BEHAVIOUR_ARCBASE_H

namespace arc_behaviour {

class ArcBase {
private:
    /**
     * MOTOR SCHEMAS
     */

    /*
     * Node that this object is working on.
     */
    ros::NodeHandle *nh;

    /**
     * Setup all of the motor schemas.
     */
    bool setupSchemas(); //TODO: Throw exception if fail.

public:
    ArcBase(ros::NodeHandle *nh);

    /**
     * Activates schemas and ros publishers.
     */
    void setup();

    inline void setNodeHandle(ros::NodeHandle *nh) { this->nh = nh; }

    inline ros::NodeHandle *getNodeHandle() { return this->nh; }

    //SERVICES /** * Turn off all motor schemas. */
    void disableAll();

    /**
     * Turn on a specified motor schema.
     * @param type: Name of the schema to toggle, as specified in ros message request.
     * @return bool: True if the schema was toggled, false if otherwise.
     */
    bool toggleSchema(std::string type);


    /**
     * Does calculation of final action vector by taking sum of all intermediate
     * motor schema action vectors.
     * @return Twist message representing final vector.
     */
    geometry_msgs::TwistStamped getActionVector();
};
};

#endif //ARC_BEHAVIOUR_ARCBASE_H
