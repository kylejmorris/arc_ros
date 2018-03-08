#include "XmlRpcException.h"
#include "../include/ArcBase.h"
#include "ros/ros.h"
#include "std_srvs/SetBool.h"
#include "std_srvs/Empty.h"

using namespace arc_behaviour;
using namespace XmlRpc;
#define DEFAULT_RATE 10

ArcBase::ArcBase(ros::NodeHandle *nh) {
    ros::NodeHandle global_handle;
    this->nh = nh;
    this->global_handle = global_handle;
    this->toggle_server = this->nh->advertiseService("toggle_schema", &ArcBase::toggle_schema_cb, this);
}

bool ArcBase::setupSchemas() {
    ROS_INFO("Setting up motor schemas.");
    XmlRpcValue schema_list;

    try {

        ros::param::get("arc_base/schemas", schema_list);

        for(unsigned i=0; i< schema_list.size(); i++) {
            ROS_DEBUG("Schema list contains: %s", schema_list[i]);
            if(schema_list[i].getType()==XmlRpcValue::TypeString) {
                std::string content = schema_list[i];
                std::string topic_name = content + "/toggle";
                ROS_INFO("Found schema: %s. subscribing to topic %s", content.c_str(), topic_name.c_str());
                ros::ServiceClient client = this->global_handle.serviceClient<std_srvs::SetBool>(topic_name.c_str());

                this->motor_clients.insert({content, client});
            } else {
                ROS_WARN("ArcBase::setupSchemas: found non-string value in schema list. value ignored.");
            }
        }
    } catch(XmlRpc::XmlRpcException problem) {
        ROS_WARN("error in arc_base::setupSchemas: %s. Make sure /arc_base/schemas param is set to strings.", problem.getMessage().c_str());
    }

    return true;
}

void ArcBase::disableAll() {

}

void ArcBase::setup() {
    ROS_INFO("Initializing arc_base...");
    this->setupSchemas();
}

bool ArcBase::toggleSchema(std::string type, bool state) {
    std_srvs::SetBool req;
    req.request.data = state;
    this->motor_clients[type].call(req);
    return true;
}

bool ArcBase::toggle_schema_cb(arc_msgs::ToggleSchema::Request &req, arc_msgs::ToggleSchema::Response &res) {
    //toggle each schema requeste
    for(int pos=0; pos<req.schema.size(); pos++) {
        dynamic_reconfigure::BoolParameter param = req.schema.at(pos);

        if(this->motor_clients[param.name]!=NULL) {
            this->toggleSchema(param.name, param.value);
        } else {
            ROS_WARN("ArcBase::toggleSchema: %s was not found as a schema.", param.name.c_str());
        }
    }

    return true;
}

void ArcBase::run() {
    ros::Rate r(DEFAULT_RATE);

    while(ros::ok()) {
        ros::spinOnce();
        r.sleep();
    }
}

geometry_msgs::Twist ArcBase::getActionVector() {
    return geometry_msgs::Twist();
}
