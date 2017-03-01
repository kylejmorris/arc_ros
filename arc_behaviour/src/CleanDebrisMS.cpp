#include "CleanDebrisMS.h"
#include "arc_msgs/Debris.h"
#include "arc_msgs/DetectedDebris.h"
#include "arc_msgs/MoveAlterableObject.h"

#define ROS_RATE 10 //hz
#define MAX_QUEUE_SIZE 1000 //max number of messages to keep on queue before flushing

using namespace arc_behaviour;

CleanDebrisMS::CleanDebrisMS() {
    ros::NodeHandle global_handle;
    ros::NodeHandle local_handle("clean_debris_ms");
    this->global_handle = global_handle;
    this->local_handle = local_handle;
    ROS_INFO("Setting up clean debris ms");
    local_handle.param("max_cleaning_range", this->max_cleaning_range, this->DEFAULT_MAX_CLEANING_RANGE);
    this->toggle_server = this->local_handle.advertiseService("toggle", &CleanDebrisMS::toggle_cb, this);
    this->debris_sub = global_handle.subscribe("detect_debris_ps/debris_locations",MAX_QUEUE_SIZE, &CleanDebrisMS::process_debris_cb, this);
    this->priority = local_handle.getParam("priority", this->DEFAULT_PRIORITY);
    this->clean_debris_client = global_handle.serviceClient<arc_msgs::MoveAlterableObject>("/arc/move_alterable_object");
    local_handle.param("cleaning_time", this->cleaning_time, this->DEFAULT_CLEANING_TIME);

    ROS_INFO("CleanDebrisMS Parameter max_cleaning_range for cleaning set: %d", this->max_cleaning_range);
    ROS_INFO("CleanDebrisMS cleaning time set: %d seconds", this->cleaning_time);
    ROS_INFO("CleanDebrisMS priority set: %d", this->priority);

    //starts off disabled.
    this->toggle(false);

    ROS_ASSERT(this->cleaning_time > 0);
}

void CleanDebrisMS::setMaxRange(double max_range) {
    if(max_range<=0) {
        ROS_WARN("Unable to set parameter: max_range. Value must be > 0. Using default.");
        this->max_cleaning_range = this->DEFAULT_MAX_CLEANING_RANGE;
    } else {
        this->max_cleaning_range = max_range;
    }
}

void CleanDebrisMS::run() {
    ros::Rate r(ROS_RATE);

    while(ros::ok()) {
        ros::spinOnce();
        r.sleep();
    }

}

bool CleanDebrisMS::readyToClean(int id) {
    ROS_ASSERT(this->debris_found.count(id)>0);

    bool result = false;
    int found_time = this->debris_found.find(id)->second;
    int current_time = ros::Time::now().toSec();

    ROS_INFO("foundtime of debris %d is %d. Now is time %d", id, found_time, current_time);

    if(current_time-found_time >= this->cleaning_time) {
        result = true;
    }

    return result;
}

void CleanDebrisMS::process_debris_cb(const arc_msgs::DetectedDebris debris) {
    if(this->enabled) {
        for (int pos = 0; pos < debris.debris.size(); pos++) {
            arc_msgs::Debris curr = debris.debris.at(pos);
            int id = curr.debris_id;

            //ensure valid id, and we don't already have a record of it.
            if (!this->debrisInRecord(id) && this->debrisWithinRange(curr.pose.position.x, curr.pose.position.y)) {
                this->recordDebris(id);
            } else {
                if (this->debrisInRecord(id) && this->readyToClean(id)) {
                    this->cleanDebris(id);
                }
            }


            std::vector<int> to_delete; //The debris we wish to remove (ie it now out of range.)
            //go through all debris it currently knows of. Which ones are out of range? We will forget about them.
            for (std::map<int, int>::iterator it = this->debris_found.begin(); it != debris_found.end(); it++) {
                int key = it->first;
                bool keep = false; //whether or not to keep track of this debris record.

                for (int pos = 0; pos < debris.debris.size(); pos++) {
                    int id = debris.debris.at(pos).debris_id;
                    if (key == id) {
                        keep = true;
                        break;
                    }
                }

                if (!keep) {
                    //best not to delete from a map as we iterate through it. Keep track of what to delete.
                    to_delete.push_back(key);
                }
            }

            for (int pos = 0; pos < to_delete.size(); pos++) {
                int key = to_delete.at(pos);
                this->debris_found.erase(this->debris_found.find(key));
                ROS_INFO("Lost track of debris %d. Deleted from cache.", key);
            }
        }
    }
}

bool CleanDebrisMS::debrisInRecord(int id) {
    if(this->debris_found.count(id) != 0) {
        return true;
    } else {
        return false;
    }
}

bool CleanDebrisMS::debrisWithinRange(int x, int y) {
    double distance_away = sqrt(pow(x,2) + pow(y, 2));

    if(distance_away <= this->max_cleaning_range) {
        return true;
    } else {
        return false;
    }
}


void CleanDebrisMS::cleanDebris(int id) {
    arc_msgs::MoveAlterableObject req;
    //where the debris goes to rest after being cleaned from this world.
    const int DEBRIS_GRAVEYARD_X = -5;
    const int DEBRIS_GRAVEYARD_Y = -5;
    ROS_ASSERT(this->readyToClean(id));

    req.request.fiducial_return = id; //this is how we will identify the object.
    req.request.pose.x = -5;
    req.request.pose.y = -5;

    this->clean_debris_client.call(req);

    this->debris_found.erase(this->debris_found.find(id));
    ROS_INFO("Cleaning debris #%d", id);
}

void CleanDebrisMS::recordDebris(int id) {
    this->debris_found.insert({id, ros::Time::now().toSec()});
    ROS_DEBUG("Found debris with id %d", id);
}

bool CleanDebrisMS::toggle_cb(std_srvs::SetBoolRequest &req, std_srvs::SetBoolResponse &res) {
    this->toggle(req.data);
    return true;
}

void CleanDebrisMS::toggle(bool state) {
    this->enabled = state;

    if(this->enabled) {
        ROS_INFO("CleanDebrisMS has been enabled.");
    } else {
        ROS_INFO("CleanDebrisMS has been disabled.");
    }
}
