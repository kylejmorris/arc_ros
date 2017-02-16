/**
* CLASS: MarkerDetectorPS
* DATE: 13/02/17
* AUTHOR: Kyle Morris
* DESCRIPTION: Determine if markers are near by.
*/

#include "marker_msgs/MarkerDetection.h"
#include "geometry_msgs/Pose.h"
#include "arc_msgs/Debris.h"
#include "arc_msgs/DetectedDebris.h"
#ifndef ARC_BEHAVIOUR_DebrisDETECTORPS_H
#define ARC_BEHAVIOUR_DebrisDETECTORPS_H

namespace arc_behaviour {
class DetectDebrisPS {
private:
    /**
     * Collection of the markers found in given perceptual instance.
     */
    marker_msgs::MarkerDetection found_markers;

    //The debris within our range.
    arc_msgs::DetectedDebris debris_list;

    /**
     * Maximum range that we can detect markers from robots position.
     */
    int max_range;

    /**
     * Publish Debris information
     */
    ros::Publisher debris_location_publisher;
    ros::Subscriber debris_detector_sub;

    //for public interface with ros
    ros::NodeHandle *global_handle; //the handler for this node.

    //for private node material
    ros::NodeHandle local_handle;

    //callbacks
    //update us with most recent stage information
    void process_detect_debris_cb(const marker_msgs::MarkerDetection marker_info);
public:
    /**
     * Setup the detector with all of the ros topic publishers.
     * @param handle : the node handle to setup with.
     */
    DetectDebrisPS();

    /**
     * Check output from stage and prune Debris to ones only within our maximum range.
     */
    void ProcessStageFiducial();

    void setMaxRange(int new_range);

    ros::NodeHandle *getNodeHandle();

    arc_msgs::DetectedDebris getDebris();

    /**
     * Main loop. Publishes Debris status information.
     */
    void run();
};
}

#endif //ARC_BEHAVIOUR_DebrisDETECTORPS_H
