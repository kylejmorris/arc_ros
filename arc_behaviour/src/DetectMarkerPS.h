/**
* CLASS: MarkerDetectorPS
* DATE: 13/02/17
* AUTHOR: Kyle Morris
* DESCRIPTION: Determine if markers are near by.
*/

#include "marker_msgs/MarkerDetection.h"
#ifndef ARC_BEHAVIOUR_MARKERDETECTORPS_H
#define ARC_BEHAVIOUR_MARKERDETECTORPS_H

namespace arc_behaviour {
class DetectMarkerPS {
private:
    /**
     * Collection of the markers found in given perceptual instance.
     */
    marker_msgs::MarkerDetection found_markers;

    /**
     * Maximum range that we can detect markers from robots position.
     */
    int max_range;

    /**
     * Publish marker information, ie a simple boolean if markers are nearby.
     */
    ros::Publisher marker_status_publisher;
    ros::Subscriber marker_detector_sub;

    //for public interface with ros
    ros::NodeHandle *global_handle; //the handler for this node.

    //for private node material
    ros::NodeHandle local_handle;

    //callbacks

    //update us with most recent stage information
    void process_detect_marker_cb(const marker_msgs::MarkerDetection marker_info);
public:
    /**
     * Setup the detector with all of the ros topic publishers.
     * @param handle : the node handle to setup with.
     */
    DetectMarkerPS();

    /**
     * Check output from stage and prune markers to ones only within our maximum range.
     */
    void ProcessStageFiducial();

    /**
     * @return True if at least 1 marker is nearby. False otherwise.
     */
    bool areMarkersNearby();

    void setMaxRange(int new_range);

    ros::NodeHandle *getNodeHandle();

    /**
     * Main loop. Publishes marker status information.
     */
    void run();
};
}

#endif //ARC_BEHAVIOUR_MARKERDETECTORPS_H
