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
     * Publish marker information.
     */
    ros::Publisher marker_publisher;

    ros::NodeHandle *handle; //the handler for this node.
public:
    /**
     * Setup the detector with all of the ros topic publishers.
     * @param handle : the node handle to setup with.
     */
    DetectMarkerPS();

    /**
     * Check output from Stage.
     */
    void ProcessStageFiducial();

    /**
     *
     * @return
     */
    bool areMarkersNearby();

    void setMaxRange(int new_range);

    ros::NodeHandle *getNodeHandle();
};
}

#endif //ARC_BEHAVIOUR_MARKERDETECTORPS_H
