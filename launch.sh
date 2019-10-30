#!/bin/bash

set -e

# YOUR CODE BELOW THIS LINE
# ----------------------------------------------------------------------------
#roslaunch vehicle_detection vehicle_detection_node.launch veh:=![vehicle]
#roslaunch vehicle_detection vehicle_filter_node.launch veh:=cristina
roslaunch detectionpack detectionpack.launch veh:=$VEHICLE_NAME

