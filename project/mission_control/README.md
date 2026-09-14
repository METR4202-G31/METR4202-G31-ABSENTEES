Central controller node for the autonomous exploration. Handles the following tasks:
 - reading incoming aruco pose information and building a list of global marker positions
 - publishing the markers to /markers eg. for Rvizz visualisation
 - managing the transition between stage 1 'explore' mode and stage 2 'search' mode

Run this node with "ros2 run mission_control mission_control_node". All other accompanying nodes (aruco_detector) must be running for this node to run with full functionality.

To see the stored markers on Rvizz, add the topic 'Markers" to the displayed objects on the left menu.
