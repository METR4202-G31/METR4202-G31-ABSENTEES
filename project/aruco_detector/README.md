This package processes camera information from /camera/image_raw and /camera/camera_info to detect aruco markers and calculate pose. When this package it run, messages to /aruco will be published every 0.1 seconds with the following information:

detected = True
id = #the id of the marker that can be seen on camera
pose = information about the position and orientation of the aruco marker with respect to the camera. (uses geometry_msgs/Pose)

For "position" information in pose:
 x:  left/right (+ means aruco to the right, - means aruco to the left)
y: height (- means aruco is higher than robot + means aruco is lower than robot)
z: front/back (+ means aruco in front, - aruco is impossibly behind camera)

If more than one aruco marker is detected on camera, multiple messages will be published at once, every 0.1 seconds (one for each marker). Each will have the above format.

Once installed, to run this package do ros2 run aruco_detector aruco_detector_node
