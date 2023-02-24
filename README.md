# non-linear-error-detection
non-linear error detection for both sensors and actuators simultanuously

roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch 
roslaunch nuise turtlebot3_slam.launch
roslaunch aloam_velodyne aloam_velodyne_HDL_32.launch
rosrun nuise nuise
rosrun follower follower
