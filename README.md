# non-linear-error-detection
non-linear error detection for both sensors and actuators simultanuously

urdf file is attached

export TURTLEBOT3_MODEL=waffle_pi
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch \
roslaunch nuise turtlebot3_slam.launch \
roslaunch aloam_velodyne aloam_velodyne_HDL_32.launch \
rosrun nuise nuise \
rosrun follower follower \

Depend on turtlebot3_gazebo turtlebot3_teleop(modified) gmapping
