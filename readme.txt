These packages have been tested under ROS Fuerte and Diamondback. It may work with other ROS versions, but has not been tested.

=============================================================
Installation instruction (assuming you have ros installed)
=============================================================

Download the ardrone_brown and ar_recog packages from the following repository

https://code.google.com/p/brown-ros-pkg/source/browse/#svn%2Ftrunk%2Fexperimental

Follow the instructions here
https://code.google.com/p/brown-ros-pkg/wiki/ardrone_brown

and here
https://code.google.com/p/brown-ros-pkg/wiki/ar_recog

to build the packages.

Download and build the joy_teleop and tag_follow packages from this repository. You might need to do a make clean before issuing rosmake.


=============================================================
Running Instructions
=============================================================
Execute the following sequence of commands or

ros core
rosrun ardrone_brown ardrone_driver

rosrun joy joy_node
rosrun joy_teleop joy_teleop 

rosrun tag_follow tag_follow 

roscd ar_recog/bin
rosparam set aov 0.67
rosrun ar_recog ar_recog
rosrun ar_recog ar_recog image:=ardrone/image_raw
rosrun image_view image_view image:=ar/image &

