catkin_make clean
rm -rf build/ devel/
catkin_make
source devel/setup.sh
roslaunch launch/styx.launch

