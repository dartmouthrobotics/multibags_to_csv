# multibags_to_csv package

This package converts multiple .bags including multiple robot topics including ais_info, boundary.

* Key feature: 
    * Automatically makes subscriber and message synchronizer depending on the number of obstacles in a bag file.
    * Automatically goes over all the bags in sub-directories of a main folder.

* reference: 
    * http://wiki.ros.org/message_filters/ApproximateTime

## License
This project is licensed under the MIT License.

Authors: Mingi Jeong

## Prerequisites
You need to have ROS (equal to or above kinetic), Ubuntu (equal to or above 16.04).
For Kinetic, you need to change the replace code of `/opt/ros/kinetic/lib/python2.7/dist-packages/roslaunch/config.py` of `roslaunch api` with the following code: https://raw.githubusercontent.com/sputnick1124/ros_comm/9e0cf2e52c28540b9515b2b43eacf56652008753/tools/roslaunch/src/roslaunch/config.py


### Dependencies

1. obstacle_avoidance_ros_pkg
    * this pkg includes messages such as `ais_info`, `boundary`, `running_time`.


## Install
Clone this package into your `catkin_ws` and do `catkin_make` for building.

```
cd catkin_workspace/src
git clone https://github.com/dartmouthrobotics/multibags_to_csv.git
cd ..
catkin_make
```

## Usage

### 1. configuration of main bag path
* Inside `nodes/convert_harness`, change `path=/home/minkbrook/Desktop/testbag/` as per the main path in your computer which includes all bags.
* The code will walk through all sub-folders to find `.bag` files.

### 2. Running 

* run the auto converter node
    ```
    roslaunch multibags_to_csv auto_converter.launch
    ```
### Additional script
* running time converter script
Note that the bag file contains `running_time` topic for computational time records.
    ```
    python ConverterRuntime.py
    ```
* pose converter script
Note that the bag file contains each agent position topic for pose.
    ```
    python ConvertertPosition.py
    ```


### TODO
* another node for running time and robot_0 pose


