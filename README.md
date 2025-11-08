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
2. mavros-related
```
sudo apt-get install ros-{yourdistro}-mavros ros-{yourdistro}-mavros-extras
```

## Install
Clone this package into your `catkin_ws` and do `catkin_make` for building.

```
cd catkin_workspace/src
git clone https://github.com/dartmouthrobotics/multibags_to_csv.git
cd ..
catkin_make
```

## Usage

### Multiple bag converter 
--- 
In principle, this converter is playing the bag and running time sync converter as a single .csv combined. 

### 1. configuration of main bag path
* Inside `param/param.yaml`, change `bag_path:/home/minkbrook/Desktop/testbag/MOA+/` as per the main path in your computer which includes all bags.
Make sure to close with `/` at the end of the string.
* The code will walk through all sub-folders to find `.bag` files.

### 2. Running 

* run the auto converter node (time-sync csv across all agents)
    ```
    roslaunch multibags_to_csv auto_converter.launch
    ```

### Additional script
--- 
This below does not require playing the bag and directly convert to .csv, e.g., each drone to separate .csv
* running time converter script (old)
    * change the `path` inside `ConverterRuntime.py` 
    * Note that the bag file contains `running_time` topic for computational time records.
    ```
    python ConverterRuntime.py
    ```
* pose converter script (old)
    * change the `path` inside `ConvertertPosition.py` 
    * Note that the bag file contains each agent position topic for pose.
    ```
    python ConvertertPosition.py
    ```
* Single python script that can do all (__new__) 
    * change indiv-converter.yaml file first! 
    * no need to change param.yaml
    * change PARAM_INTEREST in `ConverterParams.py`
    * give the -p as the folder where (multiple) bag is located
    ```
    python ConverterParams.py -p /mnt/mydrive/catabot_bag/barbados2024/test/test/
    ```

### TODO
* ground truth extractor such as ConverterGT.py