#!/usr/bin/env python

"""
    ConvertHarness.py

    - Class definition of 'ConvertHarness' 
    - basic modular part of converting multiple .bags
"""


# essential packages
from time import sleep
import csv
import copy
from datetime import datetime

# import of relevant ROS libraries
import rospy
import roslaunch  
import rospkg
from rosgraph_msgs.msg import Clock

# import custom modules

TEST_INITIALIZATION_WAIT_SECS = 3.0
DETECT_WAIT_SECS = 15.0
DEFAULT_BAG_STATE_TOPIC = 'clock'


class ConvertHarness:
    # reference: http://wiki.ros.org/roslaunch/API%20Usage

    def __init__(self):
        rospy.sleep(1.0)
   
        # self.roslaunch_args = None

        self.evaluation_subscriber = rospy.Subscriber(DEFAULT_BAG_STATE_TOPIC, Clock, self.clock_callback, queue_size=10)

        self.logging_time = rospy.Time.now()


    def clock_initialize(self):
        self.logging_time = rospy.Time.now()


    def clock_callback(self, msg):
        self.logging_time = rospy.Time.now()


    def run_convert(self, convert_launch_file, launch_arguments):
        rospy.loginfo("--------------------------------------------------------")
        rospy.logwarn("conversion START")

        self.clock_initialize()

        """ api configuration """
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)


        roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(convert_launch_file)[0], 
                    launch_arguments)]

        parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)

        parent.start()
        rospy.on_shutdown(parent.shutdown) # killer

        # sleep(2.0)

        # bag_file_name extract
        bag_file_name = None
        for arg in launch_arguments:
            if 'bag_file:=' in arg:
                bag_file_name = arg[len('bag_file:='):]

        rospy.logwarn("Conversion started: {}".format(bag_file_name))
        # sleep(TEST_INITIALIZATION_WAIT_SECS)


        """ running conversion """
        while True:
            
            maintain_duration = (rospy.Time.now() - self.logging_time).to_sec()

            if maintain_duration > DETECT_WAIT_SECS:
                break

            # rospy.Rate(5).sleep()
        
        parent.shutdown()

        """ finish the test """
        rospy.loginfo("Terminating ..............................")

        sleep(10.0)





if __name__ == "__main__":
    rospy.init_node('Convert_harness')
    rospy.sleep(2.0)

    launch_file = ['/home/minkbrook/catkin_ws/src/multibags_to_csv/launch/csv_converter.launch']
    # bag_files = ["/home/minkbrook/Desktop/scenario_10_1_MOA_t4_f3.0_g4.0_h1.0_T15_D15_A10_2022-01-26-09-08-48.bag", 
    #     '/home/minkbrook/Desktop/scenario_30_1_APF_t1_2022-01-25-02-28-06.bag']

    bag_files = ['/home/minkbrook/Desktop/scenario_30_1_APF_t4_2022-01-25-13-25-34.bag']


    roslaunch_args = []
    for bag_file in bag_files:
        roslaunch_args.append(["bag_file:={}".format(bag_file)])

    rospy.logwarn("cli arguments {}". format(roslaunch_args))

    convert_harness = ConvertHarness() # initialization   
    
    for roslaunch_arg in roslaunch_args:
        convert_harness.run_convert(launch_file, roslaunch_arg)