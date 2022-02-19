#!/usr/bin/env python

"""
    ConverterRuntime.py

    - Class definition of 'ConverterRuntime' 
    - basic modular part of converting /running_time topic 
    
    reference: https://answers.ros.org/question/55037/is-there-a-way-to-save-all-rosbag-data-into-a-csv-or-text-file/
"""

import rosbag
import rospy
import os
from obstacle_avoidance_ros_pkg.msg import running_time
import pandas as pd




def convert_topic_to_csv(bag, bagfile_name, path):

    # change topic
    topic = '/running_time'
    column_names = ['timestamp', 'runtime']

    # file name extract
    file_name_start_idx = bagfile_name.find('rand_scenario_')
    file_name_end_idx = bagfile_name.find('.')
    file_name_header = bagfile_name[file_name_start_idx:file_name_end_idx]

    # pandas build
    df = pd.DataFrame(columns=column_names)

    first_msg = False

    # each msg extrat and append to DF
    for topic, msg, t in bag.read_messages(topics=topic):
        firststamp= 0 

        if not first_msg:
            firststamp = msg.header.stamp.to_sec()
            first_msg = True

        timestamp = msg.header.stamp.to_sec()
        runtime = msg.run_time

        current_time = timestamp-firststamp

        df = df.append(
            {'timestamp': current_time,
            'runtime': runtime},
            ignore_index=True
        )

    # csv saver
    df.to_csv(path + 'RUNTIME_{}.csv'.format(file_name_header))




if __name__ == "__main__":
    # bag files are located
    path = '/home/minkbrook/Desktop/random/'
    bagfiles = []
    for r, d, f in os.walk(path):
        for file in f:
            if '.bag' in file:
                bagfiles.append(os.path.join(r, file))


    print("bagfiles {}".format(bagfiles))

    # function
    for i, bagfile_name in enumerate(bagfiles):
        print("converting start ...............................")
        bag = rosbag.Bag(bagfile_name)
        convert_topic_to_csv(bag, bagfile_name, path)

        print("runningtime bag to csv converted {} / {}".format(i+1, len(bagfiles)))
        print("...................................")