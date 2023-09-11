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
import argparse

# bag files are located
path = ''

topic_name = '/robot_0/running_time'

def convert_topic_to_csv(bag, bagfile_name, path):
    column_names = ['timestamp', 'runtime']


    # file name extract
    file_name_start_idx = bagfile_name.find('rand_scenario_')
    file_name_end_idx = bagfile_name.find('.bag')
    file_name_header = bagfile_name[file_name_start_idx:file_name_end_idx]

    save_folder = bagfile_name[:file_name_end_idx]


    if not os.path.exists(save_folder):
        os.makedirs(save_folder)
        print("generated directory: {}".format(save_folder))
    
    
    # pandas build
    df = pd.DataFrame(columns=column_names)

    topic = topic_name

    first_msg_built = False

    # each msg extrat and append to DF
    for topic, msg, t in bag.read_messages(topics=topic):
        firststamp= 0 

        timestamp = msg.header.stamp.to_sec()
        runtime = msg.run_time

        current_time = timestamp-firststamp

        df = pd.concat([
                df,
                pd.DataFrame([
                    {'timestamp': current_time,
                    'runtime': runtime},
                ])]
                ,ignore_index=True
            )
        # df = df.append(
        #     {'timestamp': current_time,
        #     'runtime': runtime},
        #     ignore_index=True
        # )


        if not first_msg_built:
            firststamp = msg.header.stamp.to_sec()
            first_msg_built = True


    # csv saver
    df.to_csv('{}/RUNTIME_{}.csv'.format(save_folder,file_name_header))




if __name__ == "__main__":
    ##### Parser
    parser = argparse.ArgumentParser(description="test")

    # Adding optional argument
    parser.add_argument("-p", "--path_dir", help = "path where bag file located")

    # Read arguments from command line
    args = parser.parse_args()

    path_arg = str(args.path_dir)
    if path_arg:
        path = path_arg


    ##### iteration of all bag files
    bagfiles = []
    for r, d, f in os.walk(path):
        for file in f:
            if '.bag' in file:
                bagfiles.append(os.path.join(r, file))


    print("bagfiles {}".format(bagfiles))

    # function
    for i, bagfile_name in enumerate(bagfiles):
        print("converting start {}...............................".format(bagfile_name))
        bag = rosbag.Bag(bagfile_name)
        try:
            convert_topic_to_csv(bag, bagfile_name, path)

            print("runningtime bag to csv converted {} / {}".format(i+1, len(bagfiles)))
            print("...................................")

        except Exception as e:
            print("failed to convert {}".format(e))