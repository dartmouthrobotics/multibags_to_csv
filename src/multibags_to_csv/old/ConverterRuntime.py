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
import yaml

####
# Read YAML file
with open("../../param/indiv-converter.yaml", "r") as stream:
    data_loaded = yaml.safe_load(stream)
    print("data_loaded {}".format(data_loaded))

topic_name = data_loaded["topic_name"]["runtime"]
robots_num = data_loaded["total_robot"]
total_robot_extract_by_bag = data_loaded["total_robot_extract_by_bag"]
topic_ns = data_loaded["topic_ns"]
time_base_by_bag = data_loaded["time_base_by_bag"]
path = ""  # bag files are located
####


def convert_topic_to_csv(bag, bagfile_name, path):
    column_names = ["timestamp", "runtime"]

    # file name extract
    file_name_start_idx = bagfile_name.find("rand_scenario_")
    file_name_end_idx = bagfile_name.find(".bag")
    file_name_header = bagfile_name[file_name_start_idx:file_name_end_idx]

    save_folder = bagfile_name[:file_name_end_idx]

    if not os.path.exists(save_folder):
        os.makedirs(save_folder)
        print("generated directory: {}".format(save_folder))

    bag_start_time = bag.get_start_time()
    bag_end_time = bag.get_end_time()
    print("bag file time {}".format((bag_end_time - bag_start_time)))

    # pandas build
    df = pd.DataFrame(columns=column_names)
    topic = topic_name
    first_msg_built = False

    # each msg extrat and append to DF
    for topic, msg, t in bag.read_messages(topics=topic):

        timestamp = msg.header.stamp.to_sec()
        runtime = msg.run_time

        if time_base_by_bag:
            current_time = t.to_sec() - bag_start_time

        else:
            firststamp = 0

            # TODO by first message time stamp
            # strictly speaking, it was not synced across robots
            if not first_msg_built:
                firststamp = msg.header.stamp.to_sec()
                first_msg_built = True

            current_time = timestamp - firststamp

        df = pd.concat(
            [
                df,
                pd.DataFrame(
                    [
                        {"timestamp": current_time, "runtime": runtime},
                    ]
                ),
            ],
            ignore_index=True,
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
    df.to_csv("{}/RUNTIME_{}.csv".format(save_folder, file_name_header))


if __name__ == "__main__":
    ##### Parser
    parser = argparse.ArgumentParser(description="test")

    # Adding optional argument
    parser.add_argument("-p", "--path_dir", help="path where bag file located")

    # Read arguments from command line
    args = parser.parse_args()

    path_arg = str(args.path_dir)
    if path_arg:
        path = path_arg

    ##### iteration of all bag files
    bagfiles = []
    for r, d, f in os.walk(path):
        for file in f:
            if ".bag" in file:
                bagfiles.append(os.path.join(r, file))

    print("bagfiles {}".format(bagfiles))

    # function
    for i, bagfile_name in enumerate(bagfiles):
        print("converting start {}...............................".format(bagfile_name))
        bag = rosbag.Bag(bagfile_name)
        try:
            convert_topic_to_csv(bag, bagfile_name, path)

            print("runningtime bag to csv converted {} / {}".format(i + 1, len(bagfiles)))
            print("...................................")

        except Exception as e:
            print("failed to convert {}".format(e))
