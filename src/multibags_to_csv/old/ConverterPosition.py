#!/usr/bin/env python

"""
    ConverterPosition.py

    - direct convert of .bag to .csv by reading bag data
"""

import rosbag
import rospy
import os
import pandas as pd
import argparse
import yaml

from aux_function import extract_robot_number

####
# Read YAML file
with open("../../param/indiv-converter.yaml", "r") as stream:
    data_loaded = yaml.safe_load(stream)
    print("data_loaded {}".format(data_loaded))

topic_name = data_loaded["topic_name"]["pose"]
robots_num = data_loaded["total_robot"]
total_robot_extract_by_bag = data_loaded["total_robot_extract_by_bag"]
topic_ns = data_loaded["topic_ns"]
time_base_by_bag = data_loaded["time_base_by_bag"]
path = ""  # bag files are located

####

# TODO
# wrapper function in main --> handle pose, gps posiion, boundary everything
# get robot name based on recorded topic --> get index
# script separate --> combine


def convert_topic_to_csv(bag, bagfile_name, path):
    global robots_num
    # total robot number find
    if total_robot_extract_by_bag:
        robots_num = extract_robot_number(bagfile_name)

    ######  file name extract
    # file_name_start_idx = bagfile_name.find('rand_scenario_')
    file_name_end_idx = bagfile_name.find(".bag")
    file_name_header = bagfile_name[:file_name_end_idx]
    # file_name_header = bagfile_name[file_name_start_idx:file_name_end_idx]

    # folder generate
    if not os.path.exists(file_name_header):
        os.makedirs(file_name_header)
        print("generated directory: {}".format(file_name_header))

    bag_start_time = bag.get_start_time()
    bag_end_time = bag.get_end_time()
    print("bag file time {}".format((bag_end_time - bag_start_time)))

    ######  each robot extract
    for idx in range(robots_num):
        # change topic
        # topic = '/{}_0/{}'.format(topic_ns, topic_name) # incase only ego-vehicle
        topic = "/{}_{}/{}".format(topic_ns, idx, topic_name)
        column_names = ["timestamp", "pose_x", "pose_y"]

        # pandas build
        df = pd.DataFrame(columns=column_names)

        first_msg_built = False

        # each msg extrat and append to DF
        for topic, msg, t in bag.read_messages(topics=topic):

            timestamp = msg.header.stamp.to_sec()
            pose_msg = msg.pose

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

            # append deperecated
            df = pd.concat(
                [
                    df,
                    pd.DataFrame(
                        [
                            {
                                "timestamp": current_time,
                                "obj_ID": idx,
                                # 'pose_x': [pose_msg.pose.position.x, pose_msg.pose.position.y],
                                "pose_x": pose_msg.pose.position.x,
                                "pose_y": pose_msg.pose.position.y,
                            }
                        ]
                    ),
                ],
                ignore_index=True,
            )

        # csv saver
        df.to_csv("{}/pose_{}.csv".format(file_name_header, idx))  # file_name_header as new foler


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

            print("pose bag to csv converted {} / {}".format(i + 1, len(bagfiles)))
            print("...................................")

        except Exception as e:
            print("failed to convert {}".format(e))
