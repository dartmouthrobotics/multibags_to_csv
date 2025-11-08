#!/usr/bin/env python

"""
    ConverterBoundary.py

    - direct convert of .bag to .csv by reading bag data
"""
import argparse
import rosbag
import rospy
import os
from obstacle_avoidance_ros_pkg.msg import boundary_info
import pandas as pd
import yaml

from aux_function import extract_robot_number

####
# Read YAML file
with open("../../param/indiv-converter.yaml", "r") as stream:
    data_loaded = yaml.safe_load(stream)
    print("data_loaded {}".format(data_loaded))

topic_name = data_loaded["topic_name"]["boundary"]
robots_num = data_loaded["total_robot"]
total_robot_extract_by_bag = data_loaded["total_robot_extract_by_bag"]
topic_ns = data_loaded["topic_ns"]
time_base_by_bag = data_loaded["time_base_by_bag"]
path = ""  # bag files are located
####


def convert_topic_to_csv(bag, bagfile_name, path):
    global robots_num
    # total robot number find
    if total_robot_extract_by_bag:
        robots_num = extract_robot_number(bagfile_name)
    print("robot number {}".format(robots_num))

    ######  file name extract
    file_name_end_idx = bagfile_name.find(".bag")
    file_name_header = bagfile_name[:file_name_end_idx]

    # folder generate
    if not os.path.exists(file_name_header):
        os.makedirs(file_name_header)
        print("generated directory: {}".format(file_name_header))

    bag_start_time = bag.get_start_time()
    bag_end_time = bag.get_end_time()
    print("bag file time {}".format((bag_end_time - bag_start_time)))

    for idx in range(robots_num + 1):
        # change topic
        # topic = '/{}_0/{}'.format(topic_ns, topic_name) # incase only ego-vehicle
        topic = "/{}_{}/{}".format(topic_ns, idx, topic_name)
        column_names = [
            "timestamp",
            "boundary_pose_x",
            "boundary_pose_y",
            "boundary_pose_z",
            "boundary_orient_x",
            "boundary_orient_y",
            "boundary_orient_z",
            "boundary_orient_w",
            "boundary_scale_x",
            "boundary_scale_y",
        ]

        # pandas build
        df = pd.DataFrame(columns=column_names)

        first_msg_built = False

        # each msg extrat and append to DF
        for topic, msg, t in bag.read_messages(topics=topic):

            timestamp = msg.header.stamp.to_sec()
            boundary_msg = msg

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

            # if we want time distcrete
            # if first_msg_built and \
            # abs(current_time - df['timestamp'].iloc[-1]) < 0.5:
            #     continue

            # append deperecated
            df = pd.concat(
                [
                    df,
                    pd.DataFrame(
                        [
                            {
                                "timestamp": current_time,
                                # 'pose_x': [pose_msg.pose.position.x, pose_msg.pose.position.y],
                                "boundary_pose_x": boundary_msg.pose.position.x,
                                "boundary_pose_y": boundary_msg.pose.position.y,
                                "boundary_pose_z": boundary_msg.pose.position.z,
                                "boundary_orient_x": boundary_msg.pose.orientation.x,
                                "boundary_orient_y": boundary_msg.pose.orientation.y,
                                "boundary_orient_z": boundary_msg.pose.orientation.z,
                                "boundary_orient_w": boundary_msg.pose.orientation.w,
                                "boundary_scale_x": boundary_msg.scale.x,
                                "boundary_scale_y": boundary_msg.scale.y,
                            }
                        ]
                    ),
                ],
                ignore_index=True,
            )

        # csv saver
        df.to_csv("{}/boundary_{}.csv".format(file_name_header, idx))  # file_name_header as new foler


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

            print("boundary bag to csv converted {} / {}".format(i + 1, len(bagfiles)))
            print("...................................")

        except Exception as e:
            print("failed to convert {}".format(e))
