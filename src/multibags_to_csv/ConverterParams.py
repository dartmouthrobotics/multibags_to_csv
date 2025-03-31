#!/usr/bin/env python

"""
    ConverterParams.py

    - direct convert of .bag to .csv by reading bag data

    # Usage:
    - change yaml file first! 
    - change PARAM_INTEREST
     - python ConverterParams.py -p /mnt/mydrive/catabot_bag/barbados2024/test/test/
"""

import rosbag
import rospy
import os
import pandas as pd
import argparse
import yaml


from aux_function import extract_robot_number
from test_reader import ros2api_reader_lstm

# PARAM_INTEREST = ["pose", "boundary", "gps", "runtime"]
# PARAM_INTEREST = ["lstm"]
# PARAM_INTEREST = ["pose", "lstm"]
PARAM_INTEREST = ["imu", "gps"]

# TODO
# get robot name based on recorded topic --> get index

# reference: https://wiki.ros.org/rosbag/Cookbook


def read_yaml(_param_interest):
    """read yaml for parameter we will extract"""
    with open("../../param/indiv-converter.yaml", "r") as stream:
        data_loaded = yaml.safe_load(stream)
        print("data_loaded {}".format(data_loaded))

    topic_name = data_loaded["topic_name"][_param_interest]  # str
    robots_num = data_loaded["total_robot"]  # int
    total_robot_extract_by_bag = data_loaded["param_set"]["total_robot_extract_by_bag"]  # bool
    time_base_by_bag = data_loaded["param_set"]["time_base_by_bag"]  # bool
    name_space_flag = data_loaded["param_set"]["name_space_flag"]  # bool
    topic_ns = data_loaded["topic_ns"]  # str

    return topic_name, robots_num, total_robot_extract_by_bag, topic_ns, time_base_by_bag, name_space_flag


def time_mapper(t, bag_start_time, msg, time_base_by_bag, first_msg_built):
    """get time for recording"""
    if time_base_by_bag:
        # bag time-based rather than msg.stamp
        current_time = t.to_sec() - bag_start_time

    else:
        firststamp = 0

        # TODO by first message time stamp
        # strictly speaking, it was not synced across robots (for first start time)
        if not first_msg_built:
            firststamp = msg.header.stamp.to_sec()
            first_msg_built = True

        timestamp = msg.header.stamp.to_sec()
        current_time = timestamp - firststamp

    return current_time


def get_param_column(param_interest, name_space_flag):
    """
    Args:
        - param_interest (str): param to be extracted, e.g., pose, gps
    """
    if param_interest == "pose":
        column_names = ["timestamp", "obj_ID", "pose_x", "pose_y"]

    elif param_interest == "boundary":
        column_names = [
            "timestamp",
            "obj_ID",
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

    elif param_interest == "gps":
        column_names = ["timestamp", "obj_ID", "latitude", "longitude"]

    elif param_interest == "runtime":
        column_names = ["timestamp", "runtime"]

    elif param_interest == "lstm":
        column_names = ["timestamp", "obj_ID", "lstm_l", "lstm_r"]

    elif param_interest == "imu":
        column_names = ["timestamp", "obj_ID", "lin_x", "lin_y", "lin_z", "ang_x", "ang_y", "ang_z"]



    return column_names


def append_param_df(df, param_interest, msg, current_time, idx, name_space_flag):
    """
    depending on param interst --> append df
    """
    if param_interest == "pose":
        # append deperecated
        df = pd.concat(
            [
                df,
                pd.DataFrame(
                    [
                        {
                            "timestamp": current_time,
                            "obj_ID": idx,
                            "pose_x": msg.pose.pose.position.x,
                            "pose_y": msg.pose.pose.position.y,
                        }
                    ]
                ),
            ],
            ignore_index=True,
        )

    elif param_interest == "boundary":

        # append deperecated
        df = pd.concat(
            [
                df,
                pd.DataFrame(
                    [
                        {
                            "timestamp": current_time,
                            "obj_ID": idx,
                            "boundary_pose_x": msg.pose.position.x,
                            "boundary_pose_y": msg.pose.position.y,
                            "boundary_pose_z": msg.pose.position.z,
                            "boundary_orient_x": msg.pose.orientation.x,
                            "boundary_orient_y": msg.pose.orientation.y,
                            "boundary_orient_z": msg.pose.orientation.z,
                            "boundary_orient_w": msg.pose.orientation.w,
                            "boundary_scale_x": msg.scale.x,
                            "boundary_scale_y": msg.scale.y,
                        }
                    ]
                ),
            ],
            ignore_index=True,
        )
    elif param_interest == "gps":
        df = pd.concat(
            [
                df,
                pd.DataFrame(
                    [
                        {
                            "timestamp": current_time,
                            "obj_ID": idx,
                            "latitude": msg.latitude,
                            "longitude": msg.longitude,
                        }
                    ]
                ),
            ],
            ignore_index=True,
        )

    elif param_interest == "runtime":
        df = pd.concat(
            [
                df,
                pd.DataFrame(
                    [
                        {"timestamp": current_time, "runtime": msg.run_time},
                    ]
                ),
            ],
            ignore_index=True,
        )

    elif param_interest == "lstm":
        df = pd.concat(
            [
                df,
                pd.DataFrame(
                    [
                        {
                            "timestamp": current_time,
                            "obj_ID": idx,
                            "lstm_l": msg.items[int(idx) - 1].probability[0],
                            "lstm_r": msg.items[int(idx) - 1].probability[1],
                        },
                    ]
                ),
            ],
            ignore_index=True,
        )

    elif param_interest == "imu":
        df = pd.concat(
            [
                df,
                pd.DataFrame(
                    [
                        {
                            "timestamp": current_time,
                            "obj_ID": idx,
                            "lin_x": msg.linear_acceleration.x, 
                            "lin_y": msg.linear_acceleration.y, 
                            "lin_z": msg.linear_acceleration.z, 
                            "ang_x": msg.angular_velocity.x, 
                            "ang_y": msg.angular_velocity.y, 
                            "ang_z": msg.angular_velocity.z
                        }
                    ]
                ),
            ],
            ignore_index=True,
        )

    if not name_space_flag:
        df = df.drop('obj_ID', axis=1) # obj_ID column drop

    return df


def convert_topic_to_csv(bag, bagfile_name, path, _param_interest):

    # -----------------------------------------------------
    #### yaml read
    topic_name, robots_num, total_robot_extract_by_bag, topic_ns, time_base_by_bag, name_space_flag = read_yaml(
        _param_interest
    )
    # total robot number find
    if total_robot_extract_by_bag:
        robots_num = extract_robot_number(bagfile_name)

    # -----------------------------------------------------

    # -----------------------------------------------------
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

    # -----------------------------------------------------

    if _param_interest == "lstm":
        topic = "/{}_0/{}".format(topic_ns, topic_name) if name_space_flag else "/{}".format(topic_name) # incase only ego-vehicle\
        print("topic name {}".format(topic))

        # ros2 api-based saver
        ros2api_reader_lstm(bagfile_name, topic, robots_num, file_name_header, _param_interest, name_space_flag)

    else:
        # -----------------------------------------------------
        ######  each robot extract
        for idx in range(robots_num + 1):
            # change topic
            # topic = '/{}_0/{}'.format(topic_ns, topic_name) # incase only ego-vehicle
            topic = "/{}_{}/{}".format(topic_ns, idx, topic_name) if name_space_flag else "/{}".format(topic_name)
            print("topic name {}".format(topic))

            # pandas build
            column_names = get_param_column(param_interest=_param_interest, name_space_flag=name_space_flag)
            df = pd.DataFrame(columns=column_names)

            first_msg_built = False

            # -----------------------------------------------------

            #### each msg extrat and append to DF
            for topic, msg, t in bag.read_messages(topics=topic):
                current_time = time_mapper(
                    t, bag_start_time, msg, time_base_by_bag, first_msg_built
                )
                df = append_param_df(
                    df, param_interest=_param_interest, msg=msg, current_time=current_time, idx=idx, name_space_flag=name_space_flag
                )

            # csv saver
            df.to_csv(
                "{}/{}_{}.csv".format(file_name_header, _param_interest, idx)
            )  # file_name_header as new foler
            if _param_interest == "runtime":
                break

            # -----------------------------------------------------


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

    ##### iterate over each bag file and then each param
    for i, bagfile_name in enumerate(bagfiles):
        print("converting start {}...............................".format(bagfile_name))
        bag = rosbag.Bag(bagfile_name)
        try:
            for _param_interest in PARAM_INTEREST:
                convert_topic_to_csv(bag, bagfile_name, path, _param_interest)

                print(
                    "{} bag to csv converted {} / {}".format(_param_interest, i + 1, len(bagfiles))
                )
                print("...................................")

        except Exception as e:
            print("failed to convert {}".format(e))
