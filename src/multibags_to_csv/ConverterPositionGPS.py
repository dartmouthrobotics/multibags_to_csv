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

from aux_function import extract_robot_number

topic_name = "mavros/global_position/raw/fix" # 1) GPS-based
path = '' # bag files are located


def convert_topic_to_csv(bag, bagfile_name, path):

    # total robot number find 
    robots_num = 3
    print("robot number {}".format(robots_num))
    

    ######  file name extract
    # file_name_start_idx = bagfile_name.find('rand_scenario_')
    file_name_end_idx = bagfile_name.find('.bag')
    file_name_header = bagfile_name[:file_name_end_idx]
    # file_name_header = bagfile_name[file_name_start_idx:file_name_end_idx]

    # folder generate
    if not os.path.exists(file_name_header):
        os.makedirs(file_name_header)
        print("generated directory: {}".format(file_name_header))
    


    for idx in range(robots_num + 1):
        # change topic
        # topic = '/robot_0/ais_info' # incase only ego-vehicle
        topic  = '/robot_{}/{}'.format(idx, topic_name)
        column_names = ['timestamp','latitude', 'longitude']


        
        # pandas build
        df = pd.DataFrame(columns=column_names)

        first_msg_built = False

        # each msg extrat and append to DF
        for topic, msg, t in bag.read_messages(topics=topic):
            firststamp= 0 

            timestamp = msg.header.stamp.to_sec()
            # pose_msg = msg # gps
            pose_msg = msg.pose

            current_time = timestamp-firststamp

            # 1) GPS-based
            df  = pd.concat([df, 
                            pd.DataFrame([{'timestamp': current_time,
                            # 'pose_x': [pose_msg.pose.position.x, pose_msg.pose.position.y],
                            'latitude': pose_msg.latitude,
                            'longitude': pose_msg.longitude}])
                            ],
                            ignore_index = True)

            # 2) odom-based
            # df  = pd.concat([df, 
            #                 pd.DataFrame([{'timestamp': current_time,
            #                 # 'pose_x': [pose_msg.pose.position.x, pose_msg.pose.position.y],
            #                 'pose_x': pose_msg.pose.position.x,
            #                 'pose_y': pose_msg.pose.position.y}])
            #                 ],
            #                 ignore_index = True)


            if not first_msg_built:
                firststamp = msg.header.stamp.to_sec()
                first_msg_built = True


        # csv saver
        df.to_csv('{}/pose_{}.csv'.format(file_name_header,idx)) # file_name_header as new foler
    


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

            print("pose bag to csv converted {} / {}".format(i+1, len(bagfiles)))
            print("...................................")

        except Exception as e:
            print("failed to convert {}".format(e))