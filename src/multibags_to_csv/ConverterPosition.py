#!/usr/bin/env python

"""
    ConverterPosition.py

    - direct convert of .bag to .csv by reading bag data
"""

import rosbag
import rospy
import os
from obstacle_avoidance_ros_pkg.msg import running_time
import pandas as pd

robots = range(1, 31)
topic_name = "ais_info"

def convert_topic_to_csv(bag, bagfile_name, path):

    for idx in robots:
        # change topic
        # topic = '/robot_0/ais_info'
        topic  = '/robot_{}/{}'.format(idx, topic_name)
        column_names = ['timestamp', 'pose_x', 'pose_y']

        # file name extract
        # file_name_start_idx = bagfile_name.find('rand_scenario_')
        file_name_end_idx = bagfile_name.find('.bag')
        file_name_header = bagfile_name[:file_name_end_idx]
        # file_name_header = bagfile_name[file_name_start_idx:file_name_end_idx]

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
            pose_msg = msg.pose

            current_time = timestamp-firststamp

            df = df.append(
                {'timestamp': current_time,
                # 'pose_x': [pose_msg.pose.position.x, pose_msg.pose.position.y],
                'pose_x': pose_msg.pose.position.x,
                'pose_y': pose_msg.pose.position.y},
                ignore_index=True
            )

        # csv saver
        # df.to_csv(path + 'pose_{}_{}.csv'.format(idx, file_name_header))
        df.to_csv('{}_pose_{}.csv'.format(file_name_header,idx))



if __name__ == "__main__":
    # bag files are located
    path = '/home/minkbrook/Desktop/gazebo/'
    # path = '/media/minkbrook/Seagate Portable Drive/Mingi/accident/'

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