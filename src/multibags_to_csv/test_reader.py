from __future__ import annotations

from typing import TYPE_CHECKING

from rosbags.rosbag1 import Reader
from rosbags.serde import deserialize_cdr, ros1_to_cdr, deserialize_ros1
from rosbags.typesys import get_types_from_msg, register_types

import pandas as pd

from pathlib import Path

# reference:
# https://ternaris.gitlab.io/rosbags/topics/serde.html
# https://stackoverflow.com/questions/73420147/how-to-read-custom-message-type-using-ros2bag
# https://ternaris.gitlab.io/rosbags/examples/register_types.html


# get message type
def guess_msgtype(path: Path) -> str:
    """Guess message type name from path."""
    name = path.relative_to(path.parents[2]).with_suffix("")
    if "msg" not in name.parts:
        name = name.parent / "msg" / name.name
    return str(name)


# custom message type register
add_types = {}
for pathstr in [
    "/home/mingi/vnc-ros-noetic/workspace/src/passing_intention_lstm_ros/msg/DictionaryLSTMMsg.msg",
    "/home/mingi/vnc-ros-noetic/workspace/src/passing_intention_lstm_ros/msg/KeyValueLSTMMsg.msg",
]:
    msgpath = Path(pathstr)
    msgdef = msgpath.read_text(encoding="utf-8")
    print(guess_msgtype(msgpath))
    add_types.update(get_types_from_msg(msgdef, guess_msgtype(msgpath)))
register_types(add_types)

"""
# create reader instance
with Reader("/mnt/mydrive/catabot_bag/barbados2024/test/test/catabot-5_2024-02-20-09-47-16.bag") as reader:
    # topic and msgtype information is available on .connections list
    # for connection in reader.connections:
    #     print(connection.topic, connection.msgtype)

    # print("start time", reader.start_time)
    # print("end time", reader.end_time)
    # print("minus", reader.end_time - reader.start_time)
    # print("duration", reader.duration * (1e-9))

    # iterate over messages
    for connection, timestamp, rawdata in reader.messages():
        if connection.topic == "/robot_0/LSTM_out":
            # msg = deserialize_cdr(ros1_to_cdr(rawdata, connection.msgtype), connection.msgtype)
            msg = deserialize_ros1(rawdata, connection.msgtype)

            # LSTM message is valid
            if len(msg.items) > 0:
                for idx in range(len(msg.items)):
                    if msg.items[idx].key == idx + 1:
                        prob_l = msg.items[idx].probability[0]
                        prob_r = msg.items[idx].probability[1]

                        print("idx {} prob {}".format(idx, (prob_l, prob_r)))
                        # print(timestamp.to_sec())
                    # print(msg.items[idx].key == 1)
                    # print(msg.items[0].probability)
"""


def ros2api_reader_lstm(bagfile_name, topic_name, robots_num, file_name_header, _param_interest, name_space_flag):
    """
    bagfile_name (str): "/mnt/mydrive/catabot_bag/barbados2024/test/test/catabot-5_2024-02-20-09-47-16.bag"
    """
    column_names = ["timestamp", "obj_ID", "lstm_l", "lstm_r"]

    for idx in range(1, robots_num + 1):  # ego skip

        df = pd.DataFrame(columns=column_names)

        # create reader instance
        with Reader(bagfile_name) as reader:

            bag_start_time = reader.start_time

            # topic and msgtype information is available on .connections list
            # iterate over messages
            for connection, timestamp, rawdata in reader.messages():
                if connection.topic == topic_name:
                    msg = deserialize_ros1(rawdata, connection.msgtype)

                    # LSTM message is valid
                    if len(msg.items) > 0 and len(msg.items) >= idx:

                        if msg.items[idx - 1].key == idx:  # object id
                            prob_l = msg.items[idx - 1].probability[0]  # L
                            prob_r = msg.items[idx - 1].probability[1]  # R

                            current_time = (timestamp - bag_start_time) * (1e-9)
                            # print("idx {} prob {}".format(idx, (prob_l, prob_r)))
                            df = pd.concat(
                                [
                                    df,
                                    pd.DataFrame(
                                        [
                                            {
                                                "timestamp": current_time,
                                                "obj_ID": idx,
                                                "lstm_l": prob_l,
                                                "lstm_r": prob_r,
                                            },
                                        ]
                                    ),
                                ],
                                ignore_index=True,
                            )

                continue  # other topic skip

            if not name_space_flag:
                df = df.drop('obj_ID', axis=1) # obj_ID column drop

            # per object save
            df.to_csv(
                "{}/{}_{}.csv".format(file_name_header, _param_interest, idx)
            )  # file_name_header as new foler
