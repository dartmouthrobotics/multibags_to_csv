"""
python rename_topic.py --input_bag input.bag --output_bag output.bag
"""

import gflags
import sys

from rosbag import Bag

gflags.DEFINE_string("input_bag", "bag.bag", "Path to the bagfile.")
gflags.DEFINE_string("output_bag", "output.bag", "Output bag path.")

if __name__ == '__main__':
    # Parse flags for gflags.
    try:
        argv = gflags.FLAGS(sys.argv)
    except gflags.FlagsError as e:
        print('%s\\nUsage: %s ARGS\\n%s' % (e, sys.argv[0], gflags.FLAGS))
        sys.exit(1)

    with Bag(gflags.FLAGS.output_bag, 'w') as outbag:
        for topic, msg, t in Bag(gflags.FLAGS.input_bag):
            if 'odom' in topic:
                msg.header.frame_id = msg.header.frame_id[1:]
            outbag.write(topic, msg, t)
