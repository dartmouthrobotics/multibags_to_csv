#!/usr/bin/env python 

"""
    live_plot
    
    purpose: 
    - live_plot for x: time, y: value
"""


# essential python modules
import matplotlib
# matplotlib.use('GTKAgg') # backend choice for speed up / before pyplot
import matplotlib.pyplot as plt
import matplotlib.colors as colors
from matplotlib.projections import get_projection_class
from mpl_toolkits.mplot3d import Axes3D # need for 3D plot
import numpy.ma as ma
import numpy as np


import time
import rospy, rostopic
from std_msgs.msg import Float64



class GraphVisualizer:
    def __init__(self):
    
        # plot configure
        self.fig = plt.figure(figsize=(8,6))
        self.to_draw_ax = self.fig.add_subplot(1,1,1)
        plt.ion() # essential

        self.start_time = None
        self.compass_hdg = None
        self.compass_sub = rospy.Subscriber("mavros/global_position/compass_hdg",
                                            Float64, self.compass_callback, queue_size=1)
        

        self.x_data_list = []
        self.y_data_list = []

        self.move_figure(self.fig, 910, 320)

    def compass_callback(self, msg):
        self.compass_hdg = msg.data

    def move_figure(self, f, x, y):
        """Move figure's upper left corner to pixel (x, y)"""
        backend = matplotlib.get_backend()
        if backend == 'TkAgg':
            f.canvas.manager.window.wm_geometry("+%d+%d" % (x, y))
        elif backend == 'WXAgg':
            f.canvas.manager.window.SetPosition((x, y))
        else:
            # This works for QT and GTK
            # You can also use window.setGeometry
            f.canvas.manager.window.move(x, y)
        

    def live_plot(self, to_draw_data):
        if to_draw_data is not None:
            if self.start_time is None:
                self.start_time = rospy.Time.now().to_sec()
            # self.to_draw_ax.clear()

            print("Here")
            # data preparation
            current_time = rospy.Time.now().to_sec()
            x_data = current_time - self.start_time
            y_data = to_draw_data

            self.x_data_list.append(x_data)
            self.y_data_list.append(y_data)

            active_action_begin = 5.2
            if x_data > active_action_begin:
                plt.vlines(x = active_action_begin, ymin = 90, ymax = 150, linestyles="dashed", linewidth=2,  color = 'g', zorder=5)
                plt.text(active_action_begin, 137, 'active action', ha='center', va='center',rotation='horizontal', color = 'g', fontsize=18)

            # configure plot
            # ---------------------------------------------------
            #  compass
            self.to_draw_ax.set_xlabel('Operation time (sec)', fontsize=18)
            self.to_draw_ax.set_ylabel('Compass heading (deg)', fontsize=18)
            self.to_draw_ax.tick_params(axis='both', which='major', labelsize=14)
            self.to_draw_ax.set_ylim(100, 135)
            # ---------------------------------------------------


            # draw plot
            self.to_draw_ax.plot(self.x_data_list, self.y_data_list, color="red")
            plt.show()
            plt.pause(0.00001)

            self.to_draw_ax.cla()


    def spin(self):
        self.live_plot(self.compass_hdg)
        rospy.Rate(5).sleep()


def main():
    graph_visualizer = GraphVisualizer()

    while not rospy.is_shutdown():
        graph_visualizer.spin()


if __name__ == "__main__":
    rospy.init_node("live_plot")
    rospy.sleep(1)
    main()