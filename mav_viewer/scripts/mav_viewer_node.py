#!/usr/bin/env python
import rospy
from mav_msgs.msg import State

from mav_viewer import MAV_Viewer as MAV
import numpy as np
import pyqtgraph as pg

class Viewer:
    def __init__(self):
        self.mav_viewer = MAV()
        rospy.init_node("simulator", log_level=rospy.DEBUG)
        self.pose_sub = rospy.Subscriber('true_states', State, self.state_callback, queue_size=1)

    def state_callback(self, msg):
        state = np.array([msg.pn, msg.pe, msg.h, msg.phi, msg.theta, msg.psi])
        self.mav_viewer.update(state)

    def run(self):
        while not rospy.is_shutdown():
            rospy.spin()

if __name__ == '__main__':
    """
    This is the visualization of the simulator
    """
    #Initialize class here:
    viewer = Viewer()
    pg.QtGui.QApplication.instance().exec_()
    # viewer.run() # not needed. The previous line performs the same function
