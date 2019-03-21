#!/usr/bin/env python
import rospy
from dynamics.msg import State

from mav_viewer import MAV_Viewer as MAV
import numpy as np

class Viewer:
    def __init__(self):
        self.mav_viewer = MAV()
        self.pose_sub = rospy.Subscriber('true_state', State, self.state_callback, queue_size=1)

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

    print "Initializing node"
    rospy.init_node("simulator", log_level=rospy.DEBUG) #use when running ros

    #Initialize class here:
    viewer = Viewer()
    viewer.run() # will stay commented out while building rest of class

print "Done"
