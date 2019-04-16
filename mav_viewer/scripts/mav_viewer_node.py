#!/usr/bin/env python
import rospy
from dynamics.msg import State

from mav_viewer import MAV_Viewer as MAV
import numpy as np
# import pyqtgraph.opengl as gl
import pyqtgraph as pg

from threading import Thread

class Viewer:
    def __init__(self):
        self.mav_viewer = MAV()
        rospy.init_node("simulator", log_level=rospy.DEBUG) #initialize at end of constructor
        self.pose_sub = rospy.Subscriber('true_states', State, self.state_callback, queue_size=1)
        pg.QtGui.QApplication.instance().exec_()

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

    #set this so that mav_viewer is main window. Create separate threads inside of it to run other ros nodes.
    print "Initializing node"

    #Initialize class here:
    viewer = Viewer()
    thread = Thread(target=viewer.run()) # try running this in a separate thread
    thread.start()

print "Done"
