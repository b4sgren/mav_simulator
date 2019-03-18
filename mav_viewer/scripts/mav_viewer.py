#!/usr/bin/env python

import rospy
import numpy as np
import pyqtgraph as pg
import pyqtgraph.opengl as gl
import pyqtgraph.Vector as Vector

class MAV_Viewer:
    def __init__(self):
        self.application = pg.QtGui.QApplication([])
        self.window = gl.GLViewWidget()
        self.window.setWindowTitle('Flight Simulator')
        self.window.setGeometry(0, 0, 750, 750)
        grid = gl.GLGridItem()
        grid.scale(20, 20, 20)
        self.window.addItem(grid)
        self.window.setCameraPosition(distance=200)
        self.window.setBackgroundColor('k')
        self.window.show()
        self.window.raise_()
        self.plot_initialize = False
        # self.points, self.mesh_colors = self.getMAVPoints()

    def run(self):
        while not rospy.is_shutdown():
            rospy.spin()


if __name__ == '__main__':
    """
    This is the visualization of the simulator
    """

    print "Initializing node"

    #Initialize class here:
    mav = MAV_Viewer()
    # mav.run() # will stay commented out while building rest of class
    print "Done"
