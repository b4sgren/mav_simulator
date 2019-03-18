#!/usr/bin/env python

import rospy
from dynamics.msg import State

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
        self.points, self.mesh_colors = self.getMAVPoints()

        #Set up subscribers
        #need to create the State message
        self.pose_sub = rospy.Subscriber('true_state', State, self.state_callback, queue_size = 1)

    def state_callback(self, msg):
        mav_position = np.array([[msg.pn, msg.pe, -msg.h]]).T
        R = self.EulerToRotation(msg.phi, msg.theta, msg.psi)
        rotated_pts = self.rotatePoints(self.points, R)
        trans_pts = self.translatePoints(rotated_pts, mav_position)

        R2 = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]]) # Convert to ENU coordinates for rendering
        trans_pts = np.dot(R2, trans_pts)
        mesh = self.pointsToMesh(trans_pts)

        if not self.plot_initialize:
            self.body = gl.GLMeshItem(vertexes=mesh, #defines mesh (Nx3x3)
                                      vertexColors=self.mesh_colors,
                                      drawEdges=True,
                                      smooth=False, #speeds up rendering
                                      computeNormals=False) # speeds up rendering
            self.window.addItem(self.body)
            self.plot_initialize = True
        else:
            self.body.setMeshData(vertexes=mesh, vertexColors=self.mesh_colors)

        view_location = Vector(state.pe, state.pn, state.h) # in ENU frame
        self.window.opts['center'] = view_location

        self.application.processEvents() #redraw

    def getMAVPoints(self):
        points = np.array([[3.5, 0.0, 0.0],
                           [2.0, 1.0, -1.0],
                           [2.0, -1.0, -1.0],
                           [2.0, -1.0, 1.0],
                           [2.0, 1.0, 1.0],
                           [-7.0, 0.0, 0.0],
                           [0.0, 5.0, 0.0],
                           [-2.0, 5.0, 0.0],
                           [-2.0, -5.0, 0.0],
                           [0.0, -5.0, 0.0],
                           [-5.0, 3.0, 0.0],
                           [-7.0, 3.0, 0.0],
                           [-7.0, -3.0, 0.0],
                           [-5.0, -3.0,0.0],
                           [-5.0, 0.0, 0.0],
                           [-7.0, 0.0, -3.0]]).T
        scale = 2.5
        points = scale * points

        red = np.array([1.0, 0.0, 0.0, 1])
        green = np.array([0.0, 1.0, 0.0, 1])
        blue = np.array([0.0, 0.0, 1.0, 1])
        yellow = np.array([1.0, 1.0, 0.0, 1])
        mesh_colors = np.empty((13, 3, 4), dtype=np.float32)
        mesh_colors[0] = yellow
        mesh_colors[1] = yellow
        mesh_colors[2] = yellow
        mesh_colors[3] = yellow
        mesh_colors[4] = blue
        mesh_colors[5] = blue
        mesh_colors[6] = blue
        mesh_colors[7] = blue
        mesh_colors[8] = red
        mesh_colors[9] = red
        mesh_colors[10] = red
        mesh_colors[11] = red
        mesh_colors[12] = green

        return points, mesh_colors

    def translatePoints(self, points, T):
        trans_pts = points + np.dot(T, np.ones((1, points.shape[1])))
        return trans_pts

    def rotatePoints(self, points, R):
        rotated_pts = np.dot(R, points)
        return rotated_pts

    def pointsToMesh(self, points):
        points = points.T
        mesh = np.array([[points[0], points[1], points[2]],
                         [points[0], points[1], points[4]],
                         [points[0], points[4], points[3]],
                         [points[0], points[3], points[2]],
                         [points[1], points[2], points[5]],
                         [points[2], points[3], points[5]],
                         [points[3], points[4], points[5]],
                         [points[4], points[1], points[5]],
                         [points[7], points[6], points[9]],
                         [points[7], points[8], points[9]],
                         [points[11], points[10], points[13]],
                         [points[11], points[12], points[13]],
                         [points[5], points[14], points[15]]])
        return mesh

    def EulerToRotation(self, phi, theta, psi):
        c_psi = np.cos(psi)
        s_psi = np.sin(psi)
        c_theta = np.cos(theta)
        s_theta = np.sin(theta)
        c_phi = np.cos(phi)
        s_phi = np.sin(phi)

        Rz = np.array([[c_psi, s_psi, 0.0],
                       [-s_psi, c_psi, 0.0],
                       [0.0, 0.0, 1.0]])
        Ry = np.array([[c_theta, 0.0, -s_theta],
                       [0.0, 1.0, 0.0],
                       [s_theta, 0.0, c_theta]])
        Rx = np.array([[1.0, 0.0, 0.0],
                       [0.0, c_phi, s_phi],
                       [0.0, -s_phi, c_phi]])

        R = np.dot(np.dot(Rx, Ry), Rz) # this is the rotation from the inertial to body
        return R.T  # Return transpose to take body frame to inertial

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
    mav = MAV_Viewer()
    mav.run() # will stay commented out while building rest of class

    print "Done"
