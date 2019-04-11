#!/usr/bin/env python3
from __future__ import division

import rospy
from dynamics.msg import State
import numpy as np

from uav_plotter.Plotter import Plotter
from uav_plotter.plotter_args import PlotboxArgs, PlotArgs

class PlotWrapper:
    """
    PlotWrapper class for connecting ROS topics to the Plotter object
    """
    def __init__(self, update_freq=30, use_degrees=True):

        # Store parameters
        self.use_degrees = use_degrees

        # Setup the plotter window
        plotting_freq = 4
        time_window = 100
        self.plotter = Plotter(plotting_freq,time_window)

        # Define plots
        plots = self._define_plots()

        # Add plots to the window
        self.plotter.add_plotboxes(plots)

        # Define input vectors for easier input
        self.plotter.define_input_vector('true_state', ['pn','pe','h','Va',
                  'alpha','beta','phi','theta','chi','p','q','r','Vg','wn',
                  'we','psi','bx','by','bz'])
        self.plotter.define_input_vector('estimated_state',['pn_e','pe_e','h_e',
                  'Va_e','alpha_e','beta_e','phi_e','theta_e','chi_e','p_e',
                  'q_e','r_e','Vg_e','wn_e','we_e','psi_e','bx_e','by_e','bz_e'])
        self.plotter.define_input_vector('commands',['h_c','Va_c','phi_c',
                  'theta_c','chi_c'])

        # Subscribe to relevant ROS topics
        rospy.Subscriber('true_state', State, self.statesCallback)
        rospy.Subscriber('estimated_state', State, self.estimatesCallback)
        rospy.Subscriber('commanded_state', State, self.cmdCallback)

        # Update the plots
        rate = rospy.Rate(update_freq)
        while not rospy.is_shutdown():
            self.plotter.update_plots()
            rate.sleep()

    def _define_plots(self):
        pn_plots = PlotboxArgs(plots=['pn','pn_e'],
                               labels={'left': 'pn (m)','bottom': 'Time (s)'})
        pe_plots = PlotboxArgs(plots=['pe','pe_e'],
                               labels={'left': 'pe (m)','bottom': 'Time (s)'})
        h_plots = PlotboxArgs(plots=['h','h_e','h_c'],
                               labels={'left': 'h (m)','bottom': 'Time (s)'})
        wind_plots = PlotboxArgs(plots=['wn','wn_e','we','we_e'],
                             labels={'left': 'wind (m/s)','bottom': 'Time (s)'})
        first_row = [pn_plots, pe_plots, h_plots, wind_plots]

        Va_plots = PlotboxArgs(plots=['Va','Va_e','Va_c'],
                               labels={'left': 'Va (m/s)','bottom': 'Time (s)'})
        alpha_plots = PlotboxArgs(plots=['alpha','alpha_e'],
                            labels={'left': 'alpha (deg)','bottom': 'Time (s)'},
                            rad2deg=self.use_degrees)
        beta_plots = PlotboxArgs(plots=['beta','beta_e'],
                            labels={'left': 'beta (deg)','bottom': 'Time (s)'},
                            rad2deg=self.use_degrees)
        Vg_plots = PlotboxArgs(plots=['Vg','Vg_e'],
                            labels={'left': 'Vg (m/s)','bottom': 'Time (s)'})
        second_row = [Va_plots, alpha_plots, beta_plots, Vg_plots]

        phi_plots = PlotboxArgs(plots=['phi','phi_e','phi_c'],
                            labels={'left':'phi (deg)','bottom':'Time (s)'},
                            rad2deg=self.use_degrees)
        theta_plots = PlotboxArgs(plots=['theta','theta_e','theta_c'],
                            labels={'left':'theta (deg)','bottom':'Time (s)'},
                            rad2deg=self.use_degrees)
        psi_plots = PlotboxArgs(plots=['psi','psi_e'],
                            labels={'left': 'psi (deg)','bottom': 'Time (s)'},
                            rad2deg=self.use_degrees)
        chi_plots = PlotboxArgs(plots=['chi','chi_e','chi_c'],
                            labels={'left': 'chi (deg)','bottom': 'Time (s)'},
                            rad2deg=self.use_degrees)
        third_row = [phi_plots, theta_plots, psi_plots, chi_plots]

        p_plots = PlotboxArgs(plots=['p','p_e'],
                             labels={'left': 'p (deg/s)','bottom': 'Time (s)'},
                             rad2deg=self.use_degrees)
        q_plots = PlotboxArgs(plots=['q','q_e'],
                             labels={'left': 'q (deg/s)','bottom': 'Time (s)'},
                             rad2deg=self.use_degrees)
        r_plots = PlotboxArgs(plots=['r','r_e'],
                             labels={'left': 'r (deg)','bottom': 'Time (s)'},
                             rad2deg=self.use_degrees)
        gyro_plots = PlotboxArgs(plots=['bx','bx_e','by','by_e','bz','bz_e'],
                           labels={'left': 'bias (deg/s)','bottom': 'Time (s)'},
                           rad2deg=self.use_degrees)
        fourth_row = [p_plots, q_plots, r_plots, gyro_plots]

        plots = [first_row,second_row,third_row,fourth_row]
        return plots


    def statesCallback(self, msg):
        # Extract time
        t = msg.header.stamp.to_sec()

        true_state_list = [msg.pn, msg.pe, msg.h, msg.Va, msg.alpha, msg.beta,
                           msg.phi, msg.theta, msg.chi, msg.p, msg.q, msg.r,
                           msg.Vg, msg.wn, msg.we, msg.psi,msg.bx,msg.by,msg.bz]

        self.plotter.add_vector_measurement('true_state', true_state_list, t)

    def estimatesCallback(self, msg):
        t = msg.header.stamp.to_sec()

        est_state_list = [msg.pn, msg.pe, msg.h, msg.Va, msg.alpha, msg.beta,
                           msg.phi, msg.theta, msg.chi, msg.p, msg.q, msg.r,
                           msg.Vg, msg.wn, msg.we, msg.psi,msg.bx,msg.by,msg.bz]

        self.plotter.add_vector_measurement('estimated_state',est_state_list,t)

    def cmdCallback(self, msg):
        t = msg.header.stamp.to_sec()

        cmd_state_list = [msg.h, msg.Va, msg.phi, msg.theta, msg.chi]

        self.plotter.add_vector_measurement('commands', cmd_state_list, t)


if __name__ == '__main__':
    rospy.init_node('plotter', anonymous=False)

    try:
        obj = PlotWrapper()
    except rospy.ROSInterruptException:
        pass
