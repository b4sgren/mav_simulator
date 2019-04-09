#!/usr/bin/env python
import rospy

from state_plotter.Plotter import Plotter
from state_plotter.plotter_args import *
from dynamics.msg import State

class StateViewer:
    def __init__(self):
        time_window_length=100
        self.plotter = Plotter(plotting_frequency=100, # refresh plot every 100 time steps
                               time_window=time_window_length)  # plot last time_window seconds of data
        # set up the plot window
        # define first row
        pn_plots = PlotboxArgs(plots=['pn', 'pn_e'],
                               labels={'left': 'pn(m)', 'bottom': 'Time (s)'},
                               time_window=time_window_length)
        pe_plots = PlotboxArgs(plots=['pe', 'pe_e'],
                               labels={'left': 'pe(m)', 'bottom': 'Time (s)'},
                               time_window=time_window_length)
        h_plots = PlotboxArgs(plots=['h', 'h_e', 'h_c'],
                              labels={'left': 'h(m)', 'bottom': 'Time (s)'},
                              time_window=time_window_length)
        wind_plots = PlotboxArgs(plots=['wn', 'wn_e', 'we', 'we_e'],
                                 labels={'left': 'wind(m/s)', 'bottom': 'Time (s)'},
                                 time_window=time_window_length)
        first_row = [pn_plots, pe_plots, h_plots, wind_plots]

        # define second row
        Va_plots = PlotboxArgs(plots=['Va', 'Va_e', 'Va_c'],
                               labels={'left': 'Va(m/s)', 'bottom': 'Time (s)'},
                               time_window=time_window_length)
        alpha_plots = PlotboxArgs(plots=['alpha', 'alpha_e'],
                                  labels={'left': 'alpha(deg)', 'bottom': 'Time (s)'},
                                  rad2deg=True,
                                  time_window=time_window_length)
        beta_plots = PlotboxArgs(plots=['beta', 'beta_e'],
                                 labels={'left': 'beta(deg)', 'bottom': 'Time (s)'},
                                 rad2deg=True,
                                 time_window=time_window_length)
        Vg_plots = PlotboxArgs(plots=['Vg', 'Vg_e'],
                               labels={'left': 'Vg(m/s)', 'bottom': 'Time (s)'},
                               time_window=time_window_length)
        second_row = [Va_plots, alpha_plots, beta_plots, Vg_plots]

        # define third row
        phi_plots = PlotboxArgs(plots=['phi', 'phi_e', 'phi_c'],
                                labels={'left': 'phi(deg)', 'bottom': 'Time (s)'},
                                rad2deg=True,
                                time_window=time_window_length)
        theta_plots = PlotboxArgs(plots=['theta', 'theta_e', 'theta_c'],
                                  labels={'left': 'theta(deg)', 'bottom': 'Time (s)'},
                                  rad2deg=True,
                                  time_window=time_window_length)
        psi_plots = PlotboxArgs(plots=['psi', 'psi_e'],
                                labels={'left': 'psi(deg)', 'bottom': 'Time (s)'},
                                rad2deg=True,
                                time_window=time_window_length)
        chi_plots = PlotboxArgs(plots=['chi', 'chi_e', 'chi_c'],
                                labels={'left': 'chi(deg)', 'bottom': 'Time (s)'},
                                rad2deg=True,
                                time_window=time_window_length)
        third_row = [phi_plots, theta_plots, psi_plots, chi_plots]

        # define fourth row
        p_plots = PlotboxArgs(plots=['p', 'p_e'],
                              labels={'left': 'p(deg/s)', 'bottom': 'Time (s)'},
                              rad2deg=True,
                              time_window=time_window_length)
        q_plots = PlotboxArgs(plots=['q', 'q_e'],
                              labels={'left': 'q(deg/s)', 'bottom': 'Time (s)'},
                              rad2deg=True,
                              time_window=time_window_length)
        r_plots = PlotboxArgs(plots=['r', 'r_e'],
                              labels={'left': 'r(deg)', 'bottom': 'Time (s)'},
                              rad2deg=True,
                              time_window=time_window_length)
        gyro_plots = PlotboxArgs(plots=['bx', 'bx_e', 'by', 'by_e', 'bz', 'bz_e'],
                                 labels={'left': 'bias(deg/s)', 'bottom': 'Time (s)'},
                                 rad2deg=True,
                                 time_window=time_window_length)
        fourth_row = [p_plots, q_plots, r_plots, gyro_plots]
        plots = [first_row,
                 second_row,
                 third_row,
                 fourth_row
                 ]
        # Add plots to the window
        self.plotter.add_plotboxes(plots)
        # Define and label vectors for more convenient/natural data input
        self.plotter.define_input_vector('true_state', ['pn', 'pe', 'h', 'Va', 'alpha', 'beta', 'phi', 'theta', 'chi',
                                                        'p', 'q', 'r', 'Vg', 'wn', 'we', 'psi', 'bx', 'by', 'bz'])
        self.plotter.define_input_vector('estimated_state', ['pn_e', 'pe_e', 'h_e', 'Va_e', 'alpha_e', 'beta_e',
                                                             'phi_e', 'theta_e', 'chi_e', 'p_e', 'q_e', 'r_e',
                                                             'Vg_e', 'wn_e', 'we_e', 'psi_e', 'bx_e', 'by_e', 'bz_e'])
        self.plotter.define_input_vector('commands', ['h_c', 'Va_c', 'phi_c', 'theta_c', 'chi_c'])
        # plot timer
        self.time = 0.
        self.ts = 0.02

        self.true_state_sub = rospy.Subscriber('true_state', State, self.state_callback, queue_size=1)
        self.est_state_sub = rospy.Subscriber('estimated_state', State, self.estimated_callback, queue_size = 1)
        self.commanded_sub = rospy.Subscriber('commanded_state', State, self.commanded_callback, queue_size = 1)

    def estimated_callback(self, msg):
        estimated_state_list = [msg.pn, msg.pe, msg.h,
                                msg.Va, msg.alpha, msg.beta,
                                msg.phi, msg.theta, msg.chi,
                                msg.p, msg.q, msg.r,
                                msg.Vg, msg.wn, msg.we, msg.psi,
                                msg.bx, msg.by, msg.bz]
        self.plotter.add_vector_measurement('estimated_state', estimated_state_list, self.time)

        # Update and display the plot
        self.plotter.update_plots() # TODO How to guarantee that all 3 msgs have been received (make a member variable *_list and update in everyfunction)

        # increment time
        self.time += self.ts # This will need to change

    def commanded_callback(self, msg):
        commands = [msg.h, # h_c
                    msg.Va, # Va_c
                    msg.phi, # phi_c
                    msg.theta, # theta_c
                    msg.chi] # chi_c
        self.plotter.add_vector_measurement('commands', commands, self.time)

    def state_callback(self, msg):
        ## Add the state data in vectors
        # the order has to match the order in lines 72-76
        true_state_list = [msg.pn, msg.pe, msg.h,
                           msg.Va, msg.alpha, msg.beta,
                           msg.phi, msg.theta, msg.chi,
                           msg.p, msg.q, msg.r,
                           msg.Vg, msg.wn, msg.we, msg.psi,
                           msg.bx, msg.by, msg.bz]
        self.plotter.add_vector_measurement('true_state', true_state_list, self.time)

    def run(self):
        while not rospy.is_shutdown():
            rospy.spin()

if __name__ == '__main__':
    print "Initializing node"
    rospy.init_node("StateViewer", log_level=rospy.DEBUG) #use when running ros

    #Initialize class here:
    state_viewer = StateViewer()
    state_viewer.run() # will stay commented out while building rest of class

    print "Done"
