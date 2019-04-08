import rospy

from data_viewer import data_viewer as data_viewer

class StateViewer:
    def __init__(self):
        self.data_view = data_viewer()
        self.true_state_sub = rospy.Subscriber('true_state', State, self.state_callback, queue_size=1)
        # self.est_state_sub = rospy.Subscriber('estimated_state', State, self.estimated_callback, queue_size = 1)
        # self.commanded_sub = rospy.Subscriber('commanded_state', State, self.commanded_callback, queue_size = 1)

    def state_callback(self, msg):
        state = np.array([msg.pn, msg.pe, msg.h, msg.phi, msg.theta, msg.psi])
        self.data_viewer.update(state)

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
