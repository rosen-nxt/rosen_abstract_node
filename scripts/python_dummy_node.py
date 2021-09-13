#!/usr/bin/env python3

import rospy

from std_msgs.msg import Int16
from rosen_abstract_node.rosen_abstract_node import RosenAbstractNode


class PythonDummyNode(RosenAbstractNode):

    def __init__(self):
        super(PythonDummyNode, self).__init__()
        self.test_publisher = rospy.Publisher("~published_in_loop", Int16, queue_size=10)

    def do_init(self):
        return rospy.get_param("~dummy_node_do_init_succeeds", True)

    def do_stop(self):
        return rospy.get_param("~dummy_node_do_stop_succeeds", True)

    def do_connect(self):
        return rospy.get_param("~dummy_node_do_connect_succeeds", True)

    def do_disconnect(self):
        return rospy.get_param("~dummy_node_do_disconnect_succeeds", True)

    def do_pause(self):
        return rospy.get_param("~dummy_node_do_pause_succeeds", True)

    def do_start(self):
        return (rospy.get_param("~dummy_node_do_start_succeeds", True), rospy.get_param("~dummy_node_running", True))

    def do_resume(self):
        return rospy.get_param("~dummy_node_do_resume_succeeds", True)

    def do_step(self):
        self.test_publisher.publish(42)

def main():
    rospy.init_node("python_dummy_node")
    node = PythonDummyNode()
    node.loop()

if __name__ == "__main__":
    main()
