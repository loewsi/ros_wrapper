#!/usr/bin/env python3

# import external libraries
import rospy



# import DTROS-related classes
from duckietown.dtros import \
    DTROS, \
    NodeType, \
    TopicType, \
    DTParam, \
    ParamType

# import messages and services
from std_msgs.msg import Float32
from duckietown_msgs.msg import \
    SegmentList, \
    Segment, \
    BoolStamped

class RosWrapperNode(DTROS):
    def __init__(self, node_name):
        super(RosWrapperNode, self).__init__(
            node_name=node_name,
            node_type=NodeType.VISUALIZATION
        )
        # class implementation

if __name__ == '__main__':
    node = RosWrapperNode(node_name='ros_wrapper_node')
    rospy.spin()