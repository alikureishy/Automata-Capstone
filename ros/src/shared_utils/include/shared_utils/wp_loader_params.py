import rospy
from waypoint_tools import kmph2mps

class WLParams(object):
    """
    Waypoint Loader params
    """
    __namespace__ = lambda param, default_value : rospy.get_param("/waypoint_loader/{}".format(param), default_value)

    VELOCITY_MPS = kmph2mps(__namespace__('velocity', None))
    VELOCITY_KMPH = __namespace__('velocity', None)
    PATH = __namespace__('path', None)
