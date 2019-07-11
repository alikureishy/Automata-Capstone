import rospy

class WUParams(object):
    """
    Waypoint Updater params
    """
    NODE_NAME = "waypoint_updater"
    __namespace__ = lambda param, default_value : rospy.get_param("/waypoint_updater/{}".format(param), default_value)
