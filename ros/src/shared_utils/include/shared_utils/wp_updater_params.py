import rospy

class WUParams(object):
    """
    Waypoint Updater params
    """
    __namespace__ = lambda param, default_value : rospy.get_param("/waypoint_updater/{}".format(param), default_value)