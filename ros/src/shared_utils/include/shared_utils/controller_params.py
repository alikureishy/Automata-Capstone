import rospy

class ControllerParams(object):
    """
    Controller Params
    """
    __namespace__ = lambda param, default_value : rospy.get_param("/controller/{}".format(param), default_value)
