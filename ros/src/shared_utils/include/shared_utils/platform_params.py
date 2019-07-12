import rospy

class PlatformParams(object):
    """
    Root (Platform) params
    """
    __namespace__ = lambda param, default_value : rospy.get_param("/{}".format(param), default_value)

    ####################################################################
    # All parameters are of teh format: "/<param-name>"
    ####################################################################
    SIMULATOR_MODE = __namespace__('sim_mode', 0) # Simulator_mode parameter (1== ON, 0==OFF)
