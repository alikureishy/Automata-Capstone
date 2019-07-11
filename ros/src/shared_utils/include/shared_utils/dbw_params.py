
import rospy

class DBWParams(object):
    """
    DBW Parameter Names
    """
    __namespace__ = lambda param, default_value : rospy.get_param("/dbw_node/{}".format(param), default_value)

    ####################################################################
    # All parameters are of teh format: "/dbw_node/<param-name>"
    ####################################################################
    VEHICLE_MASS = __namespace__('vehicle_mass', 1736.35)
    FUEL_CAPACITY = __namespace__('fuel_capacity', 13.5)
    BRAKE_DEADBAND = __namespace__('brake_deadband', .1)
    DECEL_LIMIT = __namespace__('decel_limit', -5)
    ACCEL_LIMIT = __namespace__('accel_limit', 1.)
    WHEEL_RADIUS = __namespace__('wheel_radius', 0.2413)
    WHEEL_BASE = __namespace__('wheel_base', 2.8498)
    STEER_RATIO = __namespace__('steer_ratio', 14.8)
    MAX_LAT_ACCEL = __namespace__('max_lat_accel', 3.)
    MAX_STEER_ANGLE = __namespace__('max_steer_angle', 8.)
