
import rospy
from std_msgs.msg import Bool, Int32
from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane, TrafficLightArray, TrafficLight
from sensor_msgs.msg import Image
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport

class Topics(object):
    class CurrentPose(object):
        __topic_name__ = "/current_pose"
        @classmethod
        def Subscriber(cls, callback, queue_size=None):
            return rospy.Subscriber(cls.__topic_name__, PoseStamped, callback, queue_size=queue_size)
        @classmethod
        def Publisher(cls, queue_size=None):
            return rospy.Publisher(cls.__topic_name__, PoseStamped, queue_size=queue_size)

    class BaseWaypoints(object):
        __topic_name__ = "/base_waypoints"
        @classmethod
        def Subscriber(cls, callback, queue_size=None):
            return rospy.Subscriber(cls.__topic_name__, Lane, callback, queue_size=queue_size)
        @classmethod
        def Publisher(cls, queue_size=None):
            return rospy.Publisher(cls.__topic_name__, Lane, queue_size=queue_size)

    class TrafficWaypoint(object):
        __topic_name__ = "/traffic_waypoint"
        @classmethod
        def Subscriber(cls, callback, queue_size=None):
            return rospy.Subscriber(cls.__topic_name__, Int32, callback, queue_size=queue_size)
        @classmethod
        def Publisher(cls, queue_size=None):
            return rospy.Publisher(cls.__topic_name__, Int32, queue_size=queue_size)

    class ObstacleWaypoint(object):
        __topic_name__ = "/obstacle_waypoint"
        @classmethod
        def Subscriber(cls, callback, queue_size=None):
            return rospy.Subscriber(cls.__topic_name__, Int32, callback, queue_size=queue_size)
        @classmethod
        def Publisher(cls, queue_size=None):
            return rospy.Publisher(cls.__topic_name__, Int32, queue_size=queue_size)

    class TwistCmd(object):
        __topic_name__ = "/twist_cmd"
        @classmethod
        def Subscriber(cls, callback, queue_size=None):
            return rospy.Subscriber(cls.__topic_name__, TwistStamped, callback, queue_size=queue_size)
        @classmethod
        def Publisher(cls, queue_size=None):
            return rospy.Publisher(cls.__topic_name__, TwistStamped, queue_size=queue_size)

    class CurrentVelocity(object):
        __topic_name__ = "/current_velocity"
        @classmethod
        def Subscriber(cls, callback, queue_size=None):
            return rospy.Subscriber(cls.__topic_name__, TwistStamped, callback, queue_size=queue_size)
        @classmethod
        def Publisher(cls, queue_size=None):
            return rospy.Publisher(cls.__topic_name__, TwistStamped, queue_size=queue_size)

    class ImageColor(object):
        __topic_name__ = "/image_color"
        @classmethod
        def Subscriber(cls, callback, queue_size=None):
            return rospy.Subscriber(cls.__topic_name__, Image, callback, queue_size=queue_size)
        @classmethod
        def Publisher(cls, queue_size=None):
            return rospy.Publisher(cls.__topic_name__, Image, queue_size=queue_size)

    class FinalWaypoints(object):
        __topic_name__ = "final_waypoints"
        @classmethod
        def Subscriber(cls, callback, queue_size=None):
            return rospy.Subscriber(cls.__topic_name__, Lane, callback, queue_size=queue_size)
        @classmethod
        def Publisher(cls, queue_size=None):
            return rospy.Publisher(cls.__topic_name__, Lane, queue_size=queue_size)

    class Vehicle(object):
        class SteeringCmd(object):
            __topic_name__ = '/vehicle/steering_cmd'
            @classmethod
            def Subscriber(cls, callback, queue_size=None):
                return rospy.Subscriber(cls.__topic_name__, SteeringCmd, callback, queue_size=queue_size)
            @classmethod
            def Publisher(cls, queue_size=None):
                return rospy.Publisher(cls.__topic_name__, SteeringCmd, queue_size=queue_size)

        class ThrottleCmd(object):
            __topic_name__ = "/vehicle/throttle_cmd"
            @classmethod
            def Subscriber(cls, callback, queue_size=None):
                return rospy.Subscriber(cls.__topic_name__, ThrottleCmd, callback, queue_size=queue_size)
            @classmethod
            def Publisher(cls, queue_size=None):
                return rospy.Publisher(cls.__topic_name__, ThrottleCmd, queue_size=queue_size)

        class BrakeCmd(object):
            __topic_name__ = "/vehicle/brake_cmd"
            @classmethod
            def Subscriber(cls, callback, queue_size=None):
                return rospy.Subscriber(cls.__topic_name__, BrakeCmd, callback, queue_size=queue_size)
            @classmethod
            def Publisher(cls, queue_size=None):
                return rospy.Publisher(cls.__topic_name__, BrakeCmd, queue_size=queue_size)

        class TrafficLights(object):
            __topic_name__ = "/vehicle/traffic_lights"
            @classmethod
            def Subscriber(cls, callback, queue_size=None):
                return rospy.Subscriber(cls.__topic_name__, TrafficLightArray, callback, queue_size=queue_size)
            @classmethod
            def Publisher(cls, queue_size=None):
                return rospy.Publisher(cls.__topic_name__, TrafficLightArray, queue_size=queue_size)

        class DBWEnabled(object):
            __topic_name__ = "/vehicle/dbw_enabled"
            @classmethod
            def Subscriber(cls, callback, queue_size=None):
                return rospy.Subscriber(cls.__topic_name__, Bool, callback, queue_size=queue_size)
            @classmethod
            def Publisher(cls, queue_size=None):
                return rospy.Publisher(cls.__topic_name__, Bool, queue_size=queue_size)
