
import rospy
from std_msgs.msg import Bool, Int32
from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane, TrafficLightArray, TrafficLight
from sensor_msgs.msg import Image
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport

class Topics(object):
    class CurrentPose(object):
        __name__ = "/current_pose"
        @staticmethod
        def Subscriber(callback, queue_size=None):
            return rospy.Subscriber(__name__, PoseStamped, callback, queue_size=queue_size)
        @staticmethod
        def Publisher(queue_size=None):
            return rospy.Publisher(__name__, PoseStamped, queue_size=queue_size)

    class BaseWaypoints(object):
        __name__ = "/base_waypoints"
        @staticmethod
        def Subscriber(callback, queue_size=None):
            return rospy.Subscriber(__name__, Lane, callback, queue_size=queue_size)
        @staticmethod
        def Publisher(queue_size=None):
            return rospy.Publisher(__name__, Lane, queue_size=queue_size)

    class TrafficWaypoint(object):
        __name__ = "/traffic_waypoint"
        @staticmethod
        def Subscriber(callback, queue_size=None):
            return rospy.Subscriber(__name__, Int32, callback, queue_size=queue_size)
        @staticmethod
        def Publisher(queue_size=None):
            return rospy.Publisher(__name__, Int32, queue_size=queue_size)

    class TwistCmd(object):
        __name__ = "/twist_cmd"
        @staticmethod
        def Subscriber(callback, queue_size=None):
            return rospy.Subscriber(__name__, TwistStamped, callback, queue_size=queue_size)
        @staticmethod
        def Publisher(queue_size=None):
            return rospy.Publisher(__name__, TwistStamped, queue_size=queue_size)

    class CurrentVelocity(object):
        __name__ = "/current_pose"
        @staticmethod
        def Subscriber(callback, queue_size=None):
            return rospy.Subscriber(__name__, TwistStamped, callback, queue_size=queue_size)
        @staticmethod
        def Publisher(queue_size=None):
            return rospy.Publisher(__name__, TwistStamped, queue_size=queue_size)

    class ImageColor(object):
        __name__ = "/image_color"
        @staticmethod
        def Subscriber(callback, queue_size=None):
            return rospy.Subscriber(__name__, Image, callback, queue_size=queue_size)
        @staticmethod
        def Publisher(queue_size=None):
            return rospy.Publisher(__name__, Image, queue_size=queue_size)

    class TrafficWaypoint(object):
        __name__ = "/traffic_waypoint"
        @staticmethod
        def Subscriber(callback, queue_size=None):
            return rospy.Subscriber(__name__, Int32, callback, queue_size=queue_size)
        @staticmethod
        def Publisher(queue_size=None):
            return rospy.Publisher(__name__, Int32, queue_size=queue_size)

    class Vehicle(object):
        class SteeringCmd(object):
            __name__ = '/vehicle/steering_cmd'
            @staticmethod
            def Subscriber(callback, queue_size=None):
                return rospy.Subscriber(__name__, SteeringCmd, callback, queue_size=queue_size)
            @staticmethod
            def Publisher(queue_size=None):
                return rospy.Publisher(__name__, SteeringCmd, queue_size=queue_size)

        class ThrottleCmd(object):
            __name__ = "/vehicle/throttle_cmd"
            @staticmethod
            def Subscriber(callback, queue_size=None):
                return rospy.Subscriber(__name__, ThrottleCmd, callback, queue_size=queue_size)
            @staticmethod
            def Publisher(queue_size=None):
                return rospy.Publisher(__name__, ThrottleCmd, queue_size=queue_size)

        class BrakeCmd(object):
            __name__ = "/vehicle/brake_cmd"
            @staticmethod
            def Subscriber(callback, queue_size=None):
                return rospy.Subscriber(__name__, BrakeCmd, callback, queue_size=queue_size)
            @staticmethod
            def Publisher(queue_size=None):
                return rospy.Publisher(__name__, BrakeCmd, queue_size=queue_size)

        class TrafficLights(object):
            __name__ = "/vehicle/traffic_lights"
            @staticmethod
            def Subscriber(callback, queue_size=None):
                return rospy.Subscriber(__name__, TrafficLightArray, callback, queue_size=queue_size)
            @staticmethod
            def Publisher(queue_size=None):
                return rospy.Publisher(__name__, TrafficLightArray, queue_size=queue_size)

        class DBWEnabled(object):
            __name__ = "/vehicle/dbw_enabled"
            @staticmethod
            def Subscriber(callback, queue_size=None):
                return rospy.Subscriber(__name__, Bool, callback, queue_size=queue_size)
            @staticmethod
            def Publisher(queue_size=None):
                return rospy.Publisher(__name__, Bool, queue_size=queue_size)

        class BrakeCmd(object):
            __name__ = "/vehicle/brake_cmd"
            @staticmethod
            def Subscriber(callback, queue_size=None):
                return rospy.Subscriber(__name__, BrakeCmd, callback, queue_size=queue_size)
            @staticmethod
            def Publisher(queue_size=None):
                return rospy.Publisher(__name__, BrakeCmd, queue_size=queue_size)
