
import rospy
from std_msgs.msg import Bool, Int32
from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane, TrafficLightArray, TrafficLight
from std_msgs.msg import Float32 as Float
from std_msgs.msg import Header, Bool
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import Image
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport, ThrottleReport, BrakeReport

TopicTypeMappings = {
    'bool': Bool,
    'float': Float,
    'pose': PoseStamped,
    'pcl': PointCloud2,
    'twist': TwistStamped,
    'steer': SteeringReport,
    'trafficlights': TrafficLightArray,
    'steer_cmd': SteeringCmd,
    'brake_cmd': BrakeCmd,
    'throttle_cmd': ThrottleCmd,
    'path_draw': Lane,
    'image': Image
}


class Topics(object):
    class CurrentPose(object):
        text = "/current_pose"
        @classmethod
        def Subscriber(cls, callback, queue_size=None):
            return rospy.Subscriber(cls.text, PoseStamped, callback, queue_size=queue_size)
        @classmethod
        def Publisher(cls, queue_size=None, latch=False):
            return rospy.Publisher(cls.text, PoseStamped, queue_size=queue_size, latch=latch)

    class BaseWaypoints(object):
        text = "/base_waypoints"
        @classmethod
        def Subscriber(cls, callback, queue_size=None):
            return rospy.Subscriber(cls.text, Lane, callback, queue_size=queue_size)
        @classmethod
        def Publisher(cls, queue_size=None, latch=False):
            return rospy.Publisher(cls.text, Lane, queue_size=queue_size, latch=latch)

    class TrafficWaypoint(object):
        text = "/traffic_waypoint"
        @classmethod
        def Subscriber(cls, callback, queue_size=None):
            return rospy.Subscriber(cls.text, Int32, callback, queue_size=queue_size)
        @classmethod
        def Publisher(cls, queue_size=None, latch=False):
            return rospy.Publisher(cls.text, Int32, queue_size=queue_size, latch=latch)

    class ObstacleWaypoint(object):
        text = "/obstacle_waypoint"
        @classmethod
        def Subscriber(cls, callback, queue_size=None):
            return rospy.Subscriber(cls.text, Int32, callback, queue_size=queue_size)
        @classmethod
        def Publisher(cls, queue_size=None, latch=False):
            return rospy.Publisher(cls.text, Int32, queue_size=queue_size, latch=latch)

    class TwistCmd(object):
        text = "/twist_cmd"
        @classmethod
        def Subscriber(cls, callback, queue_size=None):
            return rospy.Subscriber(cls.text, TwistStamped, callback, queue_size=queue_size)
        @classmethod
        def Publisher(cls, queue_size=None, latch=False):
            return rospy.Publisher(cls.text, TwistStamped, queue_size=queue_size, latch=latch)

    class CurrentVelocity(object):
        text = "/current_velocity"
        @classmethod
        def Subscriber(cls, callback, queue_size=None):
            return rospy.Subscriber(cls.text, TwistStamped, callback, queue_size=queue_size)
        @classmethod
        def Publisher(cls, queue_size=None, latch=False):
            return rospy.Publisher(cls.text, TwistStamped, queue_size=queue_size, latch=latch)

    class ImageColor(object):
        text = "/image_color"
        @classmethod
        def Subscriber(cls, callback, queue_size=None):
            return rospy.Subscriber(cls.text, Image, callback, queue_size=queue_size)
        @classmethod
        def Publisher(cls, queue_size=None, latch=False):
            return rospy.Publisher(cls.text, Image, queue_size=queue_size, latch=latch)

    class FinalWaypoints(object):
        text = "/final_waypoints"
        @classmethod
        def Subscriber(cls, callback, queue_size=None):
            return rospy.Subscriber(cls.text, Lane, callback, queue_size=queue_size)
        @classmethod
        def Publisher(cls, queue_size=None, latch=False):
            return rospy.Publisher(cls.text, Lane, queue_size=queue_size, latch=latch)

    class Vehicle(object):
        class SteeringCmd(object):
            text = '/vehicle/steering_cmd'
            @classmethod
            def Subscriber(cls, callback, queue_size=None):
                return rospy.Subscriber(cls.text, SteeringCmd, callback, queue_size=queue_size)
            @classmethod
            def Publisher(cls, queue_size=None, latch=False):
                return rospy.Publisher(cls.text, SteeringCmd, queue_size=queue_size, latch=latch)

        class ThrottleCmd(object):
            text = "/vehicle/throttle_cmd"
            @classmethod
            def Subscriber(cls, callback, queue_size=None):
                return rospy.Subscriber(cls.text, ThrottleCmd, callback, queue_size=queue_size)
            @classmethod
            def Publisher(cls, queue_size=None, latch=False):
                return rospy.Publisher(cls.text, ThrottleCmd, queue_size=queue_size, latch=latch)

        class BrakeCmd(object):
            text = "/vehicle/brake_cmd"
            @classmethod
            def Subscriber(cls, callback, queue_size=None):
                return rospy.Subscriber(cls.text, BrakeCmd, callback, queue_size=queue_size)
            @classmethod
            def Publisher(cls, queue_size=None, latch=False):
                return rospy.Publisher(cls.text, BrakeCmd, queue_size=queue_size, latch=latch)

        class TrafficLights(object):
            text = "/vehicle/traffic_lights"
            @classmethod
            def Subscriber(cls, callback, queue_size=None):
                return rospy.Subscriber(cls.text, TrafficLightArray, callback, queue_size=queue_size)
            @classmethod
            def Publisher(cls, queue_size=None, latch=False):
                return rospy.Publisher(cls.text, TrafficLightArray, queue_size=queue_size, latch=latch)

        class DBWEnabled(object):
            text = "/vehicle/dbw_enabled"
            @classmethod
            def Subscriber(cls, callback, queue_size=None):
                return rospy.Subscriber(cls.text, Bool, callback, queue_size=queue_size)
            @classmethod
            def Publisher(cls, queue_size=None, latch=False):
                return rospy.Publisher(cls.text, Bool, queue_size=queue_size, latch=latch)

        class Obstacle(object):
            text = "/vehicle/obstacle"
            @classmethod
            def Subscriber(cls, callback, queue_size=None):
                return rospy.Subscriber(cls.text, PoseStamped, callback, queue_size=queue_size)
            @classmethod
            def Publisher(cls, queue_size=None, latch=False):
                return rospy.Publisher(cls.text, PoseStamped, queue_size=queue_size, latch=latch)

        class ObstaclePoints(object):
            text = "/vehicle/obstacle_points"
            @classmethod
            def Subscriber(cls, callback, queue_size=None):
                return rospy.Subscriber(cls.text, PointCloud2, callback, queue_size=queue_size)
            @classmethod
            def Publisher(cls, queue_size=None, latch=False):
                return rospy.Publisher(cls.text, PointCloud2, queue_size=queue_size, latch=latch)

        class Lidar(object):
            text = "/vehicle/lidar"
            @classmethod
            def Subscriber(cls, callback, queue_size=None):
                return rospy.Subscriber(cls.text, PointCloud2, callback, queue_size=queue_size)
            @classmethod
            def Publisher(cls, queue_size=None, latch=False):
                return rospy.Publisher(cls.text, PointCloud2, queue_size=queue_size, latch=latch)

        class SteeringReport(object):
            text = "/vehicle/steering_report"
            @classmethod
            def Subscriber(cls, callback, queue_size=None):
                return rospy.Subscriber(cls.text, SteeringReport, callback, queue_size=queue_size)
            @classmethod
            def Publisher(cls, queue_size=None, latch=False):
                return rospy.Publisher(cls.text, SteeringReport, queue_size=queue_size, latch=latch)

        class ThrottleReport(object):
            text = "/vehicle/throttle_report"
            @classmethod
            def Subscriber(cls, callback, queue_size=None):
                return rospy.Subscriber(cls.text, ThrottleReport, callback, queue_size=queue_size)
            @classmethod
            def Publisher(cls, queue_size=None, latch=False):
                return rospy.Publisher(cls.text, ThrottleReport, queue_size=queue_size, latch=latch)

        class BrakeReport(object):
            text = "/vehicle/brake_report"
            @classmethod
            def Subscriber(cls, callback, queue_size=None):
                return rospy.Subscriber(cls.text, BrakeReport, callback, queue_size=queue_size)
            @classmethod
            def Publisher(cls, queue_size=None, latch=False):
                return rospy.Publisher(cls.text, BrakeReport, queue_size=queue_size, latch=latch)
