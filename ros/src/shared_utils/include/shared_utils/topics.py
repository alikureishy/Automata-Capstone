
import rospy
from std_msgs.msg import Bool, Int32, Float32
from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane, TrafficLightArray, TrafficLight
from sensor_msgs.msg import PointCloud2, CameraInfo, Image
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport, ThrottleReport, BrakeReport

class Topic(object):
    __registry__ = {}

    def __init__(self, topic, type):
        self.__topic__ = topic
        self.__type__ = type
        Topic.register(self)

    @classmethod
    def register(cls, topic):
        if not cls.__registry__.has_key(topic.Topic()):
            cls.__registry__[topic.Topic()] = topic
            rospy.logdebug("[TopicRegistry]: Registered topic: '%s' (type: '%s')", topic.Topic(), topic.Type())

    @classmethod
    def Lookup(cls, item):
        assert isinstance(item, basestring), "Can only lookup topic using the topic text. But was given: {}".format(item)
        rospy.logdebug("[TopicRegistry]: Lookup topic by name: '%s'", item)
        value = cls.__registry__[item]
        if value is None:
            rospy.logwarn("TopicRegistry: No topic registered with name: '%s'. Maybe the namespacing was not configured correctly?", item)
        return value

    def Topic(self):
        return self.__topic__
    def Type(self):
        return self.__type__
    def Subscriber(self, callback, queue_size=None):
        rospy.logdebug("[Topic]: Creating rospy Subscriber() for topic '%s' (type '%s')", self.__topic__, self.__type__)
        return rospy.Subscriber(self.__topic__, self.__type__, callback, queue_size=queue_size)
    def Publisher(self, queue_size=None, latch=False):
        rospy.logdebug("[Topic]: Creating rospy Publisher() for topic '%s' (type '%s')", self.__topic__, self.__type__)
        return rospy.Publisher(self.__topic__, self.__type__, queue_size=queue_size, latch=latch)

class Topics(object):
    CurrentPose             = Topic("/current_pose",            PoseStamped)
    BaseWaypoints           = Topic("/base_waypoints",          Lane)
    TrafficWaypoint         = Topic("/traffic_waypoint",        Int32)
    ObstacleWaypoint        = Topic("/obstacle_waypoint",       Int32)
    TwistCmd                = Topic("/twist_cmd",               TwistStamped)
    CurrentVelocity         = Topic("/current_velocity",        TwistStamped)
    ImageColor              = Topic("/image_color",             Image)
    FinalWaypoints          = Topic("/final_waypoints",         Lane)
    CameraInfo              = Topic("/camera_info",             CameraInfo)

    class Vehicle(object):
        SteeringCmd         = Topic('/vehicle/steering_cmd',    SteeringCmd)
        ThrottleCmd         = Topic("/vehicle/throttle_cmd",    ThrottleCmd)
        BrakeCmd            = Topic("/vehicle/brake_cmd",       BrakeCmd)
        TrafficLights       = Topic("/vehicle/traffic_lights",  TrafficLightArray)
        DBWEnabled          = Topic("/vehicle/dbw_enabled",     Bool)
        Obstacle            = Topic("/vehicle/obstacle",        PoseStamped)
        ObstaclePoints      = Topic("/vehicle/obstacle_points", PointCloud2)
        Lidar               = Topic("/vehicle/lidar",           PointCloud2)
        SteeringReport      = Topic("/vehicle/steering_report", SteeringReport)
        ThrottleReport      = Topic("/vehicle/throttle_report", Float32)
        BrakeReport         = Topic("/vehicle/brake_report",    Float32)
