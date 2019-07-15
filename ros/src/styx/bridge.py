import base64
import math
import random
from io import BytesIO

import numpy as np
import rospy
import sensor_msgs.point_cloud2 as pcl2
import tf
from PIL import Image as PIL_Image
from cv_bridge import CvBridge
from dbw_mkz_msgs.msg import SteeringReport
from geometry_msgs.msg import PoseStamped, Quaternion, TwistStamped
from shared_utils.node_names import NodeNames
from shared_utils.params import Params
from shared_utils.topics import Topics, Topic
from std_msgs.msg import Bool
from std_msgs.msg import Float32 as Float
from std_msgs.msg import Header
from styx_msgs.msg import TrafficLight, TrafficLightArray


class Bridge(object):
    def __init__(self, conf, server):
        rospy.init_node(NodeNames.STYX_SERVER, log_level=rospy.DEBUG)
        self.server = server
        self.vel = 0.
        self.yaw = None
        self.angular_vel = 0.
        self.bridge = CvBridge()

        self.callbacks = {
            Topics.Vehicle.SteeringCmd.Topic() : self.callback_steering,
            Topics.Vehicle.ThrottleCmd.Topic(): self.callback_throttle,
            Topics.Vehicle.BrakeCmd.Topic(): self.callback_brake,
            Topics.FinalWaypoints.Topic(): self.callback_path
        }

        rospy.logdebug("Subscribers received via configuration:")
        for subscriber in conf.subscribers:
            rospy.logdebug("\t '%s' : '%s'", subscriber.topic, subscriber.type)

        self.subscribers = [Topic.Lookup(subscriber.topic).Subscriber(self.callbacks[subscriber.topic])
                            for subscriber in conf.subscribers]


        """
            These are the publishers generally received from the configuration parameter:
                /vehicle/throttle_report ==> std_msgs/Float32
                /vehicle/dbw_enabled ==> std_msgs/Bool
                /vehicle/lidar ==> sensor_msgs/PointCloud2
                /vehicle/steering_report ==> dbw_mkz_msgs/SteeringReport
                /vehicle/brake_report ==> std_msgs/Float32
                /current_velocity ==> geometry_msgs/TwistStamped
                /vehicle/obstacle ==> geometry_msgs/PoseStamped
                /vehicle/traffic_lights ==> styx_msgs/TrafficLightArray
                /current_pose ==> geometry_msgs/PoseStamped
                /image_color ==> sensor_msgs/Image
                /vehicle/obstacle_points ==> sensor_msgs/PointCloud2
        """
        rospy.logdebug("Publishers received via configuration:")
        for publisher in conf.publishers:
            rospy.logdebug("\t '%s' : '%s'", publisher.topic, publisher.type)

        self.publishers = {publisher.topic: Topic.Lookup(publisher.topic).Publisher(queue_size=1)
                           for publisher in conf.publishers}


    def create_light(self, x, y, z, yaw, state):
        light = TrafficLight()

        light.header = Header()
        light.header.stamp = rospy.Time.now()
        light.header.frame_id = '/world'

        light.pose = self.create_pose(x, y, z, yaw)
        light.state = state

        return light

    def create_pose(self, x, y, z, yaw=0.):
        pose = PoseStamped()

        pose.header = Header()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = '/world'

        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z

        q = tf.transformations.quaternion_from_euler(0., 0., math.pi * yaw/180.)
        pose.pose.orientation = Quaternion(*q)

        return pose

    def create_float(self, val):
        fl = Float()
        fl.data = val
        return fl

    def create_twist(self, velocity, angular):
        tw = TwistStamped()
        tw.twist.linear.x = velocity
        tw.twist.angular.z = angular
        return tw

    def create_steer(self, val):
        st = SteeringReport()
        st.steering_wheel_angle = val * math.pi/180.
        st.enabled = True
        st.speed = self.vel
        return st

    def calc_angular(self, yaw):
        angular_vel = 0.
        if self.yaw is not None:
            angular_vel = (yaw - self.yaw)/(rospy.get_time() - self.prev_time)
        self.yaw = yaw
        self.prev_time = rospy.get_time()
        return angular_vel

    def create_point_cloud_message(self, pts):
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = '/world'
        cloud_message = pcl2.create_cloud_xyz32(header, pts)
        return cloud_message

    def broadcast_transform(self, name, position, orientation):
        br = tf.TransformBroadcaster()
        br.sendTransform(position,
            orientation,
            rospy.Time.now(),
            name,
            "world")

    def publish_odometry(self, data):
        pose = self.create_pose(data['x'], data['y'], data['z'], data['yaw'])

        position = (data['x'], data['y'], data['z'])
        orientation = tf.transformations.quaternion_from_euler(0, 0, math.pi * data['yaw']/180.)
        self.broadcast_transform("base_link", position, orientation)

        self.publishers[Topics.CurrentPose.Topic()].publish(pose)
        self.vel = data['velocity']* 0.44704
        self.angular = self.calc_angular(data['yaw'] * math.pi/180.)
        self.publishers[Topics.CurrentVelocity.Topic()].publish(self.create_twist(self.vel, self.angular))


    def publish_controls(self, data):
        steering, throttle, brake = data['steering_angle'], data['throttle'], data['brake']
        self.publishers[Topics.Vehicle.SteeringReport.Topic()].publish(self.create_steer(steering))
        self.publishers[Topics.Vehicle.ThrottleReport.Topic()].publish(self.create_float(throttle))
        self.publishers[Topics.Vehicle.BrakeReport.Topic()].publish(self.create_float(brake))

    def publish_obstacles(self, data):
        for obs in data['obstacles']:
            pose = self.create_pose(obs[0], obs[1], obs[2])
            self.publishers[Topics.Vehicle.Obstacle.Topic()].publish(pose)
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = '/world'
        cloud = pcl2.create_cloud_xyz32(header, data['obstacles'])
        self.publishers[Topics.Vehicle.ObstaclePoints.Topic()].publish(cloud)

    def publish_lidar(self, data):
        self.publishers[Topics.Vehicle.Lidar.Topic()].publish(self.create_point_cloud_message(zip(data['lidar_x'], data['lidar_y'], data['lidar_z'])))

    def publish_traffic(self, data):
        x, y, z = data['light_pos_x'], data['light_pos_y'], data['light_pos_z'],
        yaw = [math.atan2(dy, dx) for dx, dy in zip(data['light_pos_dx'], data['light_pos_dy'])]
        status = data['light_state']

        lights = TrafficLightArray()
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = '/world'
        lights.lights = [self.create_light(*e) for e in zip(x, y, z, yaw, status)]
        self.publishers[Topics.Vehicle.TrafficLights.Topic()].publish(lights)

    def publish_dbw_status(self, data):
        self.publishers[Topics.Vehicle.DBWEnabled.Topic()].publish(Bool(data))

    def publish_camera(self, data):
        # Probabilistically publish the image from the simulator:
        not_dropped = random.uniform(0,1) >= Params.Server.ImageDropProbability.Get()
        if not_dropped:
            imgString = data["image"]
            image = PIL_Image.open(BytesIO(base64.b64decode(imgString)))
            image_array = np.asarray(image)
            image_message = self.bridge.cv2_to_imgmsg(image_array, encoding="rgb8")
            self.publishers[Topics.ImageColor.Topic()].publish(image_message)

    def callback_steering(self, data):
        self.server('steer', data={'steering_angle': str(data.steering_wheel_angle_cmd)})

    def callback_throttle(self, data):
        self.server('throttle', data={'throttle': str(data.pedal_cmd)})

    def callback_brake(self, data):
        self.server('brake', data={'brake': str(data.pedal_cmd)})

    def callback_path(self, data):
        x_values = []
        y_values = []
        z_values = []
        for waypoint in data.waypoints:
            x = waypoint.pose.pose.position.x
            y = waypoint.pose.pose.position.y
            z = waypoint.pose.pose.position.z+0.5
            x_values.append(x)
            y_values.append(y)
            z_values.append(z)

        self.server('drawline', data={'next_x': x_values, 'next_y': y_values, 'next_z': z_values})
