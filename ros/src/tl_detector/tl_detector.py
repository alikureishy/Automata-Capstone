#!/usr/bin/env python
import os

import cv2
import rospy
import yaml
from cv_bridge import CvBridge
from scipy.spatial import KDTree
from shared_utils.node_names import NodeNames
from shared_utils.params import Params
from shared_utils.topics import Topics
from std_msgs.msg import Int32
from styx_msgs.msg import TrafficLight

from light_classification.tl_classifier import TLClassifier


class TLDetector(object):
    def __init__(self):
        rospy.init_node(NodeNames.TL_DETECTOR, log_level=rospy.DEBUG)

        self.pose = None
        self.waypoints = None
        # Waypoint KD Tree
        self.waypoints_2d = None
        self.waypoint_tree = None

        self.camera_image = None
        self.lights = []
        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0
        self.image_counter = 0

        self.bridge = CvBridge()
        if not Params.Classifier.CheatMode.Get():
            self.light_classifier = TLClassifier(Params.Classifier.ModelFilePath.Get())

        self.saving_images = Params.Classifier.SavingImages.Get()
        if self.saving_images:
            for dir in ["/red", "/green", "/yellow", "/none"]:
                if not os.path.exists(Params.Classifier.DataFolder.Get() + dir):
                    os.makedirs(Params.Classifier.DataFolder.Get() + dir)

        config_string = Params.Classifier.TrafficLightConfig.Get()
        self.config = yaml.load(config_string)

        # Subscribe Current pose and base waypoints
        Topics.CurrentPose.Subscriber(self.pose_cb, queue_size=2)
        Topics.BaseWaypoints.Subscriber(self.waypoints_cb, queue_size=8)
        rospy.logwarn("Start TL Detector")

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        Topics.Vehicle.TrafficLights.Subscriber(self.traffic_cb, queue_size=2)
        Topics.ImageColor.Subscriber(self.image_cb, queue_size=1)

        # Publish the index of the waypoint where we have to stop
        self.upcoming_red_light_pub = Topics.TrafficWaypoint.Publisher(queue_size=1)

        self.state_count_threshold = Params.Classifier.StateCountThreshold.Get()

        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

    # Note: waypoints remain static, this runs only once
    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints
        # Setup the Kd Tree which has log(n) complexity
        if not self.waypoints_2d:
            self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints]
            self.waypoint_tree = KDTree(self.waypoints_2d)


    # WARNING: lights state will not be available in real life, use only for diagnostics
    def traffic_cb(self, msg):
        self.lights = msg.lights


    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint
        Args:
            msg (Image): image from car-mounted camera
        """
        self.camera_image = msg
        light_wp, state = self.process_traffic_lights()
        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        rospy.logwarn("light: %s", state)
        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= self.state_count_threshold:
            self.last_state = self.state
            light_wp = light_wp if state == TrafficLight.RED else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1

    # Similar to function in waypoint_update.py
    def get_closest_waypoint(self, x, y):
        closest_idx = self.waypoint_tree.query([x,y], 1)[1]
        return closest_idx

    def get_light_state(self, light=None):
        """Determines the current color of the traffic light
        Args:
            light (TrafficLight): light to classify
        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)
        """
        if Params.Classifier.CheatMode.Get():
            traffic_color = -1 if light == None else light.state
            if self.saving_images:
                cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")
        else:
            cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")
            traffic_color = self.light_classifier.get_classification(cv_image)

        if self.saving_images:
            self.saveImages(traffic_color, cv_image)

        return traffic_color

    def saveImages(self, traffic_color, cv_image):
        self.image_counter += 1
        dir = self.traffic_color_to_file_name(traffic_color)
        save_file = Params.Classifier.DataFolder.Get() + dir + "/" + Params.Classifier.ImageNameFormat.Get().format(dir,
                                                                                                                    self.image_counter)
        rospy.logwarn("saving image: %s", save_file)
        cv2.imwrite(save_file, cv_image)

    def traffic_color_to_file_name(self, state):
        if state == TrafficLight.GREEN:
            return "green"
        elif state == TrafficLight.YELLOW:
            return "yellow"
        elif state == TrafficLight.RED:
            return "red"
        return "none"

    # Important function
    """Finds closest visible traffic light, if one exists, and determines its
            location and color
        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)
        """
    def process_traffic_lights(self):
        closest_light = None
        line_wp_idx = None

        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']
        if not self.saving_images:
            if (self.pose and self.waypoint_tree):
                # waypoint closest to current car pose
                car_wp_idx = self.get_closest_waypoint(self.pose.pose.position.x, self.pose.pose.position.y)
                # total number of waypoints
                diff = len(self.waypoints.waypoints)

                for i, light in enumerate(self.lights):
                    line = stop_line_positions[i]
                    temp_wp_idx = self.get_closest_waypoint(line[0], line[1])

                    d = temp_wp_idx - car_wp_idx

                    if d >= 0 and d < diff:
                        diff = d
                        closest_light = light
                        line_wp_idx = temp_wp_idx

            if closest_light:
                state = self.get_light_state(closest_light)
                return line_wp_idx, state

            return -1, TrafficLight.UNKNOWN
        else:
            return -1, self.get_light_state()


if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')