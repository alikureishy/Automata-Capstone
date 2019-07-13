import tf
import math
import numpy as np
from scipy.spatial import KDTree

KDTREE_SEARCH_COUNT = 1
KDTREE_SEARCH_RESULT_IDX = 1 # The KDTree query returns (position, index) tuple. We only need the index value here..hence '1'.


def distance(p1, p2):
    x, y, z = p1.x - p2.x, p1.y - p2.y, p1.z - p2.z
    return math.sqrt(x * x + y * y + z * z)

def kmph2mps(velocity_kmph):
    return (velocity_kmph * 1000.) / (60. * 60.)

def quaternion_from_yaw(yaw):
    return tf.transformations.quaternion_from_euler(0., 0., yaw)


def waypoint_coordinate_extractor(waypoint):
    """
    Returns the [x,y] coordinate of the given waypoint
    :param waypoint:
    :return: [x,y] coordinates of the given waypoint
    """
    return [waypoint.pose.pose.position.x, waypoint.pose.pose.position.y]

class PathAnalyzer(object):
    def __init__ (self, referenced_object_list, coordinates_extractor):
        """
        Initializes a waypoints wrapper object
        :param referenced_object_list: list [] of waypoints (of type Waypoint.msg)
        :param coordinates_extractor: A method that takes an object of the referenced list and returns its coordinates as a 2-element array ==> [x,y]
                                      If it is not provided, it will be assumed that the referenced_object_list is a coordinate list already
        """
        self.referenced_object_list = referenced_object_list
        self.coordinates_list = coordinates_extractor(coordinates_extractor(referenced_object) for referenced_object in referenced_object_list) \
                                    if coordinates_extractor is not None \
                                        else referenced_object_list
        self.coordinates_tree = KDTree(self.coordinates_list)

    def get_closest_object_to(self, (x, y), strictly_ahead = True):
        """
        :param (x,y): Coordinates of point being checked for closeness
        :return: index of closest object, and the object itself
        """

        # Look for the closest in the KD Tree...
        closest_idx = self.coordinates_tree.query([x, y], KDTREE_SEARCH_COUNT)[KDTREE_SEARCH_RESULT_IDX]
        closest_object = self.referenced_object_list[closest_idx] if self.referenced_object_list is not None else None

        if strictly_ahead:
            # Check if this object is ahead or behind the vehicle, based on the motion vector of the vehicle
            # We use coordinates here that are relative to the absolute origin (just as all waypoints are).
            # We then utilize their coordinates as vectors, and perform the desired calculation
            pose_x_y_vector = np.array([x,y])
            closest_x_y_vector = np.array(self.coordinates_list[closest_idx])
            prev_x_y_vector = np.array(self.coordinates_list[closest_idx - 1]) # This is guaranteed to be behind the vehicle. If it wasn't, it would have been returned as the closest object.

            # With hyperplane perpendicular to the closest vector position, we define the following:
            previous_to_hyperplane = closest_x_y_vector - prev_x_y_vector # Vector pointing from prev --> hyperplane
            hyperplane_to_car = pose_x_y_vector - closest_x_y_vector # Vector pointing from hyperplane --> car

            # If the two vectors above are pointing in teh same direction, it means the car is ahead of the closest index
            # and the dot product will be positive (as below), in which case we pick the next object (which is
            # guaranteed then to be AHEAD)
            product = np.dot(previous_to_hyperplane, hyperplane_to_car)
            if product > 0:
                closest_idx = (closest_idx + 1) % len(self.coordinates_list)
                closest_object = self.referenced_object_list[closest_idx] if self.referenced_object_list is not None else None
            else: # when product <= 0, the closest object is ahead of the car (or at the same position)
                pass # So we can return the closest idx as it is

        return closest_idx, closest_object