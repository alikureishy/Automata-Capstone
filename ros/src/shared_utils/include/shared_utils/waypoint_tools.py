from scipy.spatial import KDTree
import numpy as np

KDTREE_SEARCH_COUNT = 1
KDTREE_SEARCH_RESULT_IDX = 1 # The KDTree query returns (position, index) tuple. We only need the index value here..hence '1'.

class WaypointsWrapper(object):
    def __init__ (self, waypoints_list):
        """
        Initializes a waypoints wrapper object
        :param waypoints_list: list [] of waypoints (of type Waypoint.msg)
        """
        self.waypoints_list = waypoints_list
        self.waypoints_coordinates = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in waypoints_list]
        self.waypoints_tree = KDTree(self.waypoints_coordinates)

    def get_closest_waypoint_to(self, (x, y), strictly_ahead = True):
        """
        :param (x,y): Coordinates of point being checked for closeness
        :return: index of closest waypoint, and the waypoint itself
        """

        # Look for the closest in the KD Tree...
        closest_idx = self.waypoints_tree.query([x, y], KDTREE_SEARCH_COUNT)[KDTREE_SEARCH_RESULT_IDX]
        closest_waypoint = self.waypoints_list[closest_idx]

        if strictly_ahead:
            # Check if this waypoint is ahead or behind the vehicle, based on the motion vector of the vehicle
            # We use coordinates here that are relative to the absolute origin (just as all waypoints are).
            # We then utilize their coordinates as vectors, and perform the desired calculation
            pose_x_y_vector = np.array([x,y])
            closest_x_y_vector = np.array(self.waypoints_coordinates[closest_idx])
            prev_x_y_vector = np.array(self.waypoints_coordinates[closest_idx-1]) # This is guaranteed to be behind the vehicle. If it wasn't, it would have been returned as the closest waypoint.

            # With hyperplane perpendicular to the closest vector position, we define the following:
            previous_to_hyperplane = closest_x_y_vector - prev_x_y_vector # Vector pointing from prev --> hyperplane
            hyperplane_to_car = pose_x_y_vector - closest_x_y_vector # Vector pointing from hyperplane --> car

            # If the two vectors above are pointing in teh same direction, it means the car is ahead of the closest index
            # and the dot product will be positive (as below), in which case we pick the next waypoint (which is
            # guaranteed then to be AHEAD)
            product = np.dot(previous_to_hyperplane, hyperplane_to_car)
            if product > 0:
                closest_idx = (closest_idx + 1) % len(self.waypoints_coordinates)
                closest_waypoint = self.waypoints_list[closest_idx]
            else: # when product <= 0, the closest waypoint is ahead of the car (or at the same position)
                pass # So we can return the closest idx as it is

        return closest_idx, closest_waypoint