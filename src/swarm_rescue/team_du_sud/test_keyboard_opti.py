"""
This program can be launched directly.
To move the drone, you have to click on the map, then use the arrows on the
keyboard
"""

import sys
from pathlib import Path
from typing import List, Type
from enum import Enum
import cv2
import numpy as np

# Insert the parent directory of the current file's directory into sys.path.
# This allows Python to locate modules that are one level above the current
# script, in this case spg_overlay.
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from swarm_rescue.simulation.drone.controller import CommandsDict
from swarm_rescue.simulation.drone.drone_abstract import DroneAbstract
from swarm_rescue.simulation.utils.constants import MAX_RANGE_LIDAR_SENSOR
from swarm_rescue.simulation.utils.grid import Grid
from swarm_rescue.simulation.utils.pose import Pose
from swarm_rescue.simulation.ray_sensors.drone_semantic_sensor import DroneSemanticSensor
from swarm_rescue.simulation.elements.rescue_center import RescueCenter
from swarm_rescue.simulation.elements.return_area import ReturnArea
from swarm_rescue.simulation.elements.wounded_person import WoundedPerson
from swarm_rescue.simulation.gui_map.closed_playground import ClosedPlayground
from swarm_rescue.maps.map_intermediate_01 import MapIntermediate01
from swarm_rescue.simulation.gui_map.gui_sr import GuiSR
from swarm_rescue.simulation.gui_map.map_abstract import MapAbstract
from swarm_rescue.simulation.utils.misc_data import MiscData


from swarm_rescue.team_du_sud.geometry_opti import *
from swarm_rescue.team_du_sud.walls_keyboard import add_walls

class OccupancyGrid(Grid):
    """Simple occupancy grid"""

    def __init__(self,
                 size_area_world,
                 resolution: float,
                 lidar):
        super().__init__(size_area_world=size_area_world,
                         resolution=resolution)

        self.size_area_world = size_area_world
        self.resolution = resolution

        self.lidar = lidar

        self.x_max_grid: int = int(self.size_area_world[0] / self.resolution
                                   + 0.5)
        self.y_max_grid: int = int(self.size_area_world[1] / self.resolution
                                   + 0.5)

        self.grid = np.zeros((self.x_max_grid, self.y_max_grid))
        self.zoomed_grid = np.empty((self.x_max_grid, self.y_max_grid))

    def update_grid(self, pose: Pose):
        """
        Bayesian map update with new observation
        lidar : lidar data
        pose : corrected pose in world coordinates
        """
        EVERY_N = 3
        LIDAR_DIST_CLIP = 40.0
        EMPTY_ZONE_VALUE = -0.602
        OBSTACLE_ZONE_VALUE = 2.0
        FREE_ZONE_VALUE = -4.0
        THRESHOLD_MIN = -40
        THRESHOLD_MAX = 40

        lidar_dist = self.lidar.get_sensor_values()[::EVERY_N].copy()
        lidar_angles = self.lidar.ray_angles[::EVERY_N].copy()

        # Compute cos and sin of the absolute angle of the lidar
        cos_rays = np.cos(lidar_angles + pose.orientation)
        sin_rays = np.sin(lidar_angles + pose.orientation)

        max_range = MAX_RANGE_LIDAR_SENSOR * 0.9

        # For empty zones
        # points_x and point_y contains the border of detected empty zone
        # We use a value a little bit less than LIDAR_DIST_CLIP because of the
        # noise in lidar
        lidar_dist_empty = np.maximum(lidar_dist - LIDAR_DIST_CLIP, 0.0)
        # All values of lidar_dist_empty_clip are now <= max_range
        lidar_dist_empty_clip = np.minimum(lidar_dist_empty, max_range)
        points_x = pose.position[0] + np.multiply(lidar_dist_empty_clip,
                                                  cos_rays)
        points_y = pose.position[1] + np.multiply(lidar_dist_empty_clip,
                                                  sin_rays)

        for pt_x, pt_y in zip(points_x, points_y):
            self.add_value_along_line(pose.position[0], pose.position[1],
                                      pt_x, pt_y,
                                      EMPTY_ZONE_VALUE)

        # For obstacle zones, all values of lidar_dist are < max_range
        select_collision = lidar_dist < max_range

        points_x = pose.position[0] + np.multiply(lidar_dist, cos_rays)
        points_y = pose.position[1] + np.multiply(lidar_dist, sin_rays)

        points_x = points_x[select_collision]
        points_y = points_y[select_collision]

        self.add_points(points_x, points_y, OBSTACLE_ZONE_VALUE)

        # the current position of the drone is free !
        self.add_points(pose.position[0], pose.position[1], FREE_ZONE_VALUE)

        # threshold values
        self.grid = np.clip(self.grid, THRESHOLD_MIN, THRESHOLD_MAX)

        # compute zoomed grid for displaying
        self.zoomed_grid = self.grid.copy()
        new_zoomed_size = (int(self.size_area_world[1] * 0.5),
                           int(self.size_area_world[0] * 0.5))
        self.zoomed_grid = cv2.resize(self.zoomed_grid, new_zoomed_size,
                                      interpolation=cv2.INTER_NEAREST)

# --------------------------
# STOCKAGE CLASS
# --------------------------

class Stockage():
    def __init__(self):
        self.drone_start_position = None
        self.rescue_center = None
        self.return_area = None
        self.wounded_position = []
        self.walls_positions = []
        self.NoGPS_position = None

    # RETURN AREA
    def initialize_return_area_position(self, pose):
        if self.drone_start_position[0] == pose.position[0]:
            self.return_area = line(self.drone_start_position, pose.position)
        elif self.drone_start_position[1] == pose.position[1]:
            self.return_area = line(self.drone_start_position, pose.position)
        else:
            self.return_area = box_from_two_points(self.drone_start_position, pose.position)

    def update_return_area_position(self, pose):
        if self.return_area is None:
            self.initialize_return_area_position(pose)
            return

        if self.return_area.shape[0]==4:  # it's a line
            if is_on_line(self.return_area, pose.position):
                return
            else:
                self.return_area = extend_line(self.return_area, pose.position)
        else:
            if box_contains(self.return_area, pose.position):
                return
            else:
                self.return_area = extend_box(self.return_area, pose.position)

    # RESCUE CENTER
    def update_rescue_center_position(self, new_point):
        if self.rescue_center is None:
            self.rescue_center = new_point
            return

        if self.rescue_center.shape[0]==2:  # Point
            if np.allclose(new_point, self.rescue_center): return
            if new_point[0]==self.rescue_center[0]:
                self.rescue_center = line(self.rescue_center, new_point)
            elif new_point[1]==self.rescue_center[1]:
                self.rescue_center = line(self.rescue_center, new_point)
            else:
                self.rescue_center = box_from_two_points(self.rescue_center, new_point)
        elif self.rescue_center.shape[0]==4:  # Line
            if is_on_line(self.rescue_center, new_point):
                return
            else:
                self.rescue_center = extend_line(self.rescue_center, new_point)
        else:  # Box
            self.rescue_center = extend_box(self.rescue_center, new_point)

    # WOUNDED
    def update_wounded_position(self, new_point):
        for i, wounded in enumerate(self.wounded_position):
            if distance(wounded, new_point)<60:
                self.wounded_position[i]=new_point
                return
        self.wounded_position.append(new_point)

    # WALLS
    def update_walls_positions(self, p1, p2, middle):
        t, _ = are_aligned(np.vstack([p1,p2]))
        if t=="undefined": return

        if t=="vertical":
            h = max(abs(p1[1]), abs(p2[1]))
            wall = np.hstack([point(middle[0]-3,h), point(middle[0]+3,h),
                              point(middle[0]+3,-h), point(middle[0]-3,-h)])
        else:
            l = max(abs(p1[0]), abs(p2[0]))
            wall = np.hstack([point(l,middle[1]-3), point(l,middle[1]+3),
                              point(-l,middle[1]+3), point(-l,middle[1]-3)])
        self.walls_positions.append(wall)

class Path():
    def _init_(self):
        self.strategic_points = []
        self.return_area_postion = None
        self.rescue_center_position = None

# --------------------------
# DRONE CLASS
# --------------------------

class MyDroneTest(DroneAbstract):

    class Activity(Enum):
        EXPLORING = 1
        RESCUING = 2

    class Rescuing_Activities(Enum):
        GRASPING_WOUNDED = 3
        DROPPING_AT_RESCUE_CENTER = 4

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.iteration = 0
        self.pose = Pose()
        self.grid = OccupancyGrid(size_area_world=self.size_area, resolution=8, lidar=self.lidar())
        self.data = Stockage()
        self.path = Path()
        self.state = self.Activity.EXPLORING
        self.last_inside = False

    def define_message_for_all(self):
        pass

    def control(self):
        command = {"forward":0.0,"lateral":0.0,"rotation":0.0,"grasper":0}

        self.iteration+=1
        self.pose = Pose(np.asarray(self.measured_gps_position()), self.measured_compass_angle())
        self.grid.update_grid(pose=self.pose)

        if self.iteration%5==0:
            self.grid.display(self.grid.grid,self.pose)
            self.grid.display(self.grid.zoomed_grid,self.pose)

        if self.state==self.Activity.EXPLORING:
            if self.data.drone_start_position is None and self.is_inside_return_area:
                self.data.drone_start_position=self.pose.position.copy()
            elif self.is_inside_return_area != self.last_inside:
                self.last_inside=self.is_inside_return_area
                self.data.update_return_area_position(self.pose)
                if self.data.return_area is not None:
                    self.path.return_area_postion = box_center(self.data.return_area)
            self.process_lidar_semantic_sensors()

        return command

    def process_lidar_semantic_sensors(self):
        lidar_values = self.lidar_values()
        lidar_angles = self.lidar_rays_angles()
        zones = detect_local_zones(lidar_values)
        angles_features = [[lidar_angles[i] for i in zone] for zone in zones]

        semantic_values = self.semantic_values()
        if semantic_values is not None:
            for data in semantic_values:
                for i in range(len(angles_features)):
                    if angles_features[i][0] <= data.angle <= angles_features[i][-1]:
                        self.update_data(data=data)
                        zones.pop(i)
                        angles_features.pop(i)
                        break

        for i in range(len(zones)):
            if len(zones[i])>2:
                self.update_data(data=None,
                                 values_features=[zones[i][0],zones[i][-1],zones[i][len(zones[i])//2]],
                                 angles_features=[angles_features[i][0],angles_features[i][-1],angles_features[i][len(angles_features[i])//2]])

    def update_data(self, data=None, values_features=None, angles_features=None):
        if data is not None:
            new_point = add(self.pose.position, self.pose.orientation, data.distance, data.angle)
            if data.entity_type==DroneSemanticSensor.TypeEntity.RESCUE_CENTER:
                self.data.update_rescue_center_position(new_point)
            elif data.entity_type==DroneSemanticSensor.TypeEntity.WOUNDED_PERSON:
                self.data.update_wounded_position(new_point)
        else:
            p1 = add(self.pose.position, self.pose.orientation, values_features[0], angles_features[0])
            p2 = add(self.pose.position, self.pose.orientation, values_features[1], angles_features[1])
            pm = add(self.pose.position, self.pose.orientation, values_features[2], angles_features[2])
            self.data.update_walls_positions(p1,p2,pm)

class MyMapKeyboard(MapAbstract):

    def __init__(self, drone_type: Type[DroneAbstract]):
        super().__init__(drone_type=drone_type)

        # PARAMETERS MAP
        self._size_area = (600, 600)

        self._rescue_center = RescueCenter(size=(100, 100))
        self._rescue_center_pos = ((0, 100), 0)

        self._return_area = ReturnArea(size=(150, 100))
        self._return_area_pos = ((0, -20), 0)

        self._wounded_persons_pos = [(200, 0), (-200, 0),
                                     (200, -200), (-200, -200)]

        self._number_wounded_persons = len(self._wounded_persons_pos)
        self._wounded_persons: List[WoundedPerson] = []

        self._number_drones = 1
        self._drones_pos = [((0, 0), 0)]
        self._drones = []

        self._playground = ClosedPlayground(size=self._size_area)

        self._playground.add(self._rescue_center, self._rescue_center_pos)

        self._playground.add(self._return_area, self._return_area_pos)

        # POSITIONS OF THE WOUNDED PERSONS
        for i in range(self._number_wounded_persons):
            wounded_person = WoundedPerson(rescue_center=self._rescue_center)
            self._wounded_persons.append(wounded_person)
            pos = (self._wounded_persons_pos[i], 0)
            self._playground.add(wounded_person, pos)

        # POSITIONS OF THE DRONES
        misc_data = MiscData(size_area=self._size_area,
                             number_drones=self._number_drones,
                             max_timestep_limit=self._max_timestep_limit,
                             max_walltime_limit=self._max_walltime_limit)
        for i in range(self._number_drones):
            drone = drone_type(identifier=i, misc_data=misc_data)
            self._drones.append(drone)
            self._playground.add(drone, self._drones_pos[i])


def print_keyboard_man():
    print("How to use the keyboard to direct the drone?")
    print("\t- up / down key : forward and backward")
    print("\t- left / right key : turn left / right")
    print("\t- shift + left/right key : left/right lateral movement")
    print("\t- W key : grasp wounded person")
    print("\t- L key : display (or not) the lidar sensor")
    print("\t- S key : display (or not) the semantic sensor")
    print("\t- P key : draw position from GPS sensor")
    print("\t- C key : draw communication between drones")
    print("\t- M key : print messages between drones")
    print("\t- Q key : exit the program")
    print("\t- R key : reset")


def main():
    print_keyboard_man()
    the_map = MapIntermediate01(drone_type=MyDroneTest)

    # draw_lidar_rays : enable the visualization of the lidar rays
    # draw_semantic_rays : enable the visualization of the semantic rays
    gui = GuiSR(the_map=the_map,
                draw_lidar_rays=False,
                draw_semantic_rays=False,
                use_keyboard=True,
                )
    gui.run()

    score_health_returned = the_map.compute_score_health_returned()
    print("score_health_returned = ", score_health_returned)


if __name__ == '__main__':
    main()