"""
This program can be launched directly.
To move the drone, you have to click on the map, then use the arrows on the
keyboard
"""

import sys
from pathlib import Path
from typing import List, Type
from enum import Enum
import arcade
import numpy as np

# Insert the parent directory of the current file's directory into sys.path.
# This allows Python to locate modules that are one level above the current
# script, in this case spg_overlay.
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from swarm_rescue.simulation.drone.controller import CommandsDict
from swarm_rescue.simulation.drone.drone_abstract import DroneAbstract
from swarm_rescue.simulation.ray_sensors.drone_semantic_sensor import DroneSemanticSensor
from swarm_rescue.simulation.elements.rescue_center import RescueCenter
from swarm_rescue.simulation.elements.return_area import ReturnArea
from swarm_rescue.maps.map_intermediate_01 import MapIntermediate01
from swarm_rescue.simulation.utils.pose import Pose
from swarm_rescue.simulation.elements.wounded_person import WoundedPerson
from swarm_rescue.simulation.gui_map.closed_playground import ClosedPlayground
from swarm_rescue.simulation.gui_map.gui_sr import GuiSR
from swarm_rescue.simulation.gui_map.map_abstract import MapAbstract
from swarm_rescue.simulation.utils.misc_data import MiscData


from swarm_rescue.team_du_sud.geometry import Point, Line, Box, build_box_with_line_and_point, detect_local_zones, build_box_with_2_opposite_points
from swarm_rescue.team_du_sud.walls_keyboard import add_walls


class Stockage():
    def __init__(self):
        self.drone_start_position = None
        self.rescue_center = None
        self.return_area = None
        self.wounded_position = None
        self.walls_positions = None
        self.NoGPS_position = None

    # RETURN AREA
    def initialize_return_area_position(self, actual_drone_position):
        self.actual_drone_position = Point(actual_drone_position[0], actual_drone_position[1]) 
        if self.drone_start_position.x == self.actual_drone_position.x:
            line = Line(self.actual_drone_position, self.drone_start_position)
            line.update_type("vertical")
            self.return_area = line
        elif self.drone_start_position.y == self.actual_drone_position.y:
            line = Line(self.actual_drone_position, self.drone_start_position)
            line.update_type("horizontal")
            self.return_area = line
        else:
            box = build_box_with_2_opposite_points(self.drone_start_position, self.actual_drone_position)
            self.return_area = box

    def update_return_area_position(self, actual_drone_position):
        self.actual_drone_position = Point(actual_drone_position[0], actual_drone_position[1])
        if self.return_area is not None:
            if isinstance(self.return_area, Line):
                if self.return_area.is_on_line(self.actual_drone_position):
                    pass
                elif self.return_area.is_aligned(self.actual_drone_position):
                    self.return_area.extend_line(self.actual_drone_position)
                else:
                    new_box = build_box_with_line_and_point(self.return_area, self.actual_drone_position)
                    self.return_area = new_box
            elif isinstance(self.return_area, Box):
                if self.return_area.is_inside(self.actual_drone_position):
                    pass
                else:
                    self.return_area.extend_box(self.actual_drone_position)
        else:
            self.initialize_return_area_position(actual_drone_position)

    # RESCUE CENTER    
    def update_rescue_center_position(self, new_point : Point):
        if self.rescue_center is None:
            self.rescue_center = new_point

        elif isinstance(self.rescue_center, Point):
            if new_point == self.rescue_center :
                pass
            else:
                if new_point.x == self.rescue_center.x:
                    self.rescue_center = Line(self.rescue_center, new_point)
                    self.rescue_center.update_type("vertical")
                elif new_point.y == self.rescue_center.y:
                    self.rescue_center = Line(self.rescue_center, new_point)
                    self.rescue_center.update_type("horizontal")
                else:
                    new_box = build_box_with_2_opposite_points(self.rescue_center, new_point)
                    self.rescue_center = new_box

        elif isinstance(self.rescue_center, Line):
            if self.rescue_center.is_on_line(new_point):
                pass
            elif self.rescue_center.is_aligned(new_point):
                self.rescue_center.extend_line(new_point)
            else:
                new_box = build_box_with_line_and_point(self.rescue_center, new_point)
                self.rescue_center = new_box
    
        elif isinstance(self.rescue_center, Box):
            self.rescue_center.extend_box(new_point)

    # WOUNDED
    def update_wounded_position(self, new_point: Point):
        if self.wounded_position is None:
            self.wounded_position = [new_point]
        else:
            for wounded in self.wounded_position:
                if new_point.distance_to(wounded) < 60:
                # Remplace l'ancienne position par la nouvelle plus précise
                    self.wounded_position.remove(wounded)
                    self.wounded_position.append(new_point)
                    break
            else:
                # Aucun blessé existant à moins de 30 pixels → c'est un nouveau
                if new_point not in self.wounded_position:
                    self.wounded_position.append(new_point)

    # WALLS
    def update_walls_positions(self, new_point_1:Point, new_point_2:Point, middle_point:Point):
        aligned_type = middle_point.are_aligned_in_type(new_point_1, new_point_2)
        if aligned_type[0] == "undefined":
            pass
        else:
            if aligned_type[0] == "vertical":
                max_height = max(abs(new_point_1.y), abs(new_point_2.y))
                point_1 = Point(middle_point.x-3, max_height)
                point_2 = Point(middle_point.x+3, max_height)
                point_3 = Point(middle_point.x+3, -max_height)
                point_4 = Point(middle_point.x-3, -max_height)
                new_wall = Box(point_1, point_2, point_3, point_4)
            elif aligned_type[0] == "horizontal":
                max_length = max(abs(new_point_1.x), abs(new_point_2.x))
                point_1 = Point(max_length, middle_point.y-3)
                point_2 = Point(max_length, middle_point.y+3)
                point_3 = Point(-max_length, middle_point.y+3)
                point_4 = Point(-max_length, middle_point.y-3)
                new_wall = Box(point_1, point_2, point_3, point_4)

            if self.walls_positions is None:
                self.walls_positions = [new_wall]
            else:
                self.walls_positions.append(new_wall)

    # NOGPS ZONE
    def update_NoGPS_position(self):
        pass

class Path():
    def __init__(self):
        self.n_points = 0
        self.references = {}
        self.graph = {}
        self.to_explore = []

class MyDroneTest(DroneAbstract):
    class Activity(Enum):
        """
        The drone is either exploring the map or rescuing wounded persons.
        """
        EXPLORING = 1
        RESCUING = 2
        RESET = 3

    class Exploring_Activities(Enum):
        DEFINE_NEXT_POINT = 6
        GOING_TO_POINT = 5

    class Rescuing_Activities(Enum):
        """
        All the states of the drone during the rescue of wounded persons.
        """
        GRASPING_WOUNDED = 3
        DROPPING_AT_RESCUE_CENTER = 4


    def __init__(self, **kwargs):
        super().__init__(**kwargs)

        self.last_inside = False
        self.state = self.Activity.EXPLORING
        self.activity = self.Exploring_Activities.DEFINE_NEXT_POINT
        self.wounded_detected = False
        #self.data = Stockage()
        self.path = Path()
        self.return_area_is_detected = False
        self.rescue_center_is_detected = False
        self.points_to_analyse = []
        self.fixed_position = 0
        self.fixed_orientation = 0
        self.previous_point = None

    def define_message_for_all(self):
        """
        Here, we don't need communication...
        """
        pass

    def control(self):
        command: CommandsDict = {"forward": 0.0,
                                 "lateral": 0.0,
                                 "rotation": 0.0,
                                 "grasper": 0}
        
        # Get the position of the drone
        self.actual_position = Point(self.measured_gps_position()[0], self.measured_gps_position()[1])

        # Get the orientation of the drone
        self.orientation = round(self.compass_values(), 2)
        '''
        0,0347 degrees between each ray
        print(self.semantic().ray_angles[17])
        print(len(self.semantic().get_sensor_values()))
        print(self.compass_values()) # Absolute orientation of the nose of the drone
        print(self.lidar().ray_angles[0]) # -PI à Droite (relative position)
        print(self.lidar().ray_angles[90]) # Nose of the drone
        print(self.lidar().ray_angles[180]) # PI à Gauche (relative position)
        print(self.lidar().get_sensor_values()[90]) # Distance juste devant le drone
        '''
        # EXPLORING STATE
        if self.state == self.Activity.EXPLORING:

            # The fist point of the graph is the return area
            if len(self.path.graph) == 0 and self.is_inside_return_area :
                self.path.graph["RA"] = ["explored"]
                self.path.references["RA"] = self.actual_position
                self.return_area_is_detected = True
                UP, DOWN, LEFT, RIGHT = ("UP",Point(self.actual_position.x, self.actual_position.y+50)), ("DOWN",Point(self.actual_position.x, self.actual_position.y-50)), ("LEFT",Point(self.actual_position.x+50, self.actual_position.y)), ("RIGHT",Point(self.actual_position.x-50, self.actual_position.y))
                self.points_to_analyse = [UP, DOWN, LEFT, RIGHT]
                self.check_cardinal_points(lidar_sensor_values)
                self.previous_point = "RA"
                self.state = self.Exploring_Activities.GOING_TO_POINT

                return command

            # Define points and build the graph by exploring them
            else:
                if self.activity == self.Exploring_Activities.DEFINE_NEXT_POINT: # The drone is immobilized during this phase
                    # Measures are noisy so we set a reference
                    if self.fixed_orientation == 0 and self.fixed_position == 0:
                        self.fixed_orientation = self.orientation
                        self.fixed_position = self.actual_drone_position

                    if len(self.points_to_analyse) == 0:
                        UP, DOWN, LEFT, RIGHT = ("UP",Point(self.fixed_position.x, self.fixed_position.y+50)), ("DOWN",Point(self.fixed_position.x, self.fixed_position.y-50)), ("LEFT",Point(self.fixed_position.x+50, self.fixed_position.y)), ("RIGHT",Point(self.fixed_position.x-50, self.fixed_position.y))
                        self.points_to_analyse = [UP, DOWN, LEFT, RIGHT]

                    self.previous_point = self.path.n_points
                    lidar_sensor_values = self.lidar_values()
                    self.check_cardinal_points(lidar_sensor_values)
                    self.state = self.Exploring_Activities.GOING_TO_POINT

                    return command

                if self.activity == self.Exploring_Activities.GOING_TO_POINT: # The drone moves to the next point
                    if self.path.n_points == self.previous_point: # Nothing was added in the graph
                        pass
                
                    return command

        # RESCUING STATE
        elif self.state == self.Activity.RESCUING:
            pass

        return command

    # Drawings
    def draw_bottom_layer(self):
        self.draw_graph()

    def draw_graph(self):
        for point in self.path.graph:
            arcade.draw_point(self.path.references[point].x+400, self.path.references[point].y+250, arcade.color.RED, 10)

    # NO GPS CASE
    def control_without_gps(self):
        return {"forward": 0.0, "lateral": 0.0, "rotation": 0.0, "grasper": 0}
    
    # Check if the cardinal points can be explored
    def check_cardinal_points(self, lidar_sensor_values):
        feature_points=[]
        for direction in self.points_to_analyse:
            if direction[0] == "UP":
                diff = (np.pi/2 - self.fixed_orientation + np.pi) % (2 * np.pi) - np.pi
                idx = max(0, min(180, 90 + int(round(diff / 0.0347))))
            elif direction[0] == "DOWN":
                diff = (-np.pi/2 - self.fixed_orientation + np.pi) % (2 * np.pi) - np.pi
                idx = max(0, min(180, 90 + int(round(diff / 0.0347))))
            elif direction[0] == "LEFT":
                diff = (0 - self.fixed_orientation + np.pi) % (2 * np.pi) - np.pi
                idx = max(0, min(180, 90 + int(round(diff / 0.0347))))
            else:
                diff = (np.pi - self.fixed_orientation + np.pi) % (2 * np.pi) - np.pi
                idx = max(0, min(180, 90 + int(round(diff / 0.0347))))

            # Check if an already defined point is nearby
            if (lidar_sensor_values[idx] > 60):
                self.path.n_points += 1
                self.path.graph[self.path.n_points] = ["explored", self.previous_point]
                self.path.references[self.path.n_points] = direction[1]
            else:
                feature_points.append((direction, idx))

            self.points_to_analyse.remove(direction)

        # Specific obstacles were found
        if len(feature_points) != 0:
            self.process_semantic_sensor(feature_points)

    def process_semantic_sensor(self, feature_points):
        semantic_sensor_values = self.semantic_values()
                







    def process_lidar_semantic_sensors(self):
        lidar_sensor_values = self.lidar_values()
        lidar_sensor_angles = self.lidar_rays_angles()
        values_features = detect_local_zones(lidar_sensor_values)
        angles_features = [[lidar_sensor_angles[i] for i in zone] for zone in values_features]

        semantic_sensor_values = self.semantic_values()
        if semantic_sensor_values is not None:
            for data in semantic_sensor_values:
                for i in range(len(angles_features)):
                    if angles_features[i][0] <= data.angle <= angles_features[i][-1]:
                        self.update_data(data, None, None)
                        values_features.pop(i)
                        angles_features.pop(i)
                        break
    
        if values_features != []:
            for i in range(len(values_features)):
                if len(values_features[i]) == 2:
                    pass
                else:
                    self.update_data(None, [values_features[i][0], values_features[i][-1], values_features[i][int(len(values_features[i])/2)+1]], [angles_features[i][0], angles_features[i][-1], angles_features[i][int(len(angles_features[i])/2)+1]])


    def update_data(self, data, values_features, angles_features):
        orientation = self.compass_values()
        if data is not None: # WOUNDED or RESCUE CENTER
            gps_position = self.measured_gps_position()
            self.actual_drone_position = Point(gps_position[0], gps_position[1])
            new_point = self.actual_drone_position.add(orientation, data.distance, data.angle)

            if data.entity_type == DroneSemanticSensor.TypeEntity.RESCUE_CENTER:
                self.data.update_rescue_center_position(new_point)

            elif data.entity_type == DroneSemanticSensor.TypeEntity.WOUNDED_PERSON:
                self.data.update_wounded_position((new_point))
                print(len(self.data.wounded_position))

        else: # WALL
            new_point_1 = self.actual_drone_position.add(orientation, values_features[0], angles_features[0])
            new_point_2 = self.actual_drone_position.add(orientation, values_features[1], angles_features[1])
            middle_point = self.actual_drone_position.add(orientation, values_features[2], angles_features[2])
            self.data.update_walls_positions(new_point_1, new_point_2, middle_point)

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

def main():
    the_map = MapIntermediate01(drone_type=MyDroneTest)

    # draw_lidar_rays : enable the visualization of the lidar rays
    # draw_semantic_rays : enable the visualization of the semantic rays
    gui = GuiSR(the_map=the_map,
                draw_lidar_rays=True,
                draw_semantic_rays=True,
                use_keyboard=True,
                )
    gui.run()

    score_health_returned = the_map.compute_score_health_returned()
    print("score_health_returned = ", score_health_returned)


if __name__ == '__main__':
    main()