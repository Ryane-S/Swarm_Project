import math
from abc import abstractmethod
from enum import IntEnum, auto
from typing import Optional, Union

import arcade
import numpy as np

from spg.agent.agent import Agent
from spg.agent.communicator.communicator import Communicator
from spg.agent.interactor import GraspMagnet
from spg.element import PhysicalElement, ColorWall
from spg.playground import Playground, get_colliding_entities

from spg_overlay.entities.drone_base import DroneBase
from spg_overlay.entities.drone_abstract import DroneAbstract
from spg_overlay.entities.drone_distance_sensors import DroneLidar, DroneSemanticSensor
from spg_overlay.entities.wounded_person import WoundedPerson
from spg_overlay.entities.drone_sensors import DroneGPS, DroneCompass, DroneOdometer
from spg_overlay.entities.normal_wall import NormalWall, NormalBox
from spg_overlay.utils.constants import DRONE_INITIAL_HEALTH, RANGE_COMMUNICATION
from spg_overlay.utils.misc_data import MiscData

import matplotlib.pyplot as plt

from spg_overlay.utils.timer import Timer
from spg_overlay.utils.utils import normalize_angle

from maps.map_random import MyMapRandom

from .geometry import Point, Line, Box

# Classe où le drône stocke les données importantes de la map
class Stockage():
    def __init__(self, drone_start_position):
        self.drone_start_position = Point(drone_start_position[0], drone_start_position[1])
        self.rescue_center_position = None
        self.return_area_position = None
        self.wounded_position = None
        self.NoGPS_position = None

    def initialize_return_area_position(self, actual_drone_position):
        self.actual_drone_position = Point(actual_drone_position[0], actual_drone_position[1]) 
        if self.drone_start_position.x == self.actual_drone_position.x:
            line = Line(self.actual_drone_position, self.drone_start_position)
            line.update_type("vertical")
            self.return_area_position = line
        elif self.drone_start_position.y == self.actual_drone_position.y:
            line = Line(self.actual_drone_position, self.drone_start_position)
            line.update_type("horizontal")
            self.return_area_position = line
        else:
            points = [self.drone_start_position, Point(self.actual_drone_position.x, self.drone_start_position.y), self.actual_drone_position, Point(self.actual_drone_position.y, self.drone_start_position.x)]
            box = Box(points[0], points[1], points[2], points[3])
            self.return_area_position = box

    def update_return_area_position(self, actual_drone_position):
        pass

    def update_rescue_center_position(self):
        pass

    def update_wounded_position(self):
        pass
    
    def update_NoGPS_position(self):
        pass

# Classe qui construit des chemins à suivre selon les données captées
class Path():
    def _init_(self, drone_actual_position):
        self.strategic_points = [drone_actual_position]

# Create a custom Drone class that inherits from DroneAbstract
class MyDroneTest(DroneAbstract):
    def __init__(self, identifier = None, misc_data = None, display_lidar_graph=False, **kwargs):
        super().__init__(identifier, misc_data, display_lidar_graph, **kwargs)
        self.stockage = Stockage(self, self.measured_gps_position())
        self.path = Path(self.measured_gps_position())
        self.last_inside = False

    def define_message_for_all(self):
        '''
        # Define the message to be sent to other drones
        msg_data = (self.identifier, (self.measured_gps_position(),
        self.measured_compass_angle()))
        return msg_data
        '''

    def process_lidar_sensor(self):
        """
        Returns True if the drone collided an obstacle
        """
        if self.lidar_values() is None:
            return False

        collided = False
        dist = min(self.lidar_values())

        if dist < 40:
            collided = True

        return collided
    
    """
    def control(self):
       # Define the control command for the drone
        command = {"forward": 0.5, "lateral": 0.0, "rotation": 0.0, "grasper": 1} # Rotation entre -1 et 1 <-> -pi et pi
        
        
        collided = self.process_lidar_sensor()
        if collided:
            command["grasper"] = 1
        print(len(self.base.grasper.grasped_entities))
        
        if command_semantic is not None:
            command = command_semantic
            command["grasper"] = 1
            
        self.display_lidar_graph()
    """
    
    def control(self):
        command = {"forward": 0.0,
                   "lateral": 0.0,
                   "rotation": 0.0,
                   "grasper": 0}

        if self.is_inside_return_area != self.last_inside:
            print("is_inside_return_area : ", self.is_inside_return_area)
            self.last_inside = self.is_inside_return_area

        return command
    
    def process_semantic_sensor(self):
        """
        According to his state in the state machine, the Drone will move
        towards a wound person or the rescue center
        """
        detection_semantic = self.semantic_values()
        if detection_semantic is not None:
            for data in detection_semantic: 
                # If the wounded person detected is held by nobody
                if (data.entity_type ==
                        DroneSemanticSensor.TypeEntity.WOUNDED_PERSON):
                    print("type: wounded, angle: ",data.angle, "d: ",data.distance)
                    return True