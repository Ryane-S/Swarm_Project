"""
This program can be launched directly.
"""
# This line add, to sys.path, the path to parent path of this file
import math
import pathlib
import sys
from typing import List, Type, Tuple

import arcade
import os
import numpy as np

sys.path.insert(0,os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent / "src"))

from swarm_rescue.simulation.drone.controller import CommandsDict
from swarm_rescue.simulation.utils.path import Path
from swarm_rescue.simulation.utils.pose import Pose
from swarm_rescue.simulation.utils.utils import clamp
from swarm_rescue.simulation.drone.drone_abstract import DroneAbstract
from swarm_rescue.simulation.gui_map.closed_playground import ClosedPlayground
from swarm_rescue.simulation.gui_map.gui_sr import GuiSR
from swarm_rescue.simulation.gui_map.map_abstract import MapAbstract
from swarm_rescue.simulation.utils.misc_data import MiscData

class MyDronePidTranslation(DroneAbstract):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)

        self.target_points = [
            (150, 100),  # Premier point
            (150, -100),  # Deuxième point
            (-150, 100)   # Troisième point
        ]

        self.iter_path = 0  # Compteur pour le tracé de la trajectoire
        self.path_done = Path()
        self.prev_diff_position = 0
        self.current_target_idx = 0  # Indice du point cible actuel
        self.position_setpoint = self.target_points[self.current_target_idx]
        self.Kp = 1.1
        self.Kd = 11
        self.T_cr = None  # Période d'oscillation
        self.K_cr = None  # Gain critique

    def define_message_for_all(self):
        """
        Here, we don't need communication...
        """
        pass

    def control(self):
        """
        Commande du drone vers le point cible, avec les paramètres PID ajustés.
        """
        command = {"forward": 0.0, "lateral": 0, "rotation": 0.0}
        diff_position = self.position_setpoint - np.asarray(self.gps().get_sensor_values())
        deriv_diff_position = diff_position - self.prev_diff_position
        
        forward = (self.Kp * float(diff_position[0]) +
                   self.Kd * float(deriv_diff_position[0]))
        lateral = (self.Kp * float(diff_position[1]) +
                   self.Kd * float(deriv_diff_position[1]))

        forward = clamp(forward, -1.0, 1.0)
        lateral = clamp(lateral, -1.0, 1.0)
        self.prev_diff_position = diff_position

        # Vérifier si le drone est proche du point cible
        if np.linalg.norm(diff_position) < 5:  # Tolérance de 5 pixels
            self.current_target_idx += 1
            if self.current_target_idx < len(self.target_points):
                self.position_setpoint = self.target_points[self.current_target_idx]

        return {"forward": forward, "lateral": lateral, "rotation": 0.0}


    """Dessins"""
    def draw_bottom_layer(self):
        self.draw_setpoint()
        # Dessiner la grille
        self.draw_grid()
        # Dessiner le setpoint (position cible du drone)
        self.draw_setpoint()

    def draw_grid(self):
        # Taille de la grille
        grid_size = 50  # Taille des cases de la grille
        width, height = self._size_area  # Taille de la zone de jeu (en pixels)
        
        # Couleur de la grille
        grid_color = (200, 200, 200)  # Gris clair pour la grille

        # Dessiner les lignes verticales de la grille
        for x in range(0, width, grid_size):
            arcade.draw_line(x, 0, x, height, grid_color, 1)

        # Dessiner les lignes horizontales de la grille
        for y in range(0, height, grid_size):
            arcade.draw_line(0, y, width, y, grid_color, 1)


    def draw_setpoint(self):
        half_width = self._half_size_array[0]
        half_height = self._half_size_array[1]
        arcade.draw_point(250, 500, arcade.color.RED, 10)
        arcade.draw_point(550, 500, arcade.color.RED, 10)
        arcade.draw_point(550, 300, arcade.color.RED, 10)
        arcade.draw_point(400, 400, arcade.color.BLACK, 10)


class MyMap(MapAbstract):


    def __init__(self, drone_type: Type[DroneAbstract]):
        super().__init__(drone_type=drone_type)

        # PARAMETERS MAP
        self._size_area = (800, 800)

        # POSITIONS OF THE DRONES
        self._number_drones = 1
        self._drones_pos = []
        for i in range(self._number_drones):
            pos = ((-150, 100), 0)
            self._drones_pos.append(pos)

        self._drones: List[DroneAbstract] = []
        self._playground = ClosedPlayground(size=self._size_area)

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
    my_map = MyMap(drone_type=MyDronePidTranslation)

    gui = GuiSR(the_map=my_map,
                use_keyboard=False,
                use_mouse_measure=True,
                enable_visu_noises=False,
                )
    gui.run()

    score_health_returned = my_map.compute_score_health_returned()

    print("score_health_returned = ", score_health_returned)


if __name__ == '__main__':


    main()