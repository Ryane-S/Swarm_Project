"""
This program can be launched directly.
"""

import sys, math
from pathlib import Path
from typing import List, Type
from enum import Enum

# Insert the parent directory of the current file's directory into sys.path.
# This allows Python to locate modules that are one level above the current
# script, in this case spg_overlay.
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from swarm_rescue.simulation.drone.controller import CommandsDict
from swarm_rescue.simulation.drone.drone_abstract import DroneAbstract
from swarm_rescue.simulation.ray_sensors.drone_semantic_sensor import DroneSemanticSensor
from swarm_rescue.simulation.elements.rescue_center import RescueCenter
from swarm_rescue.simulation.elements.return_area import ReturnArea
from swarm_rescue.simulation.elements.wounded_person import WoundedPerson
from swarm_rescue.simulation.gui_map.closed_playground import ClosedPlayground
from swarm_rescue.maps.map_intermediate_01 import MapIntermediate01
from swarm_rescue.simulation.gui_map.gui_sr import GuiSR
from swarm_rescue.simulation.gui_map.map_abstract import MapAbstract
from swarm_rescue.simulation.utils.misc_data import MiscData


from swarm_rescue.team_du_sud.geometry import Box, Point, Line, build_box_with_2_opposite_points, build_box_with_line_and_point, detect_local_zones
from swarm_rescue.team_du_sud.QuadTree import QuadTree, Node 

import math
from enum import Enum
from swarm_rescue.team_du_sud.geometry import Box, Point, Line, build_box_with_2_opposite_points, build_box_with_line_and_point, detect_local_zones
from swarm_rescue.team_du_sud.QuadTree import QuadTree


class Stockage():
    def __init__(self):
        self.drone_start_position = None
        self.rescue_center = None
        self.return_area = None
        self.wounded_positions = []  # Liste simplifiée
        self.walls_positions = None
        
        # QuadTree simplifiée
        self.quadtree = None
        self.min_cell_size = 0.5
        self.qt_origin = None
        self.qt_size = 50  # Plus petite zone pour commencer

    def init_quadtree(self, drone_position):
        """Initialise une QuadTree simple"""
        self.quadtree = QuadTree(w=self.qt_size, h=self.qt_size, min_size=self.min_cell_size)
        self.qt_origin = Point(
            drone_position.x - self.qt_size/2,
            drone_position.y - self.qt_size/2
        )
    
    def to_qt_coords(self, point):
        """Conversion simple"""
        if self.qt_origin:
            return Point(point.x - self.qt_origin.x, point.y - self.qt_origin.y)
        return point
'''
class MyDrone(DroneAbstract):
    class State(Enum):
        EXPLORING = 1
        GOING_TO_WOUNDED = 2
        RETURNING_WITH_WOUNDED = 3
        DROPPING_WOUNDED = 4
    
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        
        self.data = Stockage()
        self.state = self.State.EXPLORING
        self.last_inside = False
        
        # Blessé en cours
        self.current_wounded = None
        self.grasp_distance = 60
        self.return_target = None
        self.wounded_grasped = False
        self.wounded_confirmed_grasped = False  # NOUVEAU: confirmation avec data.grasped
        
        # Pour gérer le dépôt
        self.drop_attempts = 0
        self.max_drop_attempts = 50

    def define_message_for_all(self):
        return {}
    
    def control(self):
        """Contrôle principal avec gestion du grasped"""
        command = {"forward": 0.1, "lateral": 0.0, "rotation": 0.0, "grasper": 1.0}  # grasper TOUJOURS à 1
        
        # Données courantes
        gps = self.measured_gps_position()
        orientation = self.compass_values()
        semantic_data = self.semantic_values()
        current_pos = Point(gps[0], gps[1])
        
        # Initialisation QuadTree
        if self.data.quadtree is None:
            self.data.init_quadtree(current_pos)
        
        # Gestion zone de retour
        if self.data.drone_start_position is None and self.is_inside_return_area == True:
            self.data.drone_start_position = current_pos

        elif self.is_inside_return_area != self.last_inside:
            self.last_inside = self.is_inside_return_area
            if self.is_inside_return_area is not None:
                pass
        
        # Traitement sémantique AVEC grasped
        self._process_semantic_data_with_grasped(gps, orientation, semantic_data, command)
        
        # Machine à états
        if self.state == self.State.EXPLORING:
            command = self._explore_control(command, current_pos)
            
        elif self.state == self.State.GOING_TO_WOUNDED:
            command = self._go_to_wounded_control(command, gps, semantic_data)  # AJOUT: semantic_data
            
        elif self.state == self.State.RETURNING_WITH_WOUNDED:
            command = self._return_with_wounded_control(command, gps)
            
        elif self.state == self.State.DROPPING_WOUNDED:
            command = self._drop_wounded_control(command, gps, semantic_data)

        print(self.state)
        
        return command
    
    def _process_semantic_data_with_grasped(self, gps, orientation, semantic_data, command):
        """Traite les données sémantiques avec la propriété grasped"""
        if not semantic_data:
            return
        
        for data in semantic_data:
            # Calcul position
            dx = data.distance * math.cos(orientation + data.angle)
            dy = data.distance * math.sin(orientation + data.angle)
            pos = Point(gps[0] + dx, gps[1] + dy)
            
            if data.entity_type == DroneSemanticSensor.TypeEntity.RESCUE_CENTER:
                # Mémorise le centre
                self._update_rescue_center_box(pos)
                    
            elif data.entity_type == DroneSemanticSensor.TypeEntity.WOUNDED_PERSON:
                # CRITIQUE: Utilise data.grasped pour savoir si le blessé est déjà attrapé
                if not data.grasped:  # Blessé NON attrapé
                    # Si on n'est PAS déjà en train de sauver un blessé
                    if not self.wounded_grasped and self.state != self.State.RETURNING_WITH_WOUNDED:
                        # Ajoute à la liste si nouveau
                        is_new = True
                        for wounded in self.data.wounded_positions:
                            if pos.distance_to(wounded) < 60:
                                is_new = False
                                break
                        
                        if is_new:
                            self.data.wounded_positions.append(pos)
                            
                            # Si en exploration, commence le sauvetage
                            if (self.state == self.State.EXPLORING and 
                                self.current_wounded is None):
                                self.current_wounded = pos
                                self.state = self.State.GOING_TO_WOUNDED
                                self.wounded_confirmed_grasped = False  # Reset confirmation
    
    def _update_rescue_center_box(self, new_point):
        """Met à jour le rescue center"""
        if self.data.rescue_center is None:
            self.data.rescue_center = new_point
        elif isinstance(self.data.rescue_center, Point):
            self.data.rescue_center = build_box_with_2_opposite_points(
                self.data.rescue_center, new_point
            )
        elif isinstance(self.data.rescue_center, Box):
            self.data.rescue_center.extend_box(new_point)
    
    def _explore_control(self, command, current_pos):
        """Exploration"""
        # Comportement d'exploration par défaut
        command["forward"] = 0.4
        command["rotation"] = 0.05
        
        # Si on a des blessés non traités
        if (len(self.data.wounded_positions) != 0 and  
            len(self.grasped_wounded_persons()) == 0):
            self.current_wounded = self.data.wounded_positions[0]
            self.state = self.State.GOING_TO_WOUNDED
            self.wounded_confirmed_grasped = False

        
        return command
    
    def _go_to_wounded_control(self, command, gps, semantic_data):
        """Va vers un blessé - CONFIRME avec data.grasped"""
        if self.current_wounded is None:
            self.state = self.State.EXPLORING
            self.wounded_grasped = False
            self.wounded_confirmed_grasped = False
            return command
        
        current = Point(gps[0], gps[1])
        distance = current.distance_to(self.current_wounded)
        
        # 1. Si assez proche physiquement
        if distance < self.grasp_distance:
            self.wounded_grasped = True
        
            # 3. Si confirmé, va au centre
            if len(self.grasped_wounded_persons()) != 0:
                if self.data.rescue_center:
                    if isinstance(self.data.rescue_center, Point):
                        self.return_target = self.data.rescue_center
                    elif isinstance(self.data.rescue_center, Box):
                        self.return_target = self.data.rescue_center.get_closest_point(current)
                    
                    self.state = self.State.RETURNING_WITH_WOUNDED
                else:
                    self.state = self.State.EXPLORING
            else:
                # Pas encore confirmé, reste en approche
                pass
        else:
            # Continue à s'approcher
            self._go_to_point(current, self.current_wounded, command)
        
        return command
    
    def _return_with_wounded_control(self, command, gps):
        """Retourne au centre avec blessé"""
        # grasper reste à 1 (déjà dans la commande initiale)
        
        if self.return_target is None:
            current = Point(gps[0], gps[1])
            if self.data.rescue_center:
                if isinstance(self.data.rescue_center, Point):
                    self.return_target = self.data.rescue_center
                elif isinstance(self.data.rescue_center, Box):
                    self.return_target = self.data.rescue_center.get_closest_point(current)
            else:
                command["forward"] = 0.3
                return command
        
        current = Point(gps[0], gps[1])
        distance = current.distance_to(self.return_target)
        
        if distance < 30:  # Proche du centre
            # Passe en mode dépôt
            self.state = self.State.DROPPING_WOUNDED
            self.drop_attempts = 0
        else:
            self._go_to_point(current, self.return_target, command)
        
        return command
    
    def _drop_wounded_control(self, command, gps, semantic_data):
        """Dépose le blessé au centre"""
        
        if len(self.grasped_wounded_persons()) != 0:
            # Blessé toujours attrapé, essaie de le déposer
            self.drop_attempts += 1
            
            # Avance et tourne pour essayer de déposer
            command["forward"] = 0.5
            command["rotation"] = 0.05  # Tourne à gauche
            
            # Si trop d'essais, abandonne et retourne exploration
            if self.drop_attempts > self.max_drop_attempts:
                self._reset_after_drop()
        else:
            # Plus de blessé grasped = dépôt réussi !
            self._reset_after_drop()
        
        return command
    
    def _reset_after_drop(self):
        """Réinitialise après dépôt"""
        if self.current_wounded in self.data.wounded_positions:
            self.data.wounded_positions.remove(self.current_wounded)
        
        self.state = self.State.EXPLORING
        self.current_wounded = None
        self.wounded_grasped = False
        self.wounded_confirmed_grasped = False
        self.return_target = None
        self.drop_attempts = 0
    
    def _go_to_point(self, current, target, command):
        """Navigation simple"""
        dx = target.x - current.x
        dy = target.y - current.y
        target_angle = math.atan2(dy, dx)
        current_angle = self.compass_values()
        angle_diff = target_angle - current_angle
        
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi
        
        if abs(angle_diff) > 0.5:
            command["rotation"] = 0.7 if angle_diff > 0 else -0.7
            command["forward"] = 0.1
        elif abs(angle_diff) > 0.2:
            command["rotation"] = angle_diff * 0.8
            command["forward"] = 0.3
        else:
            command["rotation"] = angle_diff * 0.5
            command["forward"] = 0.4
    
    def _simple_obstacle_avoidance(self, command):
        """Évitement d'obstacles"""
        lidar = self.lidar_values()
        if lidar and min(lidar) < 0.8:
            command["forward"] = 0.5
            command["rotation"] = 0.5
        
        if self.wounded_grasped:
            command["forward"] = min(command["forward"], 0.3)
        
        return command
'''
class Stockage():
    def __init__(self):
        self.drone_start_position = None
        self.rescue_center = None
        self.return_area = None
        self.wounded_positions = []  # Liste simplifiée
        self.walls_positions = None
        
        # QuadTree simplifiée
        self.quadtree = None
        self.min_cell_size = 0.5
        self.qt_origin = None
        self.qt_size = 50  # Plus petite zone pour commencer

    def init_quadtree(self, drone_position):
        """Initialise une QuadTree simple"""
        self.quadtree = QuadTree(w=self.qt_size, h=self.qt_size, min_size=self.min_cell_size)
        self.qt_origin = Point(
            drone_position.x - self.qt_size/2,
            drone_position.y - self.qt_size/2
        )
    
    def to_qt_coords(self, point):
        """Conversion simple"""
        if self.qt_origin:
            return Point(point.x - self.qt_origin.x, point.y - self.qt_origin.y)
        return point

class MyDrone(DroneAbstract):
    class State(Enum):
        EXPLORING = 1
        GOING_TO_WOUNDED = 2
        RETURNING_WITH_WOUNDED = 3
        DROPPING_WOUNDED = 4
    
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        
        self.data = Stockage()
        self.state = self.State.EXPLORING
        self.last_inside = False
        
        # Blessé en cours
        self.current_wounded = None
        self.grasp_distance = 60
        self.return_target = None

        self.is_oriented_to_rescue = False

    def define_message_for_all(self):
        return {}
    
    def control(self):
        """Contrôle principal"""
        command = {"forward": 0.1, "lateral": 0.0, "rotation": 0.0, "grasper": 1.0}
        
        # Données courantes
        gps = self.measured_gps_position()
        orientation = self.compass_values()
        semantic_data = self.semantic_values()
        current_pos = Point(gps[0], gps[1])
        
        # Initialisation QuadTree
        if self.data.quadtree is None:
            self.data.init_quadtree(current_pos)
        
        # Gestion zone de retour
        if self.data.drone_start_position is None and self.is_inside_return_area == True:
            self.data.drone_start_position = current_pos

        elif self.is_inside_return_area != self.last_inside:
            self.last_inside = self.is_inside_return_area
            if self.is_inside_return_area is not None:
                pass
        
        # Traitement sémantique
        self._process_semantic_data(gps, orientation, semantic_data)
        
        # Machine à états
        if self.state == self.State.EXPLORING:
            command = self._explore_control(command, current_pos)
            
        elif self.state == self.State.GOING_TO_WOUNDED:
            command = self._go_to_wounded_control(command, gps)
            
        elif self.state == self.State.RETURNING_WITH_WOUNDED:
            command = self._return_with_wounded_control(command, gps)
            
        elif self.state == self.State.DROPPING_WOUNDED:
            command = self._drop_wounded_control(command, gps)

        print(self.state)
        
        return command
    
    def _process_semantic_data(self, gps, orientation, semantic_data):
        """Traite les données sémantiques"""
        if not semantic_data:
            return
        
        for data in semantic_data:
            # Calcul position
            dx = data.distance * math.cos(orientation + data.angle)
            dy = data.distance * math.sin(orientation + data.angle)
            pos = Point(gps[0] + dx, gps[1] + dy)
            
            if data.entity_type == DroneSemanticSensor.TypeEntity.RESCUE_CENTER:
                # Mémorise le centre
                self._update_rescue_center_box(pos)
                    
            elif data.entity_type == DroneSemanticSensor.TypeEntity.WOUNDED_PERSON:
                # Vérifie si le blessé est déjà attrapé (grâce à data.grasped)
                if not data.grasped:  # Blessé NON attrapé
                    # Ajoute à la liste si nouveau
                    is_new = True
                    for wounded in self.data.wounded_positions:
                        if pos.distance_to(wounded) < 60:
                            is_new = False
                            break
                    
                    if is_new:
                        self.data.wounded_positions.append(pos)
                        
                        # Si en exploration et pas déjà en train d'aller vers un blessé
                        if (self.state == self.State.EXPLORING and 
                            self.current_wounded is None):
                            self.current_wounded = pos
                            self.state = self.State.GOING_TO_WOUNDED
    
    def _update_rescue_center_box(self, new_point):
        """Met à jour le rescue center"""
        if self.data.rescue_center is None:
            self.data.rescue_center = new_point
        elif isinstance(self.data.rescue_center, Point):
            self.data.rescue_center = build_box_with_2_opposite_points(
                self.data.rescue_center, new_point
            )
        elif isinstance(self.data.rescue_center, Box):
            self.data.rescue_center.extend_box(new_point)
    
    def _explore_control(self, command, current_pos):
        """Exploration"""
        # Comportement d'exploration par défaut
        command["forward"] = 0.2
        command["rotation"] = 0.05

        # Pour éviter les obstacles (murs, rescue center)
        lidar = self.lidar_values()
        if len(lidar) !=0 and lidar[len(lidar)//2] < 40:
            command["forward"] = 0.2
            command["rotation"] = 0.5
        
        # Ralentit si un blessé est attrapé
        if len(self.grasped_wounded_persons()) != 0:
            command["forward"] = min(command["forward"], 0.3)
        
        # Si on a des blessés non traités et pas de blessé actuellement attrapé
        if (len(self.data.wounded_positions) != 0 and  
            len(self.grasped_wounded_persons()) == 0 and
            self.current_wounded is None):
            
            self.current_wounded = self.data.wounded_positions[0]
            self.state = self.State.GOING_TO_WOUNDED
        
        return command
    
    def _go_to_wounded_control(self, command, gps):
        """Va vers un blessé"""
        if self.current_wounded is None:
            self.state = self.State.EXPLORING
            return command
        
        current = Point(gps[0], gps[1])
        
        # Vérifie si un blessé est attrapé
        if len(self.grasped_wounded_persons()) != 0:
            # Un blessé est attrapé, on va au rescue center
            if self.data.rescue_center:
                # Utilise le centre de la zone rescue center comme cible
                if isinstance(self.data.rescue_center, Box):
                    self.return_target = self.data.rescue_center.get_center()
                else:
                    self.return_target = self.data.rescue_center
                
                self.state = self.State.RETURNING_WITH_WOUNDED
            else:
                # Pas de rescue center connu
                self.state = self.State.EXPLORING
                self.current_wounded = None
        elif current.distance_to(self.current_wounded) < self.grasp_distance:
            # Assez proche du blessé, on continue à s'approcher
            pass
        else:
            # Continue à s'approcher du blessé
            self._go_to_point(current, self.current_wounded, command)
        
        return command
    
    def _return_with_wounded_control(self, command, gps):
        """Retourne au centre avec blessé - s'oriente constamment vers le centre"""
        # Vérifie si on a toujours un blessé attrapé
        if len(self.grasped_wounded_persons()) == 0:
            # Plus de blessé attrapé, retour à l'exploration
            self._reset_after_drop()
            return command
        
        current = Point(gps[0], gps[1])
        
        # Si pas de cible, on en cherche une (centre du rescue center)
        if self.return_target is None:
            if self.data.rescue_center:
                if isinstance(self.data.rescue_center, Box):
                    self.return_target = self.data.rescue_center.get_center()
                else:
                    self.return_target = self.data.rescue_center
            else:
                command["forward"] = 0.3
                return command
        
        distance = current.distance_to(self.return_target)
        
        # Si proche du centre, passe en mode dépôt
        if distance < 30:
            self.state = self.State.DROPPING_WOUNDED
            return command
        
        # S'oriente constamment vers le centre du rescue center
        self._orient_to_center(current, command)
        
        # Avance tout droit une fois orienté (ou pendant l'orientation)
        command["forward"] = 0.4
        
        return command
    
    def _orient_to_center(self, current_pos, command):
        """Ajuste l'orientation pour pointer vers le centre du rescue center"""
        if self.return_target is None:
            return
        
        dx = self.return_target.x - current_pos.x
        dy = self.return_target.y - current_pos.y
        target_angle = math.atan2(dy, dx)
        current_angle = self.compass_values()
        angle_diff = target_angle - current_angle
        
        # Normalise l'angle entre -π et π
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi
        
        # Contrôle de rotation pour s'orienter vers le centre
        if abs(angle_diff) > 0.5:
            command["rotation"] = 0.7 if angle_diff > 0 else -0.7
        elif abs(angle_diff) > 0.2:
            command["rotation"] = angle_diff * 0.8
        else:
            command["rotation"] = angle_diff * 0.5
    
    def _drop_wounded_control(self, command, gps):
        """Dépose le blessé au centre en continuant à s'orienter vers le centre"""
        # Vérifie si on a toujours un blessé attrapé
        if len(self.grasped_wounded_persons()) == 0:
            # Plus de blessé attrapé = dépôt réussi
            self._reset_after_drop()
            return command
        
        current = Point(gps[0], gps[1])
        
        # Continue à s'orienter vers le centre du rescue center
        if self.data.rescue_center:
            if isinstance(self.data.rescue_center, Box):
                center = self.data.rescue_center.get_center()
            else:
                center = self.data.rescue_center
            
            self._orient_to_point(current, center, command)
        
        # Avance doucement pour déposer
        command["forward"] = 0.3
        
        return command
    
    def _orient_to_point(self, current, target, command):
        """S'oriente vers un point spécifique"""
        dx = target.x - current.x
        dy = target.y - current.y
        target_angle = math.atan2(dy, dx)
        current_angle = self.compass_values()
        angle_diff = target_angle - current_angle
        
        # Normalise l'angle entre -π et π
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi
        
        # Contrôle de rotation
        if abs(angle_diff) > 0.5:
            command["rotation"] = 0.7 if angle_diff > 0 else -0.7
        elif abs(angle_diff) > 0.2:
            command["rotation"] = angle_diff * 0.8
        else:
            command["rotation"] = angle_diff * 0.5
    
    def _reset_after_drop(self):
        """Réinitialise après dépôt"""
        if self.current_wounded in self.data.wounded_positions:
            self.data.wounded_positions.remove(self.current_wounded)
        
        self.state = self.State.EXPLORING
        self.current_wounded = None
        self.return_target = None
    
    def _go_to_point(self, current, target, command):
        """Navigation simple"""
        dx = target.x - current.x
        dy = target.y - current.y
        target_angle = math.atan2(dy, dx)
        current_angle = self.compass_values()
        angle_diff = target_angle - current_angle
        
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi
        
        if abs(angle_diff) > 0.5:
            command["rotation"] = 0.7 if angle_diff > 0 else -0.7
            command["forward"] = 0.3
        elif abs(angle_diff) > 0.2:
            command["rotation"] = angle_diff * 0.8
            command["forward"] = 0.3
        else:
            command["rotation"] = angle_diff * 0.5
            command["forward"] = 0.3

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
    the_map = MyMapKeyboard(drone_type=MyDrone)

    # draw_lidar_rays : enable the visualization of the lidar rays
    # draw_semantic_rays : enable the visualization of the semantic rays
    gui = GuiSR(the_map=the_map,
                draw_lidar_rays=True,
                draw_semantic_rays=True,
                use_keyboard=False,
                )
    gui.run()

    score_health_returned = the_map.compute_score_health_returned()
    print("score_health_returned = ", score_health_returned)


if __name__ == '__main__':
    main()