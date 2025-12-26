"""
This program can be launched directly.
"""

import sys, math, random
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
        """Met à jour le rescue center et marque la zone comme explorée"""
        if self.data.rescue_center is None:
            self.data.rescue_center = new_point
        elif isinstance(self.data.rescue_center, Point):
            self.data.rescue_center = build_box_with_2_opposite_points(
                self.data.rescue_center, new_point
            )
        elif isinstance(self.data.rescue_center, Box):
            self.data.rescue_center.extend_box(new_point)
        
        # Marque les points du rescue center comme explorés
        if self.data.quadtree:
            # Marque le centre
            if isinstance(self.data.rescue_center, Box):
                center = self.data.rescue_center.get_center()
            else:
                center = self.data.rescue_center
            
            qt_center = self.data.to_qt_coords(center)
            self.data.quadtree.insert_point(qt_center)  # Remplacé occupy par insert_point
            
            # Marque quelques points autour du centre
            for i in range(8):  # 8 points autour
                angle = i * math.pi / 4
                px = center.x + 20 * math.cos(angle)
                py = center.y + 20 * math.sin(angle)
                point = Point(px, py)
                qt_point = self.data.to_qt_coords(point)
                self.data.quadtree.insert_point(qt_point)  # Remplacé occupy par insert_point
    
    def _explore_control(self, command, current_pos):
        """Exploration intelligente avec QuadTree et évitement d'obstacles"""
        # Initialise la QuadTree si nécessaire
        if self.data.quadtree is None:
            self.data.init_quadtree(current_pos)
        
        # Marque la position actuelle comme explorée
        qt_pos = self.data.to_qt_coords(current_pos)
        self.data.quadtree.insert_point(qt_pos)
        
        # ÉVITEMENT D'OBSTACLES AMÉLIORÉ
        command = self._avoid_obstacles(command)
        
        # Si on évite un obstacle, on ne fait pas l'exploration normale
        if command["rotation"] != 0 or command["forward"] <= 0:
            return command
        
        # Trouve une cible d'exploration non explorée
        exploration_target = self._find_unexplored_target(current_pos)
        
        if exploration_target is not None:
            # Va vers la zone non explorée
            self._go_to_point(current_pos, exploration_target, command)
        else:
            # Pas de zone non explorée trouvée, exploration aléatoire
            command["forward"] = 0.2
            command["rotation"] = 0.05
        
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
        
    def _find_unexplored_target(self, current_pos):
        """Trouve une cible d'exploration dans une zone non explorée"""
        if self.data.quadtree is None:
            return None
        
        # Directions de recherche autour du drone
        search_distances = [30, 50, 70, 100]  # Distances croissantes
        angles = [0, math.pi/2, math.pi, 3*math.pi/2]  # 4 directions principales
        
        # Cherche d'abord près du drone, puis de plus en plus loin
        for distance in search_distances:
            for angle in angles:
                # Calcule une position candidate
                target_x = current_pos.x + distance * math.cos(angle)
                target_y = current_pos.y + distance * math.sin(angle)
                candidate = Point(target_x, target_y)
                
                # Convertit en coordonnées QuadTree
                qt_candidate = self.data.to_qt_coords(candidate)
                
                # Vérifie si la zone est déjà explorée
                if not self.data.quadtree.is_occupied(qt_candidate):
                    # Vérifie aussi que ce n'est pas dans le rescue center
                    if self.data.rescue_center:
                        if isinstance(self.data.rescue_center, Point):
                            if candidate.distance_to(self.data.rescue_center) < 50:
                                continue  # Trop près du rescue center
                        elif isinstance(self.data.rescue_center, Box):
                            if self.data.rescue_center.is_inside(candidate):
                                continue  # À l'intérieur du rescue center
                    
                    return candidate
        
        # Si aucune zone non explorée trouvée, cherche aléatoirement
        return self._find_random_target(current_pos)
    

    def _find_random_target(self, current_pos):
        """Trouve une cible d'exploration aléatoire"""
        # Distance aléatoire entre 30 et 80 pixels
        distance = 30 + random.random() * 50
        # Angle aléatoire
        angle = random.random() * 2 * math.pi
        
        target_x = current_pos.x + distance * math.cos(angle)
        target_y = current_pos.y + distance * math.sin(angle)
        
        return Point(target_x, target_y)

    
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

    def _avoid_obstacles(self, command):
        """Évitement d'obstacles intelligent avec analyse complète"""
        lidar = self.lidar_values()
        
        if len(lidar) == 0:
            return command
        
        # Convertir en liste
        lidar_list = list(lidar)
        num_readings = len(lidar_list)
        
        # Définir les zones
        center_idx = num_readings // 2
        
        # Zone frontale (30° au centre)
        front_start = max(0, center_idx - num_readings//12)  # ±15°
        front_end = min(num_readings, center_idx + num_readings//12 + 1)
        front_zone = lidar_list[front_start:front_end]
        
        # Zones gauche et droite (45° de chaque côté)
        left_zone = lidar_list[:num_readings//4]  # 0-90° à gauche
        right_zone = lidar_list[3*num_readings//4:]  # 270-360° à droite
        
        # Seuils (en pixels)
        DANGER_DISTANCE = 40
        WARNING_DISTANCE = 60
        
        # Vérifier d'abord le danger frontal immédiat
        if len(front_zone) > 0 and min(front_zone) < DANGER_DISTANCE:
            # Danger frontal immédiat - reculer d'abord
            command["forward"] = -0.4
            
            # Analyser les côtés pour choisir la direction
            if len(left_zone) > 0 and len(right_zone) > 0:
                avg_left = sum(left_zone) / len(left_zone)
                avg_right = sum(right_zone) / len(right_zone)
                
                # Tourner vers le côté avec le plus d'espace
                if avg_left > avg_right:
                    command["rotation"] = 0.6  # Tourne à gauche
                else:
                    command["rotation"] = -0.6  # Tourne à droite
            else:
                command["rotation"] = 0.5  # Par défaut à gauche
            
            return command
        
        # Vérifier les obstacles proches
        elif len(front_zone) > 0 and min(front_zone) < WARNING_DISTANCE:
            # Obstacle proche - analyser l'environnement
            
            # Créer un "score" pour chaque direction
            scores = []
            directions = []
            
            # Analyser plusieurs directions possibles
            for angle_offset in [-60, -30, 0, 30, 60]:  # en degrés
                idx = center_idx + int(angle_offset * num_readings / 360)
                if 0 <= idx < num_readings:
                    # Score basé sur la distance et l'angle
                    distance = lidar_list[idx]
                    if distance < DANGER_DISTANCE:
                        score = 0  # Trop dangereux
                    elif distance < WARNING_DISTANCE:
                        score = 0.3  # Risqué
                    else:
                        # Meilleur score pour les distances plus grandes
                        score = min(1.0, distance / 100)
                    
                    # Pénalité pour les virages serrés
                    turn_penalty = abs(angle_offset) / 90
                    final_score = score * (1 - turn_penalty * 0.3)
                    
                    scores.append(final_score)
                    directions.append(angle_offset)
            
            if scores:
                # Choisir la meilleure direction
                best_idx = scores.index(max(scores))
                best_direction = directions[best_idx]
                
                # Convertir l'angle en commande de rotation
                if best_direction < -10:
                    command["rotation"] = 0.4  # Tourne à gauche
                    command["forward"] = -0.2
                elif best_direction > 10:
                    command["rotation"] = -0.4  # Tourne à droite
                    command["forward"] = -0.2
                else:
                    # Direction presque droite, juste ralentir
                    command["forward"] = 0.1
                    command["rotation"] = 0.0
            
            return command
        
        return command
    
    def _go_to_point(self, current, target, command):
        """Navigation avec évitement d'obstacles"""
        # D'abord, vérifie les obstacles
        obstacle_command = self._avoid_obstacles(command.copy())
        
        # Si un obstacle est détecté, priorité à l'évitement
        if (obstacle_command["rotation"] != 0 or 
            obstacle_command["forward"] <= 0 or
            obstacle_command["forward"] != command["forward"]):
            command.update(obstacle_command)
            return
        
        # Sinon, navigation normale vers le point
        dx = target.x - current.x
        dy = target.y - current.y
        target_angle = math.atan2(dy, dx)
        current_angle = self.compass_values()
        angle_diff = target_angle - current_angle
        
        # Normalise l'angle
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi
        
        # Contrôle de navigation
        if abs(angle_diff) > 0.5:
            command["rotation"] = 0.7 if angle_diff > 0 else -0.7
            command["forward"] = 0.1
        elif abs(angle_diff) > 0.2:
            command["rotation"] = angle_diff * 0.8
            command["forward"] = 0.3
        else:
            command["rotation"] = angle_diff * 0.5
            command["forward"] = 0.3
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
        
        # Blessé en cours
        self.current_wounded = None
        self.grasp_distance = 60
        self.return_target = None
        
        # Exploration avec QuadTree
        self.current_exploration_node = None
        self.visited_nodes = set()  # IDs des noeuds déjà visités
        self.last_position = None
        self.stuck_counter = 0
        
        # Évitement d'obstacles
        self.avoidance_mode = False
        self.avoidance_direction = 0  # -1: gauche, 0: aucun, 1: droite

    def define_message_for_all(self):
        return {}
    
    def control(self):
        """Contrôle principal avec exploration QuadTree"""
        command = {"forward": 0.0, "lateral": 0.0, "rotation": 0.0, "grasper": 1.0}
        
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
        
        # Traitement sémantique
        self._process_semantic_data(gps, orientation, semantic_data)
        
        # Évitement d'obstacles (prioritaire)
        command = self._avoid_obstacles_simple(command, current_pos)
        
        # Si on évite activement un obstacle, on s'arrête là
        if self.avoidance_mode:
            return command
        
        # Machine à états
        if self.state == self.State.EXPLORING:
            command = self._explore_with_quadtree(command, current_pos)
            
        elif self.state == self.State.GOING_TO_WOUNDED:
            command = self._go_to_wounded_control(command, gps)
            
        elif self.state == self.State.RETURNING_WITH_WOUNDED:
            command = self._return_with_wounded_control(command, gps)
            
        elif self.state == self.State.DROPPING_WOUNDED:
            command = self._drop_wounded_control(command, gps)

        # Détection de blocage
        self._detect_stuck(current_pos)
        
        print(f"State: {self.state}, Node: {self.current_exploration_node}")
        
        return command
    
    def _process_semantic_data(self, gps, orientation, semantic_data):
        """Traite les données sémantiques"""
        if not semantic_data:
            return
        
        for data in semantic_data:
            dx = data.distance * math.cos(orientation + data.angle)
            dy = data.distance * math.sin(orientation + data.angle)
            pos = Point(gps[0] + dx, gps[1] + dy)
            
            if data.entity_type == DroneSemanticSensor.TypeEntity.RESCUE_CENTER:
                self._update_rescue_center_box(pos)
                    
            elif data.entity_type == DroneSemanticSensor.TypeEntity.WOUNDED_PERSON:
                if not data.grasped:  # Blessé NON attrapé
                    is_new = True
                    for wounded in self.data.wounded_positions:
                        if pos.distance_to(wounded) < 60:
                            is_new = False
                            break
                    
                    if is_new:
                        self.data.wounded_positions.append(pos)
                        
                        if (self.state == self.State.EXPLORING and 
                            self.current_wounded is None):
                            self.current_wounded = pos
                            self.state = self.State.GOING_TO_WOUNDED
                            # Abandonne l'exploration actuelle pour le blessé
                            self.current_exploration_node = None
    
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
        
        # Marque le rescue center comme exploré dans la QuadTree
        if self.data.quadtree:
            if isinstance(self.data.rescue_center, Box):
                center = self.data.rescue_center.get_center()
            else:
                center = self.data.rescue_center
            
            qt_center = self.data.to_qt_coords(center)
            self.data.quadtree.insert_point(qt_center)
    
    def _explore_with_quadtree(self, command, current_pos):
        """Exploration intelligente basée sur le QuadTree"""
        # Marque la position actuelle comme explorée
        qt_pos = self.data.to_qt_coords(current_pos)
        self.data.quadtree.insert_point(qt_pos)
        
        # Si on n'a pas de noeud cible, ou si on l'a atteint
        if (self.current_exploration_node is None or 
            current_pos.distance_to(self.current_exploration_node) < 10):
            
            # Trouve un nouveau noeud à explorer
            new_node = self._select_next_exploration_node(current_pos)
            
            if new_node:
                self.current_exploration_node = new_node
                self.visited_nodes.add(id(new_node))  # Marque comme visité
                print(f"Nouveau noeud cible: {new_node}")
            else:
                # Tous les noeuds explorés, exploration aléatoire
                command["forward"] = 0.2
                command["rotation"] = 0.05
                return command
        
        # Navigue vers le noeud cible
        if self.current_exploration_node:
            self._go_to_point(current_pos, self.current_exploration_node, command)
        
        # Vérifie s'il y a des blessés à traiter
        if (len(self.data.wounded_positions) != 0 and  
            len(self.grasped_wounded_persons()) == 0 and
            self.current_wounded is None):
            
            self.current_wounded = self.data.wounded_positions[0]
            self.state = self.State.GOING_TO_WOUNDED
            self.current_exploration_node = None
        
        return command
    
    def _select_next_exploration_node(self, current_pos):
        """Sélectionne le prochain noeud à explorer"""
        if self.data.quadtree is None:
            return None
        
        # Récupère tous les noeuds non occupés
        unoccupied_nodes = self.data.quadtree.get_unoccupied_nodes()
        
        if not unoccupied_nodes:
            return None
        
        # Convertit les noeuds en cibles mondiales
        candidates = []
        for node in unoccupied_nodes:
            # Vérifie si déjà visité
            if id(node) in self.visited_nodes:
                continue
            
            # Calcule le centre du noeud
            center = node.box.get_center()
            world_center = Point(
                center.x + self.data.qt_origin.x,
                center.y + self.data.qt_origin.y
            )
            
            # Vérifie si c'est une cible valide
            if not self._is_valid_exploration_target(world_center):
                continue
            
            # Calcule un score pour ce noeud
            distance = current_pos.distance_to(world_center)
            area = node.box.get_area()
            
            # Score : favorise les noeuds proches et grands
            # Mais pénalise les noeuds trop petits
            if area < 25:  # Noeuds trop petits
                continue
            
            score = (area / 100) * (1 / (distance + 1))
            
            # Bonus pour les noeuds adjacents aux zones explorées
            neighbors = node.adjacency_list(unoccupied_nodes)
            explored_neighbors = sum(1 for n in neighbors if id(n) in self.visited_nodes)
            score *= (1 + explored_neighbors * 0.2)
            
            candidates.append((score, world_center, node))
        
        if not candidates:
            # Tous les noeuds valides ont été visités
            # On peut soit étendre la recherche, soit réinitialiser les visites
            if len(self.visited_nodes) > 20:
                self.visited_nodes.clear()  # Réinitialise pour re-explorer
                return self._select_next_exploration_node(current_pos)
            return None
        
        # Sélectionne le meilleur candidat
        candidates.sort(key=lambda x: x[0], reverse=True)
        
        # Prend le meilleur, ou parfois un autre pour varier
        if len(candidates) > 3 and random.random() < 0.3:
            # 30% du temps, prend un candidat différent du meilleur
            chosen_idx = random.randint(1, min(3, len(candidates)-1))
        else:
            chosen_idx = 0
        
        _, best_target, best_node = candidates[chosen_idx]
        
        # Élargit le QuadTree si nécessaire
        self._extend_quadtree_if_needed(best_target)
        
        return best_target
    
    def _extend_quadtree_if_needed(self, target):
        """Élargit le QuadTree si la cible est en dehors"""
        if not self.data.qt_origin:
            return
        
        # Vérifie si la cible est dans les limites actuelles
        qt_target = self.data.to_qt_coords(target)
        
        # Si la cible est en dehors du QuadTree, on doit l'étendre
        if (qt_target.x < 0 or qt_target.x > self.data.qt_size or
            qt_target.y < 0 or qt_target.y > self.data.qt_size):
            
            # Calcule la nouvelle taille nécessaire
            new_size = max(self.data.qt_size * 2, 
                          abs(qt_target.x) * 2, 
                          abs(qt_target.y) * 2)
            
            print(f"Élargissement du QuadTree: {self.data.qt_size} -> {new_size}")
            
            # Réinitialise le QuadTree avec la nouvelle taille
            self.data.qt_size = new_size
            current_pos = Point(self.measured_gps_position()[0],
                               self.measured_gps_position()[1])
            self.data.init_quadtree(current_pos)
            
            # Réinitialise les visites
            self.visited_nodes.clear()
            self.current_exploration_node = None
    
    def _is_valid_exploration_target(self, target):
        """Vérifie si une cible est valide pour l'exploration"""
        # Pas dans le rescue center
        if self.data.rescue_center:
            if isinstance(self.data.rescue_center, Point):
                if target.distance_to(self.data.rescue_center) < 50:
                    return False
            elif isinstance(self.data.rescue_center, Box):
                if self.data.rescue_center.is_inside(target):
                    return False
        
        return True
    
    def _avoid_obstacles_simple(self, command, current_pos):
        """Évitement d'obstacles simple mais efficace avec saturation"""
        lidar = self.lidar_values()
        
        if len(lidar) == 0:
            self.avoidance_mode = False
            return command
        
        lidar_list = list(lidar)
        num_readings = len(lidar_list)
        center_idx = num_readings // 2
        
        # Zone frontale (45°)
        front_start = max(0, center_idx - num_readings//8)
        front_end = min(num_readings, center_idx + num_readings//8)
        front_readings = lidar_list[front_start:front_end]
        
        if not front_readings:
            self.avoidance_mode = False
            return command
        
        min_front = min(front_readings)
        
        # Seuils (ajustez selon vos mesures)
        TOO_CLOSE = 30
        CLOSE = 50
        
        if min_front < TOO_CLOSE:
            # Danger immédiat
            self.avoidance_mode = True
            command["forward"] = -0.3
            
            # Choisit une direction d'évitement cohérente
            if self.avoidance_direction == 0:
                # Pas de direction préférée, choisit aléatoirement
                self.avoidance_direction = 1 if random.random() > 0.5 else -1
            
            # SATURATION: rotation déjà dans les limites
            command["rotation"] = 0.5 * self.avoidance_direction
            
            # Marque la zone comme obstruée
            qt_pos = self.data.to_qt_coords(current_pos)
            self.data.quadtree.insert_point(qt_pos)
            
        elif min_front < CLOSE:
            # Obstacle proche
            self.avoidance_mode = True
            command["forward"] = 0.1
            
            if self.avoidance_direction == 0:
                self.avoidance_direction = 1 if random.random() > 0.5 else -1
            
            # SATURATION: rotation déjà dans les limites
            command["rotation"] = 0.3 * self.avoidance_direction
            
        else:
            # Pas d'obstacle
            self.avoidance_mode = False
            self.avoidance_direction = 0
        
        # SATURATION supplémentaire pour sécurité
        command["rotation"] = max(-1.0, min(1.0, command["rotation"]))
        command["forward"] = max(-1.0, min(1.0, command["forward"]))
        
        return command
    
    def _go_to_point(self, current, target, command):
        """Navigation vers un point avec limites de rotation"""
        dx = target.x - current.x
        dy = target.y - current.y
        
        # Si très proche, on a atteint la cible
        if abs(dx) < 5 and abs(dy) < 5:
            command["forward"] = 0.1
            command["rotation"] = 0.05
            return
        
        target_angle = math.atan2(dy, dx)
        current_angle = self.compass_values()
        angle_diff = target_angle - current_angle
        
        # Normalise l'angle entre -π et π
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi
        
        # Contrôle proportionnel avec saturation
        rotation_gain = 0.8
        rotation_command = angle_diff * rotation_gain
        
        # SATURATION: Limite entre -1 et 1
        rotation_command = max(-1.0, min(1.0, rotation_command))
        
        command["rotation"] = rotation_command
        
        # Vitesse adaptative
        alignment = 1 - abs(angle_diff) / math.pi
        base_speed = 0.3
        command["forward"] = base_speed * (0.3 + 0.7 * alignment)
        command["forward"] = min(command["forward"], 0.4)
    
    def _go_to_wounded_control(self, command, gps):
        """Va vers un blessé"""
        if self.current_wounded is None:
            self.state = self.State.EXPLORING
            return command
        
        current = Point(gps[0], gps[1])
        
        # Vérifie si un blessé est attrapé
        if len(self.grasped_wounded_persons()) != 0:
            if self.data.rescue_center:
                if isinstance(self.data.rescue_center, Box):
                    self.return_target = self.data.rescue_center.get_center()
                else:
                    self.return_target = self.data.rescue_center
                
                self.state = self.State.RETURNING_WITH_WOUNDED
            else:
                self.state = self.State.EXPLORING
                self.current_wounded = None
        elif current.distance_to(self.current_wounded) < self.grasp_distance:
            # Assez proche du blessé
            pass
        else:
            self._go_to_point(current, self.current_wounded, command)
        
        return command
    
    def _return_with_wounded_control(self, command, gps):
        """Retourne au centre avec blessé"""
        if len(self.grasped_wounded_persons()) == 0:
            self._reset_after_drop()
            return command
        
        current = Point(gps[0], gps[1])
        
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
        
        if distance < 30:
            self.state = self.State.DROPPING_WOUNDED
            return command
        
        # S'oriente vers le centre
        self._orient_to_center(current, command)
        command["forward"] = 0.4
        
        return command
    
    def _orient_to_center(self, current_pos, command):
        """S'oriente vers le centre du rescue center avec saturation"""
        if self.return_target is None:
            return
        
        dx = self.return_target.x - current_pos.x
        dy = self.return_target.y - current_pos.y
        target_angle = math.atan2(dy, dx)
        current_angle = self.compass_values()
        angle_diff = target_angle - current_angle
        
        # Normalise l'angle
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi
        
        # Calcul de la rotation avec saturation
        if abs(angle_diff) > 0.5:
            rotation = 0.7 if angle_diff > 0 else -0.7
        elif abs(angle_diff) > 0.2:
            rotation = angle_diff * 0.8
        else:
            rotation = angle_diff * 0.5
        
        # SATURATION: Limite entre -1 et 1
        command["rotation"] = max(-1.0, min(1.0, rotation))

    def _orient_to_point(self, current, target, command):
        """S'oriente vers un point spécifique avec saturation"""
        dx = target.x - current.x
        dy = target.y - current.y
        target_angle = math.atan2(dy, dx)
        current_angle = self.compass_values()
        angle_diff = target_angle - current_angle
        
        # Normalise l'angle
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi
        
        # Calcul de la rotation avec saturation
        if abs(angle_diff) > 0.5:
            rotation = 0.7 if angle_diff > 0 else -0.7
        elif abs(angle_diff) > 0.2:
            rotation = angle_diff * 0.8
        else:
            rotation = angle_diff * 0.5
        
        # SATURATION: Limite entre -1 et 1
        command["rotation"] = max(-1.0, min(1.0, rotation))
    
    def _drop_wounded_control(self, command, gps):
        """Dépose le blessé au centre"""
        if len(self.grasped_wounded_persons()) == 0:
            self._reset_after_drop()
            return command
        
        current = Point(gps[0], gps[1])
        
        if self.data.rescue_center:
            if isinstance(self.data.rescue_center, Box):
                center = self.data.rescue_center.get_center()
            else:
                center = self.data.rescue_center
            
            self._orient_to_point(current, center, command)
        
        command["forward"] = 0.3
        
        return command
    
    def _reset_after_drop(self):
        """Réinitialise après dépôt"""
        if self.current_wounded in self.data.wounded_positions:
            self.data.wounded_positions.remove(self.current_wounded)
        
        self.state = self.State.EXPLORING
        self.current_wounded = None
        self.return_target = None
        self.current_exploration_node = None
    
    def _detect_stuck(self, current_pos):
        """Détecte si le drone est bloqué"""
        if self.last_position:
            distance_moved = current_pos.distance_to(self.last_position)
            if distance_moved < 2:  # Presque pas bougé
                self.stuck_counter += 1
            else:
                self.stuck_counter = 0
            
            if self.stuck_counter > 50:  # Bloqué pendant 50 cycles
                print("Drone bloqué, réinitialisation de l'exploration")
                self.current_exploration_node = None
                self.visited_nodes.clear()
                self.stuck_counter = 0
        
        self.last_position = current_pos

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
    the_map = MapIntermediate01(drone_type=MyDrone)

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