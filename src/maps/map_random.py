import math
import random
import sys
from pathlib import Path
from typing import List, Type

# Insert the parent directory of the current file's directory into sys.path.
# This allows Python to locate modules that are one level above the current
# script, in this case spg_overlay.
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from spg.playground import Playground

from spg_overlay.entities.drone_abstract import DroneAbstract
from spg_overlay.entities.drone_motionless import DroneMotionless
from spg_overlay.entities.rescue_center import RescueCenter
from spg_overlay.entities.return_area import ReturnArea
from spg_overlay.entities.wounded_person import WoundedPerson
from spg_overlay.entities.sensor_disablers import ZoneType, NoComZone, NoGpsZone, KillZone
from spg_overlay.gui_map.closed_playground import ClosedPlayground
from spg_overlay.gui_map.gui_sr import GuiSR
from spg_overlay.gui_map.map_abstract import MapAbstract
from spg_overlay.reporting.evaluation import ZonesConfig
from spg_overlay.utils.misc_data import MiscData

from maps.walls_random import add_walls, add_boxes

# C'est une Map où tu peux tout modifier globalement
class MyMapRandom(MapAbstract):

    def __init__(self, zones_config: ZonesConfig = ()):
        # SPECIAL ZONES
        super().__init__(zones_config)
        
        # TIMER
        self._max_timestep_limit = 480
        self._max_walltime_limit = 22  # In seconds
        
        # SIZE AREA
        self._size_area = (1500, 700)      

        # RETURN AREA
        # Ici, la return area est la start area. C'est toujours le cas
        self._return_area = ReturnArea(size=(200, 200))
        self._return_area_pos = ((-500, -100), 0)

        # RESCUE CENTER
        self._rescue_center = RescueCenter(size=(200, 100))
        self._rescue_center_pos = ((0, 0), 0)

        # WOUNDED PERSONS
        x = -450
        y = 0
        '''
        x = random.uniform(-self._size_area[0] / 2, self._size_area[0] / 2)
        y = random.uniform(-self._size_area[1] / 2, self._size_area[1] / 2)
        while -100 <= x <= 100 or -50 <= y <= 50: # Pas dans la rescue center sinon il dépop
            x = random.uniform(-self._size_area[0] / 2, self._size_area[0] / 2)
            y = random.uniform(-self._size_area[1] / 2, self._size_area[1] / 2)
        '''
        self._wounded_persons_pos = [(x,y)] # Liste de tuples d'entiers
        self._number_wounded_persons = len(self._wounded_persons_pos)
        self._wounded_persons: List[WoundedPerson] = []

        # NO COM ZONE        
        self._no_com_zone = NoComZone(size=(402, 742))
        self._no_com_zone_pos = ((500, 100), 0)

        # NO GPS ZONE
        self._no_gps_zone = NoGpsZone(size=(574, 393))
        self._no_gps_zone_pos = ((100, 500), 0)

        # KILL ZONE
        self._kill_zone = KillZone(size=(89, 77))
        self._kill_zone_pos = ((0, 250), 0)
        
        # DRONES
        self._number_drones = 1
        self.drones_pos = []
        self._drones: List[DroneAbstract] = []


    def construct_playground(self, drone_type: Type[DroneAbstract]) -> Playground:
        # PLAYGROUND
        playground = ClosedPlayground(size=self._size_area)
        playground.add(self._rescue_center, self._rescue_center_pos)
        playground.add(self._return_area, self._return_area_pos)
        '''
        add_walls(playground)
        add_boxes(playground)
        '''
        # PLAYGROUND LIMITS
        self._explored_map.initialize_walls(playground)

        # POSITIONS OF THE DRONES
        misc_data = MiscData(size_area=self._size_area,
                             number_drones=self._number_drones,
                             max_timestep_limit=self._max_timestep_limit,
                             max_walltime_limit=self._max_walltime_limit)
        
        for i in range(self._number_drones):
            # Les drones apparaissent dans la return area qui est donc la start area ici
            x = -500
            y = 0
            angle = 0
            '''
            x = random.uniform(-600,-400)
            y = random.uniform(0,-200)
            angle = random.uniform(-math.pi, math.pi)
            dist_inter_drones = 0 # Variable à utiliser pour définir une distance min entre 2 drones
            '''
            self.drones_pos.append([(x,y), angle])
            drone = drone_type(identifier=i, misc_data=misc_data)
            self._drones.append(drone)
            playground.add(drone, ((x, y), angle))

        # POSITIONS OF THE WOUNDED PERSONS  
        for i in range(self._number_wounded_persons):
            wounded_person = WoundedPerson(rescue_center=self._rescue_center)
            self._wounded_persons.append(wounded_person)
            pos = (self._wounded_persons_pos[i], 0)
            playground.add(wounded_person, pos)

        # DISABLER ZONES
        if ZoneType.NO_COM_ZONE in self._zones_config:
            playground.add(self._no_com_zone, self._no_com_zone_pos)
        if ZoneType.NO_GPS_ZONE in self._zones_config:
            playground.add(self._no_gps_zone, self._no_gps_zone_pos)
        if ZoneType.KILL_ZONE in self._zones_config:
            playground.add(self._kill_zone, self._kill_zone_pos)

        return playground
    

if __name__ == '__main__':
    my_map = MyMapRandom()
    my_playground = my_map.construct_playground(drone_type=DroneMotionless)

    gui = GuiSR(playground=my_playground,
                the_map=my_map,
                use_mouse_measure=True,
                )
    gui.run()
