import numpy as np 

from swarm_rescue.team_du_sud.geometry import Point
from swarm_rescue.team_du_sud.geometry import Box

class Node :
        def __init__(self, box : Box) :
                self.box = box
                self.occupied = False
                self.children = None
        def subdivide(self) :  
                center = self.box.get_center()
                cx, cy = center.x, center.y
                w, h = self.box.get_dimensions()
                self.children = [
            # NW (top-left)
                Node(Box(Point(cx - w/2, cy), Point(cx, cy), Point(cx, cy + h/2), Point(cx - w/2, cy + h/2)).order_points()),
            # NE
                Node(Box(Point(cx, cy), Point(cx + w/2, cy), Point(cx + w/2, cy + h/2), Point(cx, cy + h/2)).order_points()),
            # SW
                Node(Box(Point(cx - w/2, cy - h/2), Point(cx, cy - h/2), Point(cx, cy), Point(cx - w/2, cy)).order_points()),
            # SE
                Node(Box(Point(cx, cy - h/2), Point(cx + w/2, cy - h/2), Point(cx + w/2, cy), Point(cx, cy)).order_points())
                ]
  
        def child_for(self, point : Point) :
                center = self.box.get_center()
                if point.x < center.x and point.y >= center.y :
                        return self.children[0]
                elif point.x >= center.x and point.y >= center.y :
                        return self.children[1]
                elif point.x < center.x and point.y < center.y :
                        return self.children[2]
                else :
                        return self.children[3]
        
        def insert_occupied(self, point : Point, min_size = 5) : #should define the minimum size later, most likely gonna be drone size
                if not self.box.is_inside(point) :
                        return
                
                w, h = self.box.get_dimensions()
                
                if w <= min_size or h <= min_size :
                        self.occupied = True
                        return
                
                if self.children is None :
                        self.subdivide()
                
                child = self.child_for(point)
                child.insert_occupied(point, min_size)
        
        def is_occupied(self, point) -> bool :
                if not self.box.is_inside(point) :
                        return False
                if self.children == None :
                        return self.occupied

                child = self.child_for(point)
                return child.is_occupied(point)