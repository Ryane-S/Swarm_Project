import math
import numpy as np    

# On définit ici un ensemble d'outils géométriques utiles pour le mapping et le pathing du drone

class Point():
    def __init__(self, position_x, position_y):
        self.x = position_x
        self.y = position_y

    def add(self, drone_orientation, distance, angle):
        new_x = self.x + (distance*math.cos(drone_orientation+angle))
        new_y = self.y + (distance*math.sin(drone_orientation+angle))
        new_point = Point(new_x, new_y)
        return new_point
    
    import numpy as np

    def are_aligned_in_type(self, point_1, point_2, epsilon=1e-6):
        x_coords = np.array([point_1.x, point_2.x, self.x])
        y_coords = np.array([point_1.y, point_2.y, self.y])
    
        var_x = np.var(x_coords)
        var_y = np.var(y_coords)
    
        if var_x < epsilon and var_y < epsilon:
            return ("undefined", 1.0)  # Les 3 points sont quasiment confondus
        elif var_x < epsilon:
            return ("vertical", float('inf'))
        elif var_y < epsilon:
            return ("horizontal", float('inf'))
    
        ratio = var_y / var_x
        if ratio < 1:
            return ("vertical", 1 / ratio)  # var_y < var_x → x constants
        else:
            return ("horizontal", ratio)
    
    def distance_to(self, point):
        return ((self.x - point.x) ** 2 + (self.y - point.y) ** 2) ** 0.5
    
    def __str__(self):
        return f"({self.x}, {self.y})"

    def __eq__(self, other):
        if not isinstance(other, Point):
            return NotImplemented
        return self.x == other.x and self.y == other.y
    
    def __hash__(self):
        return hash((self.x, self.y))


class Line():
    def __init__(self, point_1: Point, point_2: Point):
        self.point_1 = point_1
        self.point_2 = point_2
        self.type = None

    def update_type(self, type: str):
        self.type = type
        
    def get_position(self):
        if self.type == "vertical":
            return [self.point_1.x, self.point_1.y, self.point_2.y]
        elif self.type == "horizontal":
            return [self.point_1.y, self.point_1.x, self.point_2.x]
        
    def get_length(self):
        return ((self.point_2.x - self.point_1.x) ** 2 + (self.point_2.y - self.point_1.y) ** 2) ** 0.57
    
    def is_on_line(self, point: Point):
        if self.type == "vertical":
            return point.x == self.point_1.x and self.point_1.y <= point.y <= self.point_2.y
        elif self.type == "horizontal":
            return point.y == self.point_1.y and self.point_1.x <= point.x <= self.point_2.x
        
    def is_aligned(self, point: Point):
        if self.type == "vertical":
            return point.x == self.point_1.x and ((point.y <= self.point_1.y and point.y <= self.point_2.y) or (point.y >= self.point_1.y and point.y >= self.point_2.y))
        elif self.type == "horizontal":
            return point.y == self.point_1.y and ((point.x <= self.point_1.x and point.x <= self.point_2.x) or (point.x >= self.point_1.x and point.x >= self.point_2.x))

    def extend_line(self, point_3: Point):
        if self.type == "vertical":
            y_positions = [self.point_1.y, self.point_2.y, point_3.y]
            new_line = Line(Point(self.point_1.x, min(y_positions)), Point(self.point_1.x, max(y_positions)))
        elif self.type == "horizontal":
            x_positions = [self.point_1.x, self.point_2.x, point_3.x]
            new_line = Line(Point(min(x_positions), self.point_1.y), Point(max(x_positions), self.point_1.y))
        self.point_1 = new_line.point_1
        self.point_2 = new_line.point_2

    def __str__(self):
        return f"Line from {self.point_1} to {self.point_2} of type {self.type}"

    def __eq__(self, other):
        if not isinstance(other, Line):
            return NotImplemented
        return (self.point_1 == other.point_1 or self.point_1 == other.point_2) and (self.point_2 == other.point_1 or self.point_2 == other.point_2)
    
    def __hash__(self):
        return hash((self.point_1, self.point_2))


class Box():
    def __init__(self, point_1: Point, point_2: Point, point_3: Point, point_4: Point):
        self.point_1 = point_1
        self.point_2 = point_2
        self.point_3 = point_3
        self.point_4 = point_4
        self.points = [point_1, point_2, point_3, point_4]

    def order_points(self): # Méthode pertiente car tout est horizontal/vertical 
        index = 1
        for i in  range(3):
            if self.point_1.x != self.points[i+1].x and self.point_1.y != self.points[i+1].y:
                index = i+1
                break
        if index != 2:
            inter_point = self.points[2]
            self.points[2] = self.points[index]
            self.points[index] = inter_point
        # Les points sont ordonnées dans le sens horaire ou trigo à partir du point 1
        self.point_1 = self.points[0]
        self.point_2 = self.points[1]
        self.point_3 = self.points[2]
        self.point_4 = self.points[3]
    
    def extend_box(self, point: Point):
        min_x, max_x, min_y, max_y = self.get_limits()
        if min_x >= point.x:
            min_x = point.x
        if max_x <= point.x:
            max_x = point.x
        if min_y >= point.y:
            min_y = point.y
        if max_y <= point.y:
            max_y = point.y
        new_box = Box(Point(min_x, min_y), Point(max_x, min_y), Point(max_x, max_y), Point(min_x, max_y))
        self.point_1 = new_box.point_1
        self.point_2 = new_box.point_2
        self.point_3 = new_box.point_3
        self.point_4 = new_box.point_4

    def get_lines(self):
        self.order_points()
        self.points = [self.point_1, self.point_2, self.point_3, self.point_4]
        lines = []
        for i in range(4):
            line = Line(self.points[i], self.points[(i+1)%4])
            lines.append(line)
        return lines # Les lignes sont dans le sens horaire ou trigo à partir du point 1

    def get_area(self):
        width, height = self.get_dimensions()
        return width * height
    
    def get_center(self):
        self.order_points()
        self.points = [self.point_1, self.point_2, self.point_3, self.point_4]
        center_x = (self.points[0].x + self.points[2].x) / 2
        center_y = (self.points[0].y + self.points[2].y) / 2
        return Point(center_x, center_y)
    
    def get_limits(self):
        self.points_x = [self.point_1.x, self.point_2.x, self.point_3.x, self.point_4.x]
        self.points_y = [self.point_1.y, self.point_2.y, self.point_3.y, self.point_4.y]
        min_x = min(self.points_x)
        max_x = max(self.points_x)
        min_y = min(self.points_y) 
        max_y = max(self.points_y)
        return min_x, max_x, min_y, max_y
    
    def get_dimensions(self):
        self.limits = self.get_limits()
        width = self.limits[1] - self.limits[0]
        height = self.limits[3] - self.limits[2]
        return width, height
    
    def is_inside(self, point: Point):
        self.order_points()
        self.points = [self.point_1, self.point_2, self.point_3, self.point_4]
        if (self.points[0].x <= point.x <= self.points[2].x and
            self.points[0].y <= point.y <= self.points[2].y):
            return True
        return False
    
    def covers_more_than_half(self, other):
        min_x1, max_x1, min_y1, max_y1 = self.get_limits()
        min_x2, max_x2, min_y2, max_y2 = other.get_limits()
        if min_x1 >= max_x2 or max_x1 <= min_x2:
            return False
        else:
            # On vérifie si la box 2 couvre plus de la moitié de la box 1
            area_1 = self.get_area()
            area_2 = other.get_area()
            intersection_width = min(max_x1, max_x2) - max(min_x1, min_x2)
            intersection_height = min(max_y1, max_y2) - max(min_y1, min_y2)
            if intersection_width <= 0 or intersection_height <= 0:
                return False
            intersection_area = intersection_width * intersection_height
            return intersection_area >= area_1 / 2
    
    def __str__(self):
        self.order_points()
        return f"Box with points {self.point_1}, {self.point_2}, {self.point_3}, {self.point_4}"
    
    def __eq__(self, other):
        if not isinstance(other, Box):
            return NotImplemented
        return (self.point_1 == other.point_1 and
                self.point_2 == other.point_2 and
                self.point_3 == other.point_3 and
                self.point_4 == other.point_4)
    
    def __hash__(self):
        return hash((self.point_1, self.point_2, self.point_3, self.point_4))
    

# Fonctions supplémentaires
def build_box_with_2_opposite_points(point_1: Point, point_2: Point):
    point_3 = Point(point_1.x, point_2.y)
    point_4 = Point(point_1.y, point_2.x)
    return Box(point_1, point_2, point_3, point_4)

def build_box_with_line_and_point(line: Line, point: Point):
    x_positions = [line.point_1.x, line.point_2.x, point.x]
    y_positions = [line.point_1.y, line.point_2.y, point.y]
    min_x = min(x_positions)
    max_x = max(x_positions)
    min_y = min(y_positions)
    max_y = max(y_positions)
    return Box(Point(min_x, min_y), Point(max_x, min_y), Point(max_x, max_y), Point(min_x, max_y))

def detect_local_zones(liste, max_value=150, max_diff=50, min_zone_length=2):
    zones = []
    current_zone = []

    for i in range(len(liste)):
        val = liste[i]

        if val > max_value:
            if len(current_zone) >= min_zone_length:
                zones.append(current_zone)
            current_zone = []
            continue

        if not current_zone:
            current_zone.append(i)
        else:
            prev_val = liste[current_zone[-1]]
            if abs(val - prev_val) <= max_diff:
                current_zone.append(i)
            else:
                if len(current_zone) >= min_zone_length:
                    zones.append(current_zone)
                current_zone = [i]

    if len(current_zone) >= min_zone_length:
        zones.append(current_zone)

    return zones