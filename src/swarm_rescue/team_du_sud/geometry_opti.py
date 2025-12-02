import numpy as np

# --------------------------
# POINTS
# --------------------------

def point(x, y):
    """Crée un point numpy."""
    return np.array([x, y], dtype=float)

def add(p, drone_orientation, distance, angle):
    """Equivalent de Point.add → translation selon angle global."""
    total_angle = drone_orientation + angle
    vec = np.array([np.cos(total_angle), np.sin(total_angle)]) * distance
    return p + vec

def distance(p1, p2):
    """Distance entre deux points."""
    return np.linalg.norm(p1 - p2)

def are_aligned(points, eps=1e-6):
    """Détermine l'alignement type et un facteur de ratio."""
    var = points.var(axis=0)  # [var_x, var_y]

    if np.all(var < eps):
        return ("undefined", 1.0)
    if var[0] < eps:
        return ("vertical", np.inf)
    if var[1] < eps:
        return ("horizontal", np.inf)

    ratio = var[1] / var[0]
    return ("vertical", 1/ratio) if ratio < 1 else ("horizontal", ratio)

# --------------------------
# LINES
# --------------------------

def line(p1, p2):
    return np.vstack([p1, p2])

def line_length(l):
    return np.linalg.norm(l[1] - l[0])

def line_type(l, eps=1e-6):
    """Renvoie 'vertical', 'horizontal' ou 'undefined'."""
    dx = abs(l[1,0] - l[0,0])
    dy = abs(l[1,1] - l[0,1])
    if dx < eps and dy > eps:
        return "vertical"
    if dy < eps and dx > eps:
        return "horizontal"
    return "undefined"

def is_on_line(l, p):
    """Point sur segment."""
    t = line_type(l)
    if t == "vertical":
        return abs(p[0] - l[0,0]) < 1e-6 and l[:,1].min() <= p[1] <= l[:,1].max()
    if t == "horizontal":
        return abs(p[1] - l[0,1]) < 1e-6 and l[:,0].min() <= p[0] <= l[:,0].max()
    return False

# --------------------------
# BOXES
'''
P1 bas-gauche
P2 bas-droit
P3 haut-droit
P4 haut-gauche
'''
# --------------------------

def make_box_from_limits(min_x, max_x, min_y, max_y):
    return np.array([
        [min_x, min_y],
        [max_x, min_y],
        [max_x, max_y],
        [min_x, max_y]
    ], dtype=float)

def box_from_two_points(p1, p2):
    xs = sorted([p1[0], p2[0]])
    ys = sorted([p1[1], p2[1]])
    return make_box_from_limits(xs[0], xs[1], ys[0], ys[1])

def extend_box(box, p):
    xs = np.array([*box[:,0], p[0]])
    ys = np.array([*box[:,1], p[1]])
    return make_box_from_limits(xs.min(), xs.max(), ys.min(), ys.max())

def box_limits(box):
    xs = box[:,0]
    ys = box[:,1]
    return xs.min(), xs.max(), ys.min(), ys.max()

def box_dimensions(box):
    min_x, max_x, min_y, max_y = box_limits(box)
    return max_x - min_x, max_y - min_y

def box_area(box):
    w, h = box_dimensions(box)
    return w * h

def box_center(box):
    min_x, max_x, min_y, max_y = box_limits(box)
    return np.array([(min_x + max_x)/2, (min_y + max_y)/2])

def box_contains(box, p):
    min_x, max_x, min_y, max_y = box_limits(box)
    return (min_x <= p[0] <= max_x) and (min_y <= p[1] <= max_y)

def box_overlap_half(b1, b2):
    min_x1, max_x1, min_y1, max_y1 = box_limits(b1)
    min_x2, max_x2, min_y2, max_y2 = box_limits(b2)

    inter_w = max(0, min(max_x1, max_x2) - max(min_x1, min_x2))
    inter_h = max(0, min(max_y1, max_y2) - max(min_y1, min_y2))
    inter_area = inter_w * inter_h

    return inter_area >= box_area(b1) / 2

def detect_local_zones(arr, max_value=150, max_diff=50, min_zone_length=2):
    arr = np.array(arr)
    zones = []
    current = []
    prev = None

    for i, val in enumerate(arr):
        if val > max_value:
            if len(current) >= min_zone_length:
                zones.append(current)
            current = []
            prev = None
            continue

        if not current:
            current = [i]
        else:
            if abs(val - prev) <= max_diff:
                current.append(i)
            else:
                if len(current) >= min_zone_length:
                    zones.append(current)
                current = [i]

        prev = val

    if len(current) >= min_zone_length:
        zones.append(current)

    return zones