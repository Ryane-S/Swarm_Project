from swarm_rescue.team_du_sud.geometry import Box, Point

# y increasing upward vs downward


class QuadTree:
    def __init__(self, w, h, min_size=5):
        self.root = Node(
            Box(Point(0, 0), Point(w, 0), Point(w, h), Point(0, h)).order_points()
        )
        self.min_size = min_size

    def insert_point(self, point):
        self.root.insert_occupied(point, self.min_size)

    def get_unoccupied_nodes(self):  # might remove this too, see end of class
        return self.root.collect_unoccupied()


class Node:
    def __init__(self, box: Box):
        self.box = box
        self.occupied = False
        self.children = None

    def subdivide(self):
        min_x, max_x, min_y, max_y = self.box.get_limits()
        mid_x = (min_x + max_x) / 2
        mid_y = (min_y + max_y) / 2

        SW = Box(
            Point(min_x, min_y),
            Point(mid_x, min_y),
            Point(mid_x, mid_y),
            Point(min_x, mid_y),
        ).order_points()

        SE = Box(
            Point(mid_x, min_y),
            Point(max_x, min_y),
            Point(max_x, mid_y),
            Point(mid_x, mid_y),
        ).order_points()

        NW = Box(
            Point(min_x, mid_y),
            Point(mid_x, mid_y),
            Point(mid_x, max_y),
            Point(min_x, max_y),
        ).order_points()

        NE = Box(
            Point(mid_x, mid_y),
            Point(max_x, mid_y),
            Point(max_x, max_y),
            Point(mid_x, max_y),
        ).order_points()

        self.children = [
            Node(NW),  # index 0 — top-left
            Node(NE),  # index 1 — top-right
            Node(SW),  # index 2 — bottom-left
            Node(SE),  # index 3 — bottom-right
        ]

    def child_for(self, point: Point):
        center = self.box.get_center()
        if point.x < center.x and point.y >= center.y:
            return self.children[0]
        elif point.x >= center.x and point.y >= center.y:
            return self.children[1]
        elif point.x < center.x and point.y < center.y:
            return self.children[2]
        else:
            return self.children[3]

    def insert_occupied(
        self, point: Point, min_size=5
    ):  # should define the minimum size later, most likely gonna be drone size, update: maybe 1/4th?
        if not self.box.is_inside(point):
            return

        w, h = self.box.get_dimensions()

        if w <= min_size or h <= min_size:
            self.occupied = True
            return

        if self.children is None:
            self.subdivide()

        child = self.child_for(point)
        child.insert_occupied(point, min_size)

        if all(child.occupied for child in self.children):
            self.children = None
            self.occupied = True

    def is_occupied(self, point) -> bool:
        if not self.box.is_inside(point):
            return False
        if self.children == None:
            return self.occupied

        child = self.child_for(point)
        return child.is_occupied(point)

    def collect_unoccupied(self):  # most likely gonna remove this
        if self.occupied:
            return []
        if not self.children:
            return [self]  # This is an unoccupied leaf
        leaves = []
        for child in self.children:
            leaves.extend(child.collect_unoccupied())
        return leaves
