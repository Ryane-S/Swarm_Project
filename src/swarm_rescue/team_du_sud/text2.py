import matplotlib.pyplot as plt

from swarm_rescue.team_du_sud.QuadTree import Node
from swarm_rescue.team_du_sud.geometry import Box, Point
from swarm_rescue.team_du_sud.dijkstra import Dijkstra


# Monkey-patch a robust neighbor check (bypasses buggy geometry.are_neighbors)
def _boxes_touch(b1: Box, b2: Box, eps: float = 1e-9) -> bool:
    min_x1, max_x1, min_y1, max_y1 = b1.get_limits()
    min_x2, max_x2, min_y2, max_y2 = b2.get_limits()

    # Overlap helper
    def overlap(a0, a1, b0, b1):
        return min(a1, b1) - max(a0, b0)

    horiz_overlap = overlap(min_x1, max_x1, min_x2, max_x2)
    vert_overlap = overlap(min_y1, max_y1, min_y2, max_y2)

    # Adjacent vertically (share a horizontal edge)
    if abs(max_y1 - min_y2) <= eps or abs(max_y2 - min_y1) <= eps:
        return horiz_overlap > 0
    # Adjacent horizontally (share a vertical edge)
    if abs(max_x1 - min_x2) <= eps or abs(max_x2 - min_x1) <= eps:
        return vert_overlap > 0
    return False


Box.are_neighbors = _boxes_touch  # type: ignore

# Lightweight incremental map + graph test
W, H = 200, 150
MIN_SIZE = 8  # quadtree min cell size

root = Node(Box(Point(0, 0), Point(W, 0), Point(W, H), Point(0, H)).order_points())
dijkstra = Dijkstra()

obstacle_points = []

def hline(x0, x1, y, step=1):
    x_start, x_end = sorted((x0, x1))
    for x in range(x_start, x_end + 1, step):
        obstacle_points.append(Point(x, y))
        root.insert_occupied(obstacle_points[-1], min_size=MIN_SIZE)


def vline(x, y0, y1, step=1):
    y_start, y_end = sorted((y0, y1))
    for y in range(y_start, y_end + 1, step):
        obstacle_points.append(Point(x, y))
        root.insert_occupied(obstacle_points[-1], min_size=MIN_SIZE)


steps = [
    ("Step 1: vertical wall", lambda: vline(60, 20, 130, step=1)),
    ("Step 2: horizontal wall", lambda: hline(30, 170, 70, step=1)),
    ("Step 3: small box", lambda: (hline(110, 140, 40), hline(110, 140, 60), vline(110, 40, 60), vline(140, 40, 60))),
    ("Step 4: right barrier", lambda: vline(170, 20, 120, step=1)),
    ("Step 5: bottom shelf", lambda: hline(40, 120, 20, step=1)),
]

# Track graph incrementally
graph = {}
seen_nodes = set()

for label, action in steps:
    action()

    # Collect unoccupied leaf nodes and identify new ones
    unocc = root.collect_unoccupied()
    new_nodes = [n for n in unocc if n not in seen_nodes]
    for n in new_nodes:
        seen_nodes.add(n)

    graph = dijkstra.build_incremental_graph(new_nodes, existing_graph=graph)

    total_edges = sum(len(v) for v in graph.values())
    print(f"\n{label}")
    print(f"  Obstacles so far:   {len(obstacle_points)} points")
    print(f"  Unoccupied leaves:  {len(unocc)}")
    print(f"  Graph nodes:        {len(graph)}")
    print(f"  Graph edges (dir):  {total_edges}")

# Final visualization of quadtree subdivision
fig, ax = plt.subplots(figsize=(8, 6))
ax.set_title("Quadtree subdivision after incremental walls", fontsize=12)


def draw_quadtree(ax, node):
    xs = [p.x for p in node.box.points]
    ys = [p.y for p in node.box.points]
    x_min, y_min = min(xs), min(ys)
    w, h = max(xs) - x_min, max(ys) - y_min
    edgecolor = "darkred" if node.occupied else "lightgray"
    linewidth = 1.1 if node.occupied else 0.4
    rect = plt.Rectangle((x_min, y_min), w, h, edgecolor=edgecolor, facecolor="none", linewidth=linewidth)
    ax.add_patch(rect)
    if node.children:
        for child in node.children:
            draw_quadtree(ax, child)


draw_quadtree(ax, root)
ax.scatter([p.x for p in obstacle_points], [p.y for p in obstacle_points], c="black", s=6, marker="s", alpha=0.8, zorder=3)
ax.set_xlim(-5, W + 5)
ax.set_ylim(-5, H + 5)
ax.set_aspect("equal")
ax.tick_params(left=False, bottom=False, labelleft=False, labelbottom=False)
ax.set_xlabel("")
ax.set_ylabel("")
ax.legend(["wall points"], loc="upper right")
plt.tight_layout()
plt.show()
