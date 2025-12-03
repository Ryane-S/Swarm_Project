import numpy as np
import matplotlib.pyplot as plt
from swarm_rescue.team_du_sud.geometry import Point, Box
from swarm_rescue.team_du_sud.QuadTree import Node

# --------------------------
# Root quadtree node (rectangular map, 1.5x bigger)
# --------------------------
W, H = 300, 180   # bigger rectangle
root = Node(Box(Point(0,0), Point(W,0), Point(W,H), Point(0,H)).order_points())

points = []

# --------------------------
# Dense vertical wall (x=150, full height)
# --------------------------
for y in range(0, H+1, 1):   # step=1 → dense
    points.append(Point(150, y))

# --------------------------
# Dense horizontal wall (y=90, full width)
# --------------------------
for x in range(0, W+1, 1):
    points.append(Point(x, 90))

# --------------------------
# Small box obstacle (20x20 at bottom-right)
# --------------------------
x0, y0 = 220, 30
x1, y1 = 240, 50
for x in range(x0, x1+1, 1):
    points.append(Point(x, y0))
    points.append(Point(x, y1))
for y in range(y0, y1+1, 1):
    points.append(Point(x0, y))
    points.append(Point(x1, y))

# --------------------------
# Maze in top-left corner (60x60 block)
# --------------------------
# Outer frame
for x in range(0, 61, 1):
    points.append(Point(x, 120))
    points.append(Point(x, 180))
for y in range(120, 181, 1):
    points.append(Point(0, y))
    points.append(Point(60, y))

# Internal maze walls
for x in range(0, 61, 1):
    points.append(Point(x, 150))   # horizontal divider
for y in range(120, 181, 1):
    points.append(Point(30, y))    # vertical divider

# Small inner box inside maze
for x in range(10, 21, 1):
    points.append(Point(x, 130))
    points.append(Point(x, 140))
for y in range(130, 141, 1):
    points.append(Point(10, y))
    points.append(Point(20, y))

# --------------------------
# Random scatter points (to push >500 total)
# --------------------------
rng = np.random.default_rng(42)
for _ in range(300):   # add 300 random points
    x = rng.integers(0, W)
    y = rng.integers(0, H)
    points.append(Point(x, y))

print(f"Generated {len(points)} points")

# --------------------------
# Insert points into quadtree
# --------------------------
for p in points:
    root.insert_occupied(p, min_size=5)   # small min_size → deeper subdivision

# --------------------------
# Recursive drawing function
# --------------------------
def draw_node(ax, node):
    xs = [p.x for p in node.box.points]
    ys = [p.y for p in node.box.points]
    x_min, y_min = min(xs), min(ys)
    w, h = max(xs) - x_min, max(ys) - y_min
    color = 'red' if node.occupied else 'white'
    rect = plt.Rectangle((x_min, y_min), w, h,
                         edgecolor='black', facecolor=color,
                         linewidth=0.5, alpha=0.5)
    ax.add_patch(rect)
    if node.children:
        for child in node.children:
            draw_node(ax, child)

# --------------------------
# Plot result
# --------------------------
fig, ax = plt.subplots(figsize=(12,8))
draw_node(ax, root)
ax.scatter([p.x for p in points], [p.y for p in points], c='blue', s=5, label="Inserted points")
ax.set_xlim(0, W)
ax.set_ylim(0, H)
ax.set_aspect('equal')
ax.legend()
plt.title("Quadtree with 500+ points on a large rectangular map")
plt.show()
