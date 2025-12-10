import matplotlib.pyplot as plt

from swarm_rescue.team_du_sud.geometry import Box, Point
from swarm_rescue.team_du_sud.QuadTree import Node

# --------------------------
# Map dimensions (aligned to the sketch proportions)
# --------------------------
W, H = 1035, 757
root = Node(Box(Point(0, 0), Point(W, 0), Point(W, H), Point(0, H)).order_points())

obstacle_points = []


# --------------------------
# Helpers to add dense wall points (step=1 for tight succession)
# --------------------------
def hline(x0, x1, y, step=1):
    x_start, x_end = sorted((x0, x1))
    for x in range(x_start, x_end + 1, step):
        obstacle_points.append(Point(x, y))


def vline(x, y0, y1, step=1):
    y_start, y_end = sorted((y0, y1))
    for y in range(y_start, y_end + 1, step):
        obstacle_points.append(Point(x, y))


def rect_outline(x0, y0, x1, y1, step=1):
    hline(x0, x1, y0, step)
    hline(x0, x1, y1, step)
    vline(x0, y0, y1, step)
    vline(x1, y0, y1, step)


# --------------------------
# Walls derived from the provided floorplan sketch
# --------------------------

# Outer frame
rect_outline(10, 10, 1025, 747)

# Top black bars and top-right square
rect_outline(430, 660, 510, 740)
rect_outline(520, 660, 600, 740)
rect_outline(620, 660, 700, 740)
rect_outline(340, 560, 420, 640)
rect_outline(890, 620, 960, 690)

# Large central block
rect_outline(300, 320, 520, 540)

# Tall vertical in top-right area
vline(820, 420, 740)

# Long horizontal corridors (mid-height)
hline(220, 940, 420)
hline(220, 940, 360)
vline(940, 360, 400)

# Left rectangle area
rect_outline(40, 180, 220, 500)
# Crosses inside the left rectangle
hline(100, 140, 420)
vline(120, 400, 440)
hline(80, 160, 300)
vline(120, 260, 340)

# Top-left small corridor/indent
hline(40, 160, 560)
hline(40, 160, 740)
vline(160, 560, 740)

# Bottom-left black blocks
rect_outline(30, 30, 110, 120)
rect_outline(140, 30, 220, 150)

# Bottom-left verticals to frame rooms
vline(40, 10, 180)
hline(40, 220, 180)

# Bottom-middle complex (rooms/doors)
vline(260, 10, 260)
vline(340, 10, 260)
vline(420, 10, 300)
vline(500, 10, 320)
hline(260, 500, 260)
hline(300, 420, 200)
hline(300, 360, 140)
vline(330, 140, 240)
vline(360, 20, 140)

# Central vertical spine and connectors
vline(520, 200, 700)
hline(470, 650, 220)
hline(470, 650, 320)
vline(600, 220, 320)
vline(520, 320, 420)

# Bottom-right corridors and stubs
hline(520, 960, 220)
vline(740, 220, 420)
vline(880, 220, 320)
hline(600, 360, 680)
vline(680, 340, 360)
vline(960, 120, 320)
hline(820, 120, 980)

# Small downward stubs along the long corridors
vline(320, 360, 400)
vline(560, 360, 400)
vline(780, 360, 400)

print(f"Generated {len(obstacle_points)} wall points")

# --------------------------
# Insert points into quadtree
# --------------------------
for p in obstacle_points:
    root.insert_occupied(p, min_size=10)


# --------------------------
# Helper functions for analysis
# --------------------------
def count_nodes(node):
    """Count total nodes in quadtree"""
    count = 1
    if node.children:
        for child in node.children:
            count += count_nodes(child)
    return count


def get_max_depth(node, depth=0):
    """Get maximum depth of quadtree"""
    if not node.children:
        return depth
    return max(get_max_depth(child, depth + 1) for child in node.children)


def get_occupied_nodes(node):
    """Count occupied nodes in quadtree"""
    count = 1 if node.occupied else 0
    if node.children:
        for child in node.children:
            count += get_occupied_nodes(child)
    return count


# --------------------------
# Draw quadtree recursively
# --------------------------
def draw_quadtree(ax, node):
    xs = [p.x for p in node.box.points]
    ys = [p.y for p in node.box.points]
    x_min, y_min = min(xs), min(ys)
    w, h = max(xs) - x_min, max(ys) - y_min
    
    if node.occupied:
        edgecolor = "darkred"
        linewidth = 1.2
    else:
        edgecolor = "lightgray"
        linewidth = 0.4

    rect = plt.Rectangle(
        (x_min, y_min),
        w,
        h,
        edgecolor=edgecolor,
        facecolor="none",  # transparent fill to see the subdivision boxes clearly
        linewidth=linewidth,
    )
    ax.add_patch(rect)
    
    if node.children:
        for child in node.children:
            draw_quadtree(ax, child)


# --------------------------
# Create visualization (single pane)
# --------------------------
fig, ax = plt.subplots(figsize=(12, 8))
ax.set_title("Quadtree Subdivision (Red = Occupied, Green = Empty)", fontsize=14, fontweight="bold")

draw_quadtree(ax, root)

# Overlay wall points
ax.scatter(
    [p.x for p in obstacle_points],
    [p.y for p in obstacle_points],
    c="black",
    s=6,
    marker="s",
    alpha=0.8,
    zorder=3,
    label="Wall points",
)

ax.set_xlim(-20, W + 20)
ax.set_ylim(-20, H + 20)
ax.set_aspect("equal")
ax.tick_params(left=False, bottom=False, labelleft=False, labelbottom=False)
ax.set_xlabel("")
ax.set_ylabel("")
ax.legend(loc="upper right")

plt.tight_layout()

# --------------------------
# Print quadtree statistics
# --------------------------
total_nodes = count_nodes(root)
max_depth = get_max_depth(root)
occupied_nodes = get_occupied_nodes(root)

print("\n" + "="*60)
print("QUADTREE PERFORMANCE ANALYSIS")
print("="*60)
print(f"Map dimensions:              {W} x {H}")
print(f"Total wall points:           {len(obstacle_points)}")
print(f"\nTotal nodes in quadtree:     {total_nodes}")
print(f"Occupied nodes:              {occupied_nodes}")
print(f"Empty nodes:                 {total_nodes - occupied_nodes}")
print(f"Maximum tree depth:          {max_depth}")
print(f"Occupation ratio:            {occupied_nodes/total_nodes*100:.1f}%")
print(f"Points per occupied node:    {len(obstacle_points)/occupied_nodes:.2f}")
print("="*60 + "\n")

plt.show()
