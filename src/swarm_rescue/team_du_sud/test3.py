#!/usr/bin/env python3
"""
Print QuadTree hierarchy (ASCII tree) after inserting points.

This script builds a QuadTree, inserts a few example points to force subdivision,
and prints an ASCII hierarchical tree showing each node's id, whether it's a
leaf, whether it's occupied, and the box limits/size.

Run:
    python3 swarm_Project/src/swarm_rescue/team_du_sud/test3.py
"""

from __future__ import annotations

import os
import sys
from typing import Optional

# Try direct import first; if running from repository root, add src/ to PATH.
try:
    from swarm_rescue.team_du_sud.geometry import Point
    from swarm_rescue.team_du_sud.QuadTree import Node, QuadTree
except Exception:
    # Compute candidate src directory relative to this file:
    # file is .../src/swarm_rescue/team_du_sud/test3.py
    this_dir = os.path.dirname(__file__)
    candidate_src = os.path.dirname(os.path.dirname(os.path.dirname(this_dir)))
    if os.path.isdir(candidate_src) and candidate_src not in sys.path:
        sys.path.insert(0, candidate_src)
    try:
        from swarm_rescue.team_du_sud.geometry import Point
        from swarm_rescue.team_du_sud.QuadTree import Node, QuadTree
    except Exception:
        raise RuntimeError(
            "Could not import project modules. Run from project root or ensure `src/` is on PYTHONPATH."
        )


def _format_limits(box) -> str:
    min_x, max_x, min_y, max_y = box.get_limits()
    w, h = box.get_dimensions()
    return (
        f"limits=({min_x:.1f},{max_x:.1f},{min_y:.1f},{max_y:.1f}) size={w:.1f}x{h:.1f}"
    )


def print_tree(root: Node) -> None:
    """
    Print the quadtree as an ASCII hierarchy.

    Each node is assigned a sequential id during traversal. The output uses
    unicode box-drawing characters for a clear tree structure.
    """
    counter = {"next": 1}

    def next_id() -> int:
        nid = counter["next"]
        counter["next"] += 1
        return nid

    def recurse(node: Node, prefix: str = "", is_last: bool = True) -> None:
        nid = next_id()
        leaf = node.children is None
        occ = bool(node.occupied)
        branch = "└─" if is_last else "├─"
        print(
            f"{prefix}{branch} id={nid} leaf={leaf} occ={occ} {_format_limits(node.box)}"
        )
        if node.children:
            child_prefix = prefix + ("   " if is_last else "│  ")
            for i, child in enumerate(node.children):
                recurse(child, child_prefix, i == len(node.children) - 1)

    print("QuadTree hierarchy:")
    recurse(root, "", True)


def print_unoccupied_nodes(nodes: list) -> None:
    """
    Print the list of unoccupied leaf nodes as a flat list.

    collect_unoccupied() returns a flat list with no hierarchy information,
    so we print it as a simple numbered list.
    """
    if not nodes:
        print("(no unoccupied nodes)")
        return

    print(f"Flat list with {len(nodes)} unoccupied leaf nodes:\n")
    for i, node in enumerate(nodes):
        min_x, max_x, min_y, max_y = node.box.get_limits()
        w, h = node.box.get_dimensions()
        center = node.box.get_center()
        print(
            f"  [{i}] limits=({min_x:.1f},{max_x:.1f},{min_y:.1f},{max_y:.1f}) "
            f"size={w:.1f}x{h:.1f} center=({center.x:.1f},{center.y:.1f})"
        )


def demo() -> None:
    # Example configuration
    W, H = 100, 60
    min_size = 10  # threshold to stop subdividing

    qt = QuadTree(W, H, min_size=min_size)

    # Example points designed to occupy separate quadrants and the center.
    points = [
        Point(20, 45),  # NW
        Point(80, 45),  # NE
        Point(20, 10),  # SW
        Point(80, 10),  # SE
        Point(50, 30),  # center
    ]

    print(f"QuadTree demo: W={W}, H={H}, min_size={min_size}")
    print("Inserting points:", ", ".join(f"({p.x},{p.y})" for p in points))
    for p in points:
        qt.insert_point(p)

    print()
    print_tree(qt.root)

    print()
    print("=" * 70)
    print("OUTPUT OF collect_unoccupied() - returns a FLAT LIST:")
    print("=" * 70)
    unoccupied = qt.get_unoccupied_nodes()
    print_unoccupied_nodes(unoccupied)


if __name__ == "__main__":
    demo()
