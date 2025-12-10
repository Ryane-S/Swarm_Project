from swarm_rescue.team_du_sud.geometry import Box, Line, Point
from swarm_rescue.team_du_sud.QuadTree import Node


class GraphBuilder:
    """
    Class for building a graph from a set of points and lines.
    first itll get the self.children = None
    then itll see the if any of the added nodes have a parent in the graph, if so delete the parent
    """

    def __init__(self, existing_graph, QuadTree):
        self.existing_graph = existing_graph
        self.QuadTree = QuadTree

    def build_graph(self):
        if self.existing_graph is None:
            self.existing_graph = {}
