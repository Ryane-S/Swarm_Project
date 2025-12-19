
from swarm_rescue.team_du_sud.QuadTree import Node


class GraphBuilder:

    def __init__(self, existing_graph, unoccupied_nodes, nodes_to_prune):
        self.existing_graph = existing_graph
        self.unoccupied_nodes = unoccupied_nodes
        self.nodes_to_prune = nodes_to_prune

    def _prune(self):

        if self.existing_graph is None :
            return
        
        for node in self.nodes_to_prune :
            if node in self.existing_graph:
                for i in self.existing_graph[node] :
                    if i in self.existing_graph :
                        self.existing_graph[i].remove(node)
                del self.existing_graph[node]

    def _update_existing(self, new_nodes) :
        existing_nodes = [n for n in self.existing_graph if n not in new_nodes]
        for n in existing_nodes :
            for m in new_nodes :
                if n.box.are_neighbors(m.box) :
                    self.existing_graph[n].append(m)


    def build(self):     #complexity is O(K x U) with K the new nodes processed and U the total unoccupied nodes in the quadtree
        
        if self.existing_graph is None:
            self.existing_graph = {}

        self._prune()

        graph = self.existing_graph
        
        new_nodes = [n for n in self.unoccupied_nodes if n not in graph]
        for node in new_nodes:
                graph[node] = node.adjacency_list(self.unoccupied_nodes)
            
        self._update_existing(new_nodes)

        return graph
    
    