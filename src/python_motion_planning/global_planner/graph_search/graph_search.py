"""
@file: graph_search.py
@breif: Base class for planner based on graph searching
@author: Winter
@update: 2023.1.13
"""
import math
from python_motion_planning.utils import Env, Node, Planner, Grid, Env3D, Node3D, Planner3D, Grid3D


class GraphSearcher(Planner):
    """
    Base class for planner based on graph searching.

    Parameters:
        start (tuple): start point coordinate
        goal (tuple): goal point coordinate
        env (Grid): environment
        heuristic_type (str): heuristic function type
    """
    def __init__(self, start: tuple, goal: tuple, env: Grid, heuristic_type: str="euclidean") -> None:
        super().__init__(start, goal, env)
        # heuristic type
        self.heuristic_type = heuristic_type
        # allowed motions
        self.motions = self.env.motions
        # obstacles
        # self.obstacles = self.env.obstacles

    def h(self, node: Node, goal: Node) -> float:
        """
        Calculate heuristic.

        Parameters:
            node (Node): current node
            goal (Node): goal node

        Returns:
            h (float): heuristic function value of node
        """
        if self.heuristic_type == "manhattan":
            return abs(goal.x - node.x) + abs(goal.y - node.y)
        elif self.heuristic_type == "euclidean":
            return math.hypot(goal.x - node.x, goal.y - node.y)

    def cost(self, node1: Node, node2: Node) -> float:
        """
        Calculate cost for this motion.

        Parameters:
            node1 (Node): node 1
            node2 (Node): node 2

        Returns:
            cost (float): cost of this motion
        """
        if self.isCollision(node1, node2):
            return float("inf")
        return self.dist(node1, node2)

    def isCollision(self, node1: Node, node2: Node) -> bool:
        """
        Judge collision when moving from node1 to node2.

        Parameters:
            node1 (Node): node 1
            node2 (Node): node 2

        Returns:
            collision (bool): True if collision exists else False
        """
        if node1.current in self.obstacles or node2.current in self.obstacles:
            return True

        x1, y1 = node1.x, node1.y
        x2, y2 = node2.x, node2.y

        if x1 != x2 and y1 != y2:
            if x2 - x1 == y1 - y2:
                s1 = (min(x1, x2), min(y1, y2))
                s2 = (max(x1, x2), max(y1, y2))
            else:
                s1 = (min(x1, x2), max(y1, y2))
                s2 = (max(x1, x2), min(y1, y2))
            if s1 in self.obstacles or s2 in self.obstacles:
                return True
        return False


class GraphSearcher3D(Planner3D):
    """
    Base class for planner based on graph searching.

    Parameters:
        start (tuple): start point coordinate
        goal (tuple): goal point coordinate
        env (Grid): environment
        heuristic_type (str): heuristic function type
    """
    def __init__(self, start: tuple, goal: tuple, env: Grid3D, heuristic_type: str="euclidean") -> None:
        super().__init__(start, goal, env)
        # heuristic type
        self.heuristic_type = heuristic_type
        # allowed motions
        self.motions = self.env.motions
        # obstacles
        self.obstacles = self.env.obstacles

    def h(self, node: Node3D, goal: Node3D) -> float:
        """
        Calculate heuristic.

        Parameters:
            node (Node3D): current node
            goal (Node3D): goal node

        Returns:
            h (float): heuristic function value of node
        """
        dx = goal.x - node.x
        dy = goal.y - node.y
        dz = goal.z - node.z

        if self.heuristic_type == "manhattan":
            return abs(dx) + abs(dy) + abs(dz)
        elif self.heuristic_type == "euclidean":
            return math.sqrt(dx**2 + dy**2 + dz**2)

    def cost(self, node1: Node3D, node2: Node3D) -> float:
        """
        Calculate cost for this motion.

        Parameters:
            node1 (Node3D): node 1
            node2 (Node3D): node 2

        Returns:
            cost (float): cost of this motion
        """
        if self.isCollision(node1, node2):
            return float("inf")
        return self.dist(node1, node2)

    def isCollision(self, node1: Node3D, node2: Node3D) -> bool:
        """
        Judge collision when moving from node1 to node2.

        Parameters:
            node1 (Node3D): node 1
            node2 (Node3D): node 2

        Returns:
            collision (bool): True if collision exists else False
        """
        if node1.current in self.obstacles or node2.current in self.obstacles:
            return True

        x1, y1, z1 = node1.x, node1.y, node1.z
        x2, y2, z2 = node2.x, node2.y, node2.z

        min_x, max_x = min(x1, x2), max(x1, x2)
        min_y, max_y = min(y1, y2), max(y1, y2)
        min_z, max_z = min(z1, z2), max(z1, z2)

        for x in range(min_x, max_x + 1):
            for y in range(min_y, max_y + 1):
                for z in range(min_z, max_z + 1):
                    if (x, y, z) not in [(x1, y1, z1), (x2, y2, z2)] and (x, y, z) in self.obstacles:
                        return True
        return False
