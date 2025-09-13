"""
@file: planner.py
@breif: Abstract class for planner
@author: Winter
@update: 2023.1.17
"""
import math
from abc import abstractmethod, ABC
from ..environment.env import Env, Node
from ..plot.plot import Plot

def get_line_points(start, end):
    """Bresenham's line algorithm to get all points between start and end"""
    points = []
    x1, y1 = start
    x2, y2 = end

    dx = abs(x2 - x1)
    dy = abs(y2 - y1)
    sx = 1 if x1 < x2 else -1
    sy = 1 if y1 < y2 else -1
    err = dx - dy

    while True:
        points.append((x1, y1))
        if x1 == x2 and y1 == y2:
            break
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x1 += sx
        if e2 < dx:
            err += dx
            y1 += sy

    return points

class Planner(ABC):
    def __init__(self, start: tuple, goal: tuple, env: Env) -> None:
        # plannig start and goal
        self.start = Node(start, start, 0, 0)
        self.goal = Node(goal, goal, 0, 0)
        # environment
        self.env = env
        # graph handler
        self.plot = Plot(start, goal, env)

    def dist(self, node1: Node, node2: Node) -> float:
        return math.hypot(node2.x - node1.x, node2.y - node1.y)
    
    def angle(self, node1: Node, node2: Node) -> float:
        return math.atan2(node2.y - node1.y, node2.x - node1.x)

    def interpolate_path(self, jump_points):
        """Convert jump points to detailed path"""
        detailed_path = []

        for i in range(len(jump_points) - 1):
            start = jump_points[i]
            end = jump_points[i + 1]

            # Get all points along the line from start to end
            line_points = get_line_points(start, end)
            detailed_path.extend(line_points[:-1])  # Avoid duplicating end points

        detailed_path.append(jump_points[-1])  # Add final point
        return detailed_path

    def interpolate_continuous_path(self, continuous_points):
        """Convert continuous RRT* path to discrete points for drawing"""
        detailed_path = []

        for i in range(len(continuous_points) - 1):
            # Round to nearest integers for Bresenham
            start = (int(round(continuous_points[i][0])), int(round(continuous_points[i][1])))
            end = (int(round(continuous_points[i+1][0])), int(round(continuous_points[i+1][1])))

            # Use your existing Bresenham function
            line_points = get_line_points(start, end)
            detailed_path.extend(line_points[:-1])  # Avoid duplicates

        detailed_path.append((int(round(continuous_points[-1][0])), int(round(continuous_points[-1][1]))))
        return detailed_path

    @abstractmethod
    def plan(self):
        '''
        Interface for planning.
        '''
        pass

    @abstractmethod
    def run(self):
        '''
        Interface for running both plannig and animation.
        '''
        pass

    @abstractmethod
    def plan_path(self):
        pass


class Planner3D(ABC):
    def __init__(self, start: tuple, goal: tuple, env: Env) -> None:
        # plannig start and goal
        self.start = Node3D(start, start, 0, 0)
        self.goal = Node3D(goal, goal, 0, 0)
        # environment
        self.env = env
        # graph handler
        self.plot = Plot(start, goal, env)

    def dist(self, node1: Node, node2: Node) -> float:
        dx = node2.x - node1.x
        dy = node2.y - node1.y
        dz = node2.z - node1.z
        return math.sqrt(dx**2 + dy**2 + dz**2)

    def angle(self, node1: Node, node2: Node) -> float:
        print("Error: Planner3D.angle is not implemented!", file=sys.stderr)
        return 0.0

    @abstractmethod
    def plan(self):
        '''
        Interface for planning.
        '''
        pass

    @abstractmethod
    def run(self):
        '''
        Interface for running both plannig and animation.
        '''
        pass
