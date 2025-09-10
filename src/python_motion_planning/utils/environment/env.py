"""
@file: env.py
@breif: 2-dimension environment
@author: Winter
@update: 2023.1.13
"""
from math import sqrt
from abc import ABC, abstractmethod
from turtle import distance
from scipy.spatial import cKDTree
import numpy as np

from matplotlib import cbook, cm
from matplotlib.colors import LightSource


from .node import Node, Node3D


class Env(ABC):
    """
    Class for building 2-d workspace of robots.

    Parameters:
        x_range (int): x-axis range of enviroment
        y_range (int): y-axis range of environmet
        eps (float): tolerance for float comparison

    Examples:
        >>> from python_motion_planning.utils import Env
        >>> env = Env(30, 40)
    """
    def __init__(self, x_range: int, y_range: int, eps: float = 1e-6) -> None:
        # size of environment
        self.x_range = x_range
        self.y_range = y_range
        self.eps = eps

    @property
    def grid_map(self) -> set:
        return {(i, j) for i in range(self.x_range) for j in range(self.y_range)}

    @abstractmethod
    def init(self) -> None:
        pass


class Env3D(ABC):
    """
    Class for building 3-d workspace of robots.

    Parameters:
        x_range (int): x-axis range of enviroment
        y_range (int): y-axis range of environmet
        z_range (int): z-axis range of environmet
        eps (float): tolerance for float comparison

    Examples:
        >>> from python_motion_planning.utils import Env
        >>> env = Env(30, 40)
    """
    def __init__(self, x_range: int, y_range: int, z_range: int, eps: float = 1e-6) -> None:
        # size of environment
        self.x_range = x_range  
        self.y_range = y_range
        self.z_range = z_range
        self.eps = eps

    @property
    def grid_map(self) -> set:
        return {(i, j, k) for i in range(self.x_range) for j in range(self.y_range) for k in range(self.z_range)}
    
    @abstractmethod
    def init(self) -> None:
        pass


class Mountain(Env):
    def __init__(self):
        # Load terrain data

        with cbook.get_sample_data('jacksboro_fault_dem.npz') as dem:
            z = dem['elevation']
            nrows, ncols = z.shape
            x = np.linspace(dem['xmin'], dem['xmax'], ncols)
            y = np.linspace(dem['ymin'], dem['ymax'], nrows)
            x, y = np.meshgrid(x, y)
            region = np.s_[5:50, 5:50]

            # Conversion factors
            self.dx_meters = 111200 * dem['dx'] * np.cos(np.radians(dem['ymin']))
            self.dy_meters = 111200 * dem['dy']

            # Store both coordinate systems
            self.x_coords = x[region]  # Actual coordinates for plotting
            self.y_coords = y[region]  # Actual coordinates for plotting
            self.z = z[region]         # Elevation data

            # Grid dimensions for pathfinding (indices)
            self.rows, self.cols = self.z.shape

            # Initialize parent class with grid dimensions
            super().__init__(self.cols, self.rows)

        self.elveation_weight = 0.1
        self.obstacles = []
        self.obs_rect = []
        self.obs_circ = []
        self.boundary = [
            [-1, -1, 1, self.rows + 2],      # Left boundary
            [-1, -1, self.cols + 2, 1],      # Bottom boundary
            [self.cols, -1, 1, self.rows + 2], # Right boundary
            [-1, self.rows, self.cols + 2, 1]  # Top boundary
        ]
        self.motions = [Node((-1, 0), None, 1, 0.0), Node((-1, 1),  None, sqrt(2), 0.0),
                        Node((0, 1),  None, 1, 0.0), Node((1, 1),   None, sqrt(2), 0.0),
                        Node((1, 0),  None, 1, 0.0), Node((1, -1),  None, sqrt(2), 0.0),
                        Node((0, -1), None, 1, 0.0), Node((-1, -1), None, sqrt(2), 0.0)]

    def init(self):
        pass

    def getNeighbor(self, node):
        """Generate neighbor nodes with terrain-adjusted costs"""
        neighbors = []
        for motion in self.motions:
            # Work with integer indices
            new_x = int(node.x + motion.x)
            new_y = int(node.y + motion.y)

            # Check bounds using array dimensions
            if 0 <= new_x < self.cols and 0 <= new_y < self.rows:
                # Get elevation at new position
                new_z = self.z[new_y, new_x]
                current_z = self.z[int(node.y), int(node.x)]

                # Add elevation change penalty
                elevation_diff = abs(new_z - current_z)
                terrain_cost = motion.g + elevation_diff * self.elveation_weight

                neighbor = Node((new_x, new_y), node.current, node.g + terrain_cost, 0)
                neighbors.append(neighbor)

        return neighbors

    def index_to_coords(self, i, j):
        return self.x_coords[j, i], self.y_coords[j, i], self.z[j, i]

    def path_distance_meters(self, path):
        if len(path) < 2:
            return 0.0

        total_distance = 0.0
        for i in range(len(path) - 1):
            curr = path[i]
            next_pt = path[i + 1]

            # Grid movement (keep continuous coordinates)
            dx = next_pt[0] - curr[0]
            dy = next_pt[1] - curr[1]

            # Convert to meters
            dx_m = dx * self.dx_meters
            dy_m = dy * self.dy_meters

            # Interpolate elevation at continuous coordinates
            z_curr = self.interpolate_elevation(curr[1], curr[0])
            z_next = self.interpolate_elevation(next_pt[1], next_pt[0])
            dz = z_next - z_curr

            # 3D distance
            distance = sqrt(dx_m**2 + dy_m**2 + dz**2)
            total_distance += distance

        return total_distance

    def interpolate_elevation(self, row, col):
        """Bilinear interpolation for elevation at continuous coordinates"""
        from scipy.interpolate import RectBivariateSpline

        # If you don't want scipy dependency, use simple bilinear interpolation:
        r_floor, c_floor = int(row), int(col)
        r_ceil, c_ceil = min(r_floor + 1, self.rows - 1), min(c_floor + 1, self.cols - 1)

        # Clamp to bounds
        r_floor = max(0, min(r_floor, self.rows - 1))
        c_floor = max(0, min(c_floor, self.cols - 1))

        # Interpolation weights
        dr = row - r_floor
        dc = col - c_floor

        # Bilinear interpolation
        z1 = self.z[r_floor, c_floor] * (1 - dr) + self.z[r_ceil, c_floor] * dr
        z2 = self.z[r_floor, c_ceil] * (1 - dr) + self.z[r_ceil, c_ceil] * dr

        return z1 * (1 - dc) + z2 * dc

class Grid(Env):
    """
    Class for discrete 2-d grid map.

    Parameters:
        x_range (int): x-axis range of enviroment
        y_range (int): y-axis range of environmet
    """
    def __init__(self, x_range: int, y_range: int) -> None:
        super().__init__(x_range, y_range)
        # allowed motions
        self.motions = [Node((-1, 0), None, 1, None), Node((-1, 1),  None, sqrt(2), None),
                        Node((0, 1),  None, 1, None), Node((1, 1),   None, sqrt(2), None),
                        Node((1, 0),  None, 1, None), Node((1, -1),  None, sqrt(2), None),
                        Node((0, -1), None, 1, None), Node((-1, -1), None, sqrt(2), None)]
        # obstacles
        self.obstacles = None
        self.obstacles_tree = None
        self.init()

    def init(self) -> None:
        """
        Initialize grid map.
        """
        x, y = self.x_range, self.y_range
        obstacles = set()

        # boundary of environment
        for i in range(x):
            obstacles.add((i, 0))
            obstacles.add((i, y - 1))
        for i in range(y):
            obstacles.add((0, i))
            obstacles.add((x - 1, i))

        self.update(obstacles)

    def update(self, obstacles):
        self.obstacles = obstacles
        self.obstacles_tree = cKDTree(np.array(list(obstacles)))


class Grid3D(Env3D):
    """
    Class for discrete 2-d grid map.

    Parameters:
        x_range (int): x-axis range of enviroment
        y_range (int): y-axis range of environmet
    """
    def __init__(self, x_range: int, y_range: int, z_range: int) -> None:
        super().__init__(x_range, y_range, z_range)
        # allowed motions
        self.motions = []

        for x in [-1, 0, 1]:
            for y in [-1, 0, 1]:
                for z in [-1, 0, 1]:
                    if (x, y, z) == (0, 0, 0):
                        continue
                    distance = abs(x) + abs(y) + abs(z)
                    cost = 0
                    if distance == 1:
                        cost = 1
                    elif distance == 2:
                        cost = sqrt(2)
                    else:
                        cost = sqrt(3)
                    self.motions.append(Node((x,y,z), None, cost, None))

        # obstacles
        self.obstacles = None
        self.obstacles_tree = None
        self.init()

    def init(self) -> None:
        """
        Initialize grid map.
        """
        x, y, z = self.x_range, self.y_range, self.z_range
        obstacles = set()

        # boundary of environment
        for ix in range(x):
            for iy in range(y):
                obstacles.add((ix, iy, 0))
                obstacles.add((ix, iy, z - 1))

        for ix in range(x):
            for iz in range(z):
                obstacles.add((ix, 0, iz))
                obstacles.add((ix, y - 1, iz))

        for iy in range(y):
            for iz in range(z):
                obstacles.add((0, iy, iz))
                obstacles.add((x - 1, iy, iz))

        self.update(obstacles)

    def update(self, obstacles):
        self.obstacles = obstacles
        self.obstacles_tree = cKDTree(np.array(list(obstacles)))


class Map(Env):
    """
    Class for continuous 2-d map.

    Parameters:
        x_range (int): x-axis range of enviroment
        y_range (int): y-axis range of environmet
    """
    def __init__(self, x_range: int, y_range: int) -> None:
        super().__init__(x_range, y_range)
        self.boundary = None
        self.obs_circ = None
        self.obs_rect = None
        self.init()

    def init(self):
        """
        Initialize map.
        """
        x, y = self.x_range, self.y_range

        # boundary of environment
        self.boundary = [
            [0, 0, 1, y],
            [0, y, x, 1],
            [1, 0, x, 1],
            [x, 1, 1, y]
        ]
        self.obs_rect = []
        self.obs_circ = []

    def update(self, boundary=None, obs_circ=None, obs_rect=None):
        self.boundary = boundary if boundary else self.boundary
        self.obs_circ = obs_circ if obs_circ else self.obs_circ
        self.obs_rect = obs_rect if obs_rect else self.obs_rect
