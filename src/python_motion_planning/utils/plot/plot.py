"""
Plot tools 2D
@author: huiming zhou
"""
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.cm import ScalarMappable
import matplotlib.patches as patches

from matplotlib import cbook, cm
from matplotlib.colors import LightSource

from ..environment.env import Env, Grid, Map, Node, Mountain


class Plot:
    def __init__(self, start, goal, env: Env):
        self.start = Node(start, start, 0, 0)
        self.goal = Node(goal, goal, 0, 0)
        self.env = env
        self.fig = plt.figure("planning")
        if isinstance(self.env, Mountain):
            self.ax = self.fig.subplots(subplot_kw=dict(projection='3d'))
        else:
            self.ax = self.fig.add_subplot()

    def animation(self, path: list, name: str, cost: float = None, expand: list = None, history_pose: list = None,
                  predict_path: list = None, lookahead_pts: list = None, cost_curve: list = None,
                  ellipse: np.ndarray = None) -> None:
        name = name + "\ncost: " + str(cost) if cost else name

        self.plotEnv(name)
        if expand is not None:
            self.plotExpand(expand)
        if history_pose is not None:
            self.plotHistoryPose(history_pose, predict_path, lookahead_pts)
        if path is not None:
            self.plotPath(path)

        if cost_curve:
            plt.figure("cost curve")
            self.plotCostCurve(cost_curve, name)

        if ellipse is not None:
            self.plotEllipse(ellipse)

        plt.show()

    def plotEnv(self, title: str = "Environment"):
        """
        Plot terrain with hillshade and proper scaling so elevation differences are visible.
        Uses self.fig / self.ax so 3D axes created in __init__ are used for Mountain.
        """
        # ensure env.z exists and is float if present
        z = None
        if hasattr(self.env, "z"):
            z = np.asarray(self.env.z, dtype=float)

        # If Mountain, draw a 3D surface on the pre-created 3D axes
        if isinstance(self.env, Mountain):
            # clear and use the stored 3D axes
            self.ax.clear()
            if z is None or z.size == 0:
                raise RuntimeError("Environment elevation array is empty")

            # get coordinate grids (x_coords, y_coords) that align with z
            x = np.asarray(self.env.x_coords, dtype=float)
            y = np.asarray(self.env.y_coords, dtype=float)

            # normalize for coloring
            vmin = np.nanmin(z)
            vmax = np.nanmax(z)
            norm = plt.Normalize(vmin=vmin, vmax=vmax)

            # optional vertical exaggeration knob (increase to make relief more visible)
            vert_exag = 1.0

            # facecolors from colormap with LightSource shading
            ls = LightSource(azdeg=315, altdeg=45)
            rgb = ls.shade(z, cmap=cm.terrain, vert_exag=vert_exag, blend_mode='overlay', dx=1, dy=1)

            # plot surface using facecolors (avoid shading by mpl to preserve hillshade)
            try:
                self.ax.plot_surface(x, y, z,
                                     rstride=1, cstride=1,
                                     facecolors=cm.terrain(norm(z)),
                                     linewidth=0, antialiased=False, shade=False)
            except Exception:
                # fallback for older matplotlib: use plot_surface with rgb shading
                self.ax.plot_surface(x, y, z, rstride=1, cstride=1, facecolors=rgb, linewidth=0, antialiased=False, shade=False)

            # labels and title
            self.ax.set_title(title)
            self.ax.set_xlabel("X")
            self.ax.set_ylabel("Y")
            self.ax.set_zlabel("Elevation")

            # add colorbar on the figure
            sm = ScalarMappable(cmap=cm.terrain, norm=norm)
            sm.set_array(z)
            # remove existing colorbars to avoid duplicates
            for c in list(self.fig.axes):
                # keep only the main axes and colorbars are handled below
                pass
            self.fig.colorbar(sm, ax=self.ax, shrink=0.6, pad=0.1)

            return self.fig, self.ax

        # Non-mountain: 2D shaded image on stored 2D axes
        else:
            # ensure we have a 2D axis
            self.ax.clear()
            if z is None or z.size == 0:
                self.ax.set_title(title)
                return self.fig, self.ax

            # extent from env coordinates if available
            try:
                xmin = float(self.env.x_coords.min())
                xmax = float(self.env.x_coords.max())
                ymin = float(self.env.y_coords.min())
                ymax = float(self.env.y_coords.max())
                extent = (xmin, xmax, ymin, ymax)
            except Exception:
                extent = (0, z.shape[1], 0, z.shape[0])

            vmin = np.nanmin(z)
            vmax = np.nanmax(z)
            if vmin == vmax:
                vmax = vmin + 1.0

            ls = LightSource(azdeg=315, altdeg=45)
            rgb = ls.shade(z, cmap=cm.terrain, vert_exag=1.0, blend_mode='overlay', dx=1, dy=1)

            self.ax.imshow(rgb, extent=extent, origin='lower', aspect='auto')
            sm = ScalarMappable(cmap=cm.terrain)
            sm.set_clim(vmin, vmax)
            sm.set_array(z)
            # attach colorbar to the figure
            self.fig.colorbar(sm, ax=self.ax, fraction=0.046, pad=0.04)
            self.ax.set_title(title)
            self.ax.set_xlabel("X")
            self.ax.set_ylabel("Y")
            return self.fig, self.ax

    def plotExpand(self, expand: list) -> None:
        '''
        Plot expanded grids using in graph searching.

        Parameters
        ----------
        expand: Expanded grids during searching
        '''
        if self.start in expand:
            expand.remove(self.start)
        if self.goal in expand:
            expand.remove(self.goal)

        count = 0
        if isinstance(self.env, Grid):
            for x in expand:
                count += 1
                plt.plot(x.x, x.y, color="#dddddd", marker='s')
                plt.gcf().canvas.mpl_connect('key_release_event',
                                            lambda event: [exit(0) if event.key == 'escape' else None])
                if count < len(expand) / 3:         length = 20
                elif count < len(expand) * 2 / 3:   length = 30
                else:                               length = 40
                if count % length == 0:             plt.pause(0.001)
        
        if isinstance(self.env, Map):
            for x in expand:
                count += 1
                if x.parent:
                    plt.plot([x.parent[0], x.x], [x.parent[1], x.y], 
                        color="#dddddd", linestyle="-")
                    plt.gcf().canvas.mpl_connect('key_release_event',
                                                 lambda event:
                                                 [exit(0) if event.key == 'escape' else None])
                    if count % 10 == 0:
                        plt.pause(0.001)

        plt.pause(0.01)

    def plotPath(self, path: list, path_color: str='#ff4500', path_style: str="-") -> None:
        '''
        Plot path in global planning.

        Parameters
        ----------
        path: Path found in global planning
        '''
        if isinstance(self.env, Mountain):
            # 3D path plotting for mountain environment
            elevation_range = np.max(self.env.z) - np.min(self.env.z)
            height_offset = elevation_range * 0.1  # Elevate path above terrain

            path_x, path_y, path_z = [], [], []
            for point in path:
                x_coord, y_coord, z_coord = self.env.index_to_coords(int(point[0]), int(point[1]))
                path_x.append(x_coord)
                path_y.append(y_coord)
                path_z.append(z_coord + height_offset)  # offset so path is visible above terrain

            # Use plot3D with thicker line and high zorder
            self.ax.plot3D(path_x, path_y, path_z, path_style, linewidth=5, color=path_color, zorder=100)

            # mark start and goal with larger, contrasting markers
            try:
                sx, sy, sz = self.env.index_to_coords(int(self.start.x), int(self.start.y))
                gx, gy, gz = self.env.index_to_coords(int(self.goal.x), int(self.goal.y))
                self.ax.scatter3D([sx], [sy], [sz + height_offset], color='yellow', s=120, edgecolors='k', zorder=200)
                self.ax.scatter3D([gx], [gy], [gz + height_offset], color='cyan', s=120, edgecolors='k', zorder=200)
            except Exception:
                pass

        else:
            path_x = [path[i][0] for i in range(len(path))]
            path_y = [path[i][1] for i in range(len(path))]
            plt.plot(path_x, path_y, path_style, linewidth=3, color=path_color)
            # larger, filled markers for start/goal so they're more visible
            plt.scatter([self.start.x], [self.start.y], marker="s", color="#ff0000", s=120, edgecolors='k')
            plt.scatter([self.goal.x], [self.goal.y], marker="s", color="#1155cc", s=120, edgecolors='k')

    def plotAgent(self, pose: tuple, radius: float=1) -> None:
        '''
        Plot agent with specifical pose.

        Parameters
        ----------
        pose: Pose of agent
        radius: Radius of agent
        '''
        x, y, theta = pose
        ref_vec = np.array([[radius / 2], [0]])
        rot_mat = np.array([[np.cos(theta), -np.sin(theta)],
                            [np.sin(theta),  np.cos(theta)]])
        end_pt = rot_mat @ ref_vec + np.array([[x], [y]])

        try:
            self.ax.artists.pop()
            for art in self.ax.get_children():
                if isinstance(art, matplotlib.patches.FancyArrow):
                    art.remove()
        except:
            pass

        self.ax.arrow(x, y, float(end_pt[0]) - x, float(end_pt[1]) - y,
                width=0.1, head_width=0.40, color="r")
        circle = plt.Circle((x, y), radius, color="r", fill=False)
        self.ax.add_artist(circle)

    def plotHistoryPose(self, history_pose, predict_path=None, lookahead_pts=None) -> None:
        lookahead_handler = None
        for i, pose in enumerate(history_pose):
            if i < len(history_pose) - 1:
                plt.plot([history_pose[i][0], history_pose[i + 1][0]],
                    [history_pose[i][1], history_pose[i + 1][1]], c="#13ae00")
                if predict_path is not None:
                    plt.plot(predict_path[i][:, 0], predict_path[i][:, 1], c="#ddd")
            i += 1

            # agent
            self.plotAgent(pose)

            # lookahead
            if lookahead_handler is not None:
                lookahead_handler.remove()
            if lookahead_pts is not None:
                try:
                    lookahead_handler = self.ax.scatter(lookahead_pts[i][0], lookahead_pts[i][1], c="b")
                except:
                    lookahead_handler = self.ax.scatter(lookahead_pts[-1][0], lookahead_pts[-1][1], c="b")

            plt.gcf().canvas.mpl_connect('key_release_event',
                                        lambda event: [exit(0) if event.key == 'escape' else None])
            if i % 5 == 0:             plt.pause(0.03)

    def plotCostCurve(self, cost_list: list, name: str) -> None:
        '''
        Plot cost curve with epochs using in evolutionary searching.

        Parameters
        ----------
        cost_list: Cost with epochs
        name: Algorithm name or some other information
        '''
        plt.plot(cost_list, color="b")
        plt.xlabel("epochs")
        plt.ylabel("cost value")
        plt.title(name)
        plt.grid()

    def plotEllipse(self, ellipse: np.ndarray, color: str = 'darkorange', linestyle: str = '--', linewidth: float = 2):
        plt.plot(ellipse[0, :], ellipse[1, :], linestyle=linestyle, color=color, linewidth=linewidth)

    def connect(self, name: str, func) -> None:
        self.fig.canvas.mpl_connect(name, func)

    def clean(self):
        plt.cla()

    def update(self):
        self.fig.canvas.draw_idle()

    @staticmethod
    def plotArrow(x, y, theta, length, color):
        angle = np.deg2rad(30)
        d = 0.5 * length
        w = 2

        x_start, y_start = x, y
        x_end = x + length * np.cos(theta)
        y_end = y + length * np.sin(theta)

        theta_hat_L = theta + np.pi - angle
        theta_hat_R = theta + np.pi + angle

        x_hat_start = x_end
        x_hat_end_L = x_hat_start + d * np.cos(theta_hat_L)
        x_hat_end_R = x_hat_start + d * np.cos(theta_hat_R)

        y_hat_start = y_end
        y_hat_end_L = y_hat_start + d * np.sin(theta_hat_L)
        y_hat_end_R = y_hat_start + d * np.sin(theta_hat_R)

        plt.plot([x_start, x_end], [y_start, y_end], color=color, linewidth=w)
        plt.plot([x_hat_start, x_hat_end_L], [y_hat_start, y_hat_end_L], color=color, linewidth=w)
        plt.plot([x_hat_start, x_hat_end_R], [y_hat_start, y_hat_end_R], color=color, linewidth=w)

    @staticmethod
    def plotCar(x, y, theta, width, length, color):
        theta_B = np.pi + theta

        xB = x + length / 4 * np.cos(theta_B)
        yB = y + length / 4 * np.sin(theta_B)

        theta_BL = theta_B + np.pi / 2
        theta_BR = theta_B - np.pi / 2

        x_BL = xB + width / 2 * np.cos(theta_BL)        # Bottom-Left vertex
        y_BL = yB + width / 2 * np.sin(theta_BL)
        x_BR = xB + width / 2 * np.cos(theta_BR)        # Bottom-Right vertex
        y_BR = yB + width / 2 * np.sin(theta_BR)

        x_FL = x_BL + length * np.cos(theta)               # Front-Left vertex
        y_FL = y_BL + length * np.sin(theta)
        x_FR = x_BR + length * np.cos(theta)               # Front-Right vertex
        y_FR = y_BR + length * np.sin(theta)

        plt.plot([x_BL, x_BR, x_FR, x_FL, x_BL],
                 [y_BL, y_BR, y_FR, y_FL, y_BL],
                 linewidth=1, color=color)

        Plot.plotArrow(x, y, theta, length / 2, color)
