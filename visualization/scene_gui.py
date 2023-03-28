import matplotlib.pyplot as plt
import shapely
import numpy as np


class SceneGUI:

    def __init__(self, robot, obstacles, init_robot_state, goal, xlim, ylim, show_axis=False,
                 robot_color='k', robot_markersize=14, robot_alpha=0.7,
                 obstacle_color='lightgrey', obstacle_edge_color='k', show_obs_name=False,
                 goal_color='y', goal_marker='*', goal_markersize=16,
                 travelled_path_color='k', travelled_path_linestyle='-', travelled_path_linewidth=2):
        self.obstacles = obstacles
        self.robot = robot
        self.fig, self.ax = plt.subplots()
        self.ax.set_xlim(xlim), self.ax.set_ylim(ylim)
        if not show_axis:
            self.ax.set_axis_off()
        self.goal_handle = self.ax.plot(*goal, color=goal_color, marker=goal_marker, markersize=goal_markersize)[0]
        self.robot_handles, _ = robot.init_plot(ax=self.ax, color=robot_color, markersize=robot_markersize, alpha=robot_alpha)
        self.obstacle_handles = []
        for o in obstacles:
            lh, _ = o.init_plot(ax=self.ax, fc=obstacle_color, ec=obstacle_edge_color, show_name=show_obs_name,
                                show_reference=False)
            self.obstacle_handles.append(lh)
        self.travelled_path_handle = self.ax.plot([], [], color=travelled_path_color, linestyle=travelled_path_linestyle, linewidth=travelled_path_linewidth, zorder=0)[0]
        # Simulation ctrl
        self.fig_open = True
        self.paused = True
        self.step_once = False
        self.fig.canvas.mpl_connect('close_event', self.on_close)
        self.fig.canvas.mpl_connect('key_press_event', self.on_press)
        # Memory
        self.travelled_path = []
        #
        # self.update(init_robot_state, goal, 0)

    def update(self, robot_state=None, goal=None, time=None):
        if time:
            self.ax.set_title("Time: {:.1f} s".format(time))

        # Obstacles
        [oh[0].update_plot(oh[1]) for oh in zip(self.obstacles, self.obstacle_handles)]

        # Robot and goal position
        if robot_state is not None:
            self.travelled_path += list(self.robot.h(robot_state))
            self.robot.update_plot(robot_state, self.robot_handles)
            self.travelled_path_handle.set_data(self.travelled_path[::2], self.travelled_path[1::2])
        if goal is not None:
            self.goal_handle.set_data(*goal)


    def on_close(self, event):
        self.fig_open = False

    def on_press(self, event):
        if event.key == ' ':
            self.paused = not self.paused
        elif event.key == 'right':
            self.step_once = True
            self.paused = True
        else:
            print(event.key)


class SoadsGUI(SceneGUI):
    def __init__(self, robot, obstacles, init_robot_state, goal, xlim, ylim, show_axis=False,
                 robot_color='k', robot_markersize=14, robot_alpha=0.7,
                 obstacle_color='lightgrey', obstacle_edge_color='k', show_obs_name=False,
                 goal_color='y', goal_marker='*', goal_markersize=16,
                 travelled_path_color='k', travelled_path_linestyle='-', travelled_path_linewidth=2,
                 obstacles_star_color='r', obstacles_star_show_reference=True, obstacles_star_alpha=0.1):
        self.obstacle_star_handles = []
        self.obstacles_star_draw_options = {'fc': obstacles_star_color, 'show_reference': obstacles_star_show_reference,
                                            'alpha': obstacles_star_alpha, 'zorder': 0}
        super().__init__(robot, obstacles, init_robot_state, goal, xlim, ylim, show_axis, robot_color, robot_markersize,
                         robot_alpha, obstacle_color, obstacle_edge_color, show_obs_name, goal_color, goal_marker,
                         goal_markersize, travelled_path_color, travelled_path_linestyle, travelled_path_linewidth)

    def update(self, robot_state=None, goal=None, time=None, obstacles_star=None):
        # SceneFig update
        super().update(robot_state, goal, time)
        # Star obstacles
        [h.remove() for h in self.obstacle_star_handles if h is not None]
        self.obstacle_star_handles = []
        if obstacles_star is not None:
            for o in obstacles_star:
                lh, _ = o.draw(ax=self.ax, **self.obstacles_star_draw_options)
                self.obstacle_star_handles += lh

class TunnelMPCGUI(SceneGUI):

    def __init__(self, robot, obstacles, init_robot_state, goal, xlim, ylim, show_axis=False,
                 robot_color='y', robot_markersize=14, robot_alpha=0.7,
                 obstacle_color='lightgrey', obstacle_edge_color='k', show_obs_name=False,
                 goal_color='g', goal_marker='*', goal_markersize=16,
                 travelled_path_color='k', travelled_path_linestyle='-', travelled_path_linewidth=2,
                 target_path_color='g', target_path_linestyle='-', target_path_linewidth=4, target_path_marker=None,
                 mpc_path_color='k', mpc_path_linestyle=':', mpc_path_linewidth=4,
                 mpc_tunnel_color='r', mpc_tunnel_linestyle='--', mpc_tunnel_linewidth=2,
                 obstacles_star_color='r', obstacles_star_show_reference=False, obstacles_star_alpha=0.1,
                 mpc_horizon=None, rho0=None):
        self.obstacle_star_handles = []
        self.obstacles_star_draw_options = {'fc': obstacles_star_color, 'show_reference': obstacles_star_show_reference,
                                            'alpha': obstacles_star_alpha, 'zorder': 0}
        super().__init__(robot, obstacles, init_robot_state, goal, xlim, ylim, show_axis, robot_color, robot_markersize,
                         robot_alpha, obstacle_color, obstacle_edge_color, show_obs_name, goal_color, goal_marker,
                         goal_markersize, travelled_path_color, travelled_path_linestyle, travelled_path_linewidth)
        self.target_path_handle = self.ax.plot([], [], color=target_path_color, linestyle=target_path_linestyle, linewidth=target_path_linewidth, marker=target_path_marker, zorder=0)[0]
        self.mpc_path_handle = self.ax.plot([], [], color=mpc_path_color, linestyle=mpc_path_linestyle, linewidth=mpc_path_linewidth, zorder=0, dashes=(0.8, 0.8))[0]
        self.mpc_tunnel_handle = self.ax.plot([], [], color=mpc_tunnel_color, linestyle=mpc_tunnel_linestyle, linewidth=mpc_tunnel_linewidth, zorder=0)[0]

        self.fig_mpc_solution = None
        if mpc_horizon:
            self.fig_mpc_solution, self.ax_mpc_solution = plt.subplots(2, 2)
            self.s_handle = self.ax_mpc_solution[0, 0].plot(np.linspace(1, mpc_horizon, mpc_horizon), [None] * mpc_horizon, '-o')[0]
            self.skappa_handle = self.ax_mpc_solution[0, 0].plot([0, mpc_horizon], [None, None], 'r--')[0]
            self.ax_mpc_solution[0, 0].plot([0, mpc_horizon], [mpc_horizon, mpc_horizon], 'r--')
            self.ax_mpc_solution[0, 0].set_ylim(0, 1.1*mpc_horizon)
            self.rho_handle = self.ax_mpc_solution[1, 0].plot([0, mpc_horizon], [rho0, rho0], 'k--')[0]
            self.emax_handle = self.ax_mpc_solution[1, 0].plot([0, mpc_horizon], [None, None], 'r--')[0]
            self.e_handle = self.ax_mpc_solution[1, 0].plot(np.linspace(0, mpc_horizon, mpc_horizon+1, '-o'), [None] * (mpc_horizon+1), '-o')[0]
            self.ax_mpc_solution[1, 0].set_ylim(0, 1.1*rho0)
            # Assumes 2 control signals
            self.u1_handle = self.ax_mpc_solution[0, 1].plot(np.linspace(0, mpc_horizon, mpc_horizon+1), [None] * (mpc_horizon + 1), '-o')[0]
            self.ax_mpc_solution[0, 1].plot([0, mpc_horizon], [robot.u_min[0], robot.u_min[0]], 'r--')
            self.ax_mpc_solution[0, 1].plot([0, mpc_horizon], [robot.u_max[0], robot.u_max[0]], 'r--')
            self.u2_handle = self.ax_mpc_solution[1, 1].plot(np.linspace(0, mpc_horizon, mpc_horizon+1), [None] * (mpc_horizon + 1), '-o')[0]
            self.ax_mpc_solution[1, 1].plot([0, mpc_horizon], [robot.u_min[1], robot.u_min[1]], 'r--')
            self.ax_mpc_solution[1, 1].plot([0, mpc_horizon], [robot.u_max[1], robot.u_max[1]], 'r--')

        self.update(init_robot_state, goal, 0)

    def update(self, robot_state=None, goal=None, time=None, obstacles_star=None, target_path=None, mpc_path=None,
               s=None, u=None, e=None, rho=None, e_max=None, s_kappa=None, timing=None):
        # SceneFig update
        super().update(robot_state, goal, time)

        if timing:
            if time:
                self.ax.set_title("Time: {:.1f} s\nCompute Time ({:.1f} / {:.1f} / {:.1f})".format(time, timing['workspace'], timing['target'], timing['mpc']))
            else:
                self.ax.set_title("Compute Time ({:.1f} / {:.1f} / {:.1f})".format(timing['workspace'], timing['target'], timing['mpc']))

        # Star obstacles
        [h.remove() for h in self.obstacle_star_handles if h is not None]
        self.obstacle_star_handles = []
        if obstacles_star is not None:
            for o in obstacles_star:
                lh, _ = o.draw(ax=self.ax, **self.obstacles_star_draw_options)
                self.obstacle_star_handles += lh

        # Trajectories
        if mpc_path is not None:
            self.mpc_path_handle.set_data(mpc_path[::2], mpc_path[1::2])
        if target_path is not None:
            self.target_path_handle.set_data(target_path[::2], target_path[1::2])
            if e_max is not None:
                tunnel_polygon = shapely.geometry.LineString(list(zip(target_path[::2], target_path[1::2]))).buffer(
                    e_max)
                if tunnel_polygon.geom_type == 'Polygon':
                    self.mpc_tunnel_handle.set_data(*tunnel_polygon.exterior.xy)
                else:
                    print("[SceneFig]: Tunnel polygon not polygon.")
                    print(tunnel_polygon)

        # MPC solution plot
        if self.fig_mpc_solution is not None:
            if s is not None:
                self.s_handle.set_ydata(s[1:])
            if s_kappa is not None:
                self.skappa_handle.set_ydata([s_kappa, s_kappa])
            if e is not None:
                self.e_handle.set_ydata(e)
            if e_max is not None:
                self.emax_handle.set_ydata([e_max, e_max])
            if rho is not None:
                self.rho_handle.set_ydata([rho, rho])
            if u is not None:
                self.u1_handle.set_ydata(u[::2])
                self.u2_handle.set_ydata(u[1::2])
