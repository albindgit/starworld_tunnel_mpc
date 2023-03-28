from motion_control.tunnel_mpc_convergence import workspace_modification, path_generator, pol2pos, TunnelMpc
from starworlds.utils.misc import tic, toc
import numpy as np


class TunnelMpcController:
    def __init__(self, params, robot, verbosity=0):
        self.params = params
        self.params['dp_max'] = robot.vmax * self.params['dt']
        self.robot = robot
        self.mpc = TunnelMpc(params, robot)
        self.verbosity = verbosity
        self.reset()

    def reset(self):
        self.mpc.reset()
        self.obstacle_clusters = None
        self.obstacles_star = []
        self.free_star = None
        self.ri = None
        self.rg = None
        self.rs = None
        self.target_path = []
        self.path_pol = None
        self.rho = None
        self.epsilon = None
        self.solution = None
        self.u_prev = [0] * self.robot.nu
        self.timing = {'workspace': 0, 'target': 0, 'mpc': 0}

    def compute_u(self, x, pg, obstacles):
        p = self.robot.h(x)

        # Initialize rs to robot position
        if self.rs is None:
            self.rs = p
        # Update obstacles
        if not self.params['use_prev_workspace']:
            self.free_star = None
        self.obstacle_clusters, self.ri, self.rg, self.rho, self.free_star, self.timing['workspace'], exit_flag = \
            workspace_modification(obstacles, p, pg, self.rs, self.params['rho0'], self.params['max_obs_compute_time'],
                                   self.params['hull_epsilon'], self.params['gamma'],
                                   make_convex=self.params['make_convex'], previous_obstacle_clusters=self.obstacle_clusters,
                                   free_star_prev=self.free_star, verbosity=self.verbosity)
        self.obstacles_star = [o.cluster_obstacle for o in self.obstacle_clusters]

        # Buffer previous target path
        P_prev, s_prev = None, None
        if self.params['buffer'] and self.target_path:
            P_prev = np.array([self.target_path[::2], self.target_path[1::2]]).T
            # Shift path to start closest to current rs
            P_prev = P_prev[np.argmin(np.linalg.norm(self.rs - P_prev, axis=1)):, :]
            P_prev[0, :] = self.rs

            # ||self.rs - p|| < rho from construction
            for r in P_prev:
                if any([o.interior_point(r) for o in self.obstacles_star]):
                    if self.verbosity > 0:
                        print("[Path Generator]: No reuse of previous path. Path not collision-free.")
                    P_prev = None
                    break

            #     P_prev = None
            if P_prev is not None:
                # Cut off stand still padding in previous path
                P_prev_stepsize = np.linalg.norm(np.diff(P_prev, axis=0), axis=1)
                s_prev = np.hstack((0, np.cumsum(P_prev_stepsize) / self.params['dp_max']))
                P_prev_mask = [True] + (P_prev_stepsize > 1e-8).tolist()
                P_prev = P_prev[P_prev_mask, :]
                s_prev = s_prev[P_prev_mask]

        # Make sure all polygon representations are computed
        [o._compute_polygon_representation() for o in self.obstacles_star]

        # Check for convergence
        if np.linalg.norm(np.array(p) - np.array(pg)) < self.params['convergence_margin']:
            self.timing['target'] = 0
            self.timing['mpc'] = 0
            return np.zeros(self.robot.nu)

        # Update target path
        self.path_pol, self.epsilon, self.target_path, self.timing['target'] = \
            path_generator(self.ri, self.rg, self.obstacles_star, self.params['dp_max'], self.params['N'],
                           self.params['dt'], self.params['max_compute_time'], self.params['n_pol'],
                           ds_decay_rate=0.5, ds_increase_rate=2., max_nr_steps=1000, P_prev=P_prev, s_prev=s_prev,
                           reactivity=self.params['reactivity'], crep=self.params['crep'],
                           convergence_tolerance=self.params['convergence_tolerance'], verbosity=self.verbosity)

        # Compute MPC solution
        t0 = tic()
        # Set parameter for tracking error constraint
        if self.rho > 0:
            e_max = self.rho - max(self.epsilon)
        else:
            e_max = 1.e6

        self.s_kappa = self.params['lambda'] * self.rho / self.params['dp_max']
        self.solution = self.mpc.run(x.tolist(), self.u_prev, self.path_pol, self.params, e_max, self.rg.tolist(), self.s_kappa, obstacles, verbosity=self.verbosity)
        self.u_prev = self.solution['u'][:self.robot.nu]
        self.timing['mpc'] = toc(t0)

        # Update rs
        self.rs = self.mpc.ref(self.path_pol, self.solution['s'][1])

        # Extract first control signal
        return np.array(self.solution['u'][:self.mpc.build_params['nu']])

