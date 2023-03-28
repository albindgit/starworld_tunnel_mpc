import casadi.casadi as casadi
import opengen as og
import numpy as np
import os, sys
import yaml


class NoSolutionError(Exception):
    '''raise this when there's no solution to the mpc problem'''
    pass


class TunnelMpc:

    def __init__(self, params, robot):
        self.build_dir = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'mpc_build')
        # Load parameters
        self.build_params = None
        self.robot = robot
        self.set_build_params(params)
        rebuild, self.build_name = self.get_build_version()
        # Build if different from latest build
        if rebuild:
            self.build()
        else:
            print("Found MPC build: {}".format(self.build_name))

        sys.path.insert(1, os.path.join(self.build_dir, self.build_name))
        optimizer = __import__(self.build_name)
        self.solver = optimizer.solver()
        self.reset()

    def reset(self):
        self.sol_prev = None

    def set_build_params(self, params):
        self.build_params = {
            'mpc_ver': 5,
            'mode': params['build_mode'],
            'N': params['N'],
            'dt': params['dt'],
            'solver_tol': params['solver_tol'],
            'solver_max_time': params['solver_max_time'],
            'solver_max_inner_iterations': params['solver_max_inner_iterations'],
            'solver_max_outer_iterations': params['solver_max_outer_iterations'],
            'x_min': self.robot.x_min,
            'x_max': self.robot.x_max,
            'u_min': self.robot.u_min,
            'u_max': self.robot.u_max,
            'nx': self.robot.nx,
            'nu': self.robot.nu,
            'np': 2,
            'robot_model': self.robot.__class__.__name__,
            'n_pol': params['n_pol'],
            'integration_method': params['integration_method']
        }

    def get_build_version(self):
        builds = [name for name in os.listdir(self.build_dir)
                  if os.path.isdir(os.path.join(self.build_dir, name))]
        for build_ver in builds:
            if not os.path.isfile(os.path.join(self.build_dir, build_ver, 'build_params.yaml')):
                continue
            with open(os.path.join(self.build_dir, build_ver, 'build_params.yaml'), 'r') as file:
                build_ver_params = yaml.load(file, Loader=yaml.FullLoader)
            if self.build_params == build_ver_params:
                return False, build_ver
        ver = 0
        while 'ver' + str(ver) in os.listdir(self.build_dir):
            ver += 1
        return True, 'ver' + str(ver)


    def e_cost(self, e, ce):
        return ce * e

    def ud_cost(self, u, u_prev, R):
        ud = [(u[i] - u_prev[i]) for i in range(self.robot.nu)]
        return R[0] * ud[0] ** 2 + R[1] * ud[1] ** 2

    def ref(self, path_pol, s):
        return [sum([path_pol[j*(self.build_params['n_pol']+1) + i] * s**(self.build_params['n_pol']-i) for i in range(self.build_params['n_pol']+1)])
                for j in range(self.build_params['np'])]

    def fz(self, z, mu):
        return self.robot.f(z[:-1], mu[:-1]) + [mu[-1] / self.build_params['dt']]

    def fz_d(self, z, mu):
        nz = self.build_params['nx'] + 1
        if self.build_params['integration_method'] == 'euler':
            zd = self.fz(z, mu)
            return [z[i] + zd[i] * self.build_params['dt'] for i in range(nz)]
        # if self.build_params['integration_method'] == 'RK4':
        #     k1 = self.fz(z, u, v, path_pol)
        #     k2 = self.fz([z[i] + self.build_params['dt'] / 2 * k1[i] for i in range(nz)], u, v, path_pol)
        #     k3 = self.fz([z[i] + self.build_params['dt'] / 2 * k2[i] for i in range(nz)], u, v, path_pol)
        #     k4 = self.fz([z[i] + self.build_params['dt'] * k3[i] for i in range(nz)], u, v, path_pol)
        #     return [z[i] + (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]) * self.build_params['dt'] / 6 for i in range(nz)]
        raise NotImplemented

    def one_step_feasible(self, sol, x0, path_pol, e_max, s_kappa, obstacles, d=False):
        u, ds, x, s, e = self.unpack_solution(sol, x0, path_pol)
        sol_ok = True
        eps = 1e-3
        # x constraints
        if not all([self.build_params['x_min'][i] - eps <= x[0][i] <= self.build_params['x_max'][i] + eps for i in
                    range(self.build_params['nx'])]):
            sol_ok = False
            if d:
                print("[MPC]: Bad x")
                print(x[0])
        # s constraints
        if not (0 - eps <= s[1] <= self.build_params['N'] + eps):
            sol_ok = False
            if d:
                print("[MPC]: Bad s")
                print(s[1])
        # u constraints
        if not all([self.build_params['u_min'][i] - eps <= u[i] <= self.build_params['u_max'][i] + eps for i in
                    range(self.build_params['nu'])]):
            sol_ok = False
            if d:
                print("[MPC]: Bad u")
                print(u[0])
        # e constraints
        if not (e[1] <= e_max + eps):
            sol_ok = False
            if d:
                print("[MPC]: Bad e ({:.4f} > {:.4f})".format(e[1], e_max))
            p = x[0][:self.build_params['np']]
            if not any([o.interior_point(p) for o in obstacles]):
                sol_ok = True
                if d:
                    print("[MPC]: Position collision free. OK solution.")
        # ds constraints
        if not (s_kappa - eps <= ds[0] <= 1 + eps):
            sol_ok = False
            if d:
                print("[MPC]: Bad ds ({:.4f} < {:.4f})".format(ds[0], s_kappa))

        return sol_ok

    def is_feasible(self, sol, x0, path_pol, e_max, s_kappa, d=False):
        u, ds, x, s, e = self.unpack_solution(sol, x0, path_pol)
        sol_ok = True

        eps = 1e-2
        # x constraints
        if not all([self.build_params['x_min'][i]-eps <= xk[i] <= self.build_params['x_max'][i]+eps for i in range(self.build_params['nx']) for xk in x]):
            sol_ok = False
            if d:
                print("[MPC]: Bad x")
                print(x)
        # s constraints
        if not all([0-eps <= sk <= self.build_params['N'] + eps for sk in s]):
            sol_ok = False
            if d:
                print("[MPC]: Bad s")
                print(s)
        # u constraints
        if not all([self.build_params['u_min'][i]-eps <= uk <= self.build_params['u_max'][i]+eps for i in range(self.build_params['nu']) for uk in u[i::self.build_params['nu']]]):
            sol_ok = False
            if d:
                print("[MPC]: Bad u")
                print(u)
        # v constraints
        if not all([0-eps <= dsk <= np.inf * self.build_params['dt'] + eps for dsk in ds] or ds[0]) < s_kappa:
            sol_ok = False
            if d:
                print("[MPC]: Bad ds")
                print(ds)
                print(s_kappa)
        # e constraints
        if not all([ek <= e_max + eps for ek in e]):
            sol_ok = False
            if d:
                print("[MPC]: Bad e")
                print(e)

        # All constraints ok
        return sol_ok

    def e_sqr(self, z, path_pol):
        ref = self.ref(path_pol, z[-1])
        p = self.robot.h(z[:-1])
        e_sqr = sum([(ref[i] - p[i])**2 for i in range(self.build_params['np'])])
        return e_sqr

    def unpack_solution(self, sol, x0, path_pol):
        u = sol[:self.build_params['nu'] * self.build_params['N']]
        ds = sol[self.build_params['nu'] * self.build_params['N']:]
        zk = x0 + [0]
        x = [zk[:-1]]
        s = [zk[-1]]
        e = [self.e_sqr(zk, path_pol)]
        for k in range(self.build_params['N']):  # LOOP OVER TIME STEPS
            # Dynamics
            muk = u[self.build_params['nu'] * k:self.build_params['nu'] * (k + 1)] + [ds[k]]
            zk = self.fz_d(zk, muk)
            x += [zk[:-1]]
            s += [zk[-1]]
            e += [np.sqrt(self.e_sqr(zk, path_pol))]
        return u, ds, x, s, e

    def sol2cost(self, sol, x0, path_pol, u_prev, params):
        u, ds, x, s, e = self.unpack_solution(sol, x0, path_pol)
        u = u_prev + u
        v_cost, ud_cost, omega_cost = [0.] * 3
        for i in range(len(ds)):
            ui_prev = u[self.build_params['nu'] * i:self.build_params['nu'] * (i + 1)]
            ui = u[self.build_params['nu'] * (i+1):self.build_params['nu'] * (i + 2)]
            ud_cost += self.ud_cost(ui, ui_prev, params['R'])
            v_cost += self.v_cost(ds[i], params['cv'], params['lambda'], i)
        e_end_cost = self.e_cost(e[-1]**2, params['ce_end'])
        return v_cost, ud_cost, e_end_cost, omega_cost

    def build(self):
        # Build parametric optimizer
        # ------------------------------------
        params = {'x0': self.build_params['nx'], 'u_prev': self.build_params['nu'], 'path_pol': self.build_params['np'] * (self.build_params['n_pol']+1),
                  'cs': 1, 'ce': 1, 'R': self.build_params['nu'], 'e_max': 1, 'rg': self.build_params['np'], 'e_penalty': 1, 'cg': 1, 's_kappa': 1}
        par_dim = sum(list(params.values()))


        # Exchange parameter dimension with value
        par = casadi.SX.sym("par", par_dim)  # Parameters
        p_idx = 0
        for key, dim in params.items():
            params[key] = par[p_idx:p_idx + dim]
            p_idx += dim

        # Init value
        uk_prev = params['u_prev']
        zk = casadi.vertcat(params['x0'], 0)
        cost = 0
        # Decision variable
        mu = casadi.SX.sym('uds', (self.build_params['nu'] + 1) * self.build_params['N'])
        # Constraint variables
        f1, f1_min, f1_max = [], [], []

        # Loop over time steps
        for k in range(self.build_params['N']):
            # Decision variables
            uk = mu[k * self.build_params['nu']:(k + 1) * self.build_params['nu']]
            dsk = mu[self.build_params['nu'] * self.build_params['N'] + k]
            muk = casadi.vertcat(uk, dsk)

            # Step forward
            zk1 = casadi.vertcat(*self.fz_d(zk, muk))
            ek1_sqr = self.e_sqr(zk1, params['path_pol'])

            # Add stage cost
            cost += params['e_penalty'] * ek1_sqr
            cost += self.ud_cost(uk, uk_prev, params['R'])

            # Add state and error constraint
            f1 = casadi.vertcat(f1, zk1, ek1_sqr - params['e_max']**2)
            f1_min += self.build_params['x_min'] + [0] + [-1e10]
            f1_max += self.build_params['x_max'] + [self.build_params['N']] + [0]

            # Initial reference increment constraint
            if k == 0:
                f1 = casadi.vertcat(f1, dsk - params['s_kappa'])
                f1_min += [0]
                f1_max += [1e10]

            uk_prev = uk
            zk = zk1


        ## Terminal cost
        # Cost on last tracking error
        cost += self.e_cost(ek1_sqr, params['ce'])
        cost += - params['cs'] * zk1[-1]
        # Convergence cost
        p_end = self.robot.h(zk[:-1])
        xg_err = sum([(p_end[i] - params['rg'][i])**2 for i in range(self.build_params['np'])])
        cost += params['cg'] * xg_err
        # Final orientation cost
        # cost += params['cg_orientation'] * (1 / (xg_err + 1e-3) ) * (zk[2]-params['rg'][2])**2

        # Constraint set
        set_c = og.constraints.Rectangle(f1_min, f1_max)
        # Decision variable constraints
        mu_bounds = og.constraints.Rectangle(self.build_params['u_min'] * self.build_params['N'] +
                                               [0.] * self.build_params['N'],
                                               self.build_params['u_max'] * self.build_params['N'] +
                                               [1] * self.build_params['N'])

        # Setup builder
        problem = og.builder.Problem(mu, par, cost) \
            .with_constraints(mu_bounds) \
            .with_aug_lagrangian_constraints(f1, set_c)
        build_config = og.config.BuildConfiguration() \
            .with_build_directory(self.build_dir) \
            .with_build_mode(self.build_params['mode']) \
            .with_build_python_bindings()
        meta = og.config.OptimizerMeta() \
            .with_optimizer_name(self.build_name)
        solver_config = og.config.SolverConfiguration() \
            .with_tolerance(self.build_params['solver_tol']) \
            .with_max_duration_micros(self.build_params['solver_max_time'] * 1000) \
            .with_max_inner_iterations(self.build_params['solver_max_inner_iterations']) \
            .with_max_outer_iterations(self.build_params['solver_max_outer_iterations'])
        builder = og.builder.OpEnOptimizerBuilder(problem,
                                                  meta,
                                                  build_config,
                                                  solver_config) \
            .with_verbosity_level(1)
        builder.build()
        print('')

        # Save build params
        with open(os.path.join(self.build_dir, self.build_name, 'build_params.yaml'), 'w') as file:
            yaml.dump(self.build_params, file)

    def trivial_solution(self, x, path_pol, e_max, s_kappa):
        p_ref = self.ref(path_pol, s_kappa)

        u = [0] * (self.build_params['N'] * self.build_params['nu'])
        ds = [0] * (self.build_params['N'])
        xk = x
        sk = 0
        zk = xk + [sk]
        for k in range(self.build_params['N']):  # LOOP OVER TIME STEPS
            # Control law
            if self.build_params['robot_model'] == 'Unicycle':
                ref_dist = np.linalg.norm(np.array(xk[:2])-np.array(p_ref))
                if ref_dist > 1e-5:
                    theta_ref = np.arctan2(p_ref[1]-xk[1], p_ref[0]-xk[0])
                    theta_diff = theta_ref - xk[-1]
                    if theta_diff > np.pi:
                        theta_diff -= 2 * np.pi
                    if theta_diff < -np.pi:
                        theta_diff += 2 * np.pi
                    if abs(theta_diff) < 1e-2: # Less than 0.57 degree error
                        # Linear velocity
                        u[k*self.build_params['nu']] = min(self.build_params['u_max'][0], ref_dist / self.build_params['dt'])
                    else:
                        # Angular velocity
                        if theta_diff > 0:
                            u[k * self.build_params['nu'] + 1] = min(self.build_params['u_max'][1], theta_diff / self.build_params['dt'])
                        else:
                            u[k * self.build_params['nu'] + 1] = max(self.build_params['u_min'][1], theta_diff / self.build_params['dt'])

            # Dynamics
            muk = u[k * self.build_params['nu']:k * self.build_params['nu'] + 2] + [0]
            zk = self.fz_d(zk, muk)
            xk = zk[:-1]

        return u + ds

    def run(self, x, u_prev, path_pol, params, e_max, rg, s_kappa, obstacles, verbosity=0):
        robot_within_dg = np.linalg.norm(np.array(self.robot.h(rg))-np.array(self.robot.h(x))) < params['dg']
        cg = params['cg'] if robot_within_dg else 0

        p = x + u_prev + path_pol + [params['cs'], params['ce']] + params['R'] + \
            [0.95*e_max] + rg + [params['e_penalty'], cg, s_kappa]

        if self.sol_prev is None or not self.is_feasible(self.sol_prev, x, path_pol, 2*e_max, s_kappa):
            # Use trivial solution as initial guess
            self.sol_prev = self.trivial_solution(x, path_pol, e_max, s_kappa)
        # Run solver
        solution_data = self.solver.run(p=p, initial_guess=self.sol_prev)

        # TODO: When not one step feasible, check for actual collision with mpc before using trivial solution.
        if solution_data is None or not self.one_step_feasible(solution_data.solution, x, path_pol, e_max, s_kappa, obstacles, d=verbosity>0):
            if verbosity > 0:
                print("[MPC]: Not feasible solution. Using trivial solution.")
            sol = self.trivial_solution(x, path_pol, e_max, s_kappa)
            self.sol_prev = None
            exit_status = "Trivial solution"
            cost = -1
        else:
            sol = solution_data.solution
            exit_status = solution_data.exit_status
            cost = solution_data.cost

            self.sol_prev = sol[self.build_params['nu']:self.build_params['nu'] * self.build_params['N']] + \
                            sol[self.build_params['nu'] * (self.build_params['N']-1):self.build_params['nu'] * self.build_params['N']] \
                            + sol[self.build_params['nu'] * self.build_params['N']:]

        # Unpack solution
        u, ds, x_pred, s_pred, e_pred = self.unpack_solution(sol, x, path_pol)

        return {'u': u, 'x': x_pred, 'e': e_pred, 's': s_pred, 'cost': cost, 'exit_status': exit_status}