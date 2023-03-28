import numpy as np
import matplotlib.pyplot as plt
from config.load_config import load_config
from visualization import TunnelMPCGUI

ctrl_param_file = 'tunnel_mpc_params.yaml'
ctrl_param_file = 'tunnel_mpc_convergence_params.yaml'
scene_id = None
verbosity = 0

scene, robot, controller, x0 = load_config(ctrl_param_file=ctrl_param_file, robot_type_id=1, scene_id=scene_id, verbosity=verbosity)
gui = TunnelMPCGUI(robot, scene.obstacles, x0, scene.pg, scene.xlim, scene.ylim,
                   mpc_horizon=controller.params['N'], rho0=controller.params['rho0'], # Comment out to skip mpc solution figure
                   robot_alpha=0., mpc_path_linestyle='--', mpc_path_linewidth=4, target_path_linewidth=2,
                   goal_color='k', robot_color='orange')

# Initialize
T_max = 30
dt = controller.params['dt']
t = 0.
x = x0
u_prev = np.zeros(robot.nu)
convergence_threshold = 0.05
converged = False
timing = {'workspace': [], 'target': [], 'mpc': [], 'tot': []}

while gui.fig_open and not converged:

    if t < T_max and (not gui.paused or gui.step_once):
        gui.step_once = False
        p = robot.h(x)
        # Move obstacles
        scene.step(dt, np.array([p]), robot.width/2)
        # Compute mpc
        u = controller.compute_u(x, scene.pg, scene.obstacles)

        # Update timing
        tot_time = 0
        for k, v in controller.timing.items():
            timing[k] += [v]
            tot_time += v
        timing['tot'] += [tot_time]

        # Update plots
        mpc_path = [el for x in controller.solution['x'] for el in robot.h(x)]
        e_max = controller.rho - max(controller.epsilon)
        s_kappa = controller.s_kappa if hasattr(controller,'s_kappa') else None
        gui.update(robot_state=x, obstacles_star=controller.obstacles_star, time=t, target_path=controller.target_path,
                   mpc_path=mpc_path, e_max=e_max, timing=controller.timing, e=controller.solution['e'],
                   s=controller.solution['s'], u=u_prev.tolist() + controller.solution['u'], rho=controller.rho, s_kappa=s_kappa)

        # Integrate robot state with new control signal
        x, _ = robot.move(x, u, dt)
        u_prev = u
        t += dt

    converged = np.linalg.norm(robot.h(x)-scene.pg) < convergence_threshold

    if t >= T_max or converged:
        gui.ax.set_title("Time: {:.1f} s. Finished".format(t))
        gui.fig.canvas.draw()

    plt.pause(0.005)

gui.update(robot_state=x, obstacles_star=controller.obstacles_star)

# fig, ax = plt.subplots()
# ax.plot(timing['workspace'])
# ax.plot(timing['target'])
# ax.plot(timing['mpc'])
# ax.plot(timing['tot'])
# ax.legend(['Workspace modification', 'Target path generation', 'MPC', 'Total'])

# Wait until figure closed when converged
plt.show()
