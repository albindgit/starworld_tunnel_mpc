import numpy as np
import time
import shapely
from starworlds.utils.misc import tic, toc
from motion_control.soads.soads import f as soads_f


def pol2pos(path_pol, s):
    n_pol = len(path_pol) // 2 - 1
    return [sum([path_pol[j * (n_pol + 1) + i] * s ** (n_pol - i) for i in range(n_pol + 1)]) for j in range(2)]


def path_generator(r0, rg, obstacles, dp_max, N, dt, max_compute_time, n_pol, ds_decay_rate=0.5,
                   ds_increase_rate=2., max_nr_steps=1000, convergence_tolerance=1e-5, P_prev=None, s_prev=None,
                   reactivity=1., crep=1., tail_effect=False, reference_step_size=0.5, verbosity=0):
    t0 = tic()

    # Initialize
    ds = 1
    s = np.zeros(max_nr_steps)
    r = np.zeros((max_nr_steps, r0.size))
    if P_prev is not None:
        i = P_prev.shape[0]
        r[:i, :] = P_prev
        s[:i] = s_prev
    else:
        i = 1
        r[0, :] = r0

    while True:
        dist_to_goal = np.linalg.norm(r[i - 1, :] - rg)
        # Check exit conditions
        if dist_to_goal < convergence_tolerance:
            if verbosity > 1:
                print("[Path Generator]: Path converged. " + str(
                    int(100 * (s[i - 1] / N))) + "% of path completed.")
            break
        if s[i - 1] >= N:
            if verbosity > 1:
                print("[Path Generator]: Completed path length. " + str(
                    int(100 * (s[i - 1] / N))) + "% of path completed.")
            break
        if toc(t0) > max_compute_time:
            if verbosity > 1:
                print("[Path Generator]: Max compute time in path integrator. " + str(
                    int(100 * (s[i - 1] / N))) + "% of path completed.")
            break
        if i >= max_nr_steps:
            if verbosity > 1:
                print("[Path Generator]: Max steps taken in path integrator. " + str(
                    int(100 * (s[i - 1] / N))) + "% of path completed.")
            break

        # Movement using SOADS dynamics
        dr = min(dp_max, dist_to_goal) * soads_f(r[i - 1, :], rg, obstacles, adapt_obstacle_velocity=False,
                                                 unit_magnitude=True, crep=crep,
                                                 reactivity=reactivity, tail_effect=tail_effect,
                                                 convergence_tolerance=convergence_tolerance)

        r[i, :] = r[i - 1, :] + dr * ds

        ri_in_obstacle = False
        while any([o.interior_point(r[i, :]) for o in obstacles]):
            if verbosity > 1:
                print("[Path Generator]: Path inside obstacle. Reducing integration step from {:5f} to {:5f}.".format(ds, ds*ds_decay_rate))
            ds *= ds_decay_rate
            r[i, :] = r[i - 1, :] + dr * ds
            # Additional compute time check
            if toc(t0) > max_compute_time:
                ri_in_obstacle = True
                break
        if ri_in_obstacle:
            continue

        # Update travelled distance
        s[i] = s[i - 1] + ds
        # Try to increase step rate again
        ds = min(ds_increase_rate * ds, 1)
        # Increase iteration counter
        i += 1

    r = r[:i, :]
    s = s[:i]

    # Evenly spaced path
    s_vec = np.arange(0, s[-1] + reference_step_size, reference_step_size)
    xs, ys = np.interp(s_vec, s, r[:, 0]), np.interp(s_vec, s, r[:, 1])
    # Append not finished path with fixed final position
    s_vec = np.append(s_vec, np.arange(s[-1] + reference_step_size, N + reference_step_size, reference_step_size))
    xs = np.append(xs, xs[-1] * np.ones(len(s_vec)-len(xs)))
    ys = np.append(ys, ys[-1] * np.ones(len(s_vec)-len(ys)))

    reference_path = [el for p in zip(xs, ys) for el in p]

    # TODO: Fix when close to goal
    # TODO: Adjust for short arc length, skip higher order terms..
    path_pol = np.polyfit(s_vec, reference_path[::2], n_pol).tolist() + \
               np.polyfit(s_vec, reference_path[1::2], n_pol).tolist()
    # Force init position to be correct
    path_pol[n_pol] = reference_path[0]
    path_pol[-1] = reference_path[1]

    # Compute polyfit approximation error
    epsilon = [np.linalg.norm(np.array(reference_path[2 * i:2 * (i + 1)]) - np.array(pol2pos(path_pol, s_vec[i]))) for i in
               range(N + 1)]

    compute_time = toc(t0)
    return path_pol, epsilon, reference_path, compute_time
