import numpy as np
from starworlds.starshaped_hull import cluster_and_starify, ObstacleCluster
from starworlds.utils.misc import tic, toc
import shapely

# TODO: Check why computational time varies so much for same static scene
def workspace_modification(obstacles, p, pg, rs, rho0, max_compute_time, hull_epsilon, gamma=0.5, make_convex=0,
                            previous_obstacle_clusters=None, free_star_prev=None, verbosity=0):

    # Clearance variable initialization
    rho = rho0 / gamma  # First rho should be rho0
    t_init = tic()

    while True:
        if toc(t_init) > max_compute_time:
            if verbosity > 0:
                print("[Workspace modification]: Max compute time in rho iteration.")
            break

        # Reduce rho
        rho *= gamma

        # Pad obstacles with rho
        obstacles_rho = [o.dilated_obstacle(padding=rho, id="duplicate") for o in obstacles]

        # TODO: Fix boundaries
        free_rho = shapely.geometry.box(-20, -20, 20, 20)
        for o in obstacles_rho:
            free_rho = free_rho.difference(o.polygon())

        # TODO: Check buffering fix
        # Find P0
        Bp = shapely.geometry.Point(p).buffer(0.95 * rho)
        initial_reference_set = Bp.intersection(free_rho.buffer(-0.1 * rho))

        if not initial_reference_set.is_empty:
            break

    # Initial and goal reference position selection
    r0_sh, _ = shapely.ops.nearest_points(initial_reference_set, shapely.geometry.Point(rs))
    r0 = np.array(r0_sh.coords[0])
    rg_sh, _ = shapely.ops.nearest_points(free_rho, shapely.geometry.Point(pg))
    rg = np.array(rg_sh.coords[0])


    # TODO: Check more thoroughly
    if free_star_prev is not None:
        free_star_prev = free_star_prev.buffer(-1e-4)
    if free_star_prev is not None and free_star_prev.contains(r0_sh) and free_star_prev.contains(rg_sh) and free_rho.contains(free_star_prev):# not any([free_star_prev.covers(o.polygon()) for o in obstacles_rho]):
        if verbosity > 1:
            print("[Workspace modification]: Reuse workspace from previous time step.")
        obstacle_clusters = previous_obstacle_clusters
        exit_flag = 10
    else:
        # Apply cluster and starify
        obstacle_clusters, obstacle_timing, exit_flag, n_iter = cluster_and_starify(obstacles_rho, r0, rg,
                                                                                hull_epsilon, max_compute_time-toc(t_init),
                                                                                previous_clusters=previous_obstacle_clusters,
                                                                                    make_convex=make_convex, verbose=verbosity)

    free_star = shapely.geometry.box(-20, -20, 20, 20)
    for o in obstacle_clusters:
        free_star = free_star.difference(o.polygon())

    compute_time = toc(t_init)
    return obstacle_clusters, r0, rg, rho, free_star, compute_time, exit_flag