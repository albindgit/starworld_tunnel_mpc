import yaml
import pathlib
from config.scene import Scene, scene_description
from motion_control.soads import SoadsController
from motion_control.tunnel_mpc import TunnelMpcController
from motion_control.tunnel_mpc_convergence import TunnelMpcController as TunnelMpcControllerConvergence
from robot import Unicycle, Omnidirectional, Bicycle
import numpy as np


def load_config(scene_id=None, robot_type_id=None, ctrl_param_file=None, all_convex_obstacles=False, verbosity=0):
    # Scene init
    if scene_id is None:
        print("Select scene ID\n -------------")
        for i, description in scene_description.items():
            print(str(i) + ": " + description)
        scene_id = int(input("-------------\nScene ID: "))
    scene = Scene(scene_id, all_convex=all_convex_obstacles)

    param_path = pathlib.PurePath(__file__).parents[0].joinpath('params')

    # Load robot
    robot_param_path = str(param_path.joinpath('robot_params.yaml'))
    with open(r'' + robot_param_path) as stream:
        params = yaml.safe_load(stream)
    if robot_type_id is None:
        print("Select robot ID\n -------------")
        for i, k in enumerate(params.keys()):
            print(str(i) + ": " + k)
        robot_type_id = int(input("-------------\nRobot type: "))
    robot_type = list(params.keys())[robot_type_id]
    robot_params = params[robot_type]
    if robot_params['model'] == 'Omnidirectional':
        robot = Omnidirectional(width=robot_params['width'],
                                vel_max=robot_params['vel_max'],
                                name=robot_type)
        x0 = scene.p0
    elif robot_params['model'] == 'Unicycle':
        robot = Unicycle(width=robot_params['width'], vel_min=[robot_params['lin_vel_min'], -robot_params['ang_vel_max']],
                         vel_max=[robot_params['lin_vel_max'], robot_params['ang_vel_max']],
                         name=robot_type)
        try:
            x0 = np.append(scene.p0, [scene.theta0])
        except AttributeError:
            x0 = np.append(scene.p0, [np.arctan2(scene.pg[1]-scene.p0[1], scene.pg[0]-scene.p0[0])])
    elif robot_params['model'] == 'Bicycle':
        robot = Bicycle(width=robot_params['width'],
                        vel_min=[robot_params['lin_vel_min'], -robot_params['steer_vel_max']],
                        vel_max=[robot_params['lin_vel_max'], robot_params['steer_vel_max']],
                        steer_max=robot_params['steer_max'],
                        name=robot_type)
        try:
            x0 = np.append(scene.p0, [scene.theta0, 0])
            # x0 = np.append(scene.p0, [scene.theta0])
        except AttributeError:
            x0 = np.append(scene.p0, [np.arctan2(scene.pg[1]-scene.p0[1], scene.pg[0]-scene.p0[0]), 0])
            # x0 = np.append(scene.p0, [np.arctan2(scene.pg[1]-scene.p0[1], scene.pg[0]-scene.p0[0])])
    else:
        raise Exception("[Load Config]: Invalid robot model.\n"
                        "\t\t\tSelection: {}\n"
                        "\t\t\tValid selections: [Omnidirectional, Unicycle, Bicycle]".format(robot_params['model']))

    # Load control parameters
    ctrl_param_path = str(param_path.joinpath(ctrl_param_file))
    # param_file = str(pathlib.PurePath(pathlib.Path(__file__).parent, ctrl_param_file))
    with open(r'' + ctrl_param_path) as stream:
        params = yaml.safe_load(stream)
    if 'soads' in params:
        controller = SoadsController(params['soads'], robot, verbosity)
    elif 'tunnel_mpc' in params:
        controller = TunnelMpcController(params['tunnel_mpc'], robot, verbosity)
    elif 'tunnel_mpc_convergence' in params:
        controller = TunnelMpcControllerConvergence(params['tunnel_mpc_convergence'], robot, verbosity)
    else:
        raise Exception("[Load Config]: No valid controller selection in param file.\n"
                        "\t\t\tSelection: {}\n"
                        "\t\t\tValid selections: [soads, tunnel_mpc, tunnel_mpc_convergence]".format(str(list(params.keys())[0])))

    return scene, robot, controller, x0
