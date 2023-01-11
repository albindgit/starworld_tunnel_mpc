import yaml
import pathlib
from config.scene import Scene, scene_description
from motion_control.soads import SoadsController
from robot import Unicycle, Omnidirectional, Bicycle
import numpy as np


def load_config(scene_id=None, robot_type_id=None, ctrl_param_file=None, all_convex_obstacles=False):
    # Scene init
    if scene_id is None:
        print("Select scene ID\n -------------")
        for i, description in scene_description.items():
            print(str(i) + ": " + description)
        scene_id = int(input("-------------\nScene ID: "))
    scene = Scene(scene_id, all_convex=all_convex_obstacles)

    # Load robot
    param_file = str(pathlib.PurePath(pathlib.Path(__file__).parent, 'robot_params.yaml'))
    with open(r'' + param_file) as stream:
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
        raise Exception('Invalid robot model.')

    # Load control parameters
    if ctrl_param_file is None:
        ctrl_param_file = 'ctrl_params.yaml'
    param_file = str(pathlib.PurePath(pathlib.Path(__file__).parent, ctrl_param_file))
    with open(r'' + param_file) as stream:
        params = yaml.safe_load(stream)

    if 'soads' in params:
        # SoadsController
        controller = SoadsController(params['soads'], robot)
    # else:
    #     # MPC controller
    #     mpc_params = params['mpc']
    #     target_generator_params = params['target_generator_params']
    #     controller = MotionController(mpc_params, target_generator_params, robot)

    return scene, robot, controller, x0
    # # Initialize simulation
    # N = 100
    # dt = params['mpc']['dt']

    # q = np.zeros((N, robot.nq))
    # q[0, :2] = [scene.p0[0], scene.p0[1]]
    # qg = scene.pg
    # if robot.nq > 2:
    #     q[0, 2] = np.arctan2(scene.pg[1]-scene.p0[1], scene.pg[0]-scene.p0[0])
    #     qg = np.append(qg, [np.arctan2(scene.p0[1]-scene.pg[1], scene.p0[0]-scene.pg[0])])
    # if robot.nq == 4:
    #     qg = np.append(qg, 0)
    # MPC
    # controller = Controller(mpc_params, target_generator_params, robot)