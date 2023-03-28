import numpy as np
from starworlds.obstacles import Ellipse, StarshapedPolygon, motion_model, Polygon
import matplotlib.pyplot as plt

scene_description = {
    1: "No obstacles.",
    2: "1 static ellipse.",
    3: "2 polygons. One static, one moving slowly towards, and into, the other.",
    4: "Intersecting static ellipse and polygon. Ellipse moving towards polygon.",
    5: "6 moving ellipses.",
    6: "3 static polygons and 3 moving ellipses.",
    7: "1 static polygon and 2 static ellipses.",
    8: "3 static ellipses.",
    9: "Corridor with moving ellipse into corridor.",
    10: "Crowd.",
    11: "Boundary.",
    12: "Start stop in corridor. Ellipse obstacle."
}


class Scene:

    def __init__(self, id=None, all_convex=False, scene_setup=None):
        self.id = id
        self.obstacles = None
        self.p0 = None
        self.pg = None
        self.xlim = None
        self.ylim = None
        self._obstacles_to_plot = None

        if id is None:
            self.obstacles = scene_setup['obstacles']
            self.p0 = scene_setup['p0']
            self.pg = scene_setup['pg']
            self.xlim = scene_setup['xlim']
            self.ylim = scene_setup['ylim']
            self._obstacles_to_plot = self.obstacles
        elif id is not None:
            self.init_scene_id(id, all_convex)
        else:
            print("Bad scene initialization")

    def init_scene_id(self, id, all_convex):
        scene_id = 0
        # Scene 1
        scene_id += 1
        if id == scene_id:
            self.obstacles = [
            ]
            self.p0 = np.array([-5., 0.])
            self.pg = np.array([5., 0.])
            self.xlim = [-10, 10]
            self.ylim = [-10, 10]

        # Scene 2
        scene_id += 1
        if id == scene_id:
            self.obstacles = [
                Ellipse([1., 1.], motion_model=motion_model.Static(pos=[2., 0.])),
            ]
            self.p0 = np.array([0., -2.])
            self.pg = np.array([2., 2.])
            self.xlim = [-1, 5]
            self.ylim = [-3, 3]

        # Scene 3
        scene_id += 1
        if id == scene_id:
            self.obstacles = [
                # Polygon([[3, 0], [3, -4], [5, -4], [5, -2], [8, -2], [8, -4], [10, -4], [10, 0]]),
                StarshapedPolygon([[3, 0], [3, -4], [5, -4], [5, -2], [8, -2], [8, 0]]),
                StarshapedPolygon([[8, 0], [8, -4], [10, -4], [10, 0]]),
                StarshapedPolygon([[6, -3], [6, -7], [7, -7], [7, -3]], motion_model=motion_model.SinusVelocity(pos=[0, -1], x2_mag=0.2))
            ]
            self.p0 = np.array([0, -4])
            self.pg = np.array([11, -2])
            self.xlim = [-1, 13]
            self.ylim = [-8, 2]

        # Scene 4
        scene_id += 1
        if id == scene_id:
            self.obstacles = [
                StarshapedPolygon([[4, 0], [8, 0], [8, 2], [6, 2], [6, 6], [4, 6]]),
                Ellipse([1, 1], motion_model=motion_model.Static(pos=[4, -1])),
                Ellipse([1, 1], motion_model=motion_model.Interval([8, -3], [(5, [8, -1])])),
                # StarshapedPolygon([[1, -1], [1.5, -1], [1.5, 16], [1, 16]]),
                # StarshapedPolygon([[2, -1], [3, -1], [3, 16], [2, 16]]),
            ]
            self.p0 = np.array([3., -4.])
            # self.p0 = np.array([1.75, 0.5])
            # self.theta0 = np.pi/2
            self.pg = np.array([8., 6.])
            self.xlim = [0, 10]
            self.ylim = [-5, 8]

        # Scene 5
        scene_id += 1
        if id == scene_id:
            self.obstacles = [
                Ellipse([1, 1], motion_model=motion_model.SinusVelocity(pos=[6, -5], x1_mag=-0.2)),
                Ellipse([1, 1], motion_model=motion_model.SinusVelocity(pos=[4, -3], x1_mag=-0.5)),
                Ellipse([1, 1], motion_model=motion_model.SinusVelocity(pos=[7, -1], x1_mag=-0.2)),
                Ellipse([1, 1], motion_model=motion_model.SinusVelocity(pos=[5, -2], x1_mag=-0.25, x2_mag=-0.2)),
                Ellipse([1, 1], motion_model=motion_model.SinusVelocity(pos=[1, -5], x1_mag=0.3, x2_mag=0.5)),
                Ellipse([0.4, 1], motion_model=motion_model.SinusVelocity(pos=[8, -3], x2_mag=2, x2_period=5)),
                # Ellipse([2, 1], motion_model=motion_model.SinusVelocity(pos=[7, -2], x1_mag=2, x1_period=3)),
                Ellipse([0.4,0.4], motion_model=motion_model.SinusVelocity(pos=[8, -5], x1_mag=0.3, x2_mag=0.5)),
            ]
            self.p0 = np.array([0., -3.])
            self.pg = np.array([11., -3.])
            self.xlim = [-1, 12]
            self.ylim = [-8, 4]

            # n_pol = 10
            # t = np.linspace(0, 2 * np.pi, n_pol, endpoint=False)
            # ell_polygon = np.vstack((1 * np.cos(t), 1 * np.sin(t))).T
            # ell_polygon2 = np.vstack((0.4 * np.cos(t), 1 * np.sin(t))).T
            # ell_polygon3 = np.vstack((0.4 * np.cos(t), 0.4 * np.sin(t))).T
            # self.obstacles += [
            #     StarshapedPolygon(ell_polygon, is_convex=True, motion_model=motion_model.SinusVelocity(pos=[6, -5], x1_mag=-0.2)),
            #     StarshapedPolygon(ell_polygon, is_convex=True, motion_model=motion_model.SinusVelocity(pos=[4, -3], x1_mag=-0.5)),
            #     StarshapedPolygon(ell_polygon, is_convex=True, motion_model=motion_model.SinusVelocity(pos=[7, -1], x1_mag=-0.2)),
            #     StarshapedPolygon(ell_polygon, is_convex=True, motion_model=motion_model.SinusVelocity(pos=[5, -2], x1_mag=-0.25, x2_mag=-0.2)),
            #     StarshapedPolygon(ell_polygon, is_convex=True, motion_model=motion_model.SinusVelocity(pos=[1, -5], x1_mag=0.3, x2_mag=0.5)),
            #     StarshapedPolygon(ell_polygon2, is_convex=True, motion_model=motion_model.SinusVelocity(pos=[8, -3], x2_mag=2, x2_period=5)),
            #     StarshapedPolygon(ell_polygon3, is_convex=True, motion_model=motion_model.SinusVelocity(pos=[8, -5], x1_mag=0.3, x2_mag=0.5)),
            #     ]

        # Scene 6
        scene_id += 1
        if id == scene_id:
            self.obstacles = [
                # Polygon([(8, 0), (9, 0), (9, 5), (7, 5), (7, 7), (6, 7), (6, 3), (8, 3)], is_convex=False),
                StarshapedPolygon([(7, 3), (7, 7), (6, 7), (6, 3)], is_convex=True),
                StarshapedPolygon([(11, 4), (7, 4), (7, 3), (11, 3)], is_convex=True),
                StarshapedPolygon([(11, 1), (12, 1), (12, 5), (11, 5)], is_convex=True),
                # StarshapedPolygon([(8, 5), (7, 5), (7, 3), (8, 3)], is_convex=True),
                # StarshapedPolygon([(8, 0), (9, 0), (9, 5), (8, 5)], is_convex=True),
                StarshapedPolygon([(2, 6), (3, 6), (3, 10), (2, 10)], is_convex=True),
                # StarshapedPolygon([(6, 11), (2, 11), (2, 10), (6, 10)], is_convex=True),
                # Ellipse([1, 1], motion_model=motion_model.Waypoints([12, 3], [(9.8, 5.5), (9.8, 9.5), (7, 9.5), (3, 4)], 1.)),
                Ellipse([1, 1], motion_model=motion_model.Waypoints([3, 5.5], [(4, 5.5), (7.3, 9.5), (10, 9.5), (10.3, 4.5)], 1.)),
                Ellipse([1, 1], motion_model=motion_model.Interval([5, 8], [(3, (7.6, 7.2))])),
                # Ellipse([1, 1], motion_model=motion_model.Interval([9, 10], [(5, (4.5, 10)), (5, (4.5, 4))]))
                # Ellipse([1, 1], motion_model=motion_model.Interval([3, 4], [(5, (4., 5)), (5, (4., 9)), (4, (9, 9))]))
                Ellipse([1, 1], motion_model=motion_model.Interval([3.3, 10], [(6, (3.3, 6)), (6, (5.5, 6))]))
            ]
            self.p0 = np.array([13, 6.])
            self.pg = np.array([1., 5.])
            # self.pg = np.array([1., 5.])
            self.xlim = [0, 13]
            self.ylim = [-1, 11]

        # Scene 7
        scene_id += 1
        if id == scene_id:
            ell_ax = [0.8, 0.8]
            self.obstacles = [
                StarshapedPolygon([(8, 5), (7, 5), (7, 3), (8, 3)], is_convex=True),
                StarshapedPolygon([(7, 3), (7, 7), (6, 7), (6, 3)], is_convex=True),
                StarshapedPolygon([(8, 0), (9, 0), (9, 5), (8, 5)], is_convex=True),
                StarshapedPolygon([(3, 6), (4, 6), (4, 10), (3, 10)], is_convex=True),
                StarshapedPolygon([(9.5, 6), (10.5, 6), (10.5, 10), (9.5, 10)], is_convex=True),
                # Ellipse(ell_ax, motion_model=motion_model.Waypoints([12, 3], [(9.8, 5.5), (9.8, 9.5), (7, 9.5)], 0.8)),
                Ellipse(ell_ax, motion_model=motion_model.Static([9, 9])),
                Ellipse(ell_ax, motion_model=motion_model.Interval([12, 5], [(2, (11, 6))])),
                Ellipse(ell_ax, motion_model=motion_model.Interval([5, 8], [(3, (7.8, 7))])),
                # Ellipse(ell_ax, motion_model=motion_model.Interval([3, 4], [(5, (5, 4)), (5, (4.5, 9))])),
                Ellipse(ell_ax, motion_model=motion_model.Interval([2, 3], [(3, (4., 6))])),
                Ellipse(ell_ax, motion_model=motion_model.SinusVelocity([4.7, 12], x2_mag=-0.2)),
            ]

            # n_pol = 10
            # t = np.linspace(0, 2 * np.pi, n_pol, endpoint=False)
            # ell_polygon = np.vstack((1 * np.cos(t), 1 * np.sin(t))).T
            # self.obstacles += [StarshapedPolygon(ell_polygon, is_convex=True, motion_model=motion_model.Waypoints([12, 3], [(9.8, 5.5), (9.8, 9.5), (7, 9.5), (3, 4)], 1.))]
            # self.obstacles += [StarshapedPolygon(ell_polygon, is_convex=True, motion_model=motion_model.Interval([5, 8], [(3, (7.8, 7))]))]
            # self.obstacles += [StarshapedPolygon(ell_polygon, is_convex=True, motion_model=motion_model.Interval([3, 4], [(5, (5, 4)), (5, (4.5, 9)), (4, (9, 9))]))]

            self.p0 = np.array([12., 7.])
            self.pg = np.array([1., 4.])
            self.xlim = [0, 13]
            self.ylim = [-1, 11]
            # self.obstacles = [
            #     StarshapedPolygon([(3, 7), (6, 7), (6, 5), (7, 5), (7, 8), (3, 8)]),
            #     Ellipse([2, 0.5], motion_model=motion_model.Static([5, 4], -np.pi/3)),
            #     Ellipse([2, 1], motion_model=motion_model.Static([9, 4], np.pi/2))
            # ]
            # self.p0 = np.array([1, 6])
            # self.pg = np.array([11, 6])
            # self.xlim = [0, 12]
            # self.ylim = [0, 10]

        # Scene 8
        scene_id += 1
        if id == scene_id:
            self.obstacles = [
                Ellipse([2, 1], motion_model=motion_model.Static([5, 6], -np.pi/3)),
                Ellipse([1.2, 1.2], motion_model=motion_model.Static([4, 4], 0)),
                Ellipse([1, 1], motion_model=motion_model.Static([4, 8], 0)),
                Ellipse([1, 1], motion_model=motion_model.Static([9, 4], 0))
            ]
            self.p0 = np.array([1, 6])
            self.pg = np.array([11, 6])
            self.xlim = [0, 12]
            self.ylim = [0, 10]

        # Scene 9
        scene_id += 1
        if id == scene_id:
            self.obstacles = [
                # StarshapedPolygon([[2, 2], [8, 2], [8, 3], [2, 3]]),
                StarshapedPolygon([[2, 5], [8, 5], [8, 6], [2, 6]]),
                StarshapedPolygon([[2, 2], [8, 2], [8, 3], [2, 3]]),
                StarshapedPolygon([[2, 8], [8, 8], [8, 9], [2, 9]]),
                Ellipse([1.1, 1.1], motion_model=motion_model.Interval([-2, 4], [(13, (10, 4))])),
                # StarshapedPolygon(Ellipse([1, 1]).polygon(), motion_model=motion_model.Interval([-1, 4], [(9, (10, 4))])),
                # Ellipse([1, 1], motion_model=motion_model.Interval([-2, 4], [(9, (11, 4))])),
            ]
            # self.p0 = np.array([9, 5])
            self.p0 = np.array([9, 4])
            self.pg = np.array([0.5, 5.5])
            self.xlim = [0, 10]
            self.ylim = [2, 9]

        # Scene 10
        scene_id += 1
        if id == scene_id:
            self.obstacles = [
                # StarshapedPolygon([[7, 13], [10, 10], [11, 11], [8, 14]]),
                # StarshapedPolygon([[10, 11], [10, 7], [11, 7], [11, 11]]),
                # StarshapedPolygon([[0, 0], [0, 1], [1, 1], [1, 0]], motion_model=motion_model.Interval([5, 11], [(5, [5, 6]), (2, [8, 6])])),
                Ellipse([1, 1], motion_model=motion_model.SinusVelocity(pos=[12, 9], x1_mag=-0.2)),
                Ellipse([1, 1], motion_model=motion_model.SinusVelocity(pos=[15, 8], x1_mag=-0.8)),
                Ellipse([1, 1], motion_model=motion_model.SinusVelocity(pos=[10, 11], x1_mag=0.)),
                Ellipse([1, 1], motion_model=motion_model.SinusVelocity(pos=[10, 7], x1_mag=0.8)),
                Ellipse([1, 1], motion_model=motion_model.SinusVelocity(pos=[12, 11.5], x1_mag=0.2)),
                Ellipse([1, 1], motion_model=motion_model.SinusVelocity(pos=[15, 13.5], x2_mag=-0.1)),
                Ellipse([1, 1], motion_model=motion_model.SinusVelocity(pos=[6, 7], x1_mag=0.3)),
                Ellipse([1, 1], motion_model=motion_model.SinusVelocity(pos=[17, 5], x2_mag=0.1)),
                Ellipse([1, 1], motion_model=motion_model.Interval([16, 4], [(11, [16, 7]), (5., [16, 10])])),
                Ellipse([1, 1], motion_model=motion_model.SinusVelocity(pos=[3, 10], x1_mag=0.3)),
                Ellipse([1, 1], motion_model=motion_model.SinusVelocity(pos=[4, 8.5], x1_mag=0.1)),
                Ellipse([1, 1], motion_model=motion_model.SinusVelocity(pos=[5, 11.5], x1_mag=-0.1)),
                Ellipse([1, 1], motion_model=motion_model.SinusVelocity(pos=[8, 6], x1_mag=-0.1)),
                Ellipse([1, 1], motion_model=motion_model.SinusVelocity(pos=[10, 4], x2_mag=0.1)),
                Ellipse([1, 1], motion_model=motion_model.SinusVelocity(pos=[11, 3], x2_mag=0.2)),
                Ellipse([1, 1], motion_model=motion_model.SinusVelocity(pos=[13, 13.5], x2_mag=-0.2)),
                # Ellipse([1, 1], motion_model=motion_model.SinusVelocity(pos=[6, 7], x1_mag=0.3)),
                # StarshapedPolygon([[0, 0], [0, 1], [1, 1], [1, 0]], motion_model=motion_model.Interval([14, 3], [(5, [9.5, 6.5])])),
                # Ellipse([1, 1], motion_model=motion_model.Interval([6, -5], [])),
                # Ellipse([1, 1], motion_model=motion_model.Interval([4, -3], [])),
                # Ellipse([1, 1], motion_model=motion_model.Interval([7, -1], [])),
                # Ellipse([1, 1], motion_model=motion_model.Interval([5, -2], x1_mag=-0.25, x2_mag=-0.2)),
                # Ellipse([1, 1], motion_model=motion_model.Interval([1, -5], x1_mag=0.3, x2_mag=0.5)),
                # Ellipse([0.4, 1], motion_model=motion_model.SinusVelocity(pos=[8, -3], x1_mag=3, x1_period=4, x2_mag=2,
                #                                                           x2_period=2)),
                # Ellipse([2, 1], motion_model=motion_model.SinusVelocity(pos=[7, -2], x1_mag=2, x1_period=3)),
                # Ellipse([0.4, 0.4], motion_model=motion_model.SinusVelocity(pos=[8, -5], x1_mag=0.3, x2_mag=0.5)),
            ]
            self.p0 = np.array([1., 9.])
            self.pg = np.array([19., 9.])
            self.xlim = [0, 20]
            self.ylim = [0, 18]

        # Scene 11
        scene_id += 1
        if id == scene_id:
            self.obstacles = [
                # PolynomialObstacle(xys=np.array([[0, 0], [10, 0], [10, 3], [4, 5], [10, 7], [10, 10], [0, 10], [0, 0]], dtype=np.float64) + np.array([-2, -5]), pol_deg=60, is_boundary=True),
                # Ellipse([5, 5], is_boundary=True),
                Ellipse([1, 1], motion_model=motion_model.Interval([0., 0.5], [(0.5, [1.2, 0.5])])),
                Ellipse([1, 1], motion_model=motion_model.Static([0, -0.5])),
                Ellipse([1, 1], motion_model=motion_model.Static([0, 1.5]))
            ]
            self.p0 = np.array([-1.5, 1.5])
            # self.p0 = np.array([5.6, -1.55])
            self.pg = np.array([1.5, -1.5])
            self.xlim = [-3, 3]
            self.ylim = [-3, 3]

        # Scene 12
        scene_id += 1
        if id == scene_id:
            self.obstacles = [
                StarshapedPolygon([[0, 0], [2, 0], [2, 6], [0, 6]]),
                StarshapedPolygon([[2.5, 0], [6, 0], [6, 6], [2.5, 6]]),
                StarshapedPolygon([[7, 0], [10, 0], [10, 6], [7, 6]]),
                Ellipse([0.5, 0.5], motion_model=motion_model.Interval([4.5, -0.3], [(3, (4.5, -0.3)), (1, (4.5, -2))]))
                # StarshapedPolygon([[6.5, 0], [10, 0], [10, 6], [6.5, 6]]),
            ]
            self.p0 = np.array([2.25, 0.5])
            self.pg = np.array([6.5, 1])
            # self.pg = np.array([6.25, 1])
            self.theta0 = np.pi/2
            self.xlim = [0, 10]
            self.ylim = [-4, 2]

        # Scene 13
        scene_id += 1
        if id == scene_id:
            self.obstacles = [
                              Ellipse([0.5, 0.5], motion_model=motion_model.Static([3, 0.7])),
                              Ellipse([0.5, 0.5], motion_model=motion_model.Static([3.5, -0.1])),
                              Ellipse([0.5, 0.5], motion_model=motion_model.Static([3, -0.9])),
                              Ellipse([0.5, 0.5], motion_model=motion_model.Static([7, 0.6])),
                              Ellipse([0.5, 0.5], motion_model=motion_model.Static([8, -0.2])),
                              Ellipse([0.5, 0.5], motion_model=motion_model.Static([4.5, 1.8])),
                              Ellipse([0.5, 0.5], motion_model=motion_model.Static([5.3, 0.4])),
                              Ellipse([0.5, 0.5], motion_model=motion_model.Static([5., -1.4]))
            ]
            [o.polygon() for o in self.obstacles]


            self.p0 = np.array([1, 0])
            self.pg = np.array([9, 0])
            # self.pg = np.array([6.25, 1])
            # self.theta0 = np.pi / 2
            self.xlim = [0, 10]
            self.ylim = [-5, 5]

        # Scene 14
        scene_id += 1
        if id == scene_id:
            n_pol = 20
            self.obstacles = [
                Ellipse([0.5, 0.5], n_pol=n_pol, motion_model=motion_model.SinusVelocity([3, 0.7], x1_mag=-0.13)),
                Ellipse([0.5, 0.5], n_pol=n_pol, motion_model=motion_model.SinusVelocity([4, -0.1], x1_mag=-0.5)),
                Ellipse([0.5, 0.5], n_pol=n_pol, motion_model=motion_model.SinusVelocity([3, -0.9], x1_mag=-0.1)),
                Ellipse([0.5, 0.5], n_pol=n_pol, motion_model=motion_model.SinusVelocity([7, 0.6], x1_mag=-0.1)),
                Ellipse([0.5, 0.5], n_pol=n_pol, motion_model=motion_model.SinusVelocity([8, -0.2], x1_mag=-0.3, x2_mag=-0.1)),
                Ellipse([0.5, 0.5], n_pol=n_pol, motion_model=motion_model.SinusVelocity([6, -0.5], x1_mag=-0.1, x2_mag=0.1)),
                Ellipse([0.5, 0.5], n_pol=n_pol, motion_model=motion_model.SinusVelocity([5.3, 0.4], x1_mag=-0.1)),
                Ellipse([0.5, 0.5], n_pol=n_pol, motion_model=motion_model.SinusVelocity([5., -1.4], x2_mag=0.0)),
                Ellipse([0.5, 0.5], n_pol=n_pol, motion_model=motion_model.SinusVelocity([6.3, 1.2], x1_mag=0.)),
                Ellipse([0.5, 0.5], n_pol=n_pol, motion_model=motion_model.SinusVelocity([4.2, 1.4], x1_mag=-0.3, x2_mag=-0.2)),
                Ellipse([0.5, 0.5], n_pol=n_pol, motion_model=motion_model.SinusVelocity([8., 1.2], x1_mag=-0.1)),
                Ellipse([0.5, 0.5], n_pol=n_pol, motion_model=motion_model.SinusVelocity([8.5, -0.75], x1_mag=-0.1)),
                Ellipse([0.5, 0.5], n_pol=n_pol, motion_model=motion_model.SinusVelocity([7, -1.25], x1_mag=-0.05))
            ]
            # self.obstacles = [StarshapedPolygon(o.polygon(output_frame=Frame.OBSTACLE), motion_model=o._motion_model) for o in self.obstacles]

            self.p0 = np.array([0.9, 0])
            self.p0 = np.array([1.5, 0])
            self.pg = np.array([9., 0])
            # self.pg = np.array([6.25, 1])
            # self.theta0 = np.pi / 2
            self.xlim = [0, 10]
            self.ylim = [-5, 5]

        # Scene 15
        scene_id += 1
        if id == scene_id:
            self.obstacles = [
                Polygon([[3, 1], [4, 1], [4, 4], [2.5, 4], [2.5, 6], [1.5, 6], [1.5, 4], [0, 4], [0, 3], [3, 3]]),
                Polygon([[4, 6], [5, 6], [5, 9], [2, 9], [2, 8], [2, 7.2], [3, 7.2], [3, 8], [4, 8]]),
                StarshapedPolygon([[6, 6], [8, 6], [8, 7], [7, 7], [7, 8], [6, 8]]),
                Polygon([[10, 8], [10, 2], [6, 2], [6, 4], [7, 4], [7, 3], [9, 3], [9, 8]]),
                Ellipse([.4, .4], motion_model=motion_model.Static([5.5, 6.5])),
                Ellipse([.4, .4], motion_model=motion_model.Waypoints([8.5, 6.5], [[8.5, 4.5], [4.5, 4.5]], 0.2)),
                Ellipse([.4, .4], motion_model=motion_model.Waypoints([5, 4.5], [[5, 1]], 0.2)),
                # Ellipse([.6, .6], motion_model=motion_model.Static([2, 6.5]))
            ]
            self.p0 = np.array([1, 1])
            self.pg = np.array([5.5, 8])
            self.xlim = [0, 10]
            self.ylim = [0, 10]

        # Scene 16
        scene_id += 1
        if id == scene_id:
            # np.random.seed(2)
            No = 30
            ell_radius_mean = 0.5
            ell_radius_std = 0.2
            self.xlim = [0, 10]
            self.ylim = [0, 10]

            def random_scene_point():
                return np.array([np.random.rand() * (self.xlim[1]-self.xlim[0]) + self.xlim[0],
                                 np.random.rand() * (self.ylim[1]-self.ylim[0]) + self.ylim[0]])

            self.obstacles = [
                Ellipse(a=np.random.normal(ell_radius_mean, ell_radius_std, 2)) for _ in range(No)
            ]
            [o.set_motion_model(motion_model.Static(random_scene_point())) for o in self.obstacles]
            self.p0 = random_scene_point()
            while any([o.interior_point(self.p0) for o in self.obstacles]):
                self.p0 = random_scene_point()
            self.pg = random_scene_point()
            while any([o.interior_point(self.pg) for o in self.obstacles]):
                self.pg = random_scene_point()

        # Scene 17
        scene_id += 1
        if id == scene_id:
            self.obstacles = [
                Ellipse([0.5, 0.5], motion_model=motion_model.Static([3, 5.7])),
                Ellipse([0.5, 0.5], motion_model=motion_model.Static([3.5, 4.9])),
                Ellipse([0.5, 0.5], motion_model=motion_model.Static([3, 4.1])),
                Ellipse([0.5, 0.5], motion_model=motion_model.Static([7, 5.6])),
                Ellipse([0.5, 0.5], motion_model=motion_model.Static([8, 4.8])),
                Ellipse([0.5, 0.5], motion_model=motion_model.Static([4.5, 6.8])),
                Ellipse([0.5, 0.5], motion_model=motion_model.Static([5.3, 5.4])),
                Ellipse([0.5, 0.5], motion_model=motion_model.Static([5., 3.6])),
                Ellipse([0.5, 0.5], motion_model=motion_model.Static([5.3, 2.8])),
                Ellipse([0.5, 0.5], motion_model=motion_model.Static([7., 6.2])),
                Ellipse([0.5, 0.5], motion_model=motion_model.Static([5.3, 1.8]))
            ]
            self.p0 = np.array([1, 5])
            self.pg = np.array([9, 5])
            self.xlim = [0, 10]
            self.ylim = [0, 10]

        # Scene 18
        scene_id += 1
        if id == scene_id:
            ell_ax = [0.8, 0.8]
            self.obstacles = [
                StarshapedPolygon([(7, 5), (6, 5), (6, 3), (7, 3)], is_convex=True),
                StarshapedPolygon([(6, 3), (6, 7), (5, 7), (5, 3)], is_convex=True),
                StarshapedPolygon([(7, 1), (8, 1), (8, 5), (7, 5)], is_convex=True),
                StarshapedPolygon([(6, 7), (6, 8), (4, 8), (4, 7)], is_convex=True),
                StarshapedPolygon([(2, 6), (3, 6), (3, 10), (2, 10)], is_convex=True),
                StarshapedPolygon([(3, 9), (3, 10), (6, 10), (6, 9)], is_convex=True),
                StarshapedPolygon([(8, 6), (9, 6), (9, 10), (8, 10)], is_convex=True),
                Ellipse(ell_ax, motion_model=motion_model.Static([7.5, 8])),
                Ellipse(ell_ax, motion_model=motion_model.Static([4.8, 5])),
                # Ellipse(ell_ax, motion_model=motion_model.Static([4, 8])),
                # Ellipse(ell_ax, motion_model=motion_model.Static([2.5, 5.5])),
                Ellipse(ell_ax, motion_model=motion_model.Static([6.5, 1])),
                # Ellipse(ell_ax, motion_model=motion_model.Interval([2, 3], [(3, (4., 6))])),
                # Ellipse(ell_ax, motion_model=motion_model.SinusVelocity([4.7, 12], x2_mag=-0.2)),
            ]
            self.p0 = np.array([1., 1.])
            # self.p0 = np.array([1., 3.])
            # self.p0 = np.array([1., 5.])
            self.pg = np.array([9., 5.])
            self.xlim = [-1, 11]
            self.ylim = [-1, 11]
            self.theta0 = 0

        # Scene 19
        scene_id += 1
        if id == scene_id:
            ell_ax = [0.8, 0.8]
            self.obstacles = [
                StarshapedPolygon([(4, 1), (6, 1), (6, 9), (4, 9)], is_convex=True),
                StarshapedPolygon([(1, 4), (9, 4), (9, 6), (1, 6)], is_convex=True),
                Ellipse(ell_ax, motion_model=motion_model.Static([6.3, 0.5])),
                Ellipse(ell_ax, motion_model=motion_model.Static([1.2, 6.7])),
                # Ellipse(ell_ax, motion_model=motion_model.Static([1.7, 8])),
                Ellipse(ell_ax, motion_model=motion_model.Static([6, 0])),
                Ellipse(ell_ax, motion_model=motion_model.Static([8.6, 6.7])),
                # Ellipse(ell_ax, motion_model=motion_model.Static([2.5, 5.5])),
                # Ellipse(ell_ax, motion_model=motion_model.Static([6.5, 1])),
            ]
            self.p0 = np.array([3, 7.])
            self.p0 = np.array([7, 6.5])
            # self.p0 = np.array([7., 3.])
            self.pg = np.array([3., 3.])
            self.xlim = [-2, 10]
            self.ylim = [-2, 10]
            self.theta0 = 0

        # Scene 20
        scene_id += 1
        if id == scene_id:
            ell_ax = [0.8, 0.8]
            self.obstacles = [
                # StarshapedPolygon([[0, 0], [4, 0], [4, 1], [1, 1], [1, 3], [0, 3]]),
                StarshapedPolygon([[0, 0], [1, 0], [1, 3], [0, 3]]),
                StarshapedPolygon([[1, 0], [3, 0], [3, 1], [1, 1]]),
                StarshapedPolygon([[3, 0], [4, 0], [4, 3], [3, 3]]),
                # Polygon([[0, 0], [4, 0], [4, 3], [3, 3], [3, 1], [1, 1], [1, 3], [0, 3]]),
                Ellipse([1, 1], motion_model=motion_model.Static([-0.5, 3.7])),
                Ellipse([0.5, 1.5], motion_model=motion_model.Static([4, -1.]))
            ]
            self.p0 = np.array([0, -2.])
            self.pg = np.array([2, 2])
            # self.pg = np.array([4, 4])
            self.xlim = [-3, 6]
            self.ylim = [-3, 6]

        if not (0 < id < scene_id+1):
                text = 'Invalid scene id: ' + str(id) + '\n\nScene scenarios\n---------\n'
                for i, description in scene_description.items():
                    text += str(i) + ": " + description + "\n"
                raise Exception(text)

        # Make all obstacles convex
        self._obstacles_to_plot = self.obstacles
        if all_convex:
            convex_obstacles = []
            for o in self.obstacles:
                if o.is_convex():
                    convex_obstacles += [o]
                else:
                    pol = py2dPol.from_tuples(np.asarray(o.polygon().exterior.coords)[:-1, :])
                    conv_pols = py2dPol.convex_decompose(pol)
                    for cp in conv_pols:
                        convex_obstacles += [StarshapedPolygon([(v.x, v.y) for v in cp.points], is_convex=True)]
            self.obstacles = convex_obstacles
        # self._obstacles_to_plot = self.obstacles

        # Compute all polygon
        [o.polygon() for o in self.obstacles]
        [o.is_convex() for o in self.obstacles]

    def step(self, dt, X_exclude=None, dist=0):
        [o.move(dt, X_exclude, dist) for o in self.obstacles]
        [o.move(dt, X_exclude, dist) for o in self._obstacles_to_plot]

    def init_plot(self, ax=None, obstacle_color='lightgrey', draw_p0=1, draw_pg=1, show_obs_name=False, draw_streamlines=0):
        if ax is None:
            _, ax = plt.subplots(subplot_kw={'aspect': 'equal'})
        line_handles = []
        if draw_p0:
            ax.plot(*self.p0, 'ks', markersize=10)
        if draw_pg:
            ax.plot(*self.pg, 'k*', markersize=10)

        # lh = self.obstacles[0].init_plot(ax=ax, fc='None', ec='k', show_reference=False)
        # line_handles.append(lh)
        for o in self._obstacles_to_plot:
            # lh = o.init_plot(ax=ax, fc=obstacle_color, ec='k', show_reference=True)
            lh, _ = o.init_plot(ax=ax, fc=obstacle_color, ec='k', show_name=show_obs_name, show_reference=False)
            line_handles.append(lh)
        ax.set_xlim(self.xlim)
        ax.set_ylim(self.ylim)

        if draw_streamlines:
            from motion_control.path_generator import plot_vector_field
            plot_vector_field(self.pg, self.obstacles, ax, xlim=self.xlim, ylim=self.ylim, n=50)
        # line_handles = scene.init_plot()
        # scene.update_plot(line_handles)
        # ax = plt.gca()
        # plot_vector_field(scene.xg, scene.obstacles, ax, xlim=None, ylim=None, n=150)
        # x = np.array([6.5, -2])
        # ax.plot(*scene.obstacles[0].boundary_mapping(x), 'o')
        # ax.quiver(*x, *f(x, scene.xg, scene.obstacles, unit_magnitude=1))
        # plt.show()

        return line_handles, ax

    def update_plot(self, line_handles):
        for i, o in enumerate(self._obstacles_to_plot):
            o.update_plot(line_handles[i])

    def draw(self, ax=None, obstacle_color='lightgrey', draw_p0=1, draw_pg=1, show_obs_name=False, draw_streamlines=0):
        lh, ax = self.init_plot(ax, obstacle_color, draw_p0, draw_pg, show_obs_name, draw_streamlines)
        self.update_plot(lh)
        return lh, ax
