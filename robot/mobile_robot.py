import matplotlib.pyplot as plt
import matplotlib.patches as patches
from abc import ABC, abstractmethod
import numpy as np


class MobileRobot(ABC):

    def __init__(self, nu, nx, width, name, u_min=None, u_max=None, x_min=None, x_max=None):
        self.nx = nx
        self.nu = nu
        self.width = width
        self.name = name
        def valid_u_bound(bound): return bound is not None and len(bound) == self.nu
        def valid_q_bound(bound): return bound is not None and len(bound) == self.nx
        self.u_min = u_min if valid_u_bound(u_min) else [-np.inf] * self.nu
        self.u_max = u_max if valid_u_bound(u_max) else [np.inf] * self.nu
        self.x_min = x_min if valid_q_bound(x_min) else [-np.inf] * self.nx
        self.x_max = x_max if valid_q_bound(x_max) else [np.inf] * self.nx

    @abstractmethod
    def f(self, x, u):
        pass

    @abstractmethod
    def h(self, q):
        pass

    @abstractmethod
    def vel_min(self):
        pass

    @abstractmethod
    def vel_max(self):
        pass

    def move(self, x, u, dt):
        u_sat = np.clip(u, self.u_min, self.u_max)
        x_next = x + np.array(self.f(x, u_sat)) * dt
        x_next = np.clip(x_next, self.x_min, self.x_max)
        return x_next, u_sat

    def init_plot(self, ax=None, color='b', alpha=0.7, markersize=10):
        if ax is None:
            _, ax = plt.subplots(subplot_kw={'aspect': 'equal'})
        if self.width == 0:
            handles = plt.plot(0, 0, '+', color=color, alpha=alpha, markersize=markersize, label=self.name)
        else:
            handles = [patches.Circle((0, 0), self.width / 2, color=color, alpha=alpha, label=self.name)]
            ax.add_patch(handles[0])
        return handles, ax

    def update_plot(self, x, handles):
        if self.width == 0:
            handles[0].set_data(*self.h(x))
        else:
            handles[0].set(center=self.h(x))

    def draw(self, x, **kwargs):
        handles, ax = self.init_plot(**kwargs)
        self.update_plot(x, handles)
        return handles, ax
