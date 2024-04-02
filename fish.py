
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse



class Fish:
    def __init__(self, x, y, psi, delta, alpha1, alpha2):
        self.x = x
        self.y = y
        self.psi = psi
        self.delta = delta
        self.alpha1 = alpha1
        self.alpha2 = alpha2

    def get_state(self):
        return self.x, self.y, self.psi, self.delta, self.alpha1, self.alpha2

    def set_state(self, x, y, psi, delta, alpha1, alpha2):
        self.x = x
        self.y = y
        self.psi = psi
        self.delta = delta
        self.alpha1 = alpha1
        self.alpha2 = alpha2

    def set_shape(self, l0=0.042, l1=0.058, l2=0.022, d0=0.04, d=0.08):
        self.l0 = l0
        self.l1 = l1
        self.l2 = l2
        self.d0 = d0
        self.d = d

    def plot(self, ax):
        fish_body = Ellipse((self.x, self.y), self.d0, self.d, angle=self.psi*180/np.pi, color='red')
        fish_tail0 = plt.Line2D([self.x, self.x - self.l0*np.cos(self.psi)], [self.y, self.y - self.l0*np.sin(self.psi)], color='red')
        fish_tail1 = plt.Line2D([self.x - self.l0*np.cos(self.psi), self.x - self.l0*np.cos(self.psi) - self.l1*np.cos(self.psi + self.alpha1)], [self.y - self.l0*np.sin(self.psi), self.y - self.l0*np.sin(self.psi) - self.l1*np.sin(self.psi + self.alpha1)], color='red')
        fish_tail2 = plt.Line2D([self.x - self.l0*np.cos(self.psi), self.x - self.l0*np.cos(self.psi) - self.l2*np.cos(self.psi + self.alpha2)], [self.y - self.l0*np.sin(self.psi), self.y - self.l0*np.sin(self.psi) - self.l2*np.sin(self.psi + self.alpha2)], color='red')
        fish_tail3 = plt.Line2D([self.x - self.l0*np.cos(self.psi), self.x - self.l0*np.cos(self.psi) - self.l2*np.cos(self.psi - self.alpha2)], [self.y - self.l0*np.sin(self.psi), self.y - self.l0*np.sin(self.psi) - self.l2*np.sin(self.psi - self.alpha2)], color='red')
        
        ax.add_patch(fish_body)
        ax.add_line(fish_tail0)
        ax.add_line(fish_tail1)
        ax.add_line(fish_tail2)
        ax.add_line(fish_tail3)

