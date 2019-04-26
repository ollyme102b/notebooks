import numpy as np


class OllyYawEstimatorV1:
    def __init__(self, N=10):
        self.N = N
        self.counter = 0
        self.yaw = 0
        self.velocity_body_stack = np.zeros((2, N))
        self.velocity_inertial_stack = np.zeros((2, N))

    def _increment_counter(self):
        self.counter = (self.counter + 1) % self.N

    def _update_body_stack(self, vx, vy):
        self._update_stack(self.velocity_body_stack, vx, vy)

    def _update_inertial_stack(self, vx, vy):
        self._update_stack(self.velocity_inertial_stack, vx, vy)

    def _update_stack(self, stack, vx, vy):
        stack[0, self.counter] = vx
        stack[1, self.counter] = vy

    def update(self, vix, viy, vbx, vby):
        self._update_inertial_stack(vix, viy)
        self._update_body_stack(vbx, vby)
        self._increment_counter()

    def update_with_numpy_array(self, vi, vb):
        self.update(vi[0], vi[1], vb[0], vb[1])

    def estimate_yaw(self):
        dot_products = np.sum(np.multiply(self.velocity_body_stack, self.velocity_inertial_stack), axis=0)
        directionals = [np.sign(np.cross(self.velocity_body_stack[:,t], self.velocity_inertial_stack[:,t])) for t in range(self.N)]
        norms = np.linalg.norm(self.velocity_body_stack, axis=0) * np.linalg.norm(self.velocity_inertial_stack, axis=0)
        yaws = np.nan_to_num(np.arccos(dot_products / norms)) * directionals
        self.yaw = np.mean(yaws)
        return self.yaw
