import numpy as np

class Observer:
    def __init__(self, J, l, J_r, Ts=0.01):
        # x -> state vector [phi, theta, psi, phi_dot, theta_dot, psi_dot]'
        # u -> input vector [u2, u3, u4]'
        # x_hat = [0, 0, 0, 0, 0, 0]'  # Initial state estimate
        # J -> moment of inertia tensor of the quadrotor
        # l -> L/root2 ; L: distance from center of mass to motors
        # J_r -> rotor inertia
        self.J = J
        self.l = l
        self.J_r = J_r
        self.Ts = Ts  # Sampling time

        self.Ix = J[0][0]
        self.Iy = J[1][1]
        self.Iz = J[2][2]

        # Initial state estimate x_hat (6x1 vector)
        self.x_hat = np.zeros((6, 1))

        # State transition matrix A
        self.A = np.array([[0, 0, 0, 1, 0, 0],
                           [0, 0, 0, 0, 1, 0],
                           [0, 0, 0, 0, 0, 1],
                           [0, 0, 0, 0, 0, 0],
                           [0, 0, 0, 0, 0, 0],
                           [0, 0, 0, 0, 0, 0]])

        # Input matrix B
        self.B = np.array([[0, 0, 0],
                           [0, 0, 0],
                           [0, 0, 0],
                           [1 / self.Ix, 0, 0],
                           [0, 1 / self.Iy, 0],
                           [0, 0, 1 / self.Iz]])

        # Output matrix C (Identity matrix for direct observation of states)
        self.C = np.eye(6)

        # Observer gain matrix K
        self.K = np.array([[100, 0, 0, 5, 0, 0],
                           [0, 100, 0, 0, 5, 0],
                           [0, 0, 100, 0, 0, 5],
                           [5, 0, 0, 100, 0, 0],
                           [0, 5, 0, 0, 100, 0],
                           [0, 0, 5, 0, 0, 100]])

    def integrate_rk4(self, x, x_dot):
        # Runge-Kutta 4th order (RK4) integration method
        k1 = self.Ts * x_dot
        k2 = self.Ts * (x_dot + 0.5 * k1)
        k3 = self.Ts * (x_dot + 0.5 * k2)
        k4 = self.Ts * (x_dot + k3)
        x_next = x + (k1 + 2 * k2 + 2 * k3 + k4) / 6
        
        return x_next

    def get_residual(self, x, omega_r, u):
        phi_dot_hat = self.x_hat[3][0]
        theta_dot_hat = self.x_hat[4][0]
        psi_dot_hat = self.x_hat[5][0]
        # print(u.shape)
        h_x_hat = np.array([[0],
                            [0],
                            [0],
                            [(psi_dot_hat * theta_dot_hat * (self.Iy - self.Iz) - self.J_r * omega_r * theta_dot_hat) / self.Ix],
                            [(psi_dot_hat * phi_dot_hat * (self.Iz - self.Ix) + self.J_r * omega_r * phi_dot_hat) / self.Iy],
                            [(phi_dot_hat * theta_dot_hat * (self.Ix - self.Iy)) / self.Iz]])
        x_hat_dot = self.A @ self.x_hat + self.B @ u + h_x_hat + self.K @ (x - self.x_hat)
        self.x_hat = self.integrate_rk4(self.x_hat, x_hat_dot)
        residual = x - self.x_hat
        
        return residual