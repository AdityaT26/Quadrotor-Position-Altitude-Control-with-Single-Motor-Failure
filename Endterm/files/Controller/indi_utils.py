import numpy as np
from quadrotor import Quadrotor3D


STABILIZATION_INDI_FILT_CUTOFF_w = 10.0
STABILIZATION_INDI_FILT_CUTOFF_tau = 10.0
PERIODIC_FREQUENCY = 20
sample_time = 1.0/PERIODIC_FREQUENCY

def calc_alpha(omega_history, thrust): #calculate tourque from u using dynamics
    quad = Quadrotor3D()
    thrust*=quad.max_thrust

    h = quad.length
    k = quad.c
    quantity = np.zeros(3)
    I_v = np.array([[quad.J[0], 0, 0], 
                [0, quad.J[1], 0], 
                [0, 0, quad.J[2]]])
    
    omega = omega_history[-1]
    
    quantity[0] = (-h*thrust[0] + h*thrust[1] + h*thrust[2] - h*thrust[3] + (quad.J[1] - quad.J[2]) * omega[1] * omega[2])
    quantity[1] = (h*thrust[0] - h*thrust[1] + h*thrust[2] - h*thrust[3] + (quad.J[2] - quad.J[0]) * omega[0] * omega[2])
    quantity[2] = (k*thrust[0] + k*thrust[1] - k*thrust[2] - k*thrust[3] + (quad.J[0] - quad.J[1]) * omega[0] * omega[1])
    return calc_Td(quantity, omega_history, thrust)


def calc_Td(quantity, omega_history, thrust): 
    quad = Quadrotor3D()
    h = np.cos(np.pi / 4) * quad.length
    k = quad.c
    tau = np.zeros(3)
    I_v = np.array([[quad.J[0], 0, 0], 
                [0, quad.J[1], 0], 
                [0, 0, quad.J[2]]])

    omega_dot = (- omega_history[-2] + omega_history[-1])*PERIODIC_FREQUENCY
    tourque_asli = I_v @ omega_dot

    tourqe_ext = 0.3*(tourque_asli - quantity.reshape((3,1)))

    tourque_want_to_gen = quantity.reshape((3,1)) - tourqe_ext

    gu_3 = tourque_want_to_gen + np.cross(omega_history[-1].reshape(3,), np.dot(I_v, omega_history[-1]).reshape(3,))#3,1
    Td = np.sum(thrust)
    gu = np.array([Td, gu_3[0][0],gu_3[1][0],gu_3[2][0]]).reshape(4,1)
    print(gu.shape)
    G = np.array([[1, 1, 0, 1], 
            [-h, h, 0, -h], 
            [h, -h, 0, -h], 
            [k, k, 0, -k]])
    G_inv = np.linalg.pinv(G)

    u = G_inv @ gu
    return (u/quad.max_thrust).reshape((4,))