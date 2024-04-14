import numpy as np

def state_dot(M_inv: np.ndarray, D: np.ndarray, state: np.ndarray, tau_actuation: np.ndarray, V_current: np.ndarray = np.zeros(2)) -> np.ndarray:
    """
    Calculate the derivative of the state using the non-linear kinematics
    """
    heading = state[2]

    J = np.array(
        [[np.cos(heading), -np.sin(heading), 0],
        [np.sin(heading), np.cos(heading), 0],
        [0, 0, 1]]
    )

    A = np.zeros((6,6))

    A[:3,3:] = J
    A[3:, 3:] = - M_inv @ D

    B = np.zeros((6,3))
    B[3:,:] = M_inv

    x_dot = A @ state + B @ tau_actuation
    x_dot[0:2] += V_current # add current drift term at velocity level

    return x_dot

def RK4_integration_step(M_inv: np.ndarray, D: np.ndarray, x: np.ndarray, u: np.ndarray, dt: float) -> np.ndarray:
    # integration scheme for simulation, implements the Runge-Kutta 4 integrator
    k1 = state_dot(M_inv, D, x,         u)
    k2 = state_dot(M_inv, D, x+dt/2*k1, u)
    k3 = state_dot(M_inv, D, x+dt/2*k2, u)
    k4 = state_dot(M_inv, D, x+dt*k3,   u)
    
    x_next = x + dt/6*(k1+2*k2+2*k3+k4)

    return x_next