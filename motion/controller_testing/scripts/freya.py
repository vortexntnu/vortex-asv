import numpy as np
from scipy.linalg import expm

# Constants
mp = 2.5
rp = 0.075
V_c = 1
beta_c = 0.2
n_max = 10000
n_min = -10000
k_pos = 0.005
k_neg = 0.001

# Skew-symmetric matrix
def Smtrx(vector):
    return np.array([[0, -vector[2], vector[1]], [vector[2], 0, -vector[0]], [-vector[1], vector[0], 0]])

# Rotation matrix from Euler angles (zyx convention)
def Rzyx(phi, theta, psi):
    return np.array([[np.cos(psi) * np.cos(theta), np.cos(psi) * np.sin(theta) * np.sin(phi) - np.sin(psi) * np.cos(phi), np.cos(psi) * np.sin(theta) * np.cos(phi) + np.sin(psi) * np.sin(phi)],
                    [np.sin(psi) * np.cos(theta), np.sin(psi) * np.sin(theta) * np.sin(phi) + np.cos(psi) * np.cos(phi), np.sin(psi) * np.sin(theta) * np.cos(phi) - np.cos(psi) * np.sin(phi)],
                    [-np.sin(theta), np.cos(theta) * np.sin(phi), np.cos(theta) * np.cos(phi)]])


def model(x, tau):
    # Ensure the input vectors are the correct size
    assert len(x) == 12, 'x vector must have dimension 12!'

    # Vehicle dimensions
    x_pont = 1.2
    y_pont = 1.0
    z_pont = 0.5

    # Inertia Matrix
    m = 11.0  # mass of the otter in kg
    Ix = 1.0  # moment of inertia about x
    Iy = 1.0  # moment of inertia about y
    Iz = 1.0  # moment of inertia about z
    I = np.diag([Ix, Iy, Iz])

    # Rigid body mass matrix
    M_RB = np.block([[m * np.eye(3), np.zeros((3, 3))], [np.zeros((3, 3)), I]])

    # Added mass
    X_du = 0  # Surge (kg)
    Y_dv = 0  # Sway (kg)
    Z_dw = 0  # Heave (kg)
    K_dp = 0  # Roll (kg*m^2)
    M_dq = 0  # Pitch (kg*m^2)
    N_dr = 0  # Yaw (kg*m^2)
    M_A = -np.diag([X_du, Y_dv, Z_dw, K_dp, M_dq, N_dr])

    # Total mass matrix
    M = M_RB + M_A

    # Damping coefficients
    X_u = 2  # Linear damping - Surge (Ns/m)
    Y_v = 2  # Linear damping - Sway (Ns/m)
    Z_w = 5  # Linear damping - Heave (Ns/m)
    K_p = 0.1  # Linear damping - Roll (Ns)
    M_q = 0.1  # Linear damping - Pitch (Ns)
    N_r = 0.1  # Linear damping - Yaw (Ns)
    D_l = np.diag([X_u, Y_v, Z_w, K_p, M_q, N_r])  # Linear damping matrix

    X_uu = 0  # Quadratic damping - Surge (Ns^2/m^2)
    Y_vv = 0  # Quadratic damping - Sway (Ns^2/m^2)
    Z_ww = 0  # Quadratic damping - Heave (Ns^2/m^2)
    K_pp = 0  # Quadratic damping - Roll (Ns)
    M_qq = 0  # Quadratic damping - Pitch (Ns)
    N_rr = 0.1  # Quadratic damping - Yaw (Ns)
    D_q = np.diag([abs(X_uu * x[6]), abs(Y_vv * x[7]), abs(Z_ww * x[8]), abs(K_pp * x[9]), abs(M_qq * x[10]), abs(N_rr * x[11])])  # Quadratic damping matrix

    D = D_l + D_q  # Damping matrix

    # Coriolis and centripetal matrix
    CRB = m * np.block([[np.zeros((3, 3)), -Smtrx(np.array([x[6], x[7], x[8]]))],
                    [Smtrx(np.array([x[6], x[7], x[8]])), -Smtrx(I.dot(np.array([x[9], x[10], x[11]])))]])


    CA = m * np.block([[np.zeros((3, 3)), -Smtrx(np.array([x[0], x[1], x[2]]))],
                   [np.zeros((3, 3)), np.zeros((3, 3))]])


    C = CRB + CA

    # Propeller forces
    # n_prop = np.clip(n, n_min, n_max)
    # T_pos = k_pos * n_prop * abs(n_prop)
    # T_neg = k_neg * n_prop * abs(n_prop)
    # T = np.where(n_prop > 0, T_pos, T_neg)

    #tau = np.array([T.sum(), 0, 0, 0, 0, (x_pont / 2) * (T[1] - T[0]) + (y_pont / 2) * (T[3] - T[2])])

    # System matrices
    eta = x[0:6]
    nu = x[6:12]

    J = np.block([[np.eye(3), np.zeros((3, 3))], [np.zeros((3, 3)), Rzyx(eta[3], eta[4], eta[5])]])

    M_inv = np.linalg.inv(M)

    # # tau is defined in world, move to body
    yaw = eta[5]
    c = np.cos(yaw)
    s = np.sin(yaw)

    R = np.array(([c, -s, 0], [s, c, 0], [0, 0, 1]))

    # # nu is defined in world, move to body
    nu[:3] = R @ nu[:3]

    tau = R @ tau

    tau_body_six = np.array((tau[0], tau[1], 0.0, 0.0, 0.0, tau[2]))

    # System model
    etadot = J @ nu
    nudot = M_inv @ (tau_body_six - C @ nu - D @ nu)

    xdot = np.concatenate((etadot, nudot))

    return xdot
