from __future__ import annotations
from dataclasses import dataclass, field
import numpy as np
from geometry_msgs.msg import Point

@dataclass
class LinSys:
    """
    A class to represent the linear system.

    Attributes:
        A (np.ndarray): The coefficients matrix.
        bx (np.ndarray): The x-vector for each subpath.
        by (np.ndarray): The y-vector for each subpath.
    """
    A: np.ndarray = None
    bx: np.ndarray = np.array([])
    by: np.ndarray = np.array([])

@dataclass
class Coeff:
    """
    A class to represent the coefficients.

    Attributes:
        a (np.ndarray): The x-coefficients.
        b (np.ndarray): The y-coefficients.
        a_der (np.ndarray): The x-coefficients for derivatives.
        b_der (np.ndarray): The y-coefficients for derivatives.
    """
    a: list[np.ndarray] = field(default_factory=list)
    b: list[np.ndarray] = field(default_factory=list)
    a_der: list[list[np.ndarray]] = field(default_factory=list)
    b_der: list[list[np.ndarray]] = field(default_factory=list)

@dataclass
class Path:
    """
    A class to represent the path.

    Attributes:
        NumSubpaths (int): The number of subpaths.
        Order (int): The order of the subpaths.
        LinSys (LinSys): The linear system to solve the subpaths.
        coeff (Coeff): The coefficients.
    """
    NumSubpaths: int = 0
    Order: int = 0
    LinSys: LinSys = LinSys()
    coeff: Coeff = Coeff()

class HybridPathGenerator:
    """
    This class generates a path that goes through a set of waypoints.

    Args:
        WP (list[Point]): A list of waypoints.
        r (int): The differentiability order.
        lambda_val (float): The curvature constant.

    Attributes:
        WP (np.ndarray): The waypoints.
        r (int): The differentiability order.
        lambda_val (float): The curvature constant.
        order (int): The order of the subpaths.
        path (Path): The path object.
    """
    def __init__(self, WP: list[Point], r: int, lambda_val: float):
        WP_arr = np.array([[int(wp.x), int(wp.y)] for wp in WP])
        if len(WP_arr) == 2:
            self.WP = np.array([WP_arr[0], [(WP_arr[0][0] + WP_arr[1][0])/2, (WP_arr[0][1] + WP_arr[1][1])/2], WP_arr[1]])
        else:
            self.WP = WP_arr
        self.r = r
        self.lambda_val = lambda_val
        self.order = 2*r + 1
        self.path = Path()
        self.path.NumSubpaths = len(self.WP) - 1
        self.path.Order = self.order
        self._calculate_subpaths()

    def _construct_A_matrix(self) -> np.ndarray:
        """
        Constructs and returns the A matrix.

        Returns:
            np.ndarray: The A matrix.
        """
        ord_plus_one = self.order + 1
        A = np.zeros((ord_plus_one, ord_plus_one))
        coefs = np.ones(ord_plus_one)
        A[0,0] = coefs[0]
        A[1,:] = coefs
        c_der = coefs.copy()
        for k in range(2, (self.order + 1) // 2 + 1):
            c_der = np.polyder(c_der[::-1])[::-1]
            coefs = np.concatenate([np.zeros(k-1), c_der])
            A[2*k-2, k-1] = c_der[0]
            A[2*k-1, :] = coefs
        return A

    def _calculate_subpath_coeffs(self, j: int) -> tuple[np.ndarray, np.ndarray]:
            """
            Calculate the subpath coefficients for a given index.

            Parameters:
            j (int): The index of the subpath.

            Returns:
            tuple[np.ndarray, np.ndarray]: A tuple containing two numpy arrays representing the subpath coefficients.
            """
            order_plus_one = self.order + 1
            N = self.path.NumSubpaths
            ax, bx = np.zeros(order_plus_one), np.zeros(order_plus_one)
            ax[:2] = self.WP[j:j+2, 0]
            bx[:2] = self.WP[j:j+2, 1]

            if self.order > 2:
                self._calculate_subpaths_coeffs_if_ord_greater_than_2(ax, bx, j, N)

            return ax, bx
    
    def _calculate_subpaths_coeffs_if_ord_greater_than_2(self, ax: np.ndarray, bx: np.ndarray, j: int, N: int) -> None:
        """
        Calculate the coefficients for subpaths when the order is greater than 2.
        See the README for the details.

        Args:
            ax (np.ndarray): Array to store the x-coefficients.
            bx (np.ndarray): Array to store the y-coefficients.
            j (int): Index of the current subpath.
            N (int): Total number of subpaths.

        Returns:
            None
        """
        if j == 0:
            ax[2:4] = [self.WP[j+1, 0] - self.WP[j, 0], self.lambda_val * (self.WP[j+2, 0] - self.WP[j, 0])] 
            bx[2:4] = [self.WP[j+1, 1] - self.WP[j, 1], self.lambda_val * (self.WP[j+2, 1] - self.WP[j, 1])] 

        elif j == N - 1:
            ax[2:4] = [self.lambda_val * (self.WP[j+1, 0] - self.WP[j-1, 0]), self.WP[j+1, 0] - self.WP[j, 0]]
            bx[2:4] = [self.lambda_val * (self.WP[j+1, 1] - self.WP[j-1, 1]), self.WP[j+1, 1] - self.WP[j, 1]]

        else:
            ax[2:4] = [self.lambda_val * (self.WP[j+1, 0] - self.WP[j-1, 0]), self.lambda_val * (self.WP[j+2, 0] - self.WP[j, 0])]
            bx[2:4] = [self.lambda_val * (self.WP[j+1, 1] - self.WP[j-1, 1]), self.lambda_val * (self.WP[j+2, 1] - self.WP[j, 1])]

    def solve_linear_system(self, A: np.ndarray, ax: np.ndarray, bx: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """
        Solves the linear system for the given subpath.

        Args:
            A (np.ndarray): The A matrix.
            ax (np.ndarray): The x-vector for the subpath.
            bx (np.ndarray): The y-vector for the subpath.

        Returns:
            tuple[np.ndarray, np.ndarray]: The coefficients a and b for the subpath.
        """
        a = np.linalg.solve(A, ax)
        b = np.linalg.solve(A, bx)
        return a, b

    def _calculate_derivatives(self, vec: np.ndarray) -> np.ndarray:
        """
        Calculate the derivatives of a given vector.

        Parameters:
        vec (np.ndarray): The input vector.

        Returns:
        np.ndarray: A list of arrays representing the derivatives of the input vector.
        """
        derivatives = []
        for _ in range(1, self.order + 1):
            vec = np.polyder(vec[::-1])[::-1]
            derivatives.append(vec)
        return derivatives
    
    def _append_derivatives(self, k: int, a: np.ndarray, b: np.ndarray) -> None:
        """
        Append derivatives of coefficients 'a' and 'b' to the path object.

        Parameters:
            k (int): The index of the derivative.
            a (np.ndarray): The derivative of coefficient 'a'.
            b (np.ndarray): The derivative of coefficient 'b'.

        Returns:
            None
        """
        if k < len(self.path.coeff.a_der):
            self.path.coeff.a_der[k].append(a)
            self.path.coeff.b_der[k].append(b)
        else:
            self.path.coeff.a_der.append([a])
            self.path.coeff.b_der.append([b])

    def _calculate_subpaths(self) -> None:
            """
            Calculates the subpaths of the hybrid path.

            This method constructs the A matrix, sets it as the A matrix of the path's linear system,
            and calculates the coefficients for each subpath. It then solves the linear system for each
            subpath to obtain the coefficients a and b. The derivatives of a and b are also calculated
            and appended to the path's derivatives.

            Returns:
                None
            """
            
            A = self._construct_A_matrix()
            self.path.LinSys.A = A

            N = self.path.NumSubpaths
            for j in range(N):
                ax, bx = self._calculate_subpath_coeffs(j)
                self.path.LinSys.bx = np.append(self.path.LinSys.bx, ax)
                self.path.LinSys.by = np.append(self.path.LinSys.by, bx)

                a_vec, b_vec = self.solve_linear_system(A, ax, bx)
                self.path.coeff.a.append(a_vec)
                self.path.coeff.b.append(b_vec)

                a_derivatives = self._calculate_derivatives(a_vec)
                b_derivatives = self._calculate_derivatives(b_vec)

                for k, (a, b) in enumerate(zip(a_derivatives, b_derivatives)):
                    self._append_derivatives(k, a, b)

class HybridPathSignals:
    """
    Given a path and the path parameter s, this class can get the position
    and derivatives of the path at the given point s.

    Args:
        Path (Path): The path object.
        s (float): The path parameter.

    Attributes:
        Path (Path): The path object.
        s (float): The path parameter.
    """
    def __init__(self, path: Path, s: float):
        if not isinstance(path, Path):
            raise TypeError("path must be an instance of Path")
        self.path = path
        self.s = self._clamp_s(s, self.path.NumSubpaths)

    def _clamp_s(self, s: float, num_subpaths: int) -> float:
        """
        Ensures s is within the valid range of [0, num_subpaths].

        Args:
            s (float): The path parameter.
            num_subpaths (int): The number of subpaths.

        Returns:
            float: The clamped path parameter.
        """
        if s < 0:
            return 0.0
        elif s >= num_subpaths:
            return num_subpaths - 1e-3
        return s

    
    def get_position(self) -> list[float]:
            """
            Get the position of the object along the path.

            Returns:
                list[float]: The x and y coordinates of the object's position.
            """
            index = int(self.s) + 1
            theta = self.s - (index - 1)
            theta_pow = theta ** np.arange(self.path.Order + 1)
            a = self.path.coeff.a[index - 1]
            b = self.path.coeff.b[index - 1]
            x_pos = np.dot(a, theta_pow)
            y_pos = np.dot(b, theta_pow)

            return [x_pos, y_pos]
    
    def _compute_derivatives(self, theta: float, a_vec: np.ndarray, b_vec: np.ndarray) -> list[float]:
        """
        Compute the derivatives of x and y.

        Parameters:
            theta (float): The value of theta.
            a_vec (np.ndarray): The vector of coefficients for x.
            b_vec (np.ndarray): The vector of coefficients for y.

        Returns:
            list[float]: A list containing the derivatives of x and y.

        """
        theta_pow = theta ** np.arange(len(a_vec))
        x_der = np.dot(a_vec, theta_pow)
        y_der = np.dot(b_vec, theta_pow)
        return [x_der, y_der]

    def get_derivatives(self) -> list[list[float]]:
        """
        Get the derivatives of the path at the current position.

        Returns:
            list: A list of computed derivatives.
        """
        derivatives = []
        index = int(self.s) + 1
        for k in range(1, self.path.Order + 1):
            a_vec = self.path.coeff.a_der[k - 1][index - 1]
            b_vec = self.path.coeff.b_der[k - 1][index - 1]
            derivatives.append(self._compute_derivatives(self.s, a_vec, b_vec))

        return derivatives
    
    def get_heading(self) -> float:
        """
        Get the heading of the object along the path.

        Returns:
            float: The heading of the object.
        """
        p_der = self.get_derivatives()[0]
        psi = np.arctan2(p_der[1], p_der[0])
        return psi
    
    def get_heading_derivative(self) -> float:
        """
        Get the derivative of the heading of the object along the path.

        Returns:
            float: The derivative of the heading.
        """
        p_der = self.get_derivatives()[0]
        p_dder = self.get_derivatives()[1]
        psi_der = (p_der[0] * p_dder[1] - p_der[1] * p_dder[0]) / (p_der[0]**2 + p_der[1]**2)
        return psi_der
    
    def get_heading_second_derivative(self) -> float:
        """
        Get the second derivative of the heading of the object along the path.

        Returns:
            float: The second derivative of the heading.
        """
        p_der = self.get_derivatives()[0]
        p_dder = self.get_derivatives()[1]
        p_ddder = self.get_derivatives()[2]
        psi_dder = (p_der[0] * p_ddder[1] - p_der[1] * p_ddder[0]) / (p_der[0]**2 + p_der[1]**2) - 2 * (p_der[0] * p_dder[1] - p_dder[0] * p_der[1]) * (p_der[0] * p_dder[0] - p_dder[1] * p_der[0]) / ((p_der[0]**2 + p_der[1]**2)**2)
        return psi_dder
    
    def get_vs(self, u_desired: float) -> float:
        """
        Calculate the reference velocity and its derivative.

        Args:
            u_desired (float): The desired velocity.

        Returns:
            float: The reference velocity.
        """
        p_der = self.get_derivatives()[0]
        v_s = u_desired / np.linalg.norm(p_der)
        return v_s
    
    def get_vs_derivative(self, u_desired: float) -> float:
        """
        Calculate the derivative of the reference velocity.

        Args:
            u_desired (float): The desired velocity.

        Returns:
            float: The derivative of the reference velocity.
        """
        p_der = self.get_derivatives()[0]
        p_dder = self.get_derivatives()[1]
        v_s_s = -u_desired * (np.dot(p_der, p_dder)) / (np.sqrt(p_der[0]**2 + p_der[1]**2)**3)
        return v_s_s
