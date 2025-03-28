from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
from geometry_msgs.msg import Point


@dataclass
class LinSys:
    """A class to represent the linear system.

    Attributes:
        coeff_matrix: The coefficients matrix.
        bx: The x-vector for each subpath.
        by: The y-vector for each subpath.
    """

    coeff_matrix: np.ndarray = None
    bx: list[np.ndarray] = field(default_factory=list)
    by: list[np.ndarray] = field(default_factory=list)


@dataclass
class Coeff:
    """A class to represent the coefficients.

    Attributes:
        a: The x-coefficients.
        b: The y-coefficients.
        a_der: The x-coefficients for derivatives.
        b_der: The y-coefficients for derivatives.
    """

    a: list[np.ndarray] = field(default_factory=list)
    b: list[np.ndarray] = field(default_factory=list)
    a_der: list[list[np.ndarray]] = field(default_factory=list)
    b_der: list[list[np.ndarray]] = field(default_factory=list)


@dataclass
class Path:
    """A class to represent the path.

    Attributes:
        NumSubpaths (int): The number of subpaths.
        Order (int): The order of the polynomials.
        LinSys (LinSys): The linear system to solve the subpaths.
        coeff (Coeff): The coefficients.
    """

    NumSubpaths: int = 0
    Order: int = 0
    LinSys: LinSys = LinSys()
    coeff: Coeff = Coeff()


class HybridPathGenerator:
    """This class generates a path that goes through a set of waypoints.

    Args:
        waypoints (list[Point]): A list of waypoints.
        r (int): The differentiability order.
        lambda_val (float): The curvature constant.

    Attributes:
        waypoints (np.ndarray): The waypoints.
        r (int): The differentiability order.
        lambda_val (float): The curvature constant.
        order (int): The order of the polynomials.
        path (Path): The path object.
    """

    def __init__(self, r: int, lambda_val: float):
        self.r = r
        self.lambda_val = lambda_val
        self.order = int(2 * r + 1)

    def create_path(self, waypoints: list[Point]) -> None:
        """Create a path object.

        Args:
            waypoints (list[Point]): A list of waypoints.

        Returns:
            None
        """
        self.update_waypoints(waypoints)
        self._initialize_path()
        self._calculate_subpaths()

    def _initialize_path(self):
        """Initialize the path object."""
        self.path = Path()

        self.path.coeff.a = []
        self.path.coeff.b = []
        self.path.coeff.a_der = []
        self.path.coeff.b_der = []
        self.path.LinSys.coeff_matrix = None
        self.path.LinSys.bx = []
        self.path.LinSys.by = []

        self.path.NumSubpaths = len(self.waypoints) - 1
        self.path.Order = self.order

    def update_waypoints(self, waypoints: list[Point]) -> None:
        """Updates the waypoints for the path.

        Args:
            waypoints (list[Point]): A list of waypoints.
        """
        # Convert the waypoints to a numpy array
        waypoint_arr = np.array([[wp.x, wp.y] for wp in waypoints])

        if (
            len(waypoint_arr) == 2
        ):  # The generator must have at least 3 waypoints to work
            self.waypoints = np.array(
                [
                    waypoint_arr[0],
                    [
                        (waypoint_arr[0][0] + waypoint_arr[1][0]) / 2,
                        (waypoint_arr[0][1] + waypoint_arr[1][1]) / 2,
                    ],
                    waypoint_arr[1],
                ]
            )

        elif len(waypoint_arr) < 2:
            raise ValueError("Please input at least 2 waypoints.")

        else:
            self.waypoints = waypoint_arr

    def _construct_coeff_matrix(self) -> np.ndarray:
        """Constructs and returns the coeff matrix.

        Returns:
            np.ndarray: The coeff matrix.
        """
        ord_plus_one = self.order + 1

        # Initialize the coeff matrix
        coeff_matrix = np.zeros((ord_plus_one, ord_plus_one))

        # Create an array of ones
        coefs = np.ones(ord_plus_one)

        # Set the first and second row of the A matrix
        coeff_matrix[0, 0] = coefs[0]
        coeff_matrix[1, :] = coefs

        # Copy the coefficients array to work with derivatives
        c_der = coefs.copy()

        # Loop through the rest of the rows
        for k in range(2, (self.order + 1) // 2 + 1):
            # Calculate the polynomial derivative, reverse it for correct alignment, and reverse back after differentiation
            c_der = np.polyder(c_der[::-1])[::-1]

            # Concatenate zeros to align derivatives at the correct order position
            coefs = np.concatenate([np.zeros(k - 1), c_der])

            # Update the coeff_matrix matrix with derivatives
            coeff_matrix[2 * k - 2, k - 1] = c_der[0]

            # Set every k-th row according to the adjusted coefficients
            coeff_matrix[2 * k - 1, :] = coefs

        return coeff_matrix

    def _calculate_subpath_coeffs(self, j: int) -> tuple[np.ndarray, np.ndarray]:
        """Calculate the subpath coefficients for a given index.

        Parameters:
        j (int): The index of the subpath.

        Returns:
        tuple[np.ndarray, np.ndarray]: A tuple containing two numpy arrays representing the subpath coefficients.
        """
        order_plus_one = self.order + 1
        num_subpaths = self.path.NumSubpaths

        # Initialize the coefficient arrays
        ax, bx = np.zeros(order_plus_one), np.zeros(order_plus_one)

        # Set the first two coefficients for each dimension directly from waypoints. See README under C^0 continuity.
        ax[:2] = self.waypoints[j : j + 2, 0]
        bx[:2] = self.waypoints[j : j + 2, 1]

        # Calculate the coefficients for subpaths when the order is greater than 2 i.e. C^1 continuity.
        if self.order > 2:
            self._calculate_subpaths_coeffs_if_ord_greater_than_2(
                ax, bx, j, num_subpaths
            )

        return ax, bx

    def _calculate_subpaths_coeffs_if_ord_greater_than_2(
        self, ax: np.ndarray, bx: np.ndarray, j: int, num_subpaths: int
    ) -> None:
        """Calculate the coefficients for subpaths when the order is greater than 2.

        See the README for the details, C^1 continuity.

        Args:
            ax (np.ndarray): Array to store the x-coefficients.
            bx (np.ndarray): Array to store the y-coefficients.
            j (int): Index of the current subpath.
            num_subpaths (int): Total number of subpaths.

        Returns:
            None
        """
        # Set the coefficients for the subpath if we are on the first subpath i.e. j = 0
        if j == 0:
            ax[2:4] = [
                self.waypoints[j + 1, 0] - self.waypoints[j, 0],
                self.lambda_val * (self.waypoints[j + 2, 0] - self.waypoints[j, 0]),
            ]
            bx[2:4] = [
                self.waypoints[j + 1, 1] - self.waypoints[j, 1],
                self.lambda_val * (self.waypoints[j + 2, 1] - self.waypoints[j, 1]),
            ]

        # Set the coefficients for the subpath if we are on the last subpath i.e. j = num_subpaths - 1
        elif j == num_subpaths - 1:
            ax[2:4] = [
                self.lambda_val * (self.waypoints[j + 1, 0] - self.waypoints[j - 1, 0]),
                self.waypoints[j + 1, 0] - self.waypoints[j, 0],
            ]
            bx[2:4] = [
                self.lambda_val * (self.waypoints[j + 1, 1] - self.waypoints[j - 1, 1]),
                self.waypoints[j + 1, 1] - self.waypoints[j, 1],
            ]

        # Set the coefficients for the subpath if we are on any other subpath
        else:
            ax[2:4] = [
                self.lambda_val * (self.waypoints[j + 1, 0] - self.waypoints[j - 1, 0]),
                self.lambda_val * (self.waypoints[j + 2, 0] - self.waypoints[j, 0]),
            ]
            bx[2:4] = [
                self.lambda_val * (self.waypoints[j + 1, 1] - self.waypoints[j - 1, 1]),
                self.lambda_val * (self.waypoints[j + 2, 1] - self.waypoints[j, 1]),
            ]

    def _solve_linear_system(
        self, coeff_matrix: np.ndarray, ax: np.ndarray, bx: np.ndarray
    ) -> tuple[np.ndarray, np.ndarray]:
        """Solves the linear system for the given subpath.

        Args:
            coeff_matrix (np.ndarray): The coeff matrix.
            ax (np.ndarray): The x-vector for the subpath.
            bx (np.ndarray): The y-vector for the subpath.

        Returns:
            tuple[np.ndarray, np.ndarray]: The coefficients a and b for the subpath.
        """
        a = np.linalg.solve(coeff_matrix, ax)
        b = np.linalg.solve(coeff_matrix, bx)

        return a, b

    def _calculate_derivatives(self, vec: np.ndarray) -> np.ndarray:
        """Calculate the derivatives of a given vector.

        Parameters:
        vec (np.ndarray): The input vector.

        Returns:
        np.ndarray: A list of arrays representing the derivatives of the input vector.
        """
        derivatives = []  # Initialize the list to store the derivatives

        # Loop through 1 to the order of the polynomial
        for _ in range(1, self.order + 1):
            # Calculate the derivative of the polynomial vector. The vector is reversed for calculation,
            # then reversed back to maintain the original order of coefficients after differentiation.
            vec = np.polyder(vec[::-1])[::-1]

            # Append the derivative vector to the list
            derivatives.append(vec)

        return derivatives

    def _append_derivatives(self, k: int, a: np.ndarray, b: np.ndarray) -> None:
        """Append derivatives of coefficients 'a' and 'b' to the path object.

        Parameters:
            k (int): The index of the derivative.
            a (np.ndarray): The derivative of coefficient 'a'.
            b (np.ndarray): The derivative of coefficient 'b'.

        Returns:
            None
        """
        # Check if the index k for the derivative is within the current length of the derivatives list
        if k < len(self.path.coeff.a_der):
            # If the index is within the current bounds, append the new derivative to the existing list at index k
            self.path.coeff.a_der[k].append(a)
            self.path.coeff.b_der[k].append(b)
        else:
            # If the index is beyond the current list size, it means this is a new derivative order being introduced.
            # Start a new list for this order and append
            self.path.coeff.a_der.append([a])
            self.path.coeff.b_der.append([b])

    def _calculate_subpaths(self) -> None:
        """Calculates the subpaths of the hybrid path.

        This method constructs the A matrix, sets it as the A matrix of the path's linear system,
        and calculates the coefficients for each subpath. It then solves the linear system for each
        subpath to obtain the coefficients a and b. The derivatives of a and b are also calculated
        and appended to the path's derivatives.

        Returns:
            None
        """
        # Construct the A matrix
        coeff_matrix = self._construct_coeff_matrix()
        self.path.LinSys.coeff_matrix = coeff_matrix

        # Loop through each subpath
        num_subpaths = self.path.NumSubpaths
        for j in range(num_subpaths):
            # Calculate the subpath coefficients
            ax, bx = self._calculate_subpath_coeffs(j)
            self.path.LinSys.bx.append(ax)
            self.path.LinSys.by.append(bx)

            # Solve the linear system for the subpath
            a_vec, b_vec = self._solve_linear_system(coeff_matrix, ax, bx)
            self.path.coeff.a.append(a_vec)
            self.path.coeff.b.append(b_vec)

            # Calculate the derivatives of the coefficients
            a_derivatives = self._calculate_derivatives(a_vec)
            b_derivatives = self._calculate_derivatives(b_vec)

            # Append the derivatives to the path object
            for k, (a, b) in enumerate(zip(a_derivatives, b_derivatives)):
                self._append_derivatives(k, a, b)

    def get_path(self) -> Path:
        """Get the hybrid path."""
        return self.path

    @staticmethod
    def update_s(path: Path, dt: float, u_desired: float, s: float, w: float) -> float:
        """Update the position along the hybrid path based on the desired velocity and time step.

        Args:
            path (Path): The hybrid path.
            dt (float): The time step.
            u_desired (float): The desired velocity.
            s (float): The current position along the hybrid path.
            w (float): The unit-tangent gradient update law.

        Returns:
            float: The updated position along the hybrid path.

        """
        signals = HybridPathSignals()
        signals.update_path(path)
        signals.update_s(s)
        v_s = signals.get_vs(u_desired)
        s_new = s + (v_s + w) * dt
        return s_new


class HybridPathSignals:
    """Computes position and derivatives of a path given a path parameter s.

    This class retrieves the position, heading, and derivatives of a given path
    at a specified path parameter s.

    Args:
        Path (Path): The path object.
        s (float): The path parameter.

    Attributes:
        Path (Path): The path object.
        s (float): The path parameter.
    """

    def __init__(self):
        self.path = None
        self.s = None

    @staticmethod
    def _clamp_s(s: float, num_subpaths: int) -> float:
        """Ensures s is within the valid range of [0, num_subpaths].

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
        """Get the position of the object along the path.

        Returns:
            list[float]: The x and y coordinates of the object's position.
        """
        # Get the index and theta. See README for what they represent.
        index = int(self.s) + 1
        theta = self.s - (index - 1)

        # Calculate the position using the coefficients for the polynomial for the given subpath
        theta_pow = theta ** np.arange(self.path.Order + 1)
        a = self.path.coeff.a[index - 1]
        b = self.path.coeff.b[index - 1]
        x_pos = np.dot(a, theta_pow)
        y_pos = np.dot(b, theta_pow)

        return [x_pos, y_pos]

    def _compute_derivatives(
        self, theta: float, a_vec: np.ndarray, b_vec: np.ndarray
    ) -> list[float]:
        """Compute the derivatives of x and y.

        Parameters:
            theta (float): The value of theta.
            a_vec (np.ndarray): The vector of coefficients for x.
            b_vec (np.ndarray): The vector of coefficients for y.

        Returns:
            list[float]: A list containing the derivatives of x and y.

        """
        # Calculate the derivatives of x and y using the coefficients for the polynomial for the given subpath
        theta_pow = theta ** np.arange(len(a_vec))
        x_der = np.dot(a_vec, theta_pow)
        y_der = np.dot(b_vec, theta_pow)
        return [x_der, y_der]

    def get_derivatives(self) -> list[list[float]]:
        """Get the derivatives of the path at the current position.

        Returns:
            list: A list of computed derivatives.
        """
        # Initialize the list to store the derivatives
        derivatives = []

        # Get the index and theta. See README for what they represent.
        index = int(self.s) + 1
        theta = self.s - (index - 1)

        # Loop through the order of the polynomial and calculate the derivatives for each order
        for k in range(1, self.path.Order + 1):
            a_vec = self.path.coeff.a_der[k - 1][index - 1]
            b_vec = self.path.coeff.b_der[k - 1][index - 1]
            derivatives.append(self._compute_derivatives(theta, a_vec, b_vec))

        return derivatives

    def get_heading(self) -> float:
        """Get the heading of the object along the path.

        Returns:
            float: The heading of the object.
        """
        # Get the first derivative of the position
        p_der = self.get_derivatives()[0]

        # Calculate the heading
        psi = np.arctan2(p_der[1], p_der[0])
        return psi

    def get_heading_derivative(self) -> float:
        """Get the derivative of the heading of the object along the path.

        Returns:
            float: The derivative of the heading.
        """
        # Get the first and second derivatives of the position
        p_der = self.get_derivatives()[0]
        p_dder = self.get_derivatives()[1]

        # Calculate the derivative of the heading
        psi_der = (p_der[0] * p_dder[1] - p_der[1] * p_dder[0]) / (
            p_der[0] ** 2 + p_der[1] ** 2
        )
        return psi_der

    def get_heading_second_derivative(self) -> float:
        """Get the second derivative of the heading of the object along the path.

        Returns:
            float: The second derivative of the heading.
        """
        # Get the first, second and third derivatives of the position
        p_der = self.get_derivatives()[0]
        p_dder = self.get_derivatives()[1]
        p_ddder = self.get_derivatives()[2]

        # Calculate the second derivative of the heading
        psi_dder = (p_der[0] * p_ddder[1] - p_der[1] * p_ddder[0]) / (
            p_der[0] ** 2 + p_der[1] ** 2
        ) - 2 * (p_der[0] * p_dder[1] - p_dder[0] * p_der[1]) * (
            p_der[0] * p_dder[0] - p_dder[1] * p_der[0]
        ) / ((p_der[0] ** 2 + p_der[1] ** 2) ** 2)
        return psi_dder

    def get_vs(self, u_desired) -> float:
        """Calculate the reference velocity.

        Args:
            u_desired (float): The desired velocity.

        Returns:
            float: The reference velocity.
        """
        # Get the first derivative of the position
        p_der = self.get_derivatives()[0]

        # Calculate the reference velocity
        v_s = u_desired / np.linalg.norm(p_der)
        return v_s

    def get_vs_derivative(self, u_desired) -> float:
        """Calculate the derivative of the reference velocity.

        Args:
            u_desired (float): The desired velocity.

        Returns:
            float: The derivative of the reference velocity.
        """
        # Get the first and second derivatives of the position
        p_der = self.get_derivatives()[0]
        p_dder = self.get_derivatives()[1]

        # Calculate the derivative of the reference velocity
        v_s_s = (
            -u_desired
            * (np.dot(p_der, p_dder))
            / (np.sqrt(p_der[0] ** 2 + p_der[1] ** 2) ** 3)
        )
        return v_s_s

    def get_w(self, mu: float, eta: np.ndarray) -> float:
        """Calculates and returns the value of the unit-tangent gradient update law based on the given parameters.

        See chapter 3.4 in https://folk.ntnu.no/rskjetne/Publications/SkjetnePhDthesis_B5_compressed.pdf for more details.

        Parameters:
        mu (float): A tuning value.
        eta (np.ndarray): An array representing the position and heading of the vessel.

        Returns:
        float: The calculated value of w.
        """
        # Get the desired position and heading
        eta_d = np.array(
            [self.get_position()[0], self.get_position()[1], self.get_heading()]
        )

        # Compute the error
        error = eta - eta_d

        # Get the first derivative of the position and the heading
        p_der = self.get_derivatives()[0]
        psi_der = self.get_heading_derivative()
        eta_d_s = np.array([p_der[0], p_der[1], psi_der])

        # Compute the unit-tangent gradient
        w = (mu / np.linalg.norm(eta_d_s)) * np.dot(eta_d_s, error)

        return w

    def update_path(self, path: Path) -> None:
        """Update the path object.

        Args:
            path (Path): The new path object.

        Returns:
            None
        """
        self.path = path
        self.update_s(0.0)

    def update_s(self, s: float) -> None:
        """Update the path parameter.

        Args:
            s (float): The new path parameter.

        Returns:
            None
        """
        self.s = self._clamp_s(s, self.path.NumSubpaths)
