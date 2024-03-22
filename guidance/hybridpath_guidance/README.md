# Hybrid Path Guidance

This package provides the implementation of hybrid path guidance for the Vortex ASV.

## Usage

To use the hybrid path guidance launch it using: `ros2 launch hybridpath_guidance hybridpath_guidance.launch`
Or alternatively, run it together with the hybridpath controller using the launch file `hybridpath.launch.py` in asv_setup 

## Configuration

You can configure the behavior of the hybrid path guidance by modifying the parameters in the `config` directory. 

## Theory
### Path generation
Given a set of $n + 1$ waypoints, the hybridpath generator will generate a $C^r$ path that goes through the waypoints. That is, the path is continuous in $r$ derivatives. Towards constructing the overall desired path $p_d(s)$, it is first divided into $n$ subpaths $p_{d,i}(s)$ for $i = 1, ... , n$ between the waypoints. Each of these is expressed as a polynomial in $s$ of a certain order. Then the expressions for the subpaths are concatenated at the waypoints to assemble the full path. The order of the polynomials must be chosen sufficiently high to ensure the overall path is sufficiently differentiable at the waypoints. Increasing the order of the polynomial increases the number of coefficients giving more degrees-of-freedom to satisfy these continuity constraints.

The variable $s \in [0,n)$ is called the path variable, and it tells us exactly where we are on the path. For example, $s$ will be $0$ at the first waypoint, $1$ at the second, and so on. However, using the path parameter to take values $s \in [0,n)$ is unnecessary and can be numerically problematic. Instead, we can identify each path segment $p_i$ and parametrize it by $\theta \in [0,1)$. We define 
```math
i := \lfloor s \rfloor + 1
``` 
```math
\theta = s - \lfloor s \rfloor \in [0,1)
```

to get the continuous map 
```math 
s \mapsto p_d(s)
```

Consider polynomials of order $k$, 
```math
x_{d,i}(\theta) = a_{k,i} \theta^k + ... + a_{1,i} \theta + a_{0,i}
```
```math
 y_{d,i}(\theta) = b_{k,i} \theta^k + ... + b_{1,i} \theta + b_{0,i}
```
where the coefficients {$`a_{j,i}`$, $`b_{j,i}`$} must be determined. For each subpath there are $(k + 1)$ $\cdot$ $2$ unknowns so that there are $(k + 1)$ $\cdot$ $2n$ unknown coefficients in total to be determined for the full path. This is done by setting up and solving the linear system 
```math
A\phi = b \; \; \; \; \; \; \phi^T = [a^T, b^T]
```
for each path segment and solve them in a single operation as $` \phi = A^{-1}b`$.

Assuming that the subpaths is to be connected at the waypoints, we use the following procedure to calculate the coefficients of each subpath:

$C^0$ : Continuity at the waypoints gives for i: 
```math
x_{d,i}(0) = x_{i-1} \; \; \; \; \; \; y_{d,i}(0) = y_{i-1}
```
```math
 x_{d,i}(1) = x_i  \; \; \; \; \; \; y_{d,i}(1) = y_i
```

$C^1$ : The slopes at the first and last waypoints are chosen as: 
```math
x^\theta_{d,i}(0) = x_1 - x_0 \; \; \; \; \; \; y^\theta_{d,1}(0) = y_1 - y_0
```
```math
 x^\theta_{d,n}(1) = x_n - x_{n-1} \; \; \; \; \; \; y^\theta_{d,n}(1) = y_n - y_{n-1}
``` 

Here, $x^\theta$ represents the derivative of $x$ with respect to $\theta$.

The slopes at the intermediate waypoints are chosen as: 
```math
 x^\theta_{d,i}(0) = \lambda (x_i - x_{i-2}) \; \; \; \; \; \; i = 2, ... , n
```
```math
 y^\theta_{d,i}(0) = \lambda (y_i - y_{i-2}) \; \; \; \; \; \; i = 2, ... , n
```
```math
 x^\theta_{d,i}(1) = \lambda (x_{i+1} - x_{i-1}) \; \; \; \; \; \; i = 1, ... , n-1
```
```math
 y^\theta_{d,1}(1) = \lambda (y_{i+1} - y_{i-1}) \; \; \; \; \; \; i = 1, ... , n-1
```
where $\lambda > 0$ is a design constant. Choosing $\lambda > 0.5$ gives more rounded corners, choosing $\lambda < 0.5$ gives sharper corners and choosing $\lambda = 0.5$ makes the slope at waypoint $p_i$ be the average of pointing to $p_{i-1}$ and $p_{i+1}$

$C^j$ : Setting derivatives of order $j = 2, 3, ... , k$ to zero for i:
```math
 x^{\theta^j}_{d,i}(0) = 0 \; \; \; \; \; \; y^{\theta^j}_{d,i}(0) = 0
```
```math
x^{\theta^j}_{d,i}(1) = 0 \; \; \; \; \; \; y^{\theta^j}_{d,i}(1) = 0
```
This results in the hybrid parametrization of the path,
```math
\bar{p_d}(i,\theta) = \begin{bmatrix} x_{d,i}(\theta) \\ y_{d,i}(\theta) \end{bmatrix}
```

where $\theta \in [0,1)$. 

If the differentiability requirement of the path is $C^r$, then the above equations up to $j=r$ gives $2(r+1) \cdot 2n$ equations to solve for $(k + 1) \cdot 2n$ unknown coefficients. As a result, the order $k$ of the polynomials must be 
```math
k = 2r + 1
```

### Guidance system
So far we have defined the desired path 
```math
p_d(i,\theta) = \begin{bmatrix} x_d(i,\theta) \\ y_d(i,\theta) \end{bmatrix},\; \theta \in [0,1)
```
. Now we define the desired heading 
```math
\psi_d = atan2(y^\theta_d(i,\theta), x^\theta_d(i,\theta))
```
where $atan2(y,x)$ is the four-quadrant version of $arctan(y/x)$. As mentioned previously, the pair $(i,\theta)$ tells us which path segment $i$ we are on and where on the segment we are (given by $\theta$). Since we also need the first and second derivatives of the heading for the controller, we define them as:
```math
\psi^\theta_d(i,\theta) = \frac{x^\theta_d(i,\theta)y^{\theta^2}_d(i,\theta)-x^{\theta^2}_d(i,\theta)y^\theta_d(i,\theta)}{x^\theta_d(i,\theta)^2+y^\theta_d(i,\theta)^2}
```

Differentiating one more time will get messy, but here it is:

```math
\psi^{\theta^2}_d(i,\theta) = \frac{x^\theta_d(i,\theta)y^{\theta^3}_d(i,\theta)-x^{\theta^3}_d(i,\theta)y^\theta_d(i,\theta)}{x^\theta_d(i,\theta)^2+y^\theta_d(i,\theta)^2} \\-2\frac{(x^\theta_d(i,\theta)y^{\theta^2}_d(i,\theta)-x^{\theta^2}_d(i,\theta)y^\theta_d(i,\theta))(x^\theta_d(i,\theta)x^{\theta^2}_d(i,\theta)-y^{\theta^2}_d(i,\theta)y^\theta_d(i,\theta))}{(x^\theta_d(i,\theta)^2+y^\theta_d(i,\theta)^2)^2}
```

### Example plots

These plots showcases the what the $r$ and $\lambda$ parameters does to the path.

![Path Plots](https://drive.google.com/uc?export=download&id=1CE8kN1yJSjEgiCdWGQYgjdIfSkE40V1f)

![Lambda Plots](https://drive.google.com/uc?export=download&id=1TtpIAwBRKcZhpuXAUEinpuqUhN-WKoX-)
