# Hybrid Path Controller

This package provides the implementation of hybrid path controller for the Vortex ASV.

## Usage

To use the hybrid path guidance launch it using: `ros2 launch hybridpath_controller hybridpath_controller.launch`
Or alternatively, run it together with the hybridpath guidance using the launch file `hybridpath.launch.py` in asv_setup 

## Configuration

You can configure the behavior of the hybrid path controller by modifying the parameters in the `config` directory. 

## Theory

### Vessel Model
For our controller we use the simplified 3 degree-of-freedom (DOF) model
$$\dot{\eta} = \textbf{R}(\psi) \nu$$
$$\textbf{M}\dot{\nu} = -\textbf{D}\nu + \tau$$

where $\nu = [u,v,r]^T$ is the velocity of the vessel in the body-fixed frame {b} and $\eta = [p,\psi]^T = [x,y,\psi]^T$ is the position and heading of the vessel given in the North-East-Down frame {n}. $\textbf{M}$ is the inertia matrix, $\textbf{D}$ is the linear damping matrix and $\tau$ is the control input in {n}. $\textbf{R}(\psi)$ is the 3DOF rotation matrix with the property that $\dot{\textbf{R}}(\psi) = \textbf{R}\textbf{S}(r)$ where $\textbf{S}(r)$ is skew-symmetric. Thus:
```math
\textbf{R}(\psi) := \begin{bmatrix} \cos\psi & -\sin\psi & 0 \\ \sin\psi & \cos\psi & 0 \\ 0 & 0 & 1
\end{bmatrix}
```
```math
\textbf{S}(r) := \begin{bmatrix} 0 & -r & 0 \\ r & 0 & 0 \\ 0 & 0 & 0
\end{bmatrix}
```

The control input $\tau$ contains the thrust forces acting on the vessel:
```math
\tau = \begin{bmatrix} \tau_X \\ \tau_Y \\ \tau_N \end{bmatrix} = \begin{bmatrix} F_X \\ F_Y \\ l_x F_Y - l_y F_X \end{bmatrix}
```

where $F_X$ and $F_Y$ are the forces in the respective indexed directions and $l_x$ and $l_y$ are the arms from which $\tau_N$ are acting on. 

### The maneuvering problem
The maneuvering problem for an ASV is comprimised of two tasks:
1. $\textbf{Geometric task:}$ Given a desired pose $\eta_d(s(t))$, force the vessels pose $\eta(t)$ to converge to the desired path: $$\lim_{{t \to \infty}} [\eta(t) - \eta_d(s(t))] = 0$$

2. $\textbf{Dynamic task:}$ Satisfy one or more of the following assignments:
    1. $\textbf{Speed assignment:}$ Force the path speed $\dot{s}(t)$ to converge to the desired speed $v_s(t, s(t))$:
    $$\lim_{{t \to \infty}} [\dot{s}(t) - v_s(t, s(t))] = 0$$

    2. $\textbf{Time assignment:}$ Force the path variable $s$ to converge to a desired time signal $v_t(t)$:
    $$\lim_{{t \to \infty}} [s(t) - v_t()t,s(t)] = 0$$

The maneuvering problem highlights the superior goal of convergence to the path and to fulfill the dynamic assignment. We will be considering only the geometric task and the speed assignment. 

For the speed assignment, the guidance system needs to generate the speed profile $`v_s(t,s(t))`$ and its derivatives. We let the desired path speed $`u_d(t)`$ (in m/s) be a commanded input speed. We have that: 
```math
|\dot{p}_d(s(t))| = \sqrt{x^s_d(s(t))^2\dot{s}(t)^2 + y^s_d(s(t))^2\dot{s}(t)^2} \\ = \sqrt{x^s_d(s(t))^2 + y^s_d(s(t))^2} |v_s(t,s(t))| = |u_d(t)|
```
which must hold along the path. The speed assignment is thus, by definition:
```math
v_s(t,s(t)) := \frac{u_d(t)}{\sqrt{x^s_d(s(t))^2 + y^s_d(s(t))^2}}
```
Setting the commanded input $`u_d(t) = 0`$ m/s will stop the vessel on the path, while setting $`u_d(t) > 0`$ m/s will move the vessel in positive direction along the path and $`u_d(t) < 0`$ m/s will move it in negative direction.

Combining the generated path from [the path generator](../../guidance/hybridpath_guidance/README.md) with a dynamic assignment along the path ensures that dynamic and temporal requirements are satisfied. The maneuvering problem states that the geometry of the path and the dynamical behaviour along the path can be defined and controlled separately. This means that a path can be generated, and the speed can be controlled without having to regenerate a new path.
### Nonlinear adaptive backstepping
To design the maneuvering controller, we use the nonlinear backstepping technique. In short, backstepping is a recursive technique breaking the design problem of the full system down to a sequence of sub-problems on lower-order systems, and by recursively use some states as virtual control inputs to obtain the intermediate control laws using the appropriate Control Lyapunov Functions (CLF). The design is done in two steps. After the first step, we define the dynamic update law for the parametric value $\dot{s}(t)$ to fulfill the control objective.

#### Step 1
We define the error state variables:
```math
z_1 := \textbf{R}(\psi)^T(\eta - \eta_d(s)), \\z_2 := \nu - \alpha_1, \\ \omega := \dot{s} - v_s(t,s),
```
where $`\alpha_1`$ is the virtual control to be specified later. The total derivative of $`z_1`$ is:
```math
\dot{z}_1 = \dot{\textbf{R}}(\psi)^T(\eta - \eta_d(s)) + \textbf{R}(\psi)^T(\dot{\eta} - \eta^s_d(s)\dot{s})
```
```math
 = -\textbf{S}(r)\textbf{R}(\psi)^T(\eta-\eta_d(s)) + \nu - \textbf{R}(\psi)^T\eta^s_d(s)\dot{s}
```
```math
 = -\textbf{S}(r)z_1 + z_2 + \alpha_1 - \textbf{R}(\psi)^T\eta^s_d(s)(\omega + v_s(t,s))
```

The first CLF is defined as:
```math
V_1 := \frac{1}{2}z^T_1z_1
```
and its total derivative:
```math
\dot{V}_1 = \frac{1}{2}\dot{z}^T_1z_1 + \frac{1}{2}z^T_1\dot{z}_1
 ```
```math
= \frac{1}{2}(-\textbf{S}(r)z_1 + z_2 + \alpha_1 - \textbf{R}(\psi)^T\eta^s_d(s)(\omega + v_s(t,s)))^Tz_1
```
```math
 + \frac{1}{2}z^T_1(-\textbf{S}(r)z_1 + z_2 + \alpha_1 - \textbf{R}(\psi)^T\eta^s_d(s)(\omega + v_s(t,s)))
```
```math
 = \frac{1}{2}(-z^T_1\textbf{S}(r)z_1 + z^T_1z_2 + z^T_1\alpha_1
```
```math
 -z^T_1[\textbf{R}(\psi)^T\eta^s_d(s)(\omega + v_s(t,s))] - z^T_1\textbf{S}(r)z_1 + z^T_1z_2 + z^T_1\alpha_1
```
```math
 -z^T_1[\textbf{R}(\psi)^T\eta^s_d(s)(\omega + v_s(t,s))])
```
```math
 = z^T_1z_2 + z^T_1[\alpha_1 - \textbf{R}(\psi)^T\eta^s_d(s)(\omega + v_s(t,s))]
```

and futhermore, its derivative with respect to $s$ is:
```math
V^s_1 = \frac{1}{2} z^{s\:T}z_1 + \frac{1}{2}z^T_1z^s_1
```
```math
= \frac{1}{2}(-\textbf{R}(\psi)^T\eta^s_d(s))^Tz_1 + \frac{1}{2}z^T_1(-\textbf{R}(\psi)^T\eta^s_d(s))
```
```math
= -z^T_1 \textbf{R}(\psi)^T\eta^s_d(s)
```
Now we use Young's inequality, such that:
```math
\dot{V}_1 = z^T_1z_2 + z^T_1[\alpha_1 - \textbf{R}(\psi)^T\eta^s_d(s)(\omega + v_s(t,s))]
```
```math
\leq \frac{1}{4\kappa}z^T_2z_2 + z^T_1[\kappa z_1 + \alpha_1 - \textbf{R}(\psi)^T\eta^s_d(s)(\omega + v_s(t,s))]
```
We choose our first virtual control $\alpha_1$ and tuning function $\rho_1$ as:
```math
\alpha_1 = -\textbf{K}_1z_1 + \textbf{R}(\psi)^T\eta^s_d(s)v_s(t,s) - \kappa z_1, \; \textbf{K}_1 = \textbf{K}^T_1 > 0, \; \kappa > 0
```
```math
\rho_1 = -z^T_1 \textbf{R}(\psi)^T\eta^s_d(s) = V^s_1
```
Plugging in $\alpha_1$ and $\rho_1$ into the inequality gives:
```math
\dot{V}_1 \leq -z^T_1 \textbf{K}_1 z_1 + \rho_1 \omega + \frac{1}{4\kappa}z^T_2 z_2
```
where we postpone dealing with the coupling term involving $z_2$ until the next step.

#### Dynamic Update Law Acting in Output Space:
The dynamic update law is constructed to bridge the path following objective with the speed assignment. We examine two candidates so that the hybrid path signal being sent from the guidance system can be controlled.
- Tracking update law: Choosing:
```math
\omega = 0 \Rightarrow \dot{s} = v_s(t,s)
```
satisfies the dynamic task. This also gives:
```math
\dot{V}_1 \leq -z^T_1 \textbf{K}_1 z_1.
```
Hence the tracking update law renders $\dot{V}_1$ negative definite and the equilibrium $z_1 = 0$ uniformly globally exponentially stable (UGES).

#### Step 2
The second CLF is defined as:
```math
V_2 := V_1 + \frac{1}{2} z^T_2 \textbf{M} z_2
```
and its total derivative:
```math
\dot{V}_2 = \dot{V}_1 + z^T_2 \textbf{M} \dot{z}_2
```
```math
= -z^T_1 \textbf{K} z_1 + \frac{1}{4\kappa}z^T_2 z_2 + z^T_2(\textbf{M}\dot{\nu}- \textbf{M}\dot{\alpha}_1)
```
Plugging in our 3DOF model for $\textbf{M}\dot{\nu}$ we obtain:
```math
\dot{V}_2 = -z^T_1 \textbf{K}_1 z_1 + \frac{1}{4\kappa}z^T_2 z_2
+ z^T_2(-\textbf{D}\nu + \tau - \textbf{M}\alpha_1)
```
We now choose $\tau$ to stabilize our second CLF:
```math
\tau = -\textbf{K}_2 z_2 + \textbf{D}\nu + \textbf{M}\dot{\alpha}_1, \; \textbf{K}_2 = \textbf{K}^T_2 > 0
```
And plugging into the equation for $\dot{V}_2$:
```math
\dot{V}_2 \leq -z^T_1 \textbf{K}_1 z_1 - z^T_2 (\textbf{K}_2 - \frac{1}{4\kappa}z_2) \leq 0
```
Thus $\tau$ renders $\dot{V}_2$ negative definite and thereby the equilibrium point $(z_1, z_2) = (0,0)$ UGES. Finally we must find an expression for $\dot{\alpha}_1$ since it appears in the expression for $\tau$.
```math
\dot{\alpha}_1 = \textbf{K}_1 \dot{z}_1 + \dot{\textbf{R}}(\psi)^T\eta^s_d(s)v_s(t,s) + \textbf{R}(\psi)^T\dot{\eta}^s_d(s)v_s(t,s) + \textbf{R}(\psi)^T\eta^s_d(s)\dot{v}_s(t,s)
```
```math
= -\textbf{K}_1 \dot{z}_1 - \textbf{S}(r)\textbf{R}(psi)^T\eta^s_d(s)v_s(t,s) + \textbf{R}(\psi)^T\eta^{s^2}_d(s)\dot{s}v_s(t,s) + \textbf{R}(\psi)^T \eta^s_d(s)(v^t_s(t, s) + v^s_s(t, s)\dot{s})
```
By plugging in for $\dot{z}_1$ the expression can be further simplified:
```math
\dot{\alpha}_1 = \textbf{K}_1 \textbf{S}(r)z_1 - \textbf{K}_1\nu - \textbf{S}(r)\textbf{R}(\psi)^T\eta^s_d(s)v_s(t,s)
```
```math
+ \textbf{R}(\psi)^T\eta^s_d(s) v^t_s(t,s) + [\textbf{K}_1 \textbf{R}(\psi)^T\eta^s_d(s) + \textbf{R}(\psi)^T\eta^{s^2}(s)v_s(t,s)
```
```math
+ \textbf{R}(\psi)^T\eta^s_d(s)v^s_s(t,s)]\dot{s}
```
The terms inside the square brackets constitutes $\alpha^s_1$. If we now define:
```math
\sigma_1 := \textbf{K}_1 \textbf{S}(r)z_1 - \textbf{K}_1\nu - \textbf{S}(r)\textbf{R}(\psi)^T\eta^s_d(s)v_s(t,s) + \textbf{R}(\psi)^T\eta^s_d(s)v^t_s(t,s)
```
We can write:
```math
\dot{\alpha}_1 = \sigma_1 + \alpha^s_1\dot{s}
```