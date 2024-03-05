import rclpy
import numpy as np
from rclpy.node import Node
import matplotlib.pyplot as plt
from hybridpath_controller.adaptive_backstep import AdaptiveBackstep
from geometry_msgs.msg import Wrench
from vortex_msgs.msg import HybridpathReference

class HybridPathControllerNode(Node):
    def __init__(self):
        super().__init__("hybridpath_controller_node")
        
        self.hpref_subscriber_ = self.create_subscription(HybridpathReference, "guidance/hybridpath/reference", self.hpref_cb, 1)
        self.wrench_publisher_ = self.create_publisher(Wrench, "thrust/wrench_input", 1)

        self.declare_parameters(
            namespace='',
            parameters=[
                ('hybridpath_controller.kappa', 0.5)
            ])

        self.kappa = self.get_parameter('hybridpath_controller.kappa').get_parameter_value().double_value

        self.AB_controller = AdaptiveBackstep()

        self.get_logger().info("hybridpath_controller_node started")

    def hpref_cb(self, msg: HybridpathReference):
        w_ref = msg.w_ref
        v_ref = msg.v_ref
        v_ref_t = msg.v_ref_t
        v_ref_s = msg.v_ref_s

        eta = np.array([msg.eta.x, msg.eta.y, msg.eta.theta])
        nu = np.array([msg.nu.x, msg.nu.y, msg.nu.theta])
        eta_d = np.array([msg.eta_d.x, msg.eta_d.y, msg.eta_d.theta])
        eta_d_s = np.array([msg.eta_d_s.x, msg.eta_d_s.y, msg.eta_d_s.theta])
        eta_d_ss = np.array([msg.eta_d_ss.x, msg.eta_d_ss.y, msg.eta_d_ss.theta])

        tau = self.AB_controller.control_law(eta, nu, w_ref, v_ref, v_ref_t, v_ref_s, eta_d, eta_d_s, eta_d_ss)

        wrench_msg = Wrench()
        wrench_msg.force.x  = float(tau[0])
        wrench_msg.force.y  = float(tau[1])  # Assuming tau[1] is a single-element array
        wrench_msg.force.z  = 0.0
        wrench_msg.torque.x = 0.0
        wrench_msg.torque.y = 0.0
        wrench_msg.torque.z = float(tau[2])  # Assuming tau[2] is a single-element array

        self.wrench_publisher_.publish(wrench_msg)

    # def run_simulation(self, waypoints, T, dt):
    #     generator = HybridPathGenerator(waypoints, self.path_generator_order, self.lambda_val, 1)
    #     path = generator.Path
    #     time = np.arange(0, T, dt)

    #     # Initial vectors
    #     eta_d = np.zeros((3, len(time)))
    #     eta = np.zeros((3, len(time)))
    #     eta_d[:,0] = np.array([waypoints[0,0], waypoints[0,1], np.pi/2])
    #     eta[:,0] = np.array([waypoints[0,0], waypoints[0,1], np.pi/2])
    #     nu = np.zeros((3, len(time)))
    #     tau = np.zeros((3, len(time)))
    #     AB = AdaptiveBackstep()
    #     s = 0
    #     sig = HybridPathSignals(path, s)

    #     # Simulation loop
    #     for i in range(1,len(time)):
    #         if time[i] < self.time_to_max_speed :
    #             u_desired = 1/10 * time[i]
    #         else:
    #             u_desired = 1 # Desired speed
    #         v_ref, v_ref_s = sig.calc_vs(u_desired)
    #         s += v_ref * dt
    #         sig = HybridPathSignals(path, s)
    #         pd = sig.pd # Reference position
    #         psi_d = sig.psi # Reference heading
    #         eta_d[:,i] = np.array([pd[0], pd[1], psi_d])
    #         psi0 = eta_d[2,i-1]
    #         psi1 = eta_d[2,i]
    #         psi_vec = np.array([psi0, psi1])
    #         psi_vec = np.unwrap(psi_vec, period=2*np.pi)
    #         eta_d[2,i] = psi_vec[1]

    #         # Variables needed for tau
    #         #w = 0
    #         v_ref_t = 0 # Constant speed
    #         eta_d_s = np.array([sig.pd_der[0][0], sig.pd_der[0][1], sig.psi_der])
    #         eta_d_ss = np.array([sig.pd_der[1][0], sig.pd_der[1][1], sig.psi_dder])
    #         R, R_trsp = AB.R(eta[2,i-1])
    #         eta_error = eta[:,i-1] - eta_d[:,i]
    #         my = 0.5
    #         w = my * (eta_d_s @ R @ (R_trsp @ eta_error)) / (np.linalg.norm(eta_d_s)**2)

    #         tau[:,i] = AB.control_law(eta[:,i-1], nu[:,i-1], w, v_ref, v_ref_t, v_ref_s, eta_d[:,i], eta_d_s, eta_d_ss)
            
    #         # Step in nu and eta
    #         nu_dot = np.linalg.inv(AB.M) @ tau[:, i] - np.linalg.inv(AB.M) @ AB.D @ nu[:,i-1]
    #         nu[:,i] = nu[:,i-1] + nu_dot * dt
    #         eta_dot = R @ nu[:,i]
    #         eta[:,i] = eta[:,i-1] + eta_dot * dt

    #         psi0 = eta[2,i-1]
    #         psi1 = eta[2,i]
    #         psi_vec = np.array([psi0, psi1])
    #         psi_vec = np.unwrap(psi_vec, period=2*np.pi)
    #         eta[2,i] = psi_vec[1]
        
    #     # Plotting
    #     plt.figure()
    #     plt.plot(eta_d[1,:], eta_d[0,:], label='Reference path', zorder = 0)
    #     plt.plot(eta[1,:], eta[0,:], label='Actual path', zorder = 1)
    #     for i in range(0, len(eta_d[2]), 100):
    #         plt.quiver(eta[1,i], eta[0,i], np.sin(eta[2,i]), np.cos(eta[2,i]), zorder = 2)
    #     plt.title('Actual path vs reference path')
    #     plt.xlabel('y [m]')
    #     plt.ylabel('x [m]')
    #     plt.gca().set_axisbelow(True)
    #     plt.grid()
    #     plt.legend()

    #     plt.figure()
    #     plt.plot(time, eta[0,:], label='Actual x')
    #     plt.plot(time, eta_d[0,:], label='Reference x')
    #     plt.plot(time, eta[1,:], label='Actual y')
    #     plt.plot(time, eta_d[1,:], label='Reference y')
    #     plt.title('Actual position vs reference position')
    #     plt.xlabel('Time [s]')
    #     plt.ylabel('Position [m]')
    #     plt.gca().set_axisbelow(True)
    #     plt.grid()
    #     plt.legend()

    #     plt.figure()
    #     plt.plot(time, eta[2,:], label='Actual heading')
    #     plt.plot(time, eta_d[2,:], label='Reference heading')
    #     plt.title('Actual heading vs reference heading')
    #     plt.xlabel('Time [s]')
    #     plt.ylabel('Heading [rad]')
    #     plt.gca().set_axisbelow(True)
    #     plt.grid()
    #     plt.legend()

    #     plt.figure()
    #     plt.plot(time, nu[0,:], label='Surge velocity')
    #     plt.plot(time, nu[1,:], label='Sway velocity')
    #     plt.title('velocity')
    #     plt.xlabel('Time [s]')
    #     plt.ylabel('Velocity [m/s]')
    #     plt.gca().set_axisbelow(True)
    #     plt.grid()
    #     plt.legend()

    #     plt.figure()
    #     plt.plot(time, tau[0,:], label='Surge force')
    #     plt.plot(time, tau[1,:], label='Sway force')
    #     plt.title('Force')
    #     plt.xlabel('Time [s]')
    #     plt.ylabel('Force [N]')
    #     plt.gca().set_axisbelow(True)
    #     plt.grid()
    #     plt.legend()

    #     plt.show()

def main(args=None):
    rclpy.init(args=args)
    node = HybridPathControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
