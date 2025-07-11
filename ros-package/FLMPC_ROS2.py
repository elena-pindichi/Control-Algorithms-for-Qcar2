import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy
import numpy as np
import casadi
from motion_capture_tracking_interfaces.msg import NamedPoseArray
from qcar2_interfaces.msg import MotorCommands
from tf_transformations import euler_from_quaternion
from mpc_controller.gen_traj import get_ref
from scipy.spatial import ConvexHull
import time


class MPCControllerNode(Node):
    def __init__(self):
        super().__init__('mpc_virtual_node')

        qos_profile = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )
        self.callback_group = ReentrantCallbackGroup()

        self.pose_subscriber = self.create_subscription(
            NamedPoseArray,
            '/poses',
            self.poses_callback,
            qos_profile,
            callback_group=self.callback_group
        )

        self.cmd_publisher = self.create_publisher(
            MotorCommands,
            '/qcar2_motor_speed_cmd',
            10
        )

        msg_out = MotorCommands()
        msg_out.motor_names = ['steering_angle', 'motor_throttle']
        msg_out.values = [0.0, 0.0]
        self.cmd_publisher.publish(msg_out)

        self.target_name = 'qcar2'
        self.phi = 0.0
        self.last_time = None
        self.idx = 0
        self.dt = 0.3
        self.tf = 25

        self.DX = 4
        self.DZ = 2
        self.DU = 2
        self.NPRED = 5
        self.L = 0.256
        self.DELTA = 0.35

        self.curr_state = np.zeros((self.DX, 1))
        self.saved_states = []
        self.saved_inputs = []
        self.computation_times = []
        self.last_yaw = None
        self.phi = 0.0
        self.PHIMAX = np.pi / 5

        self.setup_mpc()
        self.timer = self.create_timer(self.dt, self.timer_callback)


    def LinOutput(self, x):
        return np.array([
            x[0] + self.L * np.cos(x[2]) + self.DELTA * np.cos(x[2] + x[3]),
            x[1] + self.L * np.sin(x[2]) + self.DELTA * np.sin(x[2] + x[3])
        ])

    def LinMatrix(self, eta):
        theta, phi = eta
        s1 = np.sin(theta + phi)
        c1 = np.cos(theta + phi)
        return np.array([
            [np.cos(theta) - np.tan(phi)*(np.sin(theta) + self.DELTA * s1 / self.L), -self.DELTA * s1],
            [np.sin(theta) + np.tan(phi)*(np.cos(theta) + self.DELTA * c1 / self.L),  self.DELTA * c1]
        ])

    def LinDyna(self, eta, L, Delta):
        s1 = casadi.sin(eta[0] + eta[1])
        c1 = casadi.cos(eta[0] + eta[1])
        O = casadi.vertcat(
            casadi.horzcat(casadi.sin(eta[1]) * c1 / L,             casadi.sin(eta[1]) * s1 / L),
            casadi.horzcat(-casadi.sin(eta[1]) * c1 / L - s1 / Delta, -casadi.sin(eta[1]) * s1 / L - c1 / Delta)
        )
        return O
    
    def InConstr(self, eta, l, Delta):
        s1 = casadi.sin(eta[0] + eta[1])
        c1 = casadi.cos(eta[0] + eta[1])
        s2 = casadi.sin(eta[0] + 2 * eta[1])
        c2 = casadi.cos(eta[0] + 2 * eta[1])

        L = casadi.vertcat(
            casadi.horzcat(-0.5 * (casadi.cos(eta[0]) + c2),              -0.5 * (casadi.sin(eta[0]) + s1)),
            casadi.horzcat(1 / (2 * l * Delta) * (Delta * s1 - Delta * casadi.sin(eta[0]) + 2 * l * s2),
                        1 / (2 * l * Delta) * (-Delta * c2 + Delta * casadi.cos(eta[0]) - 2 * l * c1)),
            casadi.horzcat(0.5 * (casadi.cos(eta[0]) + c2),               0.5 * (casadi.sin(eta[0]) + s1)),
            casadi.horzcat(-1 / (2 * l * Delta) * (Delta * s1 - Delta * casadi.sin(eta[0]) + 2 * l * s2),
                        -1 / (2 * l * Delta) * (-Delta * c2 + Delta * casadi.cos(eta[0]) - 2 * l * c1))
        )

        return L



    def setup_mpc(self):
        # Problem parameters
        Q = 300 * np.eye(self.DZ)
        R = np.diag([0.4, 5])
        P = 10 * Q

        # Q = 30 * np.eye(self.DZ)
        # R = np.diag([0.1, 5])
        # P = 10 * Q # pt lin cstr

        # Q = 1500 * np.eye(self.DZ)
        # R = np.diag([0.4, 5])
        # P = 10 * Q

        # for tests
        # Q = 300 * np.eye(self.DZ)
        # R = np.diag([2, 5])
        # P = 10 * Q

        self.solver = casadi.Opti()
        self.Z = self.solver.variable(self.DZ, self.NPRED)
        self.ETA = self.solver.variable(self.DZ, self.NPRED)
        self.W = self.solver.variable(self.DU, self.NPRED)

        self.ZINIT = self.solver.parameter(self.DZ, 1)
        self.ETAINIT = self.solver.parameter(self.DZ, 1)
        self.ZREF = self.solver.parameter(self.DZ, self.NPRED)
        self.WREF = self.solver.parameter(self.DU, self.NPRED)

        self.solver.subject_to(self.Z[:, 0] == self.ZINIT)
        self.solver.subject_to(self.ETA[:, 0] == self.ETAINIT)

        A = np.eye(self.DZ)
        B = self.dt * np.eye(self.DU)

        rhat = min(self.DELTA * self.L * 10 / np.sqrt(self.DELTA**2 + self.L**2), 0.6)

        angles = np.linspace(0, 2 * np.pi - 1e-4, 10)
        ptsU = np.array([[rhat * np.cos(theta), rhat * np.sin(theta)] for theta in angles])
        hull = ConvexHull(ptsU)
        U_approx = ptsU[hull.vertices]

        U_A = hull.equations[:, :-1]
        U_b = -hull.equations[:, -1]

        # Constraints for optimizarion problem
        for k in range(self.NPRED - 1):
            self.solver.subject_to(self.Z[:, k+1] == A @ self.Z[:, k] + B @ self.W[:, k])
            self.solver.subject_to(casadi.mtimes(self.W[:, k].T, self.W[:, k]) <= rhat)
            # O = self.LinDyna(self.ETA[:, k], self.L, self.DELTA)
            # self.solver.subject_to(self.ETA[:, k+1] == self.ETA[:, k] + self.dt * casadi.mtimes(O, self.W[:, k]))
            L = self.InConstr(self.ETA[:, k], self.L, self.DELTA)
            self.solver.subject_to(casadi.mtimes(L, self.W[:, k]) <= np.array([1,1,1,1]).T)
            # self.solver.subject_to(casadi.mtimes(U_A, self.W[:, k]) <= U_b)

        obj = 0
        for k in range(self.NPRED - 1):
            obj += casadi.mtimes([(self.Z[:, k] - self.ZREF[:, k]).T, Q, (self.Z[:, k] - self.ZREF[:, k])])
            obj += casadi.mtimes([(self.W[:, k] - self.WREF[:, k]).T, R, (self.W[:, k] - self.WREF[:, k])])
        obj += casadi.mtimes([(self.Z[:, -1] - self.ZREF[:, -1]).T, P, (self.Z[:, -1] - self.ZREF[:, -1])])
        self.solver.minimize(obj)
        self.solver.solver('ipopt', {'ipopt.print_level': 0, 'print_time': 0})
        self.create_reference_trajectory()

    def create_reference_trajectory(self):
        # Circle
        t = np.arange(0, self.tf, self.dt)

        # alpha   = 0.08
        # beta    = 0.08
        # l       = self.L

        # xr      = alpha * t
        # yr      = beta * t

        # dxr     = alpha + 0 * t
        # dyr     = beta + 0 * t

        # ddxr    = 0 + 0 * t
        # ddyr    = 0 + 0 * t

        # dddxr   = 0 + 0 * t
        # dddyr   = 0 + 0 * t

        alpha   = 1
        beta    = 1
        ang     = 0.5
        xr      = alpha*np.cos(ang*t)      
        dxr     = -alpha*ang*np.sin(ang*t)    
        ddxr    = -alpha*ang*ang*np.cos(ang*t)       
        dddxr   = alpha*ang*ang*ang*np.sin(ang*t)

        yr      = beta*np.sin(ang*t)       
        dyr     = beta*ang*np.cos(ang*t)      
        ddyr    = -beta*ang*ang*np.sin(ang*t)        
        dddyr   = -beta*ang*ang*ang*np.cos(ang*t)

        Vr      = np.sqrt(dxr ** 2 + dyr ** 2)

        epsilon = 1e-8
        Vr_safe = np.maximum(Vr, epsilon)

        numerator   = (dddyr * dxr - dddxr * dyr) * (Vr_safe ** 2) - 3 * (ddyr * dxr - ddxr * dyr) * (dxr * ddxr + dyr * ddyr)
        denominator = (Vr_safe ** 6 + (self.L ** 2) * (ddyr * dxr - ddxr * dyr) ** 2)
        omegar  = self.L * Vr_safe * numerator / denominator

        thetar  = np.unwrap(np.arctan2(dyr / Vr_safe, dxr / Vr_safe))

        phir    = np.arctan((self.L * (ddyr * dxr - ddxr * dyr)) / (Vr_safe ** 3))
            
        # Spline
        # ref = get_ref(psi=0, Tsim=self.tf, dt = self.dt)
        # omegar = ref["omegar"]
        # Vr = ref["Vr"]
        # epsilon = 1e-8
        # Vr_safe = np.maximum(Vr, epsilon)
        # XREF = ref["XREF_FULL"]

        # self.XREF_FULL = XREF
        # xr = self.XREF_FULL[0, :]
        # yr = self.XREF_FULL[1, :]
        # thetar = self.XREF_FULL[2, :]
        # phir = self.XREF_FULL[3, :]

        self.XREF_FULL = np.vstack([xr, yr, thetar, phir])
        self.UREF_FULL = np.vstack([Vr, omegar])

        self.ZREF_FULL = np.array([
            xr + self.L * np.cos(thetar) + self.DELTA * np.cos(thetar + phir),
            yr + self.L * np.sin(thetar) + self.DELTA * np.sin(thetar + phir)
        ])

        self.WREF_FULL = np.zeros((2, len(xr)))
        for i in range(len(xr)):
            M = self.LinMatrix([thetar[i], phir[i]])
            self.WREF_FULL[:, i] = M @ np.array([Vr[i], omegar[i]])

    def timer_callback(self):
        self.saved_states.append(self.curr_state.copy())

        if self.idx > self.tf / self.dt:
            msg_out = MotorCommands()
            msg_out.motor_names = ['steering_angle', 'motor_throttle']
            msg_out.values = [0.0, 0.0]
            self.cmd_publisher.publish(msg_out)
            self.get_logger().info("Trajectory complete. Robot stopped.")
            self.plot_reference_and_states()
            return

        if self.idx + self.NPRED > self.ZREF_FULL.shape[1]:
            try:
                remaining_z     = self.ZREF_FULL[:, self.idx:]
                pad_count_z     = max(self.NPRED - remaining_z.shape[1], 0)
                last_col_z      = self.ZREF_FULL[:, -1].reshape(self.DZ, 1)
                padding_z       = np.repeat(last_col_z, pad_count_z, axis=1)
                padded_zref     = np.hstack((remaining_z, padding_z))

                remaining_w     = self.WREF_FULL[:, self.idx:]
                pad_count_w     = max(self.NPRED - remaining_w.shape[1], 0)
                last_col_w      = self.WREF_FULL[:, -1].reshape(self.DU, 1)
                padding_w       = np.repeat(last_col_w, pad_count_w, axis=1)
                padded_wref     = np.hstack((remaining_w, padding_w))

                z_current       = self.LinOutput(self.curr_state[:, 0])
                self.solver.set_value(self.ZINIT, z_current)
                self.solver.set_value(self.ZREF, padded_zref)
                self.solver.set_value(self.WREF, padded_wref)
                eta = self.curr_state[2:4, 0]
                self.solver.set_value(self.ETAINIT, eta)

                self.get_logger().info("In the last part of the reference horizon")

                start_time = time.time()
                sol = self.solver.solve()
                computation_time = time.time() - start_time
                self.computation_times.append(computation_time)
                self.get_logger().info(f"MPC solve time (last part): {computation_time:.6f} seconds")
                w0      = sol.value(self.W[:, 0])
     
                M       = self.LinMatrix(eta)
                u0      = np.linalg.pinv(M) @ w0

                self.phi = self.integrate_phi(self.phi, u0[1], self.dt)      

                msg_out = MotorCommands()
                msg_out.motor_names = ['steering_angle', 'motor_throttle']
                msg_out.values = [self.phi, u0[0]]
                self.cmd_publisher.publish(msg_out)

                self.get_logger().info(f"MPC step: v={u0[0]:.2f}, phi={self.phi:.2f}")

                x_next = self.curr_state[:, 0] + self.dt * self.fdynamics(self.curr_state[:, 0], u0)
                self.curr_state = x_next

            except Exception as e:
                self.get_logger().error(f"MPC solve error: {str(e)}")

        else:
            try:
                z_current   = self.LinOutput(self.curr_state[:, 0])
                self.solver.set_value(self.ZINIT, z_current)
                self.solver.set_value(self.ZREF, self.ZREF_FULL[:, self.idx:self.idx + self.NPRED])
                self.solver.set_value(self.WREF, self.WREF_FULL[:, self.idx:self.idx + self.NPRED])
                eta = self.curr_state[2:4, 0]
                self.solver.set_value(self.ETAINIT, eta)

                start_time = time.time()
                sol = self.solver.solve()
                computation_time = time.time() - start_time
                self.computation_times.append(computation_time)
                self.get_logger().info(f"MPC solve time (last part): {computation_time:.6f} seconds")
                w0      = sol.value(self.W[:, 0])

                M       = self.LinMatrix(eta)
                u0      = np.linalg.pinv(M) @ w0

                self.phi = self.integrate_phi(self.phi, u0[1], self.dt)
                self.phi = self.phi

                msg_out = MotorCommands()
                msg_out.motor_names = ['steering_angle', 'motor_throttle']
                msg_out.values = [self.phi, u0[0]]
                self.cmd_publisher.publish(msg_out)

                self.get_logger().info(f"MPC step: v={u0[0]:.2f}, phi={self.phi:.2f}")

                x_next = self.curr_state[:, 0] + self.dt * self.fdynamics(self.curr_state[:, 0], u0)
                # self.curr_state = x_next.reshape(-1, 1)
                self.curr_state = x_next

            except Exception as e:
                self.get_logger().error(f"MPC solve error: {str(e)}")

        self.idx += 1
        self.saved_inputs.append(u0[0])


    def fdynamics(self, x, u):
        return np.array([
            u[0] * np.cos(x[2]),
            u[0] * np.sin(x[2]),
            u[0] * np.tan(x[3]) / self.L,
            u[1]
        ])

    def integrate_phi(self, phi_current, omega, dt):
        PHI_MAX = self.PHIMAX
        return np.clip(phi_current + omega * dt, -PHI_MAX, PHI_MAX)

    def poses_callback(self, msg):
        now = self.get_clock().now()

        if self.last_time is None:
            self.last_time = now
            return

        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now

        for named_pose in msg.poses:
            if named_pose.name == self.target_name:
                x = named_pose.pose.position.x
                y = named_pose.pose.position.y
                q = named_pose.pose.orientation
                _, _, raw_yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
                
                if self.last_yaw is None:
                    yaw = raw_yaw
                else:
                    yaw = self.last_yaw + np.unwrap([self.last_yaw, raw_yaw])[1] - self.last_yaw

                self.last_yaw = yaw 

                current_state = np.array([x, y, yaw, self.phi])
                self.curr_state = current_state

                self.curr_state = np.array([[x], [y], [yaw], [self.phi]])

    def save_data_npz(self, filename="flmpc_data.npz"):
        states = np.array(self.saved_states)
        inputs = np.array(self.saved_inputs)
        computation_times = np.array(self.computation_times)

        np.savez(filename,
                XREF_FULL=self.XREF_FULL,
                UREF_FULL=self.UREF_FULL,
                states=states,
                inputs=inputs,
                computation_times=computation_times)
        self.get_logger().info(f"Saved MPC data to {filename}")

    def plot_reference_and_states(self):
        import matplotlib.pyplot as plt
        states = np.array(self.saved_states)
        inputs = np.array(self.saved_inputs)
        t_len = states.shape[0]
        tin_len = inputs.shape[0]
        ref = self.XREF_FULL[:, :t_len]
        ref_in = self.UREF_FULL[0, :tin_len]

        self.save_data_npz("flmpc_data.npz")

        # plt.figure()
        plt.figure(figsize=(10, 8))
        plt.plot(states[:, 0], states[:, 1], label='Tracked Position')
        plt.plot(ref[0, :], ref[1, :], '--', label='Reference Position')
        plt.xlabel('x')
        plt.ylabel('y')
        plt.legend()
        plt.grid(True)
        plt.title("Reference vs States")
        plt.savefig('FLMPC_carpos.pdf', format='pdf', bbox_inches='tight')

        # plt.figure()
        plt.figure(figsize=(10, 8))
        plt.plot(states[:, 2], label='Yaw')
        plt.plot(ref[2, :], '--', label='Reference Yaw')
        plt.xlabel('time')
        plt.ylabel('theta')
        plt.legend()
        plt.grid(True)
        plt.title("Reference vs Yaw")
        plt.savefig('FLMPC_yaw.pdf', format='pdf', bbox_inches='tight')

        # plt.figure()
        plt.figure(figsize=(10, 8))
        plt.plot(states[:, 3], label='Phi')
        plt.plot(ref[3, :], '--', label='Reference Phi')
        plt.xlabel('time')
        plt.ylabel('phi')
        plt.legend()
        plt.grid(True)
        plt.title("Reference vs Phi")
        plt.savefig('FLMPC_phi.pdf', format='pdf', bbox_inches='tight')

        # plt.figure()
        plt.figure(figsize=(10, 8))
        plt.plot(inputs, label='V')
        plt.plot(ref_in, '--', label='Reference V')
        plt.grid(True)
        plt.title("Reference vs V")
        plt.savefig('FLMPC_v.pdf', format='pdf', bbox_inches='tight')

        plt.show()



def main(args=None):
    rclpy.init(args=args)
    node = MPCControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
