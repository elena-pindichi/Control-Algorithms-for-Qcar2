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
import time


class MPCControllerNode(Node):
    def __init__(self):
        super().__init__('mpc_controller_node')

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

        self.saved_states = []
        self.saved_inputs = []
        self.computation_times = []
        self.last_yaw = None
        self.target_name = 'qcar2'
        self.phi = 0.0
        self.last_time = None
        self.idx = 0
        self.curr_state = np.zeros((4, 1))
        self.dt = 0.3
        self.tf = 25
        self.PHIMAX = np.pi / 5
        self.VMAX = 0.8
        self.WMAX = 0.8
        self.NPRED = 12
        
        self.setup_mpc()    
        self.timer = self.create_timer(self.dt, self.timer_callback)

    def timer_callback(self):
        self.saved_states.append(self.curr_state.copy())
        if self.idx > self.tf / self.dt:
            msg_out = MotorCommands()
            msg_out.motor_names = ['steering_angle', 'motor_throttle']
            msg_out.values = [0.0, 0.0]
            self.cmd_publisher.publish(msg_out)
            self.get_logger().info(f"I stopped")
            self.plot_reference_and_states()
            return

        if self.idx + self.NPRED > self.tf / self.dt:
            try:
                remaining = self.XREF_FULL[:, self.idx:]
                pad_count = max(self.NPRED - remaining.shape[1], 0)
                last_col = self.XREF_FULL[:, -1].reshape(4,1)
                padding = np.repeat(last_col, pad_count, axis=1)
                padded_ref = np.hstack((remaining, padding))

                remaining_u = self.UREF_FULL[:, self.idx:]
                pad_count_u = max(self.NPRED - remaining_u.shape[1], 0)
                last_col_u = self.UREF_FULL[:, -1].reshape(2,1)
                padding_u = np.repeat(last_col_u, pad_count_u, axis=1)
                padded_uref = np.hstack((remaining_u, padding_u))

                self.solver.set_value(self.XINIT, self.curr_state)
                self.solver.set_value(self.XREF, padded_ref)
                self.solver.set_value(self.UREF, padded_uref)

                self.get_logger().info(f"I am in the last part of the reference ")
                start_time = time.time()
                sol = self.solver.solve()
                computation_time = time.time() - start_time
                self.computation_times.append(computation_time)
                self.get_logger().info(f"MPC solve time (last part): {computation_time:.6f} seconds")
                u0 = sol.value(self.U[:, 0])

                self.phi = self.integrate_phi(self.phi, u0[1], self.dt)

                msg_out = MotorCommands()
                msg_out.motor_names = ['steering_angle', 'motor_throttle']
                msg_out.values = [self.phi, u0[0]]
                self.cmd_publisher.publish(msg_out)

                self.get_logger().info(f"MPC: v={u0[0]:.2f}, phi={self.phi:.2f}")
            except Exception as e:
                self.get_logger().error(f"MPC failed: {str(e)}")
        else:
            try:
                self.solver.set_value(self.XINIT, self.curr_state)
                self.solver.set_value(self.XREF, self.XREF_FULL[:, self.idx:(self.idx + self.NPRED)])
                self.solver.set_value(self.UREF, self.UREF_FULL[:, self.idx:(self.idx + self.NPRED)])

                start_time = time.time()
                sol = self.solver.solve()
                computation_time = time.time() - start_time
                self.get_logger().info(f"MPC solve time: {computation_time:.6f} seconds")
                self.computation_times.append(computation_time)
                u0 = sol.value(self.U[:, 0])

                self.phi = self.integrate_phi(self.phi, u0[1], self.dt)

                msg_out = MotorCommands()
                msg_out.motor_names = ['steering_angle', 'motor_throttle']
                msg_out.values = [self.phi, u0[0]]
                self.cmd_publisher.publish(msg_out)

                self.get_logger().info(f"MPC: v={u0[0]:.2f}, phi={self.phi:.2f}")
            except Exception as e:
                self.get_logger().error(f"MPC failed: {str(e)}")
        self.idx += 1
        self.saved_inputs.append(u0[0])

    def setup_mpc(self):
        
        self.L = 0.256 

        DX, DU = 4, 2
        self.solver = casadi.Opti()

        self.X = self.solver.variable(DX, self.NPRED)
        self.U = self.solver.variable(DU, self.NPRED)

        self.XINIT = self.solver.parameter(DX, 1)
        self.XREF = self.solver.parameter(DX, self.NPRED)
        self.UREF = self.solver.parameter(DU, self.NPRED)

        self.solver.subject_to(self.X[:, 0] == self.XINIT)

        PHIMAX = self.PHIMAX
        VMAX = self.VMAX
        WMAX = self.WMAX

        def fdynamics(x, u):
            return casadi.vertcat(
                u[0] * casadi.cos(x[2]),
                u[0] * casadi.sin(x[2]),
                u[0] * casadi.tan(x[3]) / self.L,
                u[1]
            )

        for k in range(self.NPRED - 1):
            self.solver.subject_to(self.X[:, k + 1] == self.X[:, k] + self.dt * fdynamics(self.X[:, k], self.U[:, k]))
            self.solver.subject_to(self.X[3, k + 1] >= -PHIMAX)
            self.solver.subject_to(self.X[3, k + 1] <= PHIMAX)
            self.solver.subject_to(self.U[0, k] >= -VMAX)
            self.solver.subject_to(self.U[0, k] <= VMAX)
            self.solver.subject_to(self.U[1, k] >= -WMAX)
            self.solver.subject_to(self.U[1, k] <= WMAX)

        Q = 600 * np.eye(DX)
        Q[2,2] = 15
        Q[3,3] = 0.1
        R = 5 * np.eye(DU)
        R = np.diag([10, 5])
        P = 10 * Q


        # for tests
        # Q = 300 * np.eye(DX)
        # Q[2,2] = 15
        # Q[3,3] = 0.1
        # R = 5 * np.eye(DU)
        # R = np.diag([2, 5])
        # P = 10 * Q

        obj = 0
        for k in range(self.NPRED - 1):
            obj += casadi.mtimes([(self.X[:, k] - self.XREF[:, k]).T, Q, (self.X[:, k] - self.XREF[:, k])])
            obj += casadi.mtimes([(self.U[:, k] - self.UREF[:, k]).T, R, (self.U[:, k] - self.UREF[:, k])])
        obj += casadi.mtimes([(self.X[:, -1] - self.XREF[:, -1]).T, P, (self.X[:, -1] - self.XREF[:, -1])])
        self.solver.minimize(obj)

        self.solver.solver('ipopt', {'ipopt.print_level': 0, 'print_time': 0})

        # Call the separated function to create reference trajectory
        self.create_reference_trajectory()

    def create_reference_trajectory(self):
        # Circle
        t = np.arange(0, self.tf, self.dt)
        t_const = np.repeat(1,np.size(t))
        alpha = 0.08
        beta = 0.08
        l = self.L

        # Positions
        xr = alpha * t
        yr = beta * t

        # First derivatives (velocities)
        dxr = alpha + 0 * t
        dyr = beta + 0 * t

        # Second derivatives (accelerations)
        ddxr = 0 + 0 * t
        ddyr = 0 + 0 * t

        # Third derivatives (jerks)
        dddxr = 0 + 0 * t
        dddyr = 0 + 0 * t

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


        # xr      = 1.2 * t_const
        # dxr     = 0   * t_const
        # ddxr    = 0   * t_const 
        # dddxr   = 0   * t_const

        # yr      = 1  * t_const
        # dyr     = 0  * t_const 
        # ddyr    = 0  * t_const  
        # dddyr   = 0  * t_const

        # Compute velocity magnitude Vr
        Vr = np.sqrt(dxr ** 2 + dyr ** 2)

        # Avoid division by zero by adding a small epsilon
        epsilon = 1e-8
        Vr_safe = np.maximum(Vr, epsilon)

        # Compute omegar 
        numerator = (dddyr * dxr - dddxr * dyr) * (Vr_safe ** 2) - 3 * (ddyr * dxr - ddxr * dyr) * (dxr * ddxr + dyr * ddyr)
        denominator = (Vr_safe ** 6 + (l ** 2) * (ddyr * dxr - ddxr * dyr) ** 2)
        omegar = l * Vr_safe * numerator / denominator

        # Compute thetar (angle reference)
        thetar = np.unwrap(np.arctan2(dyr / Vr_safe, dxr / Vr_safe))

        # Compute phir (steering angle reference)
        phir = np.arctan((l * (ddyr * dxr - ddxr * dyr)) / (Vr_safe ** 3))

        # Spline references
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

    def save_data_npz(self, filename="nmpc_data.npz"):
        states = np.array(self.saved_states)
        inputs = np.array(self.saved_inputs)
        computation_times = np.array(self.computation_times)

        # Save everything in a compressed NumPy archive
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

        self.save_data_npz("nmpc_data.npz")

        # plt.figure()
        plt.figure(figsize=(10, 8))
        plt.plot(states[:, 0], states[:, 1], label='Tracked Position')
        plt.plot(ref[0, :], ref[1, :], '--', label='Reference Position')
        plt.xlabel('x')
        plt.ylabel('y')
        plt.legend()
        plt.grid(True)
        plt.title("Reference vs States")
        plt.savefig('NMPC_carpos.pdf', format='pdf', bbox_inches='tight')

        # plt.figure()
        plt.figure(figsize=(10, 8))
        plt.plot(states[:, 2], label='Yaw')
        plt.plot(ref[2, :], '--', label='Reference Yaw')
        plt.xlabel('time')
        plt.ylabel('theta')
        plt.legend()
        plt.grid(True)
        plt.title("Reference vs Yaw")
        plt.savefig('NMPC_yaw.pdf', format='pdf', bbox_inches='tight')

        # plt.figure()
        plt.figure(figsize=(10, 8))
        plt.plot(states[:, 3], label='Phi')
        plt.plot(ref[3, :], '--', label='Reference Phi')
        plt.xlabel('time')
        plt.ylabel('phi')
        plt.legend()
        plt.grid(True)
        plt.title("Reference vs Phi")
        plt.savefig('NMPC_phi.pdf', format='pdf', bbox_inches='tight')

        # plt.figure()
        plt.figure(figsize=(10, 8))
        plt.plot(inputs, label='V')
        plt.plot(ref_in, '--', label='Reference V')
        plt.grid(True)
        plt.title("Reference vs V")
        plt.savefig('NMPC_V.pdf', format='pdf', bbox_inches='tight')

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