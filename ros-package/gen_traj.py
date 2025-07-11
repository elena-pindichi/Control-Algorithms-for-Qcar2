import casadi
import numpy as np
import mpc_controller.Bspline_conversionMatrix as BsplineM
from scipy.interpolate import BSpline
from scipy.integrate import quad
import matplotlib.pyplot as plt


def get_ref(psi, Tsim, dt):
    k = 8  # polynomial degree
    n_ctrl_pts = 88
    knot = [0, Tsim]
    g = 9.81

    knot = BsplineM.knot_vector(k, n_ctrl_pts, knot)
    tt = np.arange(min(knot), max(knot), dt)
    bs_list = BsplineM.b_spline_basis_functions(n_ctrl_pts, k, knot)

    # Conversion matrices for derivatives
    M = BsplineM.bsplineConversionMatrices(n_ctrl_pts, k, knot)

    # 2D Waypoints: shape (2, N)
    # W = np.array([[0, 0.3, 0.6, 0.6, 0.3, 0, -0.3, -0.3, 0],
    #               [0, -0.3, 0, 0.3, 0.6, 0.6, 0.3, 0, 0]])
    W = np.array([[0, 0.8, 1, 1, 0.8, 0, -0.2, -0.2, 0],
                  [0,   0, 0.3, 0.8, 1, 1, 0.8, 0.3, 0]])
    # W = np.array([[0, 0.6, 1],
    #               [0, 0.6, 0.6]])

    # W = np.array([[0, 0.4, 0.6, 0.8, 1],
    #               [0, 0.7, 0.4 , 0.4, 0]])
    W = np.array([[0, 0.5,  0.7, 0.9,   1, 1.2, 1.5,  1.8],
                  [0, 0.08, 0.1, 0, -0.2,-0.3, -0.4, -0.3]])
    W = np.array([[-1, -0.7, -0.5, -0.2, 0,0.5,  1,  1.2, 1.5, 1.6,  1.8, 1.8, 1.3, 1, 0.8, 0.5,0.2, -0.1, -0.4, -0.7, -1],
                  [0, 0.2, 0.4,  0.8 ,  1, 1, 0.6,0.4,  0, -0.2, -0.5, -0.7, -1, -1, -1, -1, -1, -1, -1, -1, -1]])
    W = np.array([[-1, -0.7, -0.5, -0.2, 0,0.5,  1,  1.2, 1.5, 1.6,  1.7, 1.7,  1.2, 0.6,0,-0.4,-1],
                  [0, 0.2, 0.4,  0.8 ,  1, 1, 0.6,0.4,  0, -0.2, -0.5, -0.7,  -1,  -1,-0.8,  -0.4, 0]])

    waypoint_time_stamps = np.linspace(min(knot), max(knot), W.shape[1])

    ######################## Optimization problem ########################
    solver = casadi.Opti()
    P = solver.variable(W.shape[0], n_ctrl_pts)

    objective = 0
    P1 = casadi.mtimes(P, M[0])
    for i in range(n_ctrl_pts + 1):
        for j in range(n_ctrl_pts + 1):
            def f_lamb(t, it=i, jt=j): return bs_list[1][it](t) * bs_list[1][jt](t)
            buff_int = quad(f_lamb, min(knot), max(knot))[0]
            objective += casadi.mtimes(casadi.transpose(casadi.mtimes(buff_int, P1[:, i])), P1[:, j])

    # Waypoint constraints
    for i in range(W.shape[1]):
        tmp_bs = np.zeros((len(bs_list[0]), 1))
        for j in range(len(bs_list[0])):
            tmp_bs[j] = bs_list[0][j](waypoint_time_stamps[i])
        solver.subject_to(casadi.mtimes(P, tmp_bs) == W[:, i])

    # Final velocity = 0
    for i in range(W.shape[1]):
        tmp_bs = np.zeros((len(bs_list[1]), 1))
        for j in range(len(bs_list[1])):
            tmp_bs[j] = bs_list[1][j](waypoint_time_stamps[-1] - dt)
        solver.subject_to(casadi.mtimes(P1, tmp_bs) == 0)

    # Final acceleration = 0
    P2 = casadi.mtimes(P, M[1])
    for i in range(W.shape[1]):
        tmp_bs = np.zeros((len(bs_list[2]), 1))
        for j in range(len(bs_list[2])):
            tmp_bs[j] = bs_list[2][j](waypoint_time_stamps[-1] - dt)
        solver.subject_to(casadi.mtimes(P2, tmp_bs) == 0)
    
    P3 = casadi.mtimes(P, M[2])
    for i in range(W.shape[1]):
        tmp_bs = np.zeros((len(bs_list[3]), 1))
        for j in range(len(bs_list[3])):
            tmp_bs[j] = bs_list[3][j](waypoint_time_stamps[-1] - dt)
        solver.subject_to(casadi.mtimes(P3, tmp_bs) == 0)

    solver.minimize(objective)
    solver_options = {'ipopt': {'print_level': 0, 'sb': 'yes'}, 'print_time': 0}
    solver.solver('ipopt', solver_options)

    print('Generating 2D reference trajectory ...')
    sol = solver.solve()
    P = sol.value(P)
    print('Optimal control points found.')

    # Bspline evaluation
    z = [BSpline(knot, P[i], k) for i in range(P.shape[0])]
    P1 = np.array(P @ M[0])
    z_d = [BSpline(knot, P1[i], k - 1) for i in range(P1.shape[0])]
    P2 = np.array(P @ M[1])
    z_dd = [BSpline(knot, P2[i], k - 2) for i in range(P2.shape[0])]

    x = z[0](tt)
    y = z[1](tt)
    dx = z_d[0](tt)
    dy = z_d[1](tt)
    ddx = z_dd[0](tt)
    ddy = z_dd[1](tt)

    # Compute 3rd derivatives (jerk)
    P3 = np.array(P @ M[2])
    z_ddd = [BSpline(knot, P3[i], k - 3) for i in range(P3.shape[0])]
    dddx = z_ddd[0](tt)
    dddy = z_ddd[1](tt)

    # Compute Vr (velocity magnitude)
    Vr = np.sqrt(dx**2 + dy**2)
    epsilon = 1e-8
    Vr_safe = np.maximum(Vr, epsilon)

    # Parameters
    l = 0.256

    # Numerator and denominator for omegar
    numerator = (dddy * dx - dddx * dy) * (Vr_safe**2) - 3 * (ddy * dx - ddx * dy) * (dx * ddx + dy * ddy)
    denominator = (Vr_safe**6 + (l**2) * (ddy * dx - ddx * dy)**2)
    omegar = l * Vr_safe * numerator / denominator

    thetar = np.unwrap(np.arctan2(dy / Vr_safe, dx / Vr_safe))

    phir = np.arctan((l * (ddy * dx - ddx * dy)) / (Vr_safe**3))

    XREF_FULL = np.vstack([x, y, thetar, phir])

    ref = {
    "trajectory": np.round(np.stack([x, y, dx, dy]), 3).T,
    "time_step": tt,
    "Nsim": tt.shape[0],
    "v_ref": np.stack([ddx, ddy]).T,
    "Vr": Vr,
    "omegar": omegar,
    "thetar": thetar,
    "phir": phir,
    "XREF_FULL": XREF_FULL
    }

    return ref


# # Generate reference
# rref = get_ref(psi=0, Tsim=30, dt=0.1)

# # Plotting 2D Trajectory
# plt.figure()
# plt.plot(rref["trajectory"][:, 0], rref["trajectory"][:, 1], label="2D Trajectory")
# plt.xlabel("X")
# plt.ylabel("Y")
# plt.title("Optimized 2D B-Spline Trajectory")
# plt.axis("equal")
# plt.grid(True)
# plt.legend()
# plt.show()
    