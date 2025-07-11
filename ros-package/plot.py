import numpy as np
import matplotlib.pyplot as plt

# Load the data
NMPC_data = np.load("nmpc_data.npz")
FLMPC_data = np.load("flmpc_data.npz")

NMPC_XREF = NMPC_data["XREF_FULL"]
NMPC_UREF = NMPC_data["UREF_FULL"]
NMPC_states = NMPC_data["states"]
NMPC_inputs = NMPC_data["inputs"]
NMPC_computation = NMPC_data["computation_times"]

FLMPC_XREF = FLMPC_data["XREF_FULL"]
FLMPC_UREF = FLMPC_data["UREF_FULL"]
FLMPC_states = FLMPC_data["states"]
FLMPC_inputs = FLMPC_data["inputs"]
FLMPC_computation = FLMPC_data["computation_times"]

dt = 0.3
time = dt * np.arange(NMPC_states.shape[0])
# print(time.shape)
# print(FLMPC_XREF.shape)
# print(FLMPC_states.shape)
# print(NMPC_XREF.shape)
# print(NMPC_states.shape)
# # print(NMPC_UREF.shape)
# print(len(NMPC_inputs))

xref = NMPC_XREF.T
err1 = np.mean((NMPC_states[:84,0] - xref[:,0])**2, axis = 0)
err2 = np.mean((NMPC_states[:84,1] - xref[:,1])**2, axis = 0)
nmse = (err1+err2)/2
print(f"MSE Error for NMPC: {nmse:.6f}")


err1 = np.mean((FLMPC_states[:84,0,0] - xref[:,0])**2, axis = 0)
err2 = np.mean((FLMPC_states[:84,1,0] - xref[:,1])**2, axis = 0)
flmse = (err1+err2)/2
print(f"MSE Error for FLMPC: {flmse:.6f}")


comp_nmpc = sum(NMPC_computation) / len(NMPC_computation)
print(f"Computation time for NMPC problem: {comp_nmpc:.6f} seconds")

comp_flmpc = sum(FLMPC_computation) / len(FLMPC_computation)
print(f"Computation time for FLMPC problem: {comp_flmpc:.6f} seconds")


# Plot trajectories
# plt.figure()
plt.figure(figsize=(10, 8))
plt.plot(NMPC_states[:, 0], NMPC_states[:, 1], label="NMPC Tracked")
plt.plot(FLMPC_states[:, 0], FLMPC_states[:, 1], label="FLMPC Tracked")
plt.plot(NMPC_XREF[0], NMPC_XREF[1], "--", label="Reference")
plt.grid(True)
plt.xlabel("x(m)")
plt.ylabel("y(m)")
plt.legend()
plt.title("Trajectory Vs Ref")
plt.savefig('circle.pdf', format='pdf', bbox_inches='tight')

# Plot yaw
# plt.figure()
plt.figure(figsize=(10, 8))
plt.plot(time,NMPC_states[:, 2], label="NMPC_Yaw")
plt.plot(time,FLMPC_states[:, 2], label="FLMPC_Yaw")
plt.plot(time[:84],NMPC_XREF[2], '--', label="Yaw ref")
plt.grid(True)
plt.legend()
plt.title("Yaw vs Ref")
plt.savefig('circle_yaw.pdf', format='pdf', bbox_inches='tight')

# Plot phi
# plt.figure()
plt.figure(figsize=(10, 8))
plt.plot(time,NMPC_states[:, 3], label="NMPC_Phi")
plt.plot(time,FLMPC_states[:, 3], label="FLMPC_Phi")
plt.plot(time[:84],NMPC_XREF[3], '--', label="Phi ref")
plt.grid(True)
plt.legend()
plt.title("Phi vs Ref")
plt.savefig('circle_phi.pdf', format='pdf', bbox_inches='tight')

# Plot velocity
# plt.figure()
plt.figure(figsize=(10, 8))
plt.plot(time[:84],NMPC_inputs, label="NMPC Input V")
plt.plot(time[:84],FLMPC_inputs, label="FLMPC Input V")
plt.plot(time[:84],NMPC_UREF[0, :len(NMPC_inputs)], '--', label="Ref V")
plt.grid(True)
plt.legend()
plt.title("Velocity vs Ref")
plt.savefig('circle_v.pdf', format='pdf', bbox_inches='tight')

plt.show()
