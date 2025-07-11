# 🚗 Running Nonlinear MPC in Simulink with CasADi

This guide explains how to run Nonlinear MPC simulations using Simulink, CasADi, and a compiled C interface for real-time control.

---

## ▶️ How to Run the Code

### Example: **Nonlinear MPC**
1. Run the MATLAB script:  
   ```matlab
   casadi_simulink_nmpc.m
   ```
2. Load the desired reference trajectory:  
   ```matlab
   load_trajectory.m
   ```
3. Open the Simulink model:  
   ```matlab
   nlmpc_simulink.slx
   ```
4. Click **Run** in Simulink.

> ✅ The procedure is the same for FLMPC (`casadi_simulink_flmpc.m`).

---

## 🧠 Code Structure & Explanation

Each `casadi_simulink_*.m` script performs the following:

- **Creates a CasADi-based MEX function**
- **Compiles it for use as an S-function** in Simulink
- Uses a `.c` file to bridge CasADi with Simulink (via I/O ports)

---

## 📝 Line-by-Line Breakdown

### 1. Define Problem Parameters
- Weight matrices, reference state, control limits, and horizon setup

```matlab
x = SX.sym('x');
y = SX.sym('y');
theta = SX.sym('theta');
phi = SX.sym('phi');
states = [x; y; theta; phi];
n_states = length(states);
```

### 2. Define Dynamics
- Nonlinear dynamics written as a CasADi function `f`

### 3. Optimization Setup
- Initialize variables for direct transcription:

```matlab
w = {}; 
w0 = []; 
lbw = []; 
ubw = [];
g = {}; 
lbg = []; 
ubg = [];
J = 0; % Objective function
```

### 4. Initial Conditions

```matlab
Xk = MX.sym('X0', n_states);
w = {w{:}, Xk};
w0 = [w0; zeros(n_states,1)];
lbw = [lbw; zeros(n_states,1)];
ubw = [ubw; zeros(n_states,1)];
```

### 5. Loop Over Prediction Horizon

For each time step `k`:

- Add control input `Uk`
- Integrate dynamics using `Fk`
- Update cost function
- Add constraints
- Add next state `Xk+1`

```matlab
Uk = MX.sym(['U_' num2str(k)], n_controls);
w = {w{:}, Uk};
w0 = [w0; 0; 0];
lbw = [lbw; -1; -1];
ubw = [ubw;  1;  1];

Fk = F('x0', Xk, 'u', Uk);
Xk_end = Fk.xf;

J = J + (Xk - x_ref)' * Q * (Xk - x_ref) + Uk' * R * Uk;

Xk = MX.sym(['X_' num2str(k+1)], n_states);
w = {w{:}, Xk};
w0 = [w0; zeros(n_states,1)];
lbw = [lbw; -inf; -inf; -inf; -pi/2];
ubw = [ubw;  inf;  inf;  inf;  pi/2];

g = {g{:}, Xk_end - Xk};
lbg = [lbg; zeros(n_states,1)];
ubg = [ubg; zeros(n_states,1)];
```

### 6. Define NLP Problem

```matlab
prob = struct('f', J, 'x', vertcat(w{:}), 'g', vertcat(g{:}));
opts = struct('ipopt', struct('print_level', 0), 'print_time', false);
solver = nlpsol('solver', 'ipopt', prob, opts);
```

### 7. Symbolic MPC Function

Use symbolic initial state `s0` as input, and extract the first control input from the optimal solution.

```matlab
s0 = MX.sym('s0', n_states);
lbw_sym = MX(lbw);
ubw_sym = MX(ubw);
lbw_sym(1:n_states) = s0;
ubw_sym(1:n_states) = s0;

sol_sym = solver('x0', w0, ...
                 'lbx', lbw_sym, ...
                 'ubx', ubw_sym, ...
                 'lbg', lbg, ...
                 'ubg', ubg);

mpc_fun = Function('mpc_fun', {s0}, {sol_sym.x(n_states+1:n_states+2)});
```

### 8. Compilation

Save and compile the symbolic function for use in real-time Simulink simulations.

```matlab
mpc_fun.save('mpc_fun.casadi');
mex(...); % Compile S-function interface
```

---

## 📎 Notes

- The `.c` file used to bridge MATLAB and Simulink is adapted from CasADi's official example:  
  🔗 [CasADi Blog – MPC in Simulink](https://web.casadi.org/blog/mpc-simulink2/)

- This file handles initialization of input/output ports and links the compiled `.mex` function to Simulink.

> ⚠️ While there may be updates in the codebase, the structure described above remains the core skeleton.

---

## ✅ You're Ready to Simulate!

Run the scripts, launch the Simulink model, and experiment with your NMPC controller. For questions or issues, feel free to check the CasADi forums or official documentation.
