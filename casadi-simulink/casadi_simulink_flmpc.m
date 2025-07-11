clc, close all, clear;
import casadi.*

N       = 5;
T       = 5;
dt      = 0.1;
l       = 0.256;
Q_val   = 10;
R_val   = 0.1;
P_val   = 10;
Delta   = 0.35;

% Weights
Q = Q_val * eye(2);
R = R_val * eye(2);
P = P_val * Q;
A = eye(2);
B = dt * eye(2);

n_states    = 2;
n_controls  = 2;

xref = [2; 12; pi/2; 0];
zref = [ xref(1)+l*cos(xref(3))+Delta*cos(xref(3)+xref(4));
         xref(2)+l*sin(xref(3))+Delta*sin(xref(3)+xref(4)) ];

Z0  = MX.sym('Z0', n_states);
W   = MX.sym('W', n_controls);
Z   = Z0;

% Build the NLP
w   = {};       % Decision variables
w0  = [];       % Initial guess
lbw = [];       % Lower bounds
ubw = [];       % Upper bounds
g   = {};       % Constraints
lbg = [];
ubg = [];
J   = 0; 

% Initial state
Zk  = MX.sym('Z0', n_states);
w   = {w{:}, Zk};
w0  = [w0; zeros(n_states,1)];
lbw = [lbw; zeros(n_states,1)];
ubw = [ubw; zeros(n_states,1)];

% ref = MX.sym('ref', n_states, N+1);
% ref = MX.sym('ref', n_states);
zref = MX.sym('zref', n_states, N+1);
wref = MX.sym('wref', n_controls, N);

% Formulate optimization problem
for k = 0 : N-1
    z_ref = zref(:, k+1);
    w_ref = wref(:, k+1);

    % Control variable
    Wk  = MX.sym(['W_' num2str(k)], n_controls);
    w   = {w{:}, Wk};
    w0  = [w0; 0; 0];
    lbw = [lbw; -inf; -inf];
    ubw = [ubw;  inf;  inf];

    Zk_end = A*Zk + B * Wk;

    % Cost function: tracking + control effort
    J = J + (Zk - z_ref)' * Q * (Zk - z_ref) + (Wk - w_ref)' * R * (Wk - w_ref);

    % New state variable
    Zk  = MX.sym(['Z_' num2str(k+1)], n_states);
    w   = {w{:}, Zk};
    w0  = [w0; zeros(n_states,1)];
    lbw = [lbw; -inf; -inf];
    ubw = [ubw;  inf; inf];

    % Add dynamics constraint
    g   = {g{:}, Zk_end - Zk, Wk' * Wk};
    lbg = [lbg; zeros(n_states,1); -inf];
    ubg = [ubg; zeros(n_states,1); 1];
end
zref_terminal = zref(:, N + 1);
J = J + (Zk - zref_terminal)' * P * (Zk - zref_terminal);

% NLP setup
ref = [zref(:); wref(:)];
prob    = struct('f', J, 'x', vertcat(w{:}), 'g', vertcat(g{:}), 'p', ref);
opts    = struct('ipopt', struct('print_level', 0), 'print_time', false);
solver  = nlpsol('solver', 'ipopt', prob, opts);

% Export initial state symbol
s0                  = MX.sym('s0', n_states);
lbw_sym             = MX(lbw);
ubw_sym             = MX(ubw);
lbw_sym(1:n_states) = s0;
ubw_sym(1:n_states) = s0;

% Solve the problem symbolically
sol_sym = solver('x0', w0, ...
                 'lbx', lbw_sym, ...
                 'ubx', ubw_sym, ...
                 'lbg', lbg, ...
                 'ubg', ubg, ...
                 'p', ref);

% Map from initial state to first control input
flmpc_fun = Function('flmpc_fun', {s0, zref, wref}, {sol_sym.x(n_states+1:n_states+2)});

% Save to file
flmpc_fun.save('flmpc_fun.casadi');

% Compile the function using CasADi C interface
inc_path = GlobalOptions.getCasadiIncludePath();
lib_path = GlobalOptions.getCasadiPath();
mex('-v',['-I' inc_path],['-L' lib_path],'-lcasadi', 'casadi_fun.c')

disp("MPC function compiled and saved.")