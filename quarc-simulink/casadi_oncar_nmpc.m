clc, close all, clear;
import casadi.*

N       = 12;
T       = 20;
dt      = 0.1;
l       = 0.256;
Q_val   = 10;
R_val   = 0.3;
P_val   = 10;

% Weights
Q       = Q_val * eye(4);
Q(3,3)  = 1;
Q(4,4)  = 1;
R       = R_val * eye(2);
P       = P_val * Q;

% Define state and control symbols
x           = SX.sym('x');    
y           = SX.sym('y');    
theta       = SX.sym('theta'); 
phi         = SX.sym('phi');
states      = [x; y; theta; phi];
n_states    = length(states);

v           = SX.sym('v');        
omega       = SX.sym('omega');
controls    = [v; omega];
n_controls  = length(controls);

% Define system dynamics
xdot = [v*cos(theta);
        v*sin(theta);
        v/l*tan(phi);
        omega];

% Define the system dynamics function
f = Function('f', {states, controls}, {xdot});

xref = [2; 12; pi/2; 0];

% ZOH
X0  = MX.sym('X0', n_states);
U   = MX.sym('U', n_controls);
X   = X0;

F = Function('F', {X0, U}, {X0 + dt*f(X0, U)}, {'x0','u'}, {'xf'});

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
Xk  = MX.sym('X0', n_states);
w   = {w{:}, Xk};
w0  = [w0; zeros(n_states,1)];
lbw = [lbw; zeros(n_states,1)];
ubw = [ubw; zeros(n_states,1)];

% Export external reference
% ref = MX.sym('ref', n_states, N+1);
% ref = MX.sym('ref', n_states);
xref = MX.sym('xref', n_states, N+1);
uref = MX.sym('uref', n_controls, N);

% Formulate optimization problem
for k = 0 : N-1
    % Reference at time k
    % x_ref = xr(:,k+1);
    x_ref = xref(:, k+1);
    u_ref = uref(:, k+1);
    % x_ref = ref;

    % Control variable
    Uk  = MX.sym(['U_' num2str(k)], n_controls);
    w   = {w{:}, Uk};
    w0  = [w0; 0; 0];
    lbw = [lbw; -1; -1];
    ubw = [ubw;  1;  1];

    % Integrate dynamics
    Fk      = F('x0', Xk, 'u', Uk);
    Xk_end  = Fk.xf;

    % Cost function: tracking + control effort
    J = J + (Xk - x_ref)' * Q * (Xk - x_ref) + (Uk - u_ref)' * R * (Uk - u_ref);

    % New state variable
    Xk  = MX.sym(['X_' num2str(k+1)], n_states);
    w   = {w{:}, Xk};
    w0  = [w0; zeros(n_states,1)];
    lbw = [lbw; -inf; -inf; -inf; -pi/2];
    ubw = [ubw;  inf; inf; inf; pi/2];

    % Add dynamics constraint
    g   = {g{:}, Xk_end - Xk};
    lbg = [lbg; zeros(n_states,1)];
    ubg = [ubg; zeros(n_states,1)];
end
x_ref_terminal = xref(:, N+1);
% x_ref_terminal = ref;
J = J + (Xk - x_ref_terminal)' * P * (Xk - x_ref_terminal);

% NLP setup
ref = [xref(:); uref(:)];
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
test_c = Function('test_c', {s0, xref, uref}, {sol_sym.x(n_states+1:n_states+2)});

% Save to file
test_c.save('test_c.casadi');

% Compile the function using CasADi C interface
inc_path = GlobalOptions.getCasadiIncludePath();
lib_path = GlobalOptions.getCasadiPath();
mex('-v',['-I' inc_path],['-L' lib_path],'-lcasadi', 'casadi_fun.c')

disp("MPC function compiled and saved.")