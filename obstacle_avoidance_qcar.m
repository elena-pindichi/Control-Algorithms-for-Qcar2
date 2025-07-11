clc, close all, clear all;
import casadi.*;
addpath(genpath('tools'))

% Define obstacle
A_ob = [1 1; 1 -1; -1 -1; -1 1];
b_ob = [1; 1; 1; 1];
O = Polyhedron(A_ob, b_ob);

% Parameters of optimization problems
dx = 4;
dz = 2;
du = 2;
Ts = 0.1;
Delta = 0.35;
l = 0.256;
N_pred_val  = 5;                      
Q_val       = 5;                          
R_val       = 1;  
P_val       = 10; 
A = eye(dz);
B = Ts * eye(dz);

% Kinematics of the model
f_dynamics = @(x, u) [ u(1) * cos(x(3));
                       u(1) * sin(x(3));
                       u(1)/l * tan(x(4));
                       u(2) ];

% Reference and initial points
u0 = zeros(du, 1);
x0 = [-4; 0.2; 0; 0];
z0 = LinOutput(x0, Delta, l);
xref = [5; 0.6; 0; 0];
zref = LinOutput(xref, Delta, l);

% Polyhedral approximation
rhat = min(Delta*l*10/sqrt(Delta*Delta + l*l), 5);
ptsU = [];
for tta = linspace(0,2*pi-1e-4,20)
    ptsU = [ptsU; [rhat*cos(tta), rhat*sin(tta)]];
end
U_approx = Polyhedron('V',ptsU).computeVRep();

% Weights
Npred   = N_pred_val;  
Q = Q_val * eye(dz);
R = R_val * eye(du);
P = P_val * Q;
[~,P] = dlqr(A,B,Q,R);
P = 150 * P;
Nsim = 30 / Ts;

solver = casadi.Opti();

%% Define variables
z   = solver.variable(dz, Npred + 1);
w   = solver.variable(du, Npred);
alpha = solver.variable(4, Npred + 1);
zinit = solver.parameter(dz, 1);
winit = solver.parameter(du, 1);

% Define obstacle constraints
H = A_ob;
b = b_ob + 0.5 * ones(4, 1);
M_cstr = 10^5;

%% Initialize constraints
solver.subject_to(z(:, 1) == zinit)
for k = 1 : Npred
    solver.subject_to(z(:, k + 1) == A * z(:, k) + B * w(:, k))
    solver.subject_to(U_approx.A*w(:, k) <= U_approx.b)
    solver.subject_to(-H * z(:, k+1) <= -b + M_cstr * alpha(:, k+1))
    solver.subject_to(sum(alpha(:, k+1)) <= 3)
end

%% Initialize objective
objective = 0;
for k = 1 : Npred
    if k ~= 1
        objective = objective + (z(:, k) - zref)' * Q * (z(:, k) - zref) + (w(:, k) - w(:, k-1))' * R * (w(:, k) - w(:, k-1));
    else
        objective = objective + (z(:, k) - zref)' * Q * (z(:, k) - zref) + w(:, k)' * R * w(:, k);
    end
end
objective = objective + (z(:, Npred + 1) - zref)' * P * (z(:, Npred + 1) - zref);

solver.minimize(objective)

%% Define the solver
z_ = reshape(zeros(dz, Npred + 1), 1, []);
w_ = reshape(zeros(du, Npred), 1, []);
alpha_ = reshape(ones(4, Npred + 1), 1, []);
option.discrete = [z_, w_, alpha_];
solver.solver('bonmin', option);

%% simulation loop
usim    = zeros(du, Nsim);
zsim    = zeros(dz, Nsim);
xsim    = zeros(dx, Nsim);
wsim    = zeros(du, Nsim);
zsim(:, 1) = z0;
xsim(:, 1) = x0;
usim_init = u0;

tfin = [];
for i = 1 : Nsim
    solver.set_value(zinit, zsim(:, i))

    t1 = tic;
    sol = solver.solve();
    tfin = [tfin toc(t1)];

    wsol = sol.value(w);
    M = LinMatrix(xsim(3:4, i), Delta, l);
    usim(:, i) = M^(-1) * wsol(:, 1);
    wsim(:, i) = wsol(:, 1);

    xsim(:, i + 1) = xsim(:, i) + Ts * f_dynamics(xsim(:, i), usim(:, i));
    zsim(:, i + 1) = A * zsim(:, i) + B * wsim(:, i);
end

computation_time = sum(tfin);
computation_time = computation_time/Nsim

err = zeros(dx,Nsim);
for i =1:Nsim
    err(:,i) = (xsim(:, i) - xref);
end

rmse_scalar = sqrt(sum(err(:).^2) / (dx * Nsim))

%% Plotting
close all;

figure
plot((1:Nsim)*Ts,err(1,:))
hold on
plot((1:Nsim)*Ts,err(2,:))
legend('err_x', 'err_y')
title('Reference tracking error')
grid

figure
scatter(xsim(1, :), xsim(2, :))
grid
title('State space');
xlabel 'x1'
ylabel 'x2'
hold on
plot(O)

t = 0 : 0.1 : Nsim;

figure
subplot(4,1,1)
plot((1:Nsim)*Ts, xsim(1,1:end-1))
hold on
plot((1:Nsim)*Ts, xref(1) * ones(1, Nsim), '--')
legend('x')
xlabel('time (s)')
grid

subplot(4,1,2)
plot((1:Nsim)*Ts, xsim(2,1:end-1))
hold on
plot((1:Nsim)*Ts, xref(2) * ones(1, Nsim), '--')
legend('y')
xlabel('time (s)')
grid

subplot(4,1,3)
plot((1:Nsim)*Ts, xsim(3,1:end-1))
hold on
plot((1:Nsim)*Ts, xref(3) * ones(1, Nsim), '--')
legend('$\theta$','interpreter','latex')
xlabel('time (s)')
grid

subplot(4,1,4)
plot((1:Nsim)*Ts, xsim(4,1:end-1))
hold on
plot((1:Nsim)*Ts, xref(4) * ones(1, Nsim), '--')
legend('$\varphi$','interpreter','latex')
xlabel('time (s)')
grid

figure
subplot(2,1,1)
plot(usim(1, :))
hold on
legend('V')
xlabel('Nsim')
grid
subplot(2,1,2)
plot(usim(2, :))
legend('\omega')
xlabel('Nsim')
grid

figure
hold on
height = 0.4; width = 0.2;
st = 10;
k=1;
while k < Nsim
   drawSteeringCar(xsim(:,k), l, height, width)
    k = k + st;
end
plot(xsim(1,:),xsim(2,:),'linewidth',2)
hold on
plot(O)
title('car position')
xlabel('x (m)')
ylabel('y (m)')

%%
% K = Polyhedron(H,b);
% figure
% scatter(xsim(1, :), xsim(2, :))
% grid
% title('State space');
% xlabel 'x1'
% ylabel 'x2'
% hold on
% plot(O)

xx = sdpvar(2,1);
figure
plot(xx'*xx <= rhat^2)
hold on
plot(U_approx)