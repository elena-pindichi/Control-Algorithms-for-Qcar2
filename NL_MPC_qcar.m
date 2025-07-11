clc; close all; clear;

import casadi.*;
load("trajectory.mat")

%% Parameters
N_pred_val  = 12;   
Q_val       = 10;
R_val       = 3;
P_val       = 10;
l           = 0.256;                % Length between front and rear

% Define prediction and simulation steps
Ts    = 0.1;                        % Sampling time
Npred = N_pred_val;                 % Prediction horizon

% Define system dimensions
dx = 4;                             % State dimensions: x, y, theta, phi
du = 2;                             % Control dimensions: V, omega

% Initial condition
x0 = [0; 2.5; 0; 0];            
u0 = zeros(du, 1);

%% Trajectories
% Choose trajectories: 1 = line, 2 = square, 3 = circle, 4 = spline
idx = 4;
[xref, uref, Nsim] = reference(idx);

%% Constraints values
Vmin     = -1; Vmax     = 1;       % Velocity limits
omegamin = -1; omegamax = 1;       % Angular velocity limits
phimax   = pi/5;                   % Front wheels orientation limits

% Weights for cost function
Q = Q_val * eye(dx);
Q(3,3) = 1;
Q(4,4) = 0.1;
R = R_val * eye(du);
P = P_val * Q;

%% CasADi nonlinear optimization
solver = casadi.Opti();

% Define optimization variables
x = solver.variable(dx, Npred + 1); 
u = solver.variable(du, Npred);     

% Define initial state as a parameter
xinit      = solver.parameter(dx, 1);
xref_param = solver.parameter(dx, Npred+1);
uref_param = solver.parameter(du, Npred+1);

% Nonlinear dynamics 
f_dynamics = @(x, u) [ u(1) * cos(x(3));
                       u(1) * sin(x(3));
                       u(1)/l * tan(x(4));
                       u(2) ];

% Add initial state constraint
solver.subject_to(x(:, 1) == xinit);

% Add constraints and dynamics for each prediction step
for k = 1 : Npred
    % State constraints
    solver.subject_to(x(:, k+1) == x(:, k) + Ts * f_dynamics(x(:, k), u(:, k)));
    solver.subject_to(-phimax <= x(4, k + 1) <= phimax);
    
    % Control input constraints
    solver.subject_to(Vmin <= u(1, k) <= Vmax);
    solver.subject_to(omegamin <= u(2, k) <= omegamax);
end

objective = 0;
for k = 1 : Npred                                                      
    objective = objective + (x(:, k) - xref_param(:, k))' * Q * (x(:, k) - xref_param(:, k)) + ...
                            (u(:, k) - uref_param(:, k))' * R * (u(:, k) - uref_param(:, k));
end

objective = objective + (x(:, Npred + 1) - xref_param(:, Npred + 1))' * P * (x(:, Npred + 1) - xref_param(:, Npred + 1));


%% Define the objective function
% Define the solver
options.ipopt.print_level = 0;
options.ipopt.sb= 'yes';
options.print_time = 0;

solver.minimize(objective)
solver.solver('ipopt', options)

% Simulation loop
usim = zeros(du, Nsim);
xsim = zeros(dx, Nsim);
xsim(:, 1) = x0;

tfin = [];
for i = 1 : Nsim
    solver.set_value(xinit, xsim(:, i))
    solver.set_value(xref_param, xref(:, i:i+Npred))
    solver.set_value(uref_param, uref(:, i:i+Npred))

    t1 = tic;
    sol = solver.solve();
    tfin = [tfin toc(t1)];

    usol = sol.value(u);
    usim(:, i) = usol(:, 1);
    xsim(:, i + 1) = xsim(:, i) + Ts * f_dynamics(xsim(:, i), usim(:, i));
end

computation_time = sum(tfin);
computation_time = computation_time/Nsim

err = zeros(dx,Nsim);
for i =1:Nsim
    err(:,i) = (xsim(:, i) - xref(:, i));
end
rmse_scalar = sqrt(sum(err(:).^2) / (dx * Nsim))

%% Plot results
figure
plot((1:Nsim)*Ts,err(1,:))
hold on
plot((1:Nsim)*Ts,err(2,:))
legend('err_x', 'err_y')
title('Reference tracking error')
grid

figure
subplot(4,1,1)
plot((1:Nsim)*Ts, xsim(1,1:Nsim))
xlabel('time (s)')
ylabel('x_1')
hold on
plot((1:Nsim)*Ts, xref(1,1:Nsim),'--')
xlabel('time (s)')
ylabel('x_1')
legend('x','$x_{ref}$','interpreter','latex')
title('Simulation and reference for x_{state}')
grid

subplot(4,1,2)
plot((1:Nsim)*Ts, xsim(2,1:Nsim))
xlabel('time (s)')
ylabel('x_2')
hold on
plot((1:Nsim)*Ts, xref(2,1:Nsim),'--')
legend('y','$y_{ref}$','interpreter','latex')
xlabel('time (s)')
ylabel('x_2')
title('Simulation and reference for y_{state}')
grid

subplot(4,1,3)
plot((1:Nsim)*Ts, xsim(3,1:end-1))
hold on
plot((1:Nsim)*Ts, xref(3,1:Nsim), '--')
legend('$\theta$','$\theta_{ref}$','interpreter','latex')
xlabel('time (s)')
ylabel('x_3')
title('Simulation and reference for \theta_{state}')
grid

subplot(4,1,4)
plot((1:Nsim)*Ts, xsim(4,1:end-1))
hold on
plot((1:Nsim)*Ts, xref(4,1:Nsim), '--')
legend('$\varphi$','$\varphi_{ref}$','interpreter','latex')
xlabel('time (s)')
ylabel('x_4')
title('Simulation and reference for \phi_{state}')
grid

figure
plot(xsim(1,:),xsim(2,:))
title('car position')
xlabel('x (m)')
ylabel('y (m)')
grid

figure
subplot(2,1,1)
plot(usim(1, :))
hold on
plot(uref(1, 1:Nsim), '--')
legend('V')
grid
subplot(2,1,2)
plot(usim(2, :))
hold on
plot(uref(2, 1:Nsim), '--')
legend('\omega')
grid

figure
hold on
height = 0.4; width = 0.2;
st = 40;
k=1;
while k < Nsim
   drawSteeringCar(xsim(:,k), l, height, width)
    k = k + st;
end
plot(xsim(1,:),xsim(2,:),'linewidth',2)

title('car position')

xlabel('x (m)')
ylabel('y (m)')

%% Trajectory Function
function [xref, uref, Nsim] = reference(idx)
    l = 0.256;
    Delta = 0.35;
    if idx == 1
        % Line reference
        % Time
        t = 0 : 0.1 : 100;
        Nsim = 200;
        alpha   = 0.5;
        beta    = 0.8;
        xr = alpha * t;     dxr = alpha + 0*t;    ddxr = 0 + 0*t;   dddxr = 0 + 0*t;
        yr = beta * t;      dyr = beta + 0*t;     ddyr = 0 + 0*t;   dddyr = 0 + 0*t;
    elseif idx == 2
        % Square trajectory
        n = 950;
        Nsim = 850;
        
        side_length = 10; 
        total_points = n;
        quarter = round(total_points / 4);
        
        xr = zeros(1, n);
        yr = zeros(1, n);
        
        % Fill square trajectory
        % Bottom edge: (0,0) to (10,0)
        xr(1:quarter) = linspace(0, side_length, quarter);
        yr(1:quarter) = 0;
        
        % Right edge: (10,0) to (10,10)
        xr(quarter+1:2*quarter) = side_length;
        yr(quarter+1:2*quarter) = linspace(0, side_length, quarter);
        
        % Top edge: (10,10) to (0,10)
        xr(2*quarter+1:3*quarter) = linspace(side_length, 0, quarter);
        yr(2*quarter+1:3*quarter) = side_length;
        
        % Left edge: (0,10) to (0,0)
        xr(3*quarter+1:end) = 0;
        yr(3*quarter+1:end) = linspace(side_length, 0, n - 3*quarter);
        
        dxr = gradient(xr, 0.3);        ddxr = gradient(dxr, 0.3);      dddxr = gradient(ddxr, 0.3);   
        dyr = gradient(yr, 0.3);        ddyr = gradient(dyr, 0.3);      dddyr = gradient(ddyr, 0.3);
    elseif idx == 3
        % Time
        t = 0 : 0.1 : 100;
        Nsim = 400;
        % Circle reference
        alpha   = 5;
        beta    = 5;
        ang     = 0.2;
        xr = alpha*cos(ang*t);      dxr = -alpha*ang*sin(ang*t);    ddxr = -alpha*ang*ang*cos(ang*t);       dddxr = alpha*ang*ang*ang*sin(ang*t);
        yr = beta*sin(ang*t);       dyr = beta*ang*cos(ang*t);      ddyr = -beta*ang*ang*sin(ang*t);        dddyr = -beta*ang*ang*ang*cos(ang*t);
    elseif idx == 4
        Nsim = 180;
        % Spline reference
        load("trajectory.mat")
        xr = xref;      dxr = dxref;        ddxr = ddxref;      dddxr = dddxref;
        yr = yref;      dyr = dyref;        ddyr = ddyref;      dddyr = dddyref;
    end
    % Computing real input reference
    Vr      = sqrt(dxr.*dxr + dyr.*dyr);
    omegar  = l * Vr .* ((dddyr.*dxr - dddxr.*dyr).*Vr.*Vr - 3 * (ddyr.*dxr - ddxr.*dyr) .* (dxr.*ddxr + dyr.*ddyr)) ...
              ./ (Vr.^6 + l*l*(ddyr.*dxr - ddxr.*dyr).^2);
    uref = [Vr; omegar];
    
    % Computing angle reference
    thetar  = unwrap(atan2(dyr ./ Vr, dxr ./ Vr));
    phir    = atan((l*(ddyr.*dxr - ddxr.*dyr)) ./ Vr.^3);

    xref = [xr; yr; thetar; phir];
end