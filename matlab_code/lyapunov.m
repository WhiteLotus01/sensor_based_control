clear; clc; close all;

%% PARAMETERS

params.R = 2.0; % Define radius  
params.omega_r = 0.1047;    
params.k1 = 22; params.k2 = 140; params.k3 = 0.14; % Define gains
%% SIMULATION SETTINGS
Tsim = [0 60];
q0_figure8 = [0 0 0];               % Start at origin for Figure-8
q0_circle = [params.R; 0; pi/2];    % Start on circle perimeter, facing upward

%% SIMULATION 1: Figure-8 Tracking

[t1,q1,errs1,cmd1,V1] = simulate_robot(Tsim, q0_figure8, params, "eight");
%% SIMULATION 2: Circular Tracking
[t2,q2,errs2,cmd2,V2] = simulate_robot(Tsim, q0_circle, params, "circle");

%% PLOT RESULTS

figure('Name','Trajectory and Control Comparison');
% Figure-8 Trajectory
subplot(3,3,1);
t_plot = linspace(Tsim(1),Tsim(2),500);
R = params.R; o = params.omega_r;
xd8 = R*sin(o*t_plot);
yd8 = R*sin(2*o*t_plot)/2;
plot(xd8, yd8,'k--','LineWidth',1.2); hold on;        % Perfect 8
plot(q1(:,1), q1(:,2),'b','LineWidth',1.5);           % Actual trajectory
plot(q1(1,1), q1(1,2),'ro','MarkerFaceColor','r');    % Starting point
xlabel('x (m)'); ylabel('y (m)'); axis equal; grid on;
title('Figure-8 Trajectory');
% Circle Trajectory
subplot(3,3,2);
th = linspace(0,2*pi,300);
plot(params.R*cos(th), params.R*sin(th),'k--','LineWidth',1.2); hold on;  % Perfect circle
plot(q2(:,1), q2(:,2),'b','LineWidth',1.5);                               % Actual trajectory
plot(q2(1,1), q2(1,2),'ro','MarkerFaceColor','r');                        % Starting point
xlabel('x (m)'); ylabel('y (m)'); axis equal; grid on;
title('Circular Trajectory');
% Lyapunov Function (Figure-8)
subplot(3,3,3);
plot(t1,V1,'k','LineWidth',1.5); grid on;
xlabel('Time (s)'); ylabel('V(t)'); title('Lyapunov (Figure-8)');
% Lyapunov Function (Circle)
subplot(3,3,4);
plot(t2,V2,'k','LineWidth',1.5); grid on;
xlabel('Time (s)'); ylabel('V(t)'); title('Lyapunov (Circle)');
% Tracking Errors (Figure-8)
subplot(3,3,5);
plot(t1,errs1(:,1),'r',t1,errs1(:,2),'b',t1,errs1(:,3),'g','LineWidth',1.2);
legend('e_x','e_y','e_\theta'); grid on;
xlabel('Time (s)'); ylabel('Error'); title('Errors (Figure-8)');
% Tracking Errors (Circle)
subplot(3,3,6);
plot(t2,errs2(:,1),'r',t2,errs2(:,2),'b',t2,errs2(:,3),'g','LineWidth',1.2);
legend('e_x','e_y','e_\theta'); grid on;
xlabel('Time (s)'); ylabel('Error'); title('Errors (Circle)');
% Control Commands (Figure-8)
subplot(3,3,7);
plot(t1,cmd1(:,1),'b',t1,cmd1(:,2),'m','LineWidth',1.2);
legend('v(t)','w(t)'); grid on;
xlabel('Time (s)'); ylabel('Velocity'); title('Control Commands (Figure-8)');
% Control Commands (Circle)
subplot(3,3,8);
plot(t2,cmd2(:,1),'b',t2,cmd2(:,2),'m','LineWidth',1.2);
legend('v(t)','w(t)'); grid on;
xlabel('Time (s)'); ylabel('Velocity'); title('Control Commands (Circle)');

%% FUNCTIONS

function [t,q,errors,cmd,V] = simulate_robot(Tsim, q0, params, mode)
    [t,q] = ode45(@(t,q) robot_model(t,q,params,mode), Tsim, q0);
    errors = zeros(length(t),3);
    cmd     = zeros(length(t),2);
    V       = zeros(length(t),1);
    for i = 1:length(t)
        [~,ex,ey,etheta,v,w] = robot_model(t(i), q(i,:), params, mode);
        errors(i,:) = [ex, ey, etheta];
        cmd(i,:)    = [v, w];
        V(i)        = 0.5*(ex^2 + ey^2 + etheta^2);
    end
end
function [dqdt,ex,ey,etheta,v,w] = robot_model(t, q, params, mode)
    % Parameters
    R = params.R; o = params.omega_r;
    k1=params.k1; k2=params.k2; k3=params.k3;
    x=q(1); y=q(2); theta=q(3);
    % Reference trajectories
    if mode == "eight"
        xd = R*sin(o*t);   yd = R*sin(2*o*t)/2;
        xdot = R*o*cos(o*t); ydot = R*o*cos(2*o*t);
        xddot = -R*o^2*sin(o*t);
        yddot = -R*o^2*sin(2*o*t);
    else % CIRCLE
        xd = R*cos(o*t);   yd = R*sin(o*t);
        xdot = -R*o*sin(o*t); ydot = R*o*cos(o*t);
        xddot = -R*o^2*cos(o*t);
        yddot = -R*o^2*sin(o*t);
    end
    thetad = atan2(ydot, xdot);
    vd = sqrt(xdot^2 + ydot^2);
    % Ref angular velocity
    wd = (xdot*yddot - ydot*xddot)/(xdot^2 + ydot^2);
    % Errors in robot frame
    ex = cos(theta)*(xd - x) + sin(theta)*(yd - y);
    ey = -sin(theta)*(xd - x) + cos(theta)*(yd - y);
    etheta = wrapToPi(thetad - theta);
    % Lyapunov Control
    v = vd*cos(etheta) + k1*ex;
    w = wd + k2*vd*ey + k3*sin(etheta);
    % Disturbances
    dx = 0.02*sin(0.5*t);
    dy = 0.02*cos(0.3*t);
    dtheta = 0.01*sin(0.7*t);
    % Dynamics
    dqdt = [v*cos(theta)+dx;
            v*sin(theta)+dy;
            w+dtheta];
end

%% Test to find the best gains

k1_list = [1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20 21 22];
k2_list = [1 10 20 30 40 50 60 70 80 90 100 110 120 130 140];
k3_list = [0.04 0.06 0.08 0.1 0.12 0.14 0.16 0.18 0.2];
best_cost = Inf;
best_gains = [];
Tsim = [0 60];
q0_circle = [params.R; 0; pi/2];
for k1 = k1_list
    for k2 = k2_list
        for k3 = k3_list
            params_try = params;
            params_try.k1 = k1;
            params_try.k2 = k2;
            params_try.k3 = k3;
            [t2,q2,errs2,cmd2,V2] = simulate_robot(Tsim, q0_circle, params_try, "circle");
            pos_err = sqrt(errs2(:,1).^2 + errs2(:,2).^2);
            cost = mean(pos_err);
            cost = cost + 0.2*var(pos_err);
            if cost < best_cost
                best_cost = cost;
                best_gains = [k1,k2,k3];
            end
        end
    end
end
disp('Best gains found:');
disp(best_gains);
disp('Best cost:');
disp(best_cost);