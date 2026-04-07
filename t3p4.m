%Author:Jack Lingbeck
%Inputs: 
%Outputs: 
%Purpose:

clc;
clear;
close all;

%Time
tspan = [0 10];

g = 9.81;
m = 0.068;

Ix = 5.8e-5;
Iy = 7.2e-5;
Iz = 1.0e-4;
I = [Ix,0,0;
     0,Iy,0;
     0,0,Iz];

var_trim = zeros(12,1);

deg5 = deg2rad(5);

case_names = { ...
    '3.3a: +5 deg roll', ...
    '3.3b: +5 deg pitch', ...
    '3.3c: +0.1 rad/s roll rate', ...
    '3.3d: +0.1 rad/s pitch rate'};

ICs = [ ...
    var_trim + [0;0;0;deg5;0;0;0;0;0;0;0;0], ...
    var_trim + [0;0;0;0;deg5;0;0;0;0;0;0;0], ...
    var_trim + [0;0;0;0;0;0;0;0;0;0.1;0;0], ...
    var_trim + [0;0;0;0;0;0;0;0;0;0;0.1;0] ];

colors = {'b-','r-','g-','m-'};

for k = 1:4
    var0 = ICs(:,k);

    [t_out, state_out] = ode45(@(t,var) QuadrotorEOM_NL_ClosedLoop(t,var,g,m,I), ...
                               tspan, var0);

    time = t_out;
    aircraft_state_array = state_out;

    npts = length(time);
    control_input_array = zeros(npts,4);

    for i = 1:npts
        [Fc, Gc] = InnerLoopFeedback(aircraft_state_array(i,:));
        control_input_array(i,:) = [Fc(3) Gc(1) Gc(2) Gc(3)];
    end

    fig = 6*(k-1) + (1:6);
    PlotAircraftSim(time, aircraft_state_array, control_input_array, fig, colors{k});

    figure(fig(1)); sgtitle(case_names{k});
    figure(fig(2)); sgtitle(case_names{k});
    figure(fig(3)); sgtitle(case_names{k});
    figure(fig(4)); sgtitle(case_names{k});
    figure(fig(5)); sgtitle(case_names{k});
    figure(fig(6)); title(case_names{k});

    disp(['Final state of ', case_names{k}])
    disp(aircraft_state_array(:,end))
end


function var_dot = QuadrotorEOM_NL_ClosedLoop(t, var, g, m, I)
    Ix = I(1,1); Iy = I(2,2); Iz = I(3,3);
    
    % states
    phi = var(4); 
    theta = var(5); 
    psi = var(6); 
    u = var(7); 
    v = var(8);
    w = var(9);
    p = var(10);  
    q = var(11);    
    r = var(12);

    % combined controller call for task 4
    [Fc, Gc] = InnerLoopFeedback(var);
    Zc = Fc(3); Lc = Gc(1); Mc = Gc(2); Nc = Gc(3);

    % body to inertial
    dxE = u*(cos(theta)*cos(psi)) + ...
          v*(sin(phi)*sin(theta)*cos(psi) - cos(phi)*sin(psi)) + ...
          w*(cos(phi)*sin(theta)*cos(psi) + sin(phi)*sin(psi));
    dyE = u*(cos(theta)*sin(psi)) + ...
          v*(sin(phi)*sin(theta)*sin(psi) + cos(phi)*cos(psi)) + ...
          w*(cos(phi)*sin(theta)*sin(psi) - sin(phi)*cos(psi));
    dzE = -u*sin(theta) + v*sin(phi)*cos(theta) + w*cos(phi)*cos(theta);

    % Rotational Kinematics
    dphi  = p + q*sin(phi)*tan(theta) + r*cos(phi)*tan(theta);
    dtheta = q*cos(phi) - r*sin(phi);
    dpsi   = q*sin(phi)*sec(theta) + r*cos(phi)*sec(theta);

    % Nonlinear Dynamics - see slides
    du = r*v - q*w - g*sin(theta);
    dv = p*w - r*u + g*cos(theta)*sin(phi);
    dw = q*u - p*v + g*cos(theta)*cos(phi) + (Zc/m);

    dp = (Lc + (Iy - Iz)*q*r) / Ix;
    dq = (Mc + (Iz - Ix)*p*r) / Iy;
    dr = (Nc + (Ix - Iy)*p*q) / Iz;

    var_dot = [dxE; dyE; dzE; dphi; dtheta; dpsi; du; dv; dw; dp; dq; dr];
end