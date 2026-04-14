function xdot = AircraftEOMDoublet(time, aircraft_state, aircraft_surfaces, ...
    doublet_size, doublet_time, wind_inertial, aircraft_parameters)
%AIRCRAFTEOMDOUBLET Aircraft EOM with an elevator doublet added to trim.
%
%   The elevator input is
%       de = de_trim + doublet_size,      0 < t <= doublet_time
%       de = de_trim - doublet_size,      doublet_time < t <= 2*doublet_time
%       de = de_trim,                     otherwise

u_ctrl = aircraft_surfaces(:);

if time > 0 && time <= doublet_time
    u_ctrl(1) = u_ctrl(1) + doublet_size;
elseif time > doublet_time && time <= 2*doublet_time
    u_ctrl(1) = u_ctrl(1) - doublet_size;
end

xdot = AircraftEOM(time, aircraft_state, u_ctrl, wind_inertial, aircraft_parameters);
end

function xdot = AircraftEOM(time, aircraft_state, aircraft_surfaces, wind_inertial, aircraft_parameters)

phi   = aircraft_state(4);
theta = aircraft_state(5);
psi   = aircraft_state(6);
u     = aircraft_state(7);
v     = aircraft_state(8);
w     = aircraft_state(9);
p     = aircraft_state(10);
q     = aircraft_state(11);
r     = aircraft_state(12);

g  = aircraft_parameters.g;
m  = aircraft_parameters.m;
Ix = aircraft_parameters.Ix;
Iy = aircraft_parameters.Iy;
Iz = aircraft_parameters.Iz;

height = -aircraft_state(3);
density = stdatmo(height);

[Fc,Gc] = AeroForcesAndMoments(aircraft_state, aircraft_surfaces, wind_inertial, density, aircraft_parameters);
Xc = Fc(1); Yc = Fc(2); Zc = Fc(3);
Lc = Gc(1); Mc = Gc(2); Nc = Gc(3);

dxE = u*cos(theta)*cos(psi) + v*(sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi)) + w*(cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi));
dyE = u*cos(theta)*sin(psi) + v*(sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi)) + w*(cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi));
dzE = -u*sin(theta) + v*sin(phi)*cos(theta) + w*cos(phi)*cos(theta);

dphi   = p + q*sin(phi)*tan(theta) + r*cos(phi)*tan(theta);
dtheta = q*cos(phi) - r*sin(phi);
dpsi   = q*sin(phi)*sec(theta) + r*cos(phi)*sec(theta);

du = r*v - q*w - g*sin(theta) + Xc/m;
dv = p*w - r*u + g*cos(theta)*sin(phi) + Yc/m;
dw = q*u - p*v + g*cos(theta)*cos(phi) + Zc/m;

dp = (Lc + (Iy-Iz)*q*r)/Ix;
dq = (Mc + (Iz-Ix)*p*r)/Iy;
dr = (Nc + (Ix-Iy)*p*q)/Iz;

xdot = [dxE;dyE;dzE;dphi;dtheta;dpsi;du;dv;dw;dp;dq;dr];
end

%% given function for aero forces and moments
function [aero_forces, aero_moments] = AeroForcesAndMoments(aircraft_state, aircraft_surfaces, wind_inertial, density, aircraft_parameters)
%
%
% aircraft_state = [xi, yi, zi, roll, pitch, yaw, u, v, w, p, q, r]
% NOTE: The function assumes the veolcity is the air relative velocity
% vector. When used with simulink the wrapper function makes the
% conversion.
%
% aircraft_surfaces = [de da dr dt];
%
% Lift and Drag are calculated in Wind Frame then rotated to body frame
% Thrust is given in Body Frame
% Sideforce calculated in Body Frame
%%% redefine states and inputs for ease of use
ap = aircraft_parameters;

psi = aircraft_state(4,1);
theta = aircraft_state(5,1);
phi = aircraft_state(6,1);

p = aircraft_state(10,1);
q = aircraft_state(11,1);
r = aircraft_state(12,1);
de = aircraft_surfaces(1,1);
da = aircraft_surfaces(2,1);
dr = aircraft_surfaces(3,1);
dt = aircraft_surfaces(4,1);

R = eul2rotm([psi, theta, phi], "ZYX");
wind_body = R * wind_inertial; 
air_rel_vel_body = aircraft_state(7:9,1) - wind_body;
[wind_angles] = WindAnglesFromVelocityBody(air_rel_vel_body);
V = wind_angles(1,1);
beta = wind_angles(2,1);
alpha = wind_angles(3,1);
alpha_dot = 0;
%Q = ap.qbar;
Q = 0.5*density*V*V;
sa = sin(alpha);
ca = cos(alpha);
%%% determine aero force coefficients
CL = ap.CL0 + ap.CLalpha*alpha + ap.CLq*q*ap.c/(2*V) + ap.CLde*de;
%CD = ap.CD0 + ap.CDalpha*alpha + ap.CDq*q*ap.c/(2*V) + ap.CDde*de;
CD = ap.CDpa + ap.K*CL*CL;
CX = -CD*ca + CL*sa;
CZ = -CD*sa - CL*ca;
CY = ap.CY0 + ap.CYbeta*beta + ap.CYp*p*ap.b/(2*V) + ap.CYr*r*ap.b/(2*V) + ap.CYda*da + ap.CYdr*dr;
%%Thrust = .5*density*ap.Sprop*ap.Cprop*((ap.kmotor*dt)^2 - V^2);
Thrust = density*ap.Sprop*ap.Cprop*(V + dt*(ap.kmotor - V))*dt*(ap.kmotor-V); %
%%% determine aero forces from coeffficients
X = Q*ap.S*CX + Thrust;
Y = Q*ap.S*CY;
Z = Q*ap.S*CZ;
aero_forces = [X;Y;Z];

%%% determine aero moment coefficients
Cl = ap.b*[ap.Cl0 + ap.Clbeta*beta + ap.Clp*p*ap.b/(2*V) + ap.Clr*r*ap.b/(2*V) + ap.Clda*da + ap.Cldr*dr];
Cm = ap.c*[ap.Cm0 + ap.Cmalpha*alpha + ap.Cmq*q*ap.c/(2*V) + ap.Cmde*de];
Cn = ap.b*[ap.Cn0 + ap.Cnbeta*beta + ap.Cnp*p*ap.b/(2*V) + ap.Cnr*r*ap.b/(2*V) + ap.Cnda*da + ap.Cndr*dr];

%%% determine aero moments from coeffficients
aero_moments = Q*ap.S*[Cl; Cm; Cn]; %[l;m;n];

end

%% given wind angle function
function [wind_angles] = WindAnglesFromVelocityBody(velocity_body)
V = norm(velocity_body);
alpha = atan2(velocity_body(3,1),velocity_body(1,1));
beta = asin(velocity_body(2,1)/V);
wind_angles = [V; beta; alpha];
end
