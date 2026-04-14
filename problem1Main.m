%% ASEN 3801 Lab 5 Problem2
clear; clc; close all;

% params
aircraft_parameters.g = 9.81; % Gravitational acceleration [m/s^2]
aircraft_parameters.S = 0.6282; %[m^2]
aircraft_parameters.b = 3.067; %[m]
aircraft_parameters.c = 0.208; %[m]
aircraft_parameters.AR = aircraft_parameters.b*aircraft_parameters.b/aircraft_parameters.S;
aircraft_parameters.m = 5.74; %[kg]
aircraft_parameters.W = aircraft_parameters.m*aircraft_parameters.g; %[N]
SLUGFT2_TO_KGM2 = 14.5939/(3.2804*3.2804);
aircraft_parameters.Ix = SLUGFT2_TO_KGM2*4106/12^2/32.2; %[kg m^2]
aircraft_parameters.Iy = SLUGFT2_TO_KGM2*3186/12^2/32.2; %[kg m^2]
aircraft_parameters.Iz = SLUGFT2_TO_KGM2*7089/12^2/32.2; %[kg m^2]
aircraft_parameters.Ixz = SLUGFT2_TO_KGM2*323.5/12^2/32.2; %[kg m^2]
aircraft_parameters.CDmin = 0.0240;
aircraft_parameters.CLmin = 0.2052;
aircraft_parameters.K = 0.0549;
aircraft_parameters.e = 1/(aircraft_parameters.K*aircraft_parameters.AR*pi);
aircraft_parameters.CD0 = aircraft_parameters.CDmin+aircraft_parameters.K*aircraft_parameters.CLmin*aircraft_parameters.CLmin;
aircraft_parameters.K1 = -2*aircraft_parameters.K*aircraft_parameters.CLmin;
aircraft_parameters.CDpa = aircraft_parameters.CD0;
aircraft_parameters.Sprop = 0.0707;
aircraft_parameters.Cprop = 1;
aircraft_parameters.kmotor = 30;
aircraft_parameters.CL0 = 0.2219;
aircraft_parameters.Cm0 = 0.0519;
aircraft_parameters.CY0 = 0;
aircraft_parameters.Cl0 = 0;
aircraft_parameters.Cn0 = 0;
aircraft_parameters.CLalpha = 6.196683; 
aircraft_parameters.Cmalpha = -1.634010; 
aircraft_parameters.CLq = 10.137584; 
aircraft_parameters.Cmq = -24.376066;
aircraft_parameters.CLalphadot = 0; 
aircraft_parameters.Cmalphadot = 0; 
aircraft_parameters.CYbeta = -0.367231; 
aircraft_parameters.Clbeta = -0.080738; 
aircraft_parameters.Cnbeta = 0.080613; 
aircraft_parameters.CYp = -0.064992;
aircraft_parameters.Clp = -0.686618;
aircraft_parameters.Cnp = -0.039384;
aircraft_parameters.Clr = 0.119718;
aircraft_parameters.Cnr = -0.052324;
aircraft_parameters.CYr = 0.213412;
  aircraft_parameters.CLde =   0.006776;
  aircraft_parameters.Cmde =  -0.06; 
  aircraft_parameters.CYda =  -0.000754;
  aircraft_parameters.Clda =  -0.02; 
  aircraft_parameters.Cnda =  -0.000078;
  aircraft_parameters.CYdr =   0.003056;
  aircraft_parameters.Cldr =   0.000157;
  aircraft_parameters.Cndr =  -0.000856;
ap = aircraft_parameters; 
wind_inertial = [0; 0; 0];
fig_indices = 1:6;
fig = [1, 2, 3, 4, 5, 6];

%% Case 1
tspan = [0 200]; %100 seconds for showing steady state isnt enough to see spiral
%prompt says 100, using 100 cuts off early. 200?
x0_1 = zeros(12,1); %mostly zeros case aside from 2 entries
x0_1(3) = -1609.34; %altitude (positive Z is down)
x0_1(7) = 21; % u = 21 m/s
u0_1 = [0; 0; 0; 0]; %no controls for case 1

[t1, x1] = ode45(@(t,x) AircraftEOM(t,x,u0_1,wind_inertial,ap), tspan, x0_1);
%stack repeated arrays for control input, err if try to only use 1 line of
%control array
u1_array = repmat(u0_1, 1, length(t1));
PlotAircraftSim(t1, x1', u1_array, fig, 'b-', 'Case 1: Near-Trim',1);

%% Case 2 Trimmed
x0_2 = [0; 0; -1800; 0; 0.0278; 0; 20.99; 0; 0.5837; 0; 0; 0];
u0_2 = [0.1079; 0; 0; 0.3182];

[t2, x2] = ode45(@(t,x) AircraftEOM(t,x,u0_2,wind_inertial,ap), tspan, x0_2);
u2_array = repmat(u0_2, 1, length(t2));
%same issue with repmat, repeated array fixes control input
PlotAircraftSim(t2, x2', u2_array, fig, 'r-', 'Case 2: Trimmed',2);

%% Case 3
x0_3 = [0; 0; -1800; deg2rad(15); deg2rad(-12); deg2rad(270); 19; 3; -2; deg2rad(0.08); deg2rad(-0.2); 0];
u0_3 = [deg2rad(5); deg2rad(2); deg2rad(-13); 0.3];

[t3, x3] = ode45(@(t,x) AircraftEOM(t,x,u0_3,wind_inertial,ap), tspan, x0_3);
u3_array = repmat(u0_3, 1, length(t3)); %repmat again to fix stacked arrays
PlotAircraftSim(t3, x3', u3_array, fig, 'g-', 'Case 3: Maneuver',3);
