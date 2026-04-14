close all;
clc;
%% aircraft parameters
aircraft_parameters.g = 9.81; % Gravitational acceleration [m/s^2]
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Aircraft geometry parameters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
aircraft_parameters.S = 0.6282; %[m^2]
aircraft_parameters.b = 3.067; %[m]
aircraft_parameters.c = 0.208; %[m]
aircraft_parameters.AR = aircraft_parameters.b*aircraft_parameters.b/aircraft_parameters.S;
aircraft_parameters.m = 5.74; %[kg]
aircraft_parameters.W = aircraft_parameters.m*aircraft_parameters.g; %[N]
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%
% Inertias from Solidworks model of Tempest
% These need to be validated, especially for Ttwistor
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

SLUGFT2_TO_KGM2 = 14.5939/(3.2804*3.2804);
aircraft_parameters.Ix = SLUGFT2_TO_KGM2*4106/12^2/32.2; %[kg m^2]
aircraft_parameters.Iy = SLUGFT2_TO_KGM2*3186/12^2/32.2; %[kg m^2]
aircraft_parameters.Iz = SLUGFT2_TO_KGM2*7089/12^2/32.2; %[kg m^2]
aircraft_parameters.Ixz = SLUGFT2_TO_KGM2*323.5/12^2/32.2; %[kg m^2]
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%
% Drag terms determined by curve fit to CFD analysis performed by Roger
% Laurence. Assumes general aircraft drag model
% CD = CDmin + K(CL-CLmin)^2
% or equivalently
% CD = CD0 + K1*CL + K*CL^2
% where
% CD0 = CDmin + K*CLmin^2
% K1 = -2K*CLmin
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%
aircraft_parameters.CDmin = 0.0240;
aircraft_parameters.CLmin = 0.2052;
aircraft_parameters.K = 0.0549;
aircraft_parameters.e = 1/(aircraft_parameters.K*aircraft_parameters.AR*pi);
aircraft_parameters.CD0 = aircraft_parameters.CDmin+aircraft_parameters.K*aircraft_parameters.CLmin*aircraft_parameters.CLmin;
aircraft_parameters.K1 = -2*aircraft_parameters.K*aircraft_parameters.CLmin;
aircraft_parameters.CDpa = aircraft_parameters.CD0;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Engine parameters, assuming model from Beard and Mclain that gives zero
% thrust for zero throttle
%%%%%%%%%%%%%%%%%%%%%%%%%%%%
aircraft_parameters.Sprop = 0.0707;
aircraft_parameters.Cprop = 1;
aircraft_parameters.kmotor = 30;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%
% Zero angle of attack aerodynamic forces and moments
% - some sources (like text used for ASEN 3128) define the body
% coordinate system as the one that gives zero total lift at
% zero angle of attack
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
aircraft_parameters.CL0 = 0.2219;
aircraft_parameters.Cm0 = 0.0519;
aircraft_parameters.CY0 = 0;
aircraft_parameters.Cl0 = 0;
aircraft_parameters.Cn0 = 0;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%
% Longtidunal nondimensional stability derivatives from AVL
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%
aircraft_parameters.CLalpha = 6.196683;
aircraft_parameters.Cmalpha = -1.634010;
aircraft_parameters.CLq = 10.137584;
aircraft_parameters.Cmq = -24.376066;
% Neglected parameters, check units below if incorporated later
aircraft_parameters.CLalphadot = 0;
aircraft_parameters.Cmalphadot = 0;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%
% Lateral-directional nondimensional stability derivatives from AVL
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%
aircraft_parameters.CYbeta = -0.367231;
aircraft_parameters.Clbeta = -0.080738;
aircraft_parameters.Cnbeta = 0.080613;
aircraft_parameters.CYp = -0.064992;
aircraft_parameters.Clp = -0.686618;
aircraft_parameters.Cnp = -0.039384;
aircraft_parameters.Clr = 0.119718;
aircraft_parameters.Cnr = -0.052324;
aircraft_parameters.CYr = 0.213412;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Control surface deflection parameters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Elevator
aircraft_parameters.CLde = 0.006776;
aircraft_parameters.Cmde = -0.06;
% Aileron
aircraft_parameters.CYda = -0.000754;
aircraft_parameters.Clda = -0.02;
aircraft_parameters.Cnda = -0.000078;
% Rudder
aircraft_parameters.CYdr = 0.003056;
aircraft_parameters.Cldr = 0.000157;
aircraft_parameters.Cndr = -0.000856;

wind_inertial = [0; 0; 0];
opts = odeset('RelTol',1e-8,'AbsTol',1e-10);

% Trim condition from Problem 2.2
x_trim = [0; 0; -1800; ...
          0; 0.02780; 0; ...
          20.99; 0; 0.5837; ...
          0; 0; 0];

u_trim = [0.1079; 0; 0; 0.3182];

doublet_size = deg2rad(15);
doublet_time = 0.25;

results = struct();

%% Problem 3.1: 3-second simulation for short-period mode
[t_sp, x_sp] = ode45(@(t,x) AircraftEOMDoublet(t, x, u_trim, doublet_size, ...
    doublet_time, wind_inertial, aircraft_parameters), [0 3], x_trim, opts);

u_hist_sp = BuildDoubletHistory(t_sp, u_trim, doublet_size, doublet_time);
PlotAircraftSim(t_sp', x_sp', u_hist_sp, 1:6, 'b-','Case 1: 3 Seconds', 1);

short_period = EstimateModeFromSignal(t_sp, x_sp(:,11), [0.5 1.5]); % q response
results.short_period = short_period;
results.short_period.t = t_sp;
results.short_period.x = x_sp;
results.short_period.u = u_hist_sp';

%% Problem 3.2: 100-second simulation for phugoid mode
[t_ph, x_ph] = ode45(@(t,x) AircraftEOMDoublet(t, x, u_trim, doublet_size, ...
    doublet_time, wind_inertial, aircraft_parameters), [0 100], x_trim, opts);

u_hist_ph = BuildDoubletHistory(t_ph, u_trim, doublet_size, doublet_time);
PlotAircraftSim(t_ph', x_ph', u_hist_ph, 7:12, 'r-', 'Case 2: 100 seconds',2);

phugoid = EstimateModeFromSignal(t_ph, x_ph(:,7), [5 100]); % u response
results.phugoid = phugoid;
results.phugoid.t = t_ph;
results.phugoid.x = x_ph;
results.phugoid.u = u_hist_ph';

%% Print results
fprintf('\nShort-period estimate (from q between 0.5 s and 1.5 s):\n');
PrintMode(short_period, 'rad/s');

fprintf('\nPhugoid estimate (from u between 5 s and 100 s):\n');
PrintMode(phugoid, 'rad/s');


function u_hist = BuildDoubletHistory(t, u_trim, doublet_size, doublet_time)
u_hist = repmat(u_trim(:), 1, numel(t));
for k = 1:numel(t)
    if t(k) > 0 && t(k) <= doublet_time
        u_hist(1,k) = u_hist(1,k) + doublet_size;
    elseif t(k) > doublet_time && t(k) <= 2*doublet_time
        u_hist(1,k) = u_hist(1,k) - doublet_size;
    end
end
end

function mode = EstimateModeFromSignal(t, y, time_window)
%ESTIMATEMODEFROMSIGNAL Estimates zeta and wn from same-sign peaks.
idx = t >= time_window(1) & t <= time_window(2);
tw = t(idx);
yw = y(idx);

if numel(tw) < 5
    error('Not enough data points inside the requested time window.');
end

% Remove bias using the mean of the last 20%% of the window
n_tail = max(3, round(0.2*numel(yw)));
yeq = mean(yw(end-n_tail+1:end));
yd = yw - yeq;

[pks_pos, tpk_pos] = LocalPeaks(tw, yd);
[pks_neg, tpk_neg] = LocalPeaks(tw, -yd);

if numel(pks_pos) >= numel(pks_neg)
    pks = pks_pos;
    tpk = tpk_pos;
else
    pks = pks_neg;
    tpk = tpk_neg;
end

mode = struct('num_peaks', numel(pks), 'peak_times', tpk, 'peak_values', pks, ...
              'wd', NaN, 'wn', NaN, 'Td', NaN, 'zeta', NaN);

mode.Td = mean(diff(tpk));
mode.wd = 2*pi / mode.Td;

delta = (1/(numel(pks)-1)) * log(pks(1)/pks(end));
mode.zeta = delta / sqrt(4*pi^2 + delta^2);
mode.wn = mode.wd / sqrt(1 - mode.zeta^2);
end

function [pks, tpk] = LocalPeaks(t, y)
%LOCALPEAKS Simple local-max detector without toolbox dependencies.
idx = find(y(2:end-1) > y(1:end-2) & y(2:end-1) >= y(3:end)) + 1;
if isempty(idx)
    pks = [];
    tpk = [];
    return;
end

threshold = 0.05 * max(abs(y));
keep = abs(y(idx)) >= threshold;
idx = idx(keep);

pks = y(idx);
tpk = t(idx);
end

function PrintMode(mode, units)
if isnan(mode.wn)
    fprintf('  unable to estimate mode from available peaks\n');
    return;
end
fprintf('  damped period Td  = %.4f s\n', mode.Td);
fprintf('  damped freq wd    = %.4f %s\n', mode.wd, units);
fprintf('  natural freq wn   = %.4f %s\n', mode.wn, units);
fprintf('  damping ratio zeta= %.4f\n', mode.zeta);
fprintf('  peaks used        = %d\n', mode.num_peaks);
end

