function xdot = AircraftEOM(time, aircraft_state, aircraft_surfaces, wind_inertial, aircraft_parameters)
    %unpacking state vector 
    % x = [xi, yi, zi, phi, theta, psi, u, v, w, p, q, r]
    phi = aircraft_state(4); theta = aircraft_state(5); psi = aircraft_state(6);
    u = aircraft_state(7); v = aircraft_state(8); w = aircraft_state(9);
    p = aircraft_state(10); q = aircraft_state(11); r = aircraft_state(12);
    
    m = aircraft_parameters.m;
    g = aircraft_parameters.g;
    Ix = aircraft_parameters.Ix; Iy = aircraft_parameters.Iy;
    Iz = aircraft_parameters.Iz; Ixz = aircraft_parameters.Ixz;

    % postion
    % go from body to inertial
    %R_bi is rotation matrix from body to inertial
    cph = cos(phi); sph = sin(phi); %setting up rotation matrix values
    cth = cos(theta); sth = sin(theta);
    cps = cos(psi); sps = sin(psi);
    
    R_bi = [cth*cps, sph*sth*cps-cph*sps, cph*sth*cps+sph*sps;
            cth*sps, sph*sth*sps+cph*cps, cph*sth*sps-sph*cps;
            -sth,    sph*cth,             cph*cth];
    
    pos_dot = R_bi * [u; v; w];

    % euler
    %see slides from 3/20 ish
    euler_dot = [1, sph*tan(theta), cph*tan(theta);
                 0, cph,           -sph;
                 0, sph/cth,        cph/cth] * [p; q; r];

    % aeroForcesandMoments function to get forces and moments
    %desntiy at boulder elevation
    density = 1.016; % given
    [Faero, Maero] = AeroForcesAndMoments(aircraft_state, aircraft_surfaces, wind_inertial, density, aircraft_parameters);
    
    % Gravity in body frame, 
    Fg_body = m * g * [-sth; sph*cth; cph*cth];
    
    accel = (1/m) * (Faero + Fg_body) - cross([p;q;r], [u;v;w]);
    % Angular acceleration solving for [pdot, qdot, rdot]
    % !! M = I * omegadot + omega x (I * omega) 
    I = [Ix, 0, -Ixz; 0, Iy, 0; -Ixz, 0, Iz];
    omega = [p; q; r];
    omega_dot = I \ (Maero - cross(omega, I * omega));

    
    %% end
    xdot = [pos_dot; euler_dot; accel; omega_dot];
end