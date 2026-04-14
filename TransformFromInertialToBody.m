function [vector_body] = TransformFromInertialToBody(vector_inertial, euler_angles)
    phi   = euler_angles(1); % Roll
    theta = euler_angles(2); % Pitch
    psi   = euler_angles(3); % Yaw
    sph = sin(phi);   cph = cos(phi);
    sth = sin(theta); cth = cos(theta);
    sps = sin(psi);   cps = cos(psi);

    % 321DCM
    % INERTIAL TO BODY****
    R = [cth*cps,                          cth*sps,                          -sth;
         sph*sth*cps - cph*sps,            sph*sth*sps + cph*cps,            sph*cth;
         cph*sth*cps + sph*sps,            cph*sth*sps - sph*cps,            cph*cth];

    % out
    vector_body = R * vector_inertial;

end