function PlotAircraftSim(time, aircraft_state_array, control_input_array, fig, col, case_name, case_num)
offset = (case_num - 1) * 6;
    current_figs = fig + offset;

    %% Inertial Position
    figure(current_figs(1));
    labels = {'x-position (m)', 'y-position (m)', 'z-position (m)'};
    for i = 1:3
        subplot(3,1,i);
        plot(time, aircraft_state_array(i,:), col, 'DisplayName', case_name); hold on; grid on;
        ylabel(labels{i});
        if i == 3, xlabel('Time (s)'); end
    end
    sgtitle(['Inertial Position: ', case_name]);

    %% Euler Angles
    figure(current_figs(2));
    labels = {'Roll (deg)', 'Pitch (deg)', 'Yaw (deg)'};
    for i = 1:3
        subplot(3,1,i);
        plot(time, aircraft_state_array(i+3,:)*180/pi, col, 'DisplayName', case_name); hold on; grid on;
        ylabel(labels{i});
        if i == 3, xlabel('Time (s)'); end
    end
    sgtitle(['Euler Angles: ', case_name]);

    %% elocity in body frame
    figure(current_figs(3));
    labels = {'u (m/s)', 'v (m/s)', 'w (m/s)'};
    for i = 1:3
        subplot(3,1,i);
        plot(time, aircraft_state_array(i+6,:), col, 'DisplayName', case_name); hold on; grid on;
        ylabel(labels{i});
        if i == 3, xlabel('Time (s)'); end
    end
    sgtitle(['Body Velocity: ', case_name]);

    %% Angular Velocity
    figure(current_figs(4));
    labels = {'p (deg/s)', 'q (deg/s)', 'r (deg/s)'};
    for i = 1:3
        subplot(3,1,i);
        plot(time, aircraft_state_array(i+9,:)*180/pi, col, 'DisplayName', case_name); hold on; grid on;
        ylabel(labels{i});
        if i == 3, xlabel('Time (s)'); end
    end
    sgtitle(['Angular Velocity: ', case_name]);

    %% Control Inputs
    figure(current_figs(5));
    labels = {'\delta_e (deg)', '\delta_a (deg)', '\delta_r (deg)', '\delta_t (0-1)'};
    for i = 1:4
        subplot(4,1,i);
        val = control_input_array(i,:);
        if i < 4, val = val * 180/pi; end
        plot(time, val, col, 'DisplayName', case_name); hold on; grid on;
        ylabel(labels{i});
        if i == 4, xlabel('Time (s)'); end
    end
    sgtitle(['Control Inputs: ', case_name]);

    %% 3D Path
    figure(current_figs(6));
    x = aircraft_state_array(1,:);
    y = aircraft_state_array(2,:);
    z = -aircraft_state_array(3,:); 
    plot3(x, y, z, col); hold on; grid on;
    plot3(x(1), y(1), z(1), 'go', 'MarkerFaceColor','g');
    plot3(x(end), y(end), z(end), 'ro', 'MarkerFaceColor','r');
    xlabel('North (m)'); ylabel('East (m)'); zlabel('Altitude (m)');
    title(['3D Path - ', case_name]);
    axis equal;
end