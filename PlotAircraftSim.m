
function PlotAircraftSim(time, aircraft_state_array, control_input_array,...
    fig, col)

%{
Purpose: Plots aircraft states, path, and control surface values with time,
with each function call plotting over the previous one 

inputs: 

time: length n vector that holds time corresponding to the 
nth set of variables

aircraft_state_array: 12 x n array of aircraft states

control_input_array: 4 x n array of control surfaces

fig: 6 x 1 vector of figure numbers to plot over

col: string indicating plotting option used for every plot eg. col = 'b-'

outputs: none
%}

%inertial position
figure(fig(1));
subplot(3,1,1);
plot(time, aircraft_state_array(1,:), col); hold on; grid on;
ylabel('x-position (m)')
subplot(3,1,2);
plot(time, aircraft_state_array(2,:), col); hold on; grid on;
ylabel('y-position (m)')
subplot(3,1,3);
plot(time, aircraft_state_array(3,:), col); hold on; grid on;
ylabel('z-position (m)')
xlabel('Time (s)')
sgtitle('Inertial Position');
ylim(ylim + [-0.1, 0.1]);

%euler angles
figure(fig(2));
subplot(3,1,1);
plot(time, aircraft_state_array(4,:)*180/pi, col); hold on; grid on;
ylabel('Roll (rad)')
ylim(ylim + [-0.1, 0.1]);
subplot(3,1,2);
plot(time, aircraft_state_array(5,:)*180/pi, col); hold on; grid on;
ylabel('Pitch (rad)')
subplot(3,1,3);
plot(time, aircraft_state_array(6,:)*180/pi, col); hold on; grid on;
xlabel('Time (s)')
ylabel('Yaw (rad)')
sgtitle('Euler Angles');

% intertial velocity
figure(fig(3));
subplot(3,1,1);
plot(time, aircraft_state_array(7,:), col); hold on; grid on;
ylabel('x-velocity (m/s)')
ylim(ylim + [-0.1, 0.1]);
subplot(3,1,2);
plot(time, aircraft_state_array(8,:), col); hold on; grid on;
ylabel('y-velocity (m/s)')
ylim(ylim + [-0.1, 0.1]);
subplot(3,1,3);
plot(time, aircraft_state_array(9,:), col); hold on; grid on;
ylabel('z-velocity (m/s)')
xlabel('Time (s)')
sgtitle('Inertial Velocity in Body Frame');
ylim(ylim + [-0.1, 0.1]);

% Angular Velocity
figure(fig(4));
subplot(3,1,1);
plot(time, aircraft_state_array(10,:)*180/pi, col); hold on; grid on;
ylabel('p (rad/s)')
subplot(3,1,2);
plot(time, aircraft_state_array(11,:)*180/pi, col); hold on; grid on;
ylabel('q (rad/s)')
subplot(3,1,3);
plot(time, aircraft_state_array(12,:)*180/pi, col); hold on; grid on;
ylabel('r (rad/s)')
xlabel('Time (s)')
sgtitle('Angular Velocity');

% Control input variables
figure(fig(5));
subplot(4,1,1);
plot(time, control_input_array(1,:)*180/pi, col); hold on; grid on;
ylabel("Elevator Deflection \delta_e (deg)")
subplot(4,1,2);
plot(time, control_input_array(2,:)*180/pi, col); hold on; grid on;
ylabel("Aileron Deflection \delta_a (deg)")
subplot(4,1,3);
plot(time, control_input_array(3,:)*180/pi, col); hold on; grid on;
ylabel("Rudder Deflection \delta_r (deg)")
subplot(4,1,4);
plot(time, control_input_array(4,:)*180/pi, col); hold on; grid on;
ylabel("Throttle \delta_t (fraction from 0 to 1)")
xlabel('Time (s)')
sgtitle('Control Input Variables');

% 3D path of aircraft
figure(fig(6));
grid on;
x = aircraft_state_array(1,:);
y = aircraft_state_array(2,:);
z = -aircraft_state_array(3,:);

plot3(x, y, z, col); hold on; 

% make 3d plot axis better
xlims = xlim;
ylims = ylim;
zlims = zlim;
axis square;
grid on;

midpts = [mean(xlims), mean(ylims), mean(zlims)];
maxspan = max([(xlims(2) - xlims(1)), (ylims(2) - ylims(1)), (zlims(2) - zlims(1))]);
x_lims = [midpts(1) - maxspan/2, midpts(1) + maxspan/2];
y_lims = [midpts(2) - maxspan/2, midpts(2) + maxspan/2];
z_lims = [midpts(3) - maxspan/2, midpts(3) + maxspan/2];

axis([x_lims y_lims z_lims]);




% Start and finish
plot3(x(1),   y(1),   z(1),   'go', 'MarkerFaceColor','g');
plot3(x(end), y(end), z(end), 'ro', 'MarkerFaceColor','r');
xlabel('North Positon (m)')
ylabel('East Positon (m)')
zlabel('Down Positon (m)')
title('3D Path of Aircraft')
end