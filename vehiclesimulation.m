% Vehicle Parameters
L = 2; % Wheelbase of the vehicle (in meters)
v = 1; % Constant velocity (m/s)
dt = 0.2; % Time step (seconds)

% Initial Coordinates and Orientation
x = 0; % Initial X coordinate
y = 0; % Initial Y coordinate
theta = 0; % Initial orientation (radians)

% PID Controller Parameters
Kp = 0.3;   % Proportional gain (slightly increased)
Ki = 0.02;  % Integral gain (slightly increased)
Kd = 0.1;   % Derivative gain (slightly increased)

% Target Coordinates
target_x = 10; % Target X coordinate
target_y = 10; % Target Y coordinate

% Initialize error and integral terms
prev_error = 0; % Previous error (for derivative calculation)
integral = 0;   % Integral of the error

% PID Control Function
function [steering_angle, prev_error, integral] = pid_control(x, y, theta, target_x, target_y, Kp, Ki, Kd, prev_error, integral)
    % Calculate the error between the target and the current position
    error = atan2(target_y - y, target_x - x) - theta;
    
    % PID correction term
    integral = integral + error;
    derivative = error - prev_error;
    steering_angle = Kp * error + Ki * integral + Kd * derivative;
    
    % Return updated error values
    prev_error = error;
end

% Simulation Loop
for t = 0:dt:50 % Run simulation for 50 seconds
    % Compute the steering angle using PID control
    [steering_angle, prev_error, integral] = pid_control(x, y, theta, target_x, target_y, Kp, Ki, Kd, prev_error, integral);
    
    % Update the vehicle's orientation and position
    theta = theta + steering_angle * dt; % Update orientation
    x = x + v * cos(theta) * dt; % Update X coordinate
    y = y + v * sin(theta) * dt; % Update Y coordinate
    
    % Visualization
    plot(x, y, 'bo', 'MarkerFaceColor', 'b'); % Plot vehicle's current position
    hold on;
    plot(target_x, target_y, 'rx', 'MarkerSize', 10); % Plot the target position
    axis([0 15 0 15]); % Set axis limits
    title('Autonomous Ground Vehicle Simulation');
    xlabel('X Coordinate');
    ylabel('Y Coordinate');
    drawnow; % Update visualization in real-time
    
    % Pause to control simulation speed
    pause(0.1);
end
