% Connect to CoppeliaSim
sim = remApi('remoteApi'); % Using the remote API of CoppeliaSim
sim.simxFinish(-1); % Close any previous connections
clientID = sim.simxStart('127.0.0.1', 19997, true, true, 5000, 5);

if clientID > -1
    disp('Connected to remote API server');
    
    % Enable the synchronous mode
    sim.simxSynchronous(clientID, true);
    
    % Start the simulation
    sim.simxStartSimulation(clientID, sim.simx_opmode_blocking);
    
    % Get handles to the quadcopter, pen, and vision sensors
    [~, quadcopter] = sim.simxGetObjectHandle(clientID, 'Quadcopter', sim.simx_opmode_blocking);
    [~, pen] = sim.simxGetObjectHandle(clientID, 'feltPen', sim.simx_opmode_blocking);
    [~, visionSensor] = sim.simxGetObjectHandle(clientID, 'Vision_sensor0', sim.simx_opmode_blocking);
    [~, visionSensorAbove] = sim.simxGetObjectHandle(clientID, 'Vision_sensor', sim.simx_opmode_blocking);

    % Check if object handles were retrieved successfully
    if quadcopter > -1 && pen > -1 && visionSensor > -1 && visionSensorAbove > -1
        disp('Quadcopter, pen, and vision sensor handles retrieved successfully');
    else
        disp('Failed to retrieve quadcopter, pen, or vision sensors handles');
        sim.simxFinish(clientID);
        sim.delete();
        return;
    end
    
    % Define parameters for drawing a circle
    radius = 0.2; % Radius of the circle
    draw_plane_height = 0; % Height of the drawing plane
    target_pen_height = 0; % Desired distance from pen tip to drawing plane
    pen_to_quadcopter_offset = 0.065; % Height offset from quadcopter base to pen tip 
    
    % Calculate the target quadcopter height
    target_quadcopter_height = draw_plane_height + target_pen_height + pen_to_quadcopter_offset;
    
    % Waypoints for the quadcopter to follow (XY plane)
    num_waypoints = 100; % Number of waypoints to approximate the circle
    theta = linspace(0, 2*pi, num_waypoints);
    [~, initialPosition] = sim.simxGetObjectPosition(clientID, quadcopter, -1, sim.simx_opmode_blocking);
    center_x = initialPosition(1);
    center_y = initialPosition(2);
    waypoints = [center_x + radius * cos(theta)', center_y + radius * sin(theta)'];
    
    % PID controller parameters for XY and Z control
    Kp_xy = 0.2;
    Ki_xy = 0.0;
    Kd_xy = 0.0;
    
    Kp_z = 1;
    Ki_z = 0.02;
    Kd_z = 0.01;
    
    % Initialize PID variables for XY and Z control
    integral_xy = [0, 0];
    previous_error_xy = [0, 0];
    
    integral_z = 0;
    previous_error_z = 0;
    
    % Initialize data collection for plotting
    time_steps = [];
    pen_heights = [];
    currentTime = 0;
    dt = 0.05; % Time step for simulation
    
    % Video recording setup
    videoWriter = VideoWriter('quadcopter_circle.avi');
    open(videoWriter);
    
    % Move the quadcopter through the waypoints
    for i = 1:size(waypoints, 1)
        targetPosition = [waypoints(i, :), target_quadcopter_height];
        
        while true
            % Get current position
            [~, currentPosition] = sim.simxGetObjectPosition(clientID, quadcopter, -1, sim.simx_opmode_blocking);
            
            % Calculate XY errors
            error_xy = targetPosition(1:2) - currentPosition(1:2);
            
            % Calculate Z error
            error_z = targetPosition(3) - currentPosition(3);
            
            % Break if close enough in XY and Z
            if norm(error_xy) < 0.05 && abs(error_z) < 0.05
                break;
            end
            
            % Update integral and derivative terms for XY control
            integral_xy = integral_xy + error_xy * dt;
            derivative_xy = (error_xy - previous_error_xy) / dt;
            previous_error_xy = error_xy;
            
            % Update integral and derivative terms for Z control
            integral_z = integral_z + error_z * dt;
            derivative_z = (error_z - previous_error_z) / dt;
            previous_error_z = error_z;
            
            % Calculate control signals for XY and Z
            control_signal_xy = Kp_xy * error_xy + Ki_xy * integral_xy + Kd_xy * derivative_xy;
            control_signal_z = Kp_z * error_z + Ki_z * integral_z + Kd_z * derivative_z;
            
            % Update target position
            new_position = [currentPosition(1) + control_signal_xy(1), ...
                            currentPosition(2) + control_signal_xy(2), ...
                            currentPosition(3) + control_signal_z];
            
            sim.simxSetObjectPosition(clientID, quadcopter, -1, new_position, sim.simx_opmode_oneshot);
            
            % Synchronous trigger
            sim.simxSynchronousTrigger(clientID);
            
            % Collect data for plotting
            pen_height = currentPosition(3) - pen_to_quadcopter_offset; % Height of the pen tip from the plane
            time_steps = [time_steps, currentTime];
            pen_heights = [pen_heights, pen_height];
            currentTime = currentTime + dt;
            
            % Pause for simulation step
            pause(dt);
            
            % Capture image from the above sensor and write to video
            [~, ~, imageAbove] = sim.simxGetVisionSensorImage2(clientID, visionSensorAbove, 0, sim.simx_opmode_blocking);
            writeVideo(videoWriter, imageAbove);
        end
    end
    
    % Move the quadcopter up to the specified height
    up_height = 1.5; % Height to move up to take a picture
    [~, currentPosition] = sim.simxGetObjectPosition(clientID, quadcopter, -1, sim.simx_opmode_blocking);
    targetPositionUp = [currentPosition(1:2), up_height];
    
    while true
        [~, currentPosition] = sim.simxGetObjectPosition(clientID, quadcopter, -1, sim.simx_opmode_blocking);
        
        % Calculate Z error for up position
        error_z = targetPositionUp(3) - currentPosition(3);
        
        % Break if close enough in Z
        if abs(error_z) < 0.05
            break;
        end
        
        % Update integral and derivative terms for Z control
        integral_z = integral_z + error_z * dt;
        derivative_z = (error_z - previous_error_z) / dt;
        previous_error_z = error_z;
        
        % Calculate control signal for Z
        control_signal_z = Kp_z * error_z + Ki_z * integral_z + Kd_z * derivative_z;
        
        % Update target position
        new_position_up = [currentPosition(1:2), currentPosition(3) + control_signal_z];
        
        sim.simxSetObjectPosition(clientID, quadcopter, -1, new_position_up, sim.simx_opmode_oneshot);
        
        % Synchronous trigger
        sim.simxSynchronousTrigger(clientID);
        
        % Pause for simulation step
        pause(dt);
        
        % Capture image from the above sensor and write to video
        [~, ~, imageAbove] = sim.simxGetVisionSensorImage2(clientID, visionSensorAbove, 0, sim.simx_opmode_blocking);
        writeVideo(videoWriter, imageAbove);
    end
    
    % Move the quadcopter to the center of the circle
    center_position = [center_x, center_y, up_height];
    
    while true
        [~, currentPosition] = sim.simxGetObjectPosition(clientID, quadcopter, -1, sim.simx_opmode_blocking);
        
        % Calculate XY errors for center position
        error_xy = center_position(1:2) - currentPosition(1:2);
        
        % Break if close enough in XY
        if norm(error_xy) < 0.05
            break;
        end
        
        % Update integral and derivative terms for XY control
        integral_xy = integral_xy + error_xy * dt;
        derivative_xy = (error_xy - previous_error_xy) / dt;
        previous_error_xy = error_xy;
        
        % Calculate control signals for XY
        control_signal_xy = Kp_xy * error_xy + Ki_xy * integral_xy + Kd_xy * derivative_xy;
        
        % Update target position
        new_position_center = [currentPosition(1) + control_signal_xy(1), ...
                               currentPosition(2) + control_signal_xy(2), ...
                               up_height];
        
        sim.simxSetObjectPosition(clientID, quadcopter, -1, new_position_center, sim.simx_opmode_oneshot);
        
        % Synchronous trigger
        sim.simxSynchronousTrigger(clientID);
        
        % Pause for simulation step
        pause(dt);
        
        % Capture image from the above sensor and write to video
        [~, ~, imageAbove] = sim.simxGetVisionSensorImage2(clientID, visionSensorAbove, 0, sim.simx_opmode_blocking);
        writeVideo(videoWriter, imageAbove);
    end
    
    % Capture an image from the vision sensor
    [~, resolution, image] = sim.simxGetVisionSensorImage2(clientID, visionSensor, 0, sim.simx_opmode_blocking);

    % Stop the simulation
    sim.simxStopSimulation(clientID, sim.simx_opmode_blocking);
    
    % Close the connection to CoppeliaSim
    sim.simxFinish(clientID);
    
    % Close the video writer
    close(videoWriter);
else
    disp('Failed connecting to remote API server');
end

sim.delete(); % Call the destructor!

% Plot the change in the distance between the pen and the plane
figure;
plot(time_steps, pen_heights);
xlabel('Time (s)');
ylabel('Pen Height (m)');
title('Pen Height over Time');
grid on;

% Display the captured image
figure;
imshow(image);
title('Captured Image of the Drawing');
