%% Robotics
% Lab 11 - Question 2 skeleton code

%% setup joystick
id = 1; % Note: may need to be changed if multiple joysticks present
joy = vrjoystick(id);
caps(joy) % display joystick information


%% Set up robot
mdl_puma560;                    % Load Puma560
robot = p560;                   % Create copy called 'robot'
robot.tool = transl(0.1,0,0);   % Define tool frame on end-effector


%% Start "real-time" simulation
q = qn;                 % Set initial robot configuration 'q'

HF = figure(1);         % Initialise figure to display robot
robot.plot(q);          % Plot robot in initial configuration
robot.delay = 0.001;    % Set smaller delay when animating
set(HF,'Position',[0.1 0.1 0.8 0.8]);

duration = 30;  % Set duration of the simulation (seconds)
dt = 0.15;      % Set time step for simulation (seconds)

n = 0;  % Initialise step count to zero 
tic;    % recording simulation start time
while( toc < duration)
    
    n=n+1; % increment step count

    % read joystick
    [axes, buttons, povs] = read(joy);
       
    % -------------------------------------------------------------
    % YOUR CODE GOES HERE
    % 1 - turn joystick input into an end-effector velocity command
    % 2 - use J inverse to calculate joint velocity
    % 3 - apply joint velocity to step robot joint angles 
    % -------------------------------------------------------------
    kv = 0.2; % linear vel gain
    kw = 0.5; % angular vel gain
    
    vx = kv*axes(1);
    vy = kv*axes(2);
    if buttons(5)==0
        vz = kv/2*axes(3)+0.1;
    else
        vz = -(kv/2*axes(3)+0.1);
    end
    
    wx = 2*kw*axes(4);
    wy = 2*kw*axes(5);
    if buttons(6)==0
        wz = kw*axes(6)+0.5;
    else
        wz = -(kw*axes(6)+0.5);
    end
    
    
    dx = [vx;vy;vz;wx;wy;wz]
    
    % 2 - use J inverse to calcualte joint velocity
    J = robot.jacob0(q);
    
    %dx = J*dq
    dq = inv(J)*dx;
    
    % 3 - apply joint velocity to step robot joint angles 
    q = q + (dt*dq)'; 
    

    % Update plot
    robot.animate(q);  
    
    % wait until loop time elapsed
    if (toc > dt*n)
        warning('Loop %i took too much time - consider increating dt',n);
    end
    while (toc < dt*n); % wait until loop time (dt) has elapsed 
    end
end
      
