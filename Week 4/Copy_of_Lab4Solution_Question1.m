%% Robotics
% Lab 4 - Questions 1: 3-link plannar draw a line then circle
function [  ] = Lab4Solution_Question1( )

clf

% Make a 3DOF model
L1 = Link('d',0.1519,'a',0,'alpha',pi/2,'qlim',deg2rad([-360 360]), 'offset', 0);
L2 = Link('d',0,'a',-0.24365,'alpha',0,'qlim', deg2rad([-90 90]), 'offset',-pi/2); % was 'offset',pi/2
L3 = Link('d',0,'a',-0.21325,'alpha',0,'qlim', deg2rad([-170 170]), 'offset', pi/2);
L4 = Link('d',0.11235,'a',0,'alpha',pi/2,'qlim',deg2rad([-360 360]),'offset', -pi/2); % was 'offset',pi/2
L5 = Link('d',0.08535,'a',0,'alpha',-pi/2,'qlim',deg2rad([-360,360]), 'offset',0);
L6 = Link('d',0.0819,'a',0,'alpha',0,'qlim',deg2rad([-360,360]), 'offset', 0);        
robot = SerialLink([L1 L2 L3 L4 L5 L6],'name','myRobot');                     


% Rotate the base around the Y axis so the Z axis faces downways
% robot.base = troty(pi);

% Make workspace big except inthe negative z
workspace = [-1 1 -1 1 -0.05 1];                                       % Set the size of the workspace when drawing the robot
        
scale = 0.5;
        
q = zeros(1,6);                                                     % Create a vector of initial joint angles
        
robot.plot(q,'workspace',workspace,'scale',scale);                  % Plot the robot
        
robot.teach;
% display('Press enter to continue');
% pause;
%%
% Get a solution for the end effector at [-0.75,-0.5,0]. Note that from
% teach you can see that there is no way to affect the Z, roll or pitch values no matter
% what you choose. Since the pen is considered to be the Z axis then we
% don't care about the yaw angle (if pen where the Z axis rotating it
% doesn't affect the result, so we just mask that out
newQ = robot.ikine(transl(-0.6,0.1,0.1),q,[1,1,0,0,0,0]);

% Now plot this new joint state
robot.plot(newQ)

% Add a plotting trail. Alternatively, close the figure and run: robot.plot(newQ,'trail','-')
rh = findobj('Tag', robot.name); ud = rh.UserData; hold on; ud.trail = plot(0,0,'-'); set(rh,'UserData',ud);

% Check how close it got to the goal transform of transl(-0.75,-0.5,0)
robot.fkine(newQ)
%%
% Now go through a loop using the previous joint as the guess to draw a
% line from [-0.75,-0.5,0] to [-0.75,0.5,0]
for y = -0.1:0.005:0.1
    newQ = robot.ikine(transl(-0.6,y,0.1),newQ,[1,1,0,0,0,0]);%,'alpha',0.01);
    robot.animate(newQ);
    drawnow();
end
% display('Press enter to continue');
% pause;

%% Now use the same robot and draw a circle around the robot at a distance of 0.5
hold on;
plot(-0.5,0,'r.')

for circleHalf = 1:2
    for x = -0.5:0.05:0.5
        if circleHalf == 1
            y = sqrt(0.5^2-x^2)
        else
            x = -x;
            y = -sqrt(0.5^2-x^2);
        end
        plot(x,y,'r.');
        newQ = robot.ikine(transl(x,y,0),newQ,[1,1,0,0,0,0]);%,'alpha',0.01);
        robot.animate(newQ);
        drawnow();
    end
end

end

