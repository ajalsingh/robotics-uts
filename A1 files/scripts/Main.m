% function Main()
%MAIN carries out the main tasks as required

%% Setup
%   clear workspace and add relevant paths
clear;
close all;

addpath('UR3/');
addpath('ply/');
addpath('scripts/');

% Assign robot base positions
ur5pos= [0.45,0,-0.25];
ur3pos=[-0.5,0,-0.15];

% build environment
env = Environment(ur3pos, ur5pos);
[ur3,ur5] = env.BuildWorld();

% Create instance of manipulator class to control arms
arms = Manipulator;
arms.MoveSingleArm(ur5.model, [0 -pi/2 0 pi/2 0 pi/2 0]);

% save initial poses of robots
ur3Origin = ur3.model.getpos;
ur5Origin = ur5.model.getpos;

%% Place Bricks

% Enter desired brick positions below:
b1 = [1,    -0.4,   0.03]'; 
b2 = [1,    -0.15,  0.03]';
b3 = [1,    0.1,    0.03]';
b4 = [1,    0.35,   0.03]'; 
b5 = [1,    0.6,    0.03]';
b6 = [1,    0.85,   0.03]';
b7 = [-1,   0.2,    -0.2]';
b8 = [-1,   -0.05,  -0.2]';
b9 = [-1,   -0.3,   -0.2]';
brickPoses = sortrows([b1 b2 b3 b4 b5 b6 b7 b8 b9]');

% Create and plot bricks
[bricksObj, brickMesh] = Bricks(brickPoses);

%Assign drop locations 
% X(min,max): 0.04, -0.025
d1 = [-0.2, -0.1, -0.1]'; 
drop = bricksObj.CalcDropPoses(d1);

%% Get max reach and volumes

values = arms.RobotPointCloud(ur3.model,ur5.model);
fprintf('\nUR3:\nMax Reach(x/y): %f m\nMax Reach(z): %f m\nApprox Volume: %f m3\n', values(1), values(2), values(3));
fprintf('\nUR5:\nMax Reach(x/y): %f m\nMax Reach(z): %f m\nApprox Volume: %f m3\n', values(4), values(5), values(6));

%% Build a Wall
% iterate through array of bricks while they have not been moved. Once each
% brick has been removed, it is removed from the array and forgotten.       
% If an arm has moved all of its allocated bricks, it returns to its
% initial position and allows the other arm to continue.
% Once the wall is built, both arms return to initial positions

while size(brickPoses,1) > 0
    
    % Move 2 bricks at a time
    if size(brickPoses,1) > 1 
        
        % Check whether remaining bricks are on 1 side and allocate
        % responsible arm. Other arm returns to initial joint states
        if brickPoses(1,1) == brickPoses(end,1)
            
            %ur5
            if brickPoses(1,1) >= 0
                arms.MoveSingleArm(ur5.model,brickPoses(1,:));
                arms.MoveSingleArm(ur5.model, ur5Origin,brickMesh(1));
                arms.MoveSingleArm(ur5.model,drop(1,:),brickMesh(1));
                arms.MoveSingleArm(ur5.model, ur5Origin);
            
            %ur3
            else
                arms.MoveSingleArm(ur3.model,brickPoses(1,:));
                arms.MoveSingleArm(ur3.model, ur3Origin);
                arms.MoveSingleArm(ur3.model,drop(1,:),brickMesh(1));
                arms.MoveSingleArm(ur3.model, ur3Origin);
            end
            brickPoses(1,:) = [];
            drop(1,:) = [];
            brickMesh(1) = [];
            armMotionComplete = 1;
            
        %   Move both arms  
        else
            arms.MoveArms(ur3.model,ur5.model,brickPoses(1,:),brickPoses(end,:));
            arms.MoveArms(ur3.model,ur5.model,ur3Origin,ur5Origin,brickMesh(1),brickMesh(end));
            arms.MoveArms(ur3.model,ur5.model,drop(1,:),drop(2,:),brickMesh(1),brickMesh(end));
            arms.MoveArms(ur3.model,ur5.model,ur3Origin,ur5Origin);

            brickPoses(1,:) = [];
            brickPoses(end,:) = [];
            drop(1,:) = [];
            drop(1,:) = [];
            brickMesh(1) = [];
            brickMesh(end) = [];
        end
        
    % If only 1 brick remains, determine the arm responsible and move brick with single arm    
    else 
        %ur5
        if brickPoses(1,1) >= 0
            arms.MoveSingleArm(ur5.model,brickPoses(1,:));
            arms.MoveSingleArm(ur5.model, ur5Origin,brickMesh(1));
            arms.MoveSingleArm(ur5.model,drop(1,:),brickMesh(1));
            arms.MoveSingleArm(ur5.model, ur5Origin);

        %ur3
        else
            arms.MoveSingleArm(ur3.model,brickPoses(1,:));
            arms.MoveSingleArm(ur3.model, ur3Origin);
            arms.MoveSingleArm(ur3.model,drop(1,:),brickMesh(1));
            arms.MoveSingleArm(ur3.model, ur3Origin);
        end
        
        brickPoses(1,:) = [];
        drop(1,:) = [];
        brickMesh(1) = [];
    end
end

arms.MoveArms(ur3.model,ur5.model,ur3Origin,ur5Origin);

%% Bonus Marks: Rosbag simulation
rosbag = CloneRosbag;

% end

