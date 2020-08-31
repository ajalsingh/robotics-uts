%% Setup environment
% Ensure MATLAB path is root folder
clc;
clear;
close all;

addpath('UR3/');
addpath('ply/');
addpath('scripts/');

% Assign robot base and brick positions
ur5pos= [0.5014,-0.00575,-0.2556];
ur3pos=[-0.5,0,-0.25];
b1 = [1,-0.4,0.03]'; 
b2 = [1,-0.15,0.03]';
b3 = [1,0.1,0.03]';
b4 = [1,0.35,0.03]'; 
b5 = [1,0.6,0.03]';
b6 = [1,0.85,0.03]';
b7 = [-1,0.2,-0.2]';
b8 = [-1,-0.05,-0.2]';
b9 = [-1,-0.3,-0.2]';
bricks = sortrows([b1 b2 b3 b4 b5 b6 b7 b8 b9]');

%Assign drop locations 
d1 = [-0.2,-0.1,-0.25]'; 
d2 = [-0.2,0.05,-0.25]';
d3 = [-0.2,0.2,-0.25]';
d4 = [-0.2,-0.1,-0.17]'; 
d5 = [-0.2,0.05,-0.17]';
d6 = [-0.2,0.2,-0.17]';
d7 = [-0.2,-0.1,-0.09]'; 
d8 = [-0.2,0.05,-0.09]';
d9 = [-0.2,0.2,-0.09]';
drop = [d1 d2 d3 d4 d5 d6 d7 d8 d9]';

% Setup environment
[ur3, ur5, brickMesh] = worldsetup(ur3pos, ur5pos, bricks);
ur3Origin = ur3.model.fkine(ur3.model.getpos);
ur5Origin = ur5.model.fkine(ur5.model.getpos);


%% Maximum reach

distances = robotpointcloud(ur3.model, ur5.model);

fprintf('\nUR3:\nMax Reach(x/y): %f m\nMax Reach(z): %f m\nApprox Volume: %f m3\n', distances(1), distances(2), distances(3));
fprintf('\nUR5:\nMax Reach(x/y): %f m\nMax Reach(z): %f m\nApprox Volume: %f m3\n', distances(4), distances(5), distances(6));
%% Build a Wall
armMotionComplete = 0
while size(bricks,1) > 0
    if size(bricks,1) > 1
        if bricks(1,1) == bricks(end,1)
            if bricks(1,1) >= 0
                %ur5
                if armMotionComplete ==0
                    MoveSingleArm(ur3.model,ur3Origin(1:3,4)');
                end
                MoveSingleArm(ur5.model,bricks(1,:));
                MoveSingleArm(ur5.model,drop(1,:),brickMesh(1));
            else
                %ur3
                if armMotionComplete == 0
                    MoveSingleArm(ur5.model,ur5Origin(1:3,4)');
                end
                MoveSingleArm(ur3.model,bricks(1,:));
                MoveSingleArm(ur3.model,drop(1,:),brickMesh(1));
            end
            bricks(1,:) = [];
            drop(1,:) = [];
            brickMesh(1) = [];
            armMotionComplete = 1;
        else
            MoveArms(ur3.model,ur5.model,bricks(1,:),bricks(end,:));
            MoveArms(ur3.model,ur5.model,drop(1,:),drop(2,:),brickMesh(1),brickMesh(end));

            bricks(1,:) = [];
            bricks(end,:) = [];
            drop(1,:) = [];
            drop(1,:) = [];
            brickMesh(1) = [];
            brickMesh(end) = [];
        end
        
        
    else
        if bricks(1)>=0
            MoveSingleArm(ur5.model,bricks(1,:));
            MoveSingleArm(ur5.model,drop(1,:),brickMesh(1));
        else
            MoveSingleArm(ur3.model,bricks(end,:));
            MoveSingleArm(ur3.model,drop(1,:),brickMesh(1));
        end
        
        bricks(1,:) = [];
        drop(1,:) = [];
        brickMesh(1) = [];
    end
end

MoveSingleArm(ur3.model,ur3Origin(1:3,4)');
MoveSingleArm(ur5.model,ur5Origin(1:3,4)');

%%
MoveSingleArm(ur3.model,bricks(1,1:3));
MoveSingleArm(ur3.model,drop(1,:),brickMesh(9));
MoveSingleArm(ur3.model,bricks(2,1:3));
MoveSingleArm(ur3.model,drop(2,:),brickMesh(8));
MoveSingleArm(ur3.model,bricks(3,1:3));
MoveSingleArm(ur3.model,drop(3,:),brickMesh(7));
MoveSingleArm(ur3.model,ur3Origin(1:3,4)');

MoveSingleArm(ur5.model,bricks(6,1:3)); 
MoveSingleArm(ur5.model,drop(4,:),brickMesh(6));
MoveSingleArm(ur5.model,bricks(5,1:3)); 
MoveSingleArm(ur5.model,drop(5,:),brickMesh(5));
MoveSingleArm(ur5.model,bricks(4,1:3)); 
MoveSingleArm(ur5.model,drop(6,:),brickMesh(4));
MoveSingleArm(ur5.model,bricks(8,1:3)); 
MoveSingleArm(ur5.model,drop(7,:),brickMesh(3));
MoveSingleArm(ur5.model,bricks(7,1:3)); 
MoveSingleArm(ur5.model,drop(8,:),brickMesh(2));
MoveSingleArm(ur5.model,bricks(6,1:3)); 
MoveSingleArm(ur5.model,drop(9,:),brickMesh(1));
MoveSingleArm(ur5.model,ur5Origin(1:3,4)');
