% function Main()
%MAIN Summary of this function goes here
%   Detailed explanation goes here
clear;
close all;

addpath('UR3/');
addpath('ply/');
addpath('scripts/');

% Assign robot base and brick positions
ur5pos= [0.5014,-0.00575,-0.2556];
ur3pos=[-0.52,0,-0.25];

env = Environmnet(ur3pos, ur5pos);
[ur3,ur5] = env.BuildWorld();

%% Place Bricks
ur3Origin = ur3.model.fkine(ur3.model.getpos);
ur5Origin = ur5.model.fkine(ur5.model.getpos);

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

[bricksObj, brickMesh] = Bricks(brickPoses);

%%%%%%%%%%%%%%%%%% If bricks poses need to be modified, comment line 30,
%%%%%%%%%%%%%%%%%% change poses above and run line 34
% bricksObj.modifyPoses(brickMesh, brickPoses);

%Assign drop locations 
% X(min,max): 0.04, -0.025
% Y(min,max): 
d1 = [-0.2, -0.1,       -0.25]'; 
drop = bricksObj.CalcDropPoses(d1);

arms = Manipulator;

%%

values = arms.RobotPointCloud(ur3.model,ur5.model);
fprintf('\nUR3:\nMax Reach(x/y): %f m\nMax Reach(z): %f m\nApprox Volume: %f m3\n', values(1), values(2), values(3));
fprintf('\nUR5:\nMax Reach(x/y): %f m\nMax Reach(z): %f m\nApprox Volume: %f m3\n', values(4), values(5), values(6));

%% Build a Wall
armMotionComplete = 0;

while size(brickPoses,1) > 0
    if size(brickPoses,1) > 1
        if brickPoses(1,1) == brickPoses(end,1)
            if brickPoses(1,1) >= 0
                %ur5
                if armMotionComplete ==0
                    arms.MoveSingleArm(ur3.model,ur3Origin(1:3,4)');
                end
                arms.MoveSingleArm(ur5.model,brickPoses(1,:));
                arms.MoveSingleArm(ur5.model,drop(1,:),brickMesh(1));
            else
                %ur3
                if armMotionComplete == 0
                    arms.MoveSingleArm(ur5.model,ur5Origin(1:3,4)');
                end
                arms.MoveSingleArm(ur3.model,brickPoses(1,:));
                arms.MoveSingleArm(ur3.model,drop(1,:),brickMesh(1));
            end
            brickPoses(1,:) = [];
            drop(1,:) = [];
            brickMesh(1) = [];
            armMotionComplete = 1;
        else
            arms.MoveArms(ur3.model,ur5.model,brickPoses(1,:),brickPoses(end,:));
            arms.MoveArms(ur3.model,ur5.model,drop(1,:),drop(2,:),brickMesh(1),brickMesh(end));

            brickPoses(1,:) = [];
            brickPoses(end,:) = [];
            drop(1,:) = [];
            drop(1,:) = [];
            brickMesh(1) = [];
            brickMesh(end) = [];
        end
        
        
    else
        if brickPoses(1)>=0
            arms.MoveSingleArm(ur5.model,brickPoses(1,:));
            arms.MoveSingleArm(ur5.model,drop(1,:),brickMesh(1));
        else
            arms.MoveSingleArm(ur3.model,brickPoses(end,:));
            arms.MoveSingleArm(ur3.model,drop(1,:),brickMesh(1));
        end
        
        brickPoses(1,:) = [];
        drop(1,:) = [];
        brickMesh(1) = [];
    end
end

arms.MoveArms(ur3.model,ur5.model,ur3Origin(1:3,4)',ur5Origin(1:3,4)');

%% Bonus Marks: Rosbag
rosbag = CloneRosbag;

% end

