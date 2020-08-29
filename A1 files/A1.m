%% Maximum reach
clc;
clear;


ur3 = UR3;
ur5 = LinearUR5(false);
close all;

distances = pointCloud(ur3.model, ur5.model);

texta = ['UR3:  Max vertical reach: ', num2str(distances(1)), ' and Max horizontal reach: ', num2str(distances(2))];
textb = ['UR5:  Max vertical reach: ', num2str(distances(3)), ' and Max horizontal reach: ', num2str(distances(4))];
disp(texta);
disp(textb);
%% Setup environment
clc;
clear;
close all;

% Assign robot base and brick positions
ur3pos= [0.5014,-0.00575,-0.2556];
ur5pos=[-0.5,0,-0.25];
b1 = [1,-0.4,0.03]'; 
b2 = [1,-0.15,0.03]';
b3 = [1,0.1,0.03]';
b4 = [1,0.35,0.03]'; 
b5 = [1,0.6,0.03]';
b6 = [1,0.85,0.03]';
b7 = [-1,0.2,-0.2]';
b8 = [-1,-0.05,-0.2]';
b9 = [-1,-0.3,-0.2]';
bricks = [b1 b2 b3 b4 b5 b6 b7 b8 b9];

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
[ur3, ur5] = worldsetup(ur3pos, ur5pos, bricks);

bricks = bricks';

%% 

% MoveArm(ur5.model,bricks(3,:),[0.1,-0.2,0]); 
MoveArm(ur3.model,bricks(9,:));
MoveArm(ur3.model,drop(1,:));

