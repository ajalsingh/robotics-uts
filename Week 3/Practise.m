%% in order to translate along Y axis by 1.5 and z acis by 0.8. Get 4x4 matrix
clc
clear all

ex1 = transl([ 1.5 0.8 0])

%% Antonomous car driving on octagonal Track. 8 sides octagon each side is 90m . car drives straight then turn left by 45 degrees(0 radius). If it starts at x = 0m. y = -100m . with heading og (se3(se2(0,-100,0))). after it has completed 1 iterations of drive 90m then turns 45 deg left. it will be at position 
% x = , y = , orientation  = 
clc
clear all 
clf
iterations = 3;
start= se3(se2(0,100,0));
side_lenght = 90;
inside_angle = 135;
%turn_angle = deg2rad(180-inside_angle);
turn_angle =deg2rad(45);
motion = se3(se2(side_lenght,0,turn_angle));

for i =1:iterations
    nextPose = start*motion;
    start=nextPose;
end

position = transl(nextPose)
rpy = tr2rpy(nextPose,'deg')


%% QUADCOPTER
% Principal Axis from the wiki (Note Z direction). Start At origin.
% Translations vec whats is the xyz in global frame
clc
clear all
clf
 view(3)
%Translational Vectors (y and z need to be inverted)
vec1 = [0,0,6];
vec2 = [6,-6,0];
vec3 = [-5,-3,-2];

trStart = eye(4);
trNext = trStart*transl(vec1);
tranimate(trStart,trNext,'fps',30);
trStart = trNext;

trNext = trStart*transl(vec2);
tranimate(trStart,trNext,'fps',30);
trStart = trNext;

trNext = trStart*transl(vec3);
tranimate(trStart,trNext,'fps',30);
trStart = trNext


%% Fill matrix for the rotation around X axis by 90 degrees. 
clc
clear all
clf
close all
tr = rotz(deg2rad(-90))

%% Create a puma560 with mdl_puma560. move the arms so that all joints are at 15 degrees. Then the robot and end effector pose to nearest milimiter
clf
clear
clear all 
view(3)
mdl_puma560

p560.teach()
% put 15 in all the categories and then get the Position in mm(YEAH mm) and
% RPY is in Degrees
%% which is untrue
% ONLY 6 angles . 6 Eulerian and 6 Cardanian

%%  RETAKE AIRRAFT QUADCOPTER FLIGHT
clc
clf
clear all
 view(3)
%Translational Vectors (y and z need to be inverted)
vec1 = [0,0,8];
vec2 = [8,-8,-12];
vec3 = [-5,-3,2];

trStart = eye(4);
trNext = trStart*transl(vec1);
tranimate(trStart,trNext,'fps',30);
trStart = trNext;

trNext = trStart*transl(vec2);
tranimate(trStart,trNext,'fps',30);
trStart = trNext;

trNext = trStart*transl(vec3);
tranimate(trStart,trNext,'fps',30);
trStart = trNext

% CRASHED (Started at lower position than the ground).

%% FIRST CALCULATE translation along z by 1.5 and x by 0.8 rotate around y by 180 get 4x4
clc
clear all
clf

tr = transl([0 1.5 0.8])*trotz(deg2rad(180))

%% FIRST CALCULATE rotate around y by 180 and then  translation along z by 1.5 and x by 0.8 get 4x4
clc 
clear all

tr = troty(deg2rad(180))*transl([0.8 0 1.5])

%% Create Puma robot. move the joints at 35 degrees. How far in radians is the closest joint limit
% RUN PUMA ROBOT FIRST

limits = p560.qlim
limit_min = limits(:,1);
limit_max = limits(:,2);
pose = p560.getpos();
pose_Transpose = transpose (pose);

diffMin = abs(limit_min-pose_Transpose);
diffMax =abs(limit_max-pose_Transpose);

closest_joint_limit = min([diffMin diffMax],[],2);

end_pose = p560.fkine(pose_Transpose);
xyx_mm = transl(1000*end_pose)
rpy = tr2rpy(end_pose,'deg')

