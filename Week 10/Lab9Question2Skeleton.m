%% Robotics
% Lab 9 - Question 2 - Static Torque

function Lab9Question2Skeleton()
%% 2.1
close all
clear all
clc


mdl_puma560                                                                 % Load the puma560 model      

tau_max = [97.6 186.4 89.4 24.2 20.1 21.3]';                                % Maximum joint torque of the Puma560

% Zero position

% Find the largest mass (in kg) that can be sustained by the Puma 560 at
% the following joint configuration:
q = zeros(1,6);                                                             % "Zero" position for the Puma 560
m = 0;
w = [0 0 m*9.81 0 0 0];

while jointTorqueOK
    t = g + J'*w;
end
disp("Max mass: ");
%% 2.2 For a given end-effector pose

% Find the largest mass (kg) that can be sustained by the Puma 560 at the
% following end-effector pose:
T1 = [roty(pi/2) [0.7; 0; 0]; zeros(1,3) 1];                                % Desired end-effector transform
q_initial = zeros(1,6);
p560.ikcon(T1, q_initial);

%% 2.3 Minimum distance
m = 40;
% w = ...
x = 0.8;
T1 = [roty(pi/2) [x; 0; 0]; zeros(1,3) 1];                                  % Desired end-effector transform
