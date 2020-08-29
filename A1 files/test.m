clear
clc
close all; 

ur3 = UR3;
ur3.model.base = transl(0.5014,-0.00575,-0.2556);% * trotx(pi/2);
% q1 = zeros(1,7);
ur3.PlotAndColourRobot();

%%
steps = 50;

T1 = ur3.model.fkine(ur3.model.getpos);

q1 = ur3.model.ikine(T1);                                                        % Derive joint angles for required end-effector transformation
T2 = transl(1,0.2,0.2); 

% Define a translation matrix            
%q2a goes crazy but q2b is smooth
q2 = ur3.model.ikine(T2)
% q2 = [-1.9632   -1.4565    0.0010   -0.0658    1.3741   -1.7258]


while max(abs(q2))>2*pi
    for i=1:size(q2,2)
        q2(i) = q2(i)/10;
    end
end

q2

s = lspb(0,1,steps);                                             	% First, create the scalar function
qMatrix = nan(steps,6);                                             % Create memory allocation for variables
for i = 1:steps
    qMatrix(i,:) = (1-s(i))*q1 + s(i)*q2;                   	% Generate interpolated joint angles
end

%%
for i=1:size(qMatrix,1)
    animate(ur3.model,qMatrix(i,:));
    drawnow;
    
end