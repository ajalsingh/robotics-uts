clear
clc
close all; 
addpath('UR3/');
ur3 = UR3;
% ur3.model.base = transl(0.5014,-0.00575,-0.2556);% * trotx(pi/2);
% q1 = zeros(1,7);
ur3.PlotAndColourRobot();

%%
steps = 50;

T1 = ur3.model.fkine(ur3.model.getpos);

q1 = ur3.model.ikine(T1);                                                        % Derive joint angles for required end-effector transformation
T2 = transl(-0.15,-0.36,0.56); 

% Define a translation matrix            
%q2a goes crazy but q2b is smooth
q2 = ur3.model.ikcon(T2, q1)
% q2 = [0 pi/2 0 0 0 0]


while max(abs(q2))>2*pi
    for i=1:size(q2,2)
        q2(i) = q2(i)/10;
    end
end

q2

qMatrix = jtraj(q1,q2,steps);    


%%
for i=1:size(qMatrix,1)
    animate(ur3.model,qMatrix(i,:));
    drawnow;
    
end