function retrieveanddrop(robot,brick,dropoff)
%RETRIEVE Summary of this function goes here
%   Detailed explanation goes here

links = size(robot.qlim,1);

brick(1,3) = brick(1,3) + 0.11;

steps = 50;

T1 = robot.fkine(robot.getpos);
q1 = robot.ikine(T1);                                                        % Derive joint angles for required end-effector transformation
T2 = transl(brick);                                                   % Define a translation matrix            
q2 = robot.ikine(T2)

% while max(abs(q2))>2*pi
%     for i=1:size(q2,2)
%         q2(i) = q2(i)/10;
%     end
% end

if links == 6
    q2=q2/1000;
end
% s = lspb(0,1,steps);                                             	% First, create the scalar function
% qMatrix = nan(steps,links);                                             % Create memory allocation for variables
% for i = 1:steps
%     qMatrix(i,:) = (1-s(i))*q1 + s(i)*q2;                   	% Generate interpolated joint angles
% end
qMatrix = jtraj(q1,q2,steps);    
   
% velocity = zeros(steps,6);
% acceleration  = zeros(steps,6);
% for i = 2:stepsqMatrix
%     velocity(i,:) = qMatrix(i,:) - qMatrix(i-1,:);                          % Evaluate relative joint velocity
%     acceleration(i,:) = velocity(i,:) - velocity(i-1,:);                    % Evaluate relative acceleration
% end

%% Plot the results
% ur5.plot(,'trail','r-')
disp("going to brick");
for i=1:size(qMatrix,1)
    animate(robot,qMatrix(i,:));
    drawnow;
    
end
%%

dropoff(1,3) = dropoff(1,3) + 0.11;
T1 = robot.fkine(robot.getpos);
q1 = robot.ikine(T1);                                                        % Derive joint angles for required end-effector transformation
T2 = transl(dropoff);                                                  % Define a translation matrix            
q2 = robot.ikine(T1)



if links == 6
    q2=q2/1000;
end

% s = lspb(0,1,steps);                                             	% First, create the scalar function
% qMatrix = nan(steps,6);                                             % Create memory allocation for variables
% for i = 1:steps
%     qMatrix(i,:) = (1-s(i))*q1 + s(i)*q2;                   	% Generate interpolated joint angles
% end
qMatrix = jtraj(q1,q2,steps);    
   
% velocity = zeros(steps,6);
% acceleration  = zeros(steps,6);
% for i = 2:stepsqMatrix
%     velocity(i,:) = qMatrix(i,:) - qMatrix(i-1,:);                          % Evaluate relative joint velocity
%     acceleration(i,:) = velocity(i,:) - velocity(i-1,:);                    % Evaluate relative acceleration
% end

%% Plot the results
% ur5.plot(,'trail','r-')
for i=1:size(qMatrix,1)
    animate(robot,qMatrix(i,:));
    drawnow;
    
end
disp("dropping brick");
end

