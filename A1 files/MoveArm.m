function MoveArm(robot,goal)
%RETRIEVE Summary of this function goes here
%   Detailed explanation goes here


goal(1,3) = goal(1,3) + 0.11;
% dropoff(1,3) = dropoff(1,3);
steps = 50;

q1 = robot.getpos;                                                        % Derive joint angles for required end-effector transformation
T2 = transl(goal);                                                   % Define a translation matrix            
q2 = robot.ikine(T2,q1, [1 1 1 0 0 0])

% while max(abs(q2))>2*pi
%     for i=1:size(q2,2)
%         q2(i) = q2(i)/10;
%     end
% end

qMatrix = jtraj(q1,q2,steps);    
% q2

%% Plot the results
% ur5.plot(,'trail','r-')
disp("get")
for i=1:size(qMatrix,1)
    animate(robot,qMatrix(i,:));
    drawnow;
    
end
end

