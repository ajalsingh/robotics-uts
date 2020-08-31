function MoveSingleArm(robot,goal,brick)
%RETRIEVE Summary of this function goes here
%   Detailed explanation goes here
clc;
if nargin ==2
    state = 0;
else
    state = 1;
end

% if state==0
%     goal(1,3) = goal(1,3) + 0.07;
% end
steps = 50;

q1 = robot.getpos;                                                        % Derive joint angles for required end-effector transformation
T2 = transl(goal);                                                   % Define a translation matrix            
q2 = robot.ikine(T2,q1, [1 1 1 0 0 0]);

qMatrix = jtraj(q1,q2,steps);    

%%
if state ==1
    [f,v,data] = plyread('ply/Brick.ply','tri');
    % Get vertex count
    brickVertexCount = size(v,1);

    % Move center point to origin
    midPoint = sum(v)/brickVertexCount;
    brickVerts = v - repmat(midPoint,brickVertexCount,1);
end

%% Plot the results
% ur5.plot(,'trail','r-')
for i=1:size(qMatrix,1)
    animate(robot,qMatrix(i,:));
    if state==1
        endP = robot.fkine(qMatrix(i,:));
        endP = transl(endP(1:3,4)');
        updatedPoints = [endP * [brickVerts,ones(brickVertexCount,1)]']';
        brick.Vertices = updatedPoints(:,1:3);
    end
    drawnow();
end

end
