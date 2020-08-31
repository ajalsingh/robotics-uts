function MoveArms(robot1, robot2,goal1,goal2,brick1,brick2)
%RETRIEVE Summary of this function goes here
%   Detailed explanation goes here

clc;
% State 0 = retrieve brick
% State 1 = place brick on wall
% State 2 = invalid number of arguments
if nargin ==4
    state = 0;      
elseif nargin==6
    state = 1;
else
    state = 2;
end

% if state==0
%     goal1(1,3) = goal1(1,3) + 0.07;
%     goal2(1,3) = goal2(1,3) + 0.07;
% end

if state ==2
    disp('Too few or too many input arguments');
else
    %%
    steps = 50;

    robot1q1 = robot1.getpos;                                                        % Derive joint angles for required end-effector transformation
    robot1T = transl(goal1);                                                   % Define a translation matrix            
    robot1q2 = robot1.ikine(robot1T,robot1q1, [1 1 1 0 0 0]);

    robot1qMatrix = jtraj(robot1q1,robot1q2,steps);
    
    robot2q1 = robot2.getpos;                                                        % Derive joint angles for required end-effector transformation
    robot2T = transl(goal2);                                                   % Define a translation matrix            
    robot2q2 = robot2.ikine(robot2T,robot2q1, [1 1 1 0 0 0]);

    robot2qMatrix = jtraj(robot2q1,robot2q2,steps);

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
    for i=1:steps
        animate(robot1,robot1qMatrix(i,:));
        
        if state==1
            endP1 = robot1.fkine(robot1qMatrix(i,:));
            endP1 = transl(endP1(1:3,4)');
            updatedPoints1 = [endP1 * [brickVerts,ones(brickVertexCount,1)]']';
            brick1.Vertices = updatedPoints1(:,1:3);
        end
                
        animate(robot2,robot2qMatrix(i,:));
        if state==1
            endP2 = robot2.fkine(robot2qMatrix(i,:));
            endP2 = transl(endP2(1:3,4)');
            updatedPoints2 = [endP2 * [brickVerts,ones(brickVertexCount,1)]']';
            brick2.Vertices = updatedPoints2(:,1:3);
        end
        drawnow();
    end
end

end

