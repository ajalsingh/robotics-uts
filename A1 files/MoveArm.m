function MoveArm(robot,goal,state)
%RETRIEVE Summary of this function goes here
%   Detailed explanation goes here


goal(1,3) = goal(1,3) + 0.11;
steps = 50;

q1 = robot.getpos;                                                        % Derive joint angles for required end-effector transformation
T2 = transl(goal);                                                   % Define a translation matrix            
q2 = robot.ikine(T2,q1, [1 1 1 0 0 0]);

qMatrix = jtraj(q1,q2,steps);    

%%
if state==1
    [f,v,data] = plyread('Brick.ply','tri');
    vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
    
    endEffectorPose = robot.fkine(qMatrix(1,:));
    endEffectorTrans = endEffectorPose(1:3,4)';
    
    brick = trisurf(f,v(:,1)+endEffectorTrans(1),v(:,2)+endEffectorTrans(2), v(:,3)+endEffectorTrans(3) ...
    ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
    end
end

%% Plot the results
% ur5.plot(,'trail','r-')
for i=1:size(qMatrix,1)
    animate(robot,qMatrix(i,:));
    
    if state==1
    endEffectorPose = robot.fkine(qMatrix(i,:));
    endEffectorTrans = endEffectorPose(1:3,4)';
    brick.
    drawnow();
end

end

