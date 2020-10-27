function  main
    %% load robot and drum
    clear;
    close all;
    clc;
    set(0,'DefaultFigureWindowStyle','docked')
    view (3)
    camlight

    % load puma560
    mdl_puma560;

    %set base
    robot_base = [1.262, 1.189, 1];
    p560.base = transl(robot_base);

    % set tool
    tool_offset = 0.2*tan(pi/4);
    p560.tool = transl([0,0,tool_offset]);

    % plot robot
    p560.plot(qn)
    xlim([-0.5,2.5]);
    ylim([0,2]);
    zlim([0,2]);
    hold on; 

    % Load Drum
    [f,v,data] = plyread('Drum.ply','tri');

    % Scale the colours to be 0-to-1 (they are originally 0-to-255
    vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;

    xoffset = 0.75;
    yoffset = 1;
    drum_transform = transl([xoffset, yoffset, 0]);

    % Then plot the trisurf
    trisurf(f,v(:,1)+xoffset,v(:,2)+yoffset, v(:,3) ...
        ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');

    %% Blast window

    pose_1 = [0.84 1.05 0.65];
    pose_2 = [0.65 0.92 0.65];

    moveArm(p560, pose_1)
    moveArm(p560, pose_2)
    moveArm(p560, qn);

end

%% Move arm
function moveArm(robot, pose)
    q1 = robot.getpos;
    
    if length(pose) == 6
        q2 = pose;
    else                                                          
        T2 = transl(pose) * troty(pi/4+pi);           
        q2 = robot.ikcon(T2,q1);
    end
    steps = 50;

    s = lspb(0, 1, steps); % First, create the scalar function
    qMatrix = nan(steps, 6); % Create memory allocation for variables
    for i = 1:steps
        qMatrix(i, :) = (1 - s(i)) * q1 + s(i) * q2; % Generate interpolated joint angles
    end
    
    for i=1:size(qMatrix,1)
        animate(robot,qMatrix(i,:));
        drawnow();
    end
end


