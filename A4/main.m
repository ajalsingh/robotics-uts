function  main
    %% load robot and drum
    clear;
    close all;
    clc;
    set(0,'DefaultFigureWindowStyle','docked')
    view (3)
    camlight
    
    %set base
    robot_base = [1.262, 1.189, 1]; % student number: 12621189
    
    % set tool
    tool_offset = 0.2*tan(pi/4);

    robot = RobotClass(robot_base, tool_offset);  
    

    % Load Drum
    [f,v,data] = plyread('Drum.ply','tri');

    % Scale the colours to be 0-to-1 (they are originally 0-to-255
    vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;

    xoffset = 0.5;
    yoffset = 1;
    drum_transform = transl([xoffset, yoffset, 0]);

    % Then plot the trisurf
    trisurf(f,v(:,1)+xoffset,v(:,2)+yoffset, v(:,3) ...
        ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');

    %% Blast window

    pose_1 = [0.57 1.06 0.75];
    pose_2 = [0.4 0.92 0.74];

    % slow enough
    robot.moveArm(pose_1, 0.5, 0)
    robot.moveArm(pose_2, 2.6, 1)
    %% 2 just fsat enough to overload joint 6
    robot.moveArm(pose_1, 0.5, 0)
    robot.moveArm(pose_2, 2.5, 1)
%     robot.moveArm(robot.home, 1.2, 0);

end


