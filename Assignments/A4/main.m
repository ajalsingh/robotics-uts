function  main
    %% load robot and drum
    clear;
    close all;
    clc;
    set(0,'DefaultFigureWindowStyle','docked')
    view (3)
    camlight
    addpath('ply');
    
    % a) set base
    robot_base_transform = transl([1.262, 1.189, 1]) % student number: 12621189
    
    % b) drum transform
    xoffset = 0.5;
    yoffset = 1;
    zoffset = 0;
    drum_transform = transl([xoffset, yoffset, zoffset])
    
    % c) calculate transform between base and drum
    transform_between_base_and_drum = (robot_base_transform + drum_transform)/2
    
    
    % set tool
    tool_offset = transl([0,0,0.2*tan(pi/4)]) * troty(-pi/4);

    robot = RobotClass(robot_base_transform, tool_offset);  
    

    % Load Drum
    [f,v,data] = plyread('ply/Drum.ply','tri');

    % Scale the colours to be 0-to-1 (they are originally 0-to-255
    vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;

    % Then plot the trisurf
    trisurf(f,v(:,1)+xoffset,v(:,2)+yoffset, v(:,3)+zoffset ...
        ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');

    %% Blast window

    pose_1 = [0.57 1.06 0.75];
    pose_2 = [0.4 0.92 0.75];

    % slow enough
    robot.moveArm(pose_1, 0.5, 0)
    robot.moveArm(pose_2, 0.75, 1)
    %% 2 just fast enough to overload joint 6
    robot.moveArm(pose_1, 0.5, 0)
    robot.moveArm(pose_2, 0.68, 1)
%     robot.moveArm(robot.home, 1.2, 0);

end


