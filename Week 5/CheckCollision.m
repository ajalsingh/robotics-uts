function collision = CheckCollision(robot, sphereCentre, radius)

    pause(0.1);
    tr = robot.fkine(robot.getpos);
    endEffectorToCentreDist = sqrt(sum((sphereCentre -tr(1:3,4)').^2));
    if endEffectorToCentreDist <=radius
        disp('Oh no a collision');
        collision = 1;
    else
        disp(['SAFE: End effector to sphere distance (', num2str(endEffectorToCentreDist)]);
        collision = 0;
    end
end

