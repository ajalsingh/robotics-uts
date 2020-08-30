function [distance] = pointCloud(robot1,robot2)
%Q2 Summary of this function goes here
%   Detailed explanation goes here
robots = [robot1 robot2];
distance = [];
disp('Calculating... ');

for i=1:length(robots)
    stepRads = deg2rad(70);
    qlim = robots(i).qlim;
    % Don't need to worry about joint 6
    PointCloudeSize = prod(floor((qlim(1:5,2)-qlim(1:5,1))/stepRads + 1));
    PointCloud = zeros(PointCloudeSize,3);
    counter = 1;
    tic
    for q1 = qlim(1,1):stepRads:qlim(1,2)
        for q2 = qlim(2,1):stepRads:qlim(2,2)
            for q3 = qlim(3,1):stepRads:qlim(3,2)
                for q4 = qlim(4,1):stepRads:qlim(4,2)
                    for q5 = qlim(5,1):stepRads:qlim(5,2)
                        if length(qlim)==6
                            q6 = 0;
                                q = [q1,q2,q3,q4,q5,q6];
                                tr = robots(i).fkine(q);                        
                                PointCloud(counter,:) = tr(1:3,4)';
                                counter = counter + 1; 
                        else
                            for q6 = qlim(6,1):stepRads:qlim(6,2)
                                q7 =0;
                                q = [q1,q2,q3,q4,q5,q6,q7];
                                tr = robots(i).fkine(q);                        
                                PointCloud(counter,:) = tr(1:3,4)';
                                counter = counter + 1; 
                            end
                        end
                    end
                end
            end
        end
    end
    side = max([PointCloud(:,1); PointCloud(:,2)]);
    top = max(PointCloud(:,3));
    distance(end+1) = side;
    distance(end+1) = top;
end
end

