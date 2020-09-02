classdef Environment
    %ENVIRONMNET Creates environment including 2 robots, 2 conveyors,
    %               floor,2 fences, and a platform 
    
    properties
        ur5;    % LinearUR5 robot
        ur3;    % UR3 robot
        
    end
    
    methods
        %%
        function self = Environment(ur3pos, ur5pos)
            %ENVIRONMNET Construct an instance of this class
            % input1: robot1 base position
            % input2: robot2 base position
            % output: class object
            
            %   Creates robots
            self.ur5 = LinearUR5(false);
            self.ur3 = UR3();
            
            %adjust robot properties and plot
            self.ur3.model.base = transl(ur3pos);
            self.ur3.model.tool = trotx(pi);
            self.ur5.model.base = transl(ur5pos) * trotx(pi/2);
%             self.ur5.model.tool = trotx(pi);
            self.ur3.PlotAndColourRobot();
            self.ur5.PlotAndColourRobot();
            hold on;
            
        end
        
        %%
        function [ur3,ur5] = BuildWorld(self)
            %METHOD1 Build world
            % output1: robot1 object
            % output2: robot2 object
            
            % Place conveyors
            % Conveyor file download: https://grabcad.com/library/conveyor-food-processing-roller-system-with-tripod-supports
            [f,v,data] = plyread('ply/conveyor.ply','tri');

            % Scale the colours to be 0-to-1 (they are originally 0-to-255
            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
            
            ur3xoffset = self.ur3.model.base(1,4) - 0.5;
            ur3zoffset = self.ur3.model.base(3,4) - 0.05;
            ur5xoffset = self.ur5.model.base(1,4) + 0.4986;
            
            % Then plot the trisurf
            trisurf(f,v(:,1)+ur5xoffset,v(:,2), v(:,3) ...
                ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
            trisurf(f,v(:,1)+ur3xoffset,v(:,2), v(:,3)+ur3zoffset ...
                ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
            
            
            
            % Place fence
            [f,v,data] = plyread('ply/fence.ply','tri');

            % Scale the colours to be 0-to-1 (they are originally 0-to-255
            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;

            ur3xoffset = self.ur3.model.base(1,4) - 1.3;
            ur3zoffset = self.ur3.model.base(3,4) - 0.35;
            ur5xoffset = self.ur5.model.base(1,4) + 1.2986;
            ur5zoffset = self.ur5.model.base(3,4) - 0.25;
            
            % Then plot the trisurf
            trisurf(f,v(:,1)+ur3xoffset,v(:,2), v(:,3)+ur3zoffset ...
                ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
            trisurf(f,v(:,1)+ur5xoffset,v(:,2), v(:,3)+ur5zoffset ...
                ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
            
            
            
            % Place floor
            [f,v,data] = plyread('ply/floor.ply','tri');

            % Scale the colours to be 0-to-1 (they are originally 0-to-255
            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;

            zoffset = self.ur3.model.base(3,4) - 0.15;
            
            % Then plot the trisurf
            trisurf(f,v(:,1),v(:,2), v(:,3)+zoffset ...
                ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
            
            
            
            % Place platform
            [f,v,data] = plyread('ply/platform.ply','tri');

            % Scale the colours to be 0-to-1 (they are originally 0-to-255
            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;

            yoffset = self.ur3.model.base(2,4) +0.2;
            zoffset = self.ur3.model.base(3,4) - 0.15;
            
            % Then plot the trisurf
            trisurf(f,v(:,1),v(:,2)+yoffset, v(:,3)+zoffset ...
                ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
            
            
            % return robots
            ur3 = self.ur3;
            ur5 = self.ur5;
        end
    end
end

