classdef Environmnet
    %ENVIRONMNET Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        ur5;
        ur3;
        
    end
    
    methods
        function self = Environmnet(ur3pos, ur5pos)
            %ENVIRONMNET Construct an instance of this class
            %   Detailed explanation goes here
            self.ur5 = LinearUR5(false);
            self.ur3 = UR3();
            
            %adjust robot base transform
            self.ur3.model.base = transl(ur3pos);
            self.ur5.model.base = transl(ur5pos) * trotx(pi/2);
            self.ur3.PlotAndColourRobot();
            self.ur5.PlotAndColourRobot();
            hold on;
            
        end
        
        function [ur3,ur5] = BuildWorld(self)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            %% Place conveyors
            % Conveyor file download: https://grabcad.com/library/conveyor-food-processing-roller-system-with-tripod-supports
            [f,v,data] = plyread('ply/conveyor.ply','tri');

            % Scale the colours to be 0-to-1 (they are originally 0-to-255
            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
            
            ur3xpos = self.ur3.model.base(1,4) - 0.5;
            ur3zpos = self.ur3.model.base(3,4) - 0.05;
            ur5xpos = self.ur5.model.base(1,4) + 0.4986;
            
            % Then plot the trisurf
            trisurf(f,v(:,1)+ur5xpos,v(:,2), v(:,3) ...
                ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
            trisurf(f,v(:,1)+ur3xpos,v(:,2), v(:,3)+ur3zpos ...
                ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
            
            %% Place fence
            [f,v,data] = plyread('ply/fence.ply','tri');

            % Scale the colours to be 0-to-1 (they are originally 0-to-255
            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;

            ur3xpos = self.ur3.model.base(1,4) - 1.3;
            ur3zpos = self.ur3.model.base(3,4) - 0.25;
            ur5xpos = self.ur5.model.base(1,4) + 1.2986;
            ur5zpos = self.ur5.model.base(3,4) - 0.2444;
            
            % Then plot the trisurf
            trisurf(f,v(:,1)+ur3xpos,v(:,2), v(:,3)+ur3zpos ...
                ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
            trisurf(f,v(:,1)+ur5xpos,v(:,2), v(:,3)+ur5zpos ...
                ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
            %% Place floor
            [f,v,data] = plyread('ply/floor.ply','tri');

            % Scale the colours to be 0-to-1 (they are originally 0-to-255
            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;

            ur3zpos = self.ur3.model.base(3,4) - 0.05;
            
            % Then plot the trisurf
            trisurf(f,v(:,1),v(:,2), v(:,3)+ur3zpos ...
                ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
            %% return robots
            ur3 = self.ur3;
            ur5 = self.ur5;
        end
    end
end

