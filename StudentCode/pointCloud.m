function [map] = pointCloud(vrep,clientID)
    operationMode = vrep.simx_opmode_blocking;
    %% Save handles of objects of interest
    [~,handles(1)]=vrep.simxGetObjectHandle(clientID,'ShelfBody',operationMode);
    [~,handles(2)]=vrep.simxGetObjectHandle(clientID,'ConveyorBelt',operationMode);
    [~,handles(3)]=vrep.simxGetObjectHandle(clientID,'ConveyorBeltBody',operationMode);
    [~,handles(4)]=vrep.simxGetObjectHandle(clientID,'ConveyorBelt_forwarder',operationMode);
    [~,handles(5)]=vrep.simxGetObjectHandle(clientID,'TagConveyorBelt',operationMode);
    [~,handles(6)]=vrep.simxGetObjectHandle(clientID,'DockStationBody0',operationMode);
    [~,handles(7)]=vrep.simxGetObjectHandle(clientID,'Cuboid0',operationMode);
    %% Get Objects Positions & lengths of vertics
    position = zeros(3,length(handles));
    for i = 1:length(handles)
%             [~, minX(i)]=vrep.simxGetObjectFloatParameter(clientID,...
%                 handles(i),15,operationMode); %3rd argument 15 = min X
%             [~, minY(i)]=vrep.simxGetObjectFloatParameter(clientID,...
%                 handles(i),16,operationMode); %3rd argument 16 = min Y
            [~, maxX(i)]=vrep.simxGetObjectFloatParameter(clientID,...
                handles(i),18,operationMode); %3rd argument 18 = max X
            [~, maxY(i)]=vrep.simxGetObjectFloatParameter(clientID,...
                handles(i),19,operationMode); %3rd argument 19 = max Y
%             lengthX(i) = maxX(i)-minX(i); %length in X
%             lengthY(i) = maxY(i)-minY(i); %length in Y
            [~,position(:,i)]=vrep.simxGetObjectPosition(clientID,handles(i),...
                -1,operationMode); % Get position of object (origin/center)
    end
    %% Mark with 3 where there's an object
    position(1,:) = position(1,:)+1.5; %Change from VREP's coordinates to MATLAB's
    position(2,:) = abs(position(2,:)-1.5); %Change from VREP's coordinates to MATLAB's
    maxXY = [maxX; maxY]; % X and Y.
    map = objScene(position, maxXY);

end