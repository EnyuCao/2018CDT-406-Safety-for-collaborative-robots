function [i,j,map] = move2position(vrep, clientID,orientHandle,sensorhandles,motor_fl, motor_fr, motor_rl, motor_rr,posxy,map,myNet,FuzzySpeed)
    accuracy = 0.5; %The size of the inner square of a grid position which the robot needs to be inside to continue on to the next position, sort of like a waypoint that the robot has to pass through to get to the next position 
    inpos = 0;

    while inpos == 0%While the robot is not inside the bounds of the next position, act until it is or until it fails
        %% Get the robots position from vrep        
        [~,~,~,robotOrient,~]=vrep.simxGetObjectGroupData(clientID,orientHandle,10,vrep.simx_opmode_blocking);
        robxy = robotOrient(1:2);
        robxymat(2) = 2-robxy(2);
        robxymat(1) = 2+robxy(1);
        robor = robotOrient(6);
        %% Is the robot inside the bounds of the next position? 
        if robxymat(1) < (posxy(1) + 0.25*accuracy) && robxymat(1) > (posxy(1) - 0.25*accuracy) && robxymat(2) < (posxy(2)+0.25*accuracy) && robxymat(2) > (posxy(2)-0.25*accuracy)
            inpos = 1;%If the robot is inside the bounds of the position that it moves towards   
        else%If the robot is out of bounds then get the wanted positions orientation with respect to the robot
            dx = posxy(1) - robxymat(1);%Difference in x-axis(horizontal, positive direction to the right) between the robot and the position it needs to arrive at
            dy = posxy(2) - robxymat(2);%Difference in y-axis(vertical, positive direction downward) between the robot and the position it needs to arrive at
            if dy > 0
               if dx > 0%Southeast
                  angle = -atan(dy/dx);
               elseif dx == 0%south
                  angle = -1.57;
               else%Southwest
                  angle = -(3.14 + atan(dy/dx));
               end
            elseif dy == 0
               if dx > 0%East
                   angle = 0;
               elseif dx < 0%West
                   angle = 3.14;
               end
            else
               if dx > 0%northeast quadrant 
                  angle = -atan(dy/dx);
               elseif dx == 0%north
                  angle = -1.57;
               else%northwest quadrant 
                  angle = 3.14 - atan(dy/dx);
               end
%                disp(inpos);
            end%Get the orientation that the robot has to rotate to
            if angle < 0 || robor < 0%convert angles from 0 --> 3.14(north domain), -3.14 --> 0 (south domain) into 0 --> 3.14(north domain), 3.14 --> 6.28(south domain)
                if robor < 0
                    robor = 6.28 + robor;
                end
                if angle < 0
                    angle = 6.28 + angle;
                end
            end%Convert into degree form 0 to 6.28
            anglediff = robor - angle;%Determine the angular difference between the orientation of the robot and the angle between the robots orientation and the point the robot wants to go to
            if abs(anglediff) > 3.14
               if anglediff < 0 
                  anglediff = 6.28 + anglediff; 
               else
                  anglediff = anglediff - 6.28;
               end
            end%Rotate the shortest angle, find the shortest angle
            [map,OiF] = diffe(vrep, clientID,orientHandle,motor_fl, motor_fr, motor_rl, motor_rr,anglediff,sensorhandles,map,myNet,FuzzySpeed);
            inpos = OiF;%If there is a object in front of the robot the robot must replan the intended path
        end
    end
    %Update robots position in grid map matrix
    i  = floor((2-robxy(2))/0.5);
    j  = floor((2+robxy(1))/0.5);
end