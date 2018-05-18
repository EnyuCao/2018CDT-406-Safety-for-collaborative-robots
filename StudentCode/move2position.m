function [i,j] = move2position(vrep, clientID,orientHandle,motor_fl, motor_fr, motor_rl, motor_rr,posxy,myNet,FuzzySpeed)
    accuracy = 0.5;
    inpos = 0;
    setang = 0;
    speccase = 0;
    while inpos == 0
        %% Get the robots position from vrep                    
        [~,~,~,robotOrient,~]=vrep.simxGetObjectGroupData(clientID,orientHandle,10,vrep.simx_opmode_blocking);
        robxy = robotOrient(1:2);
        robxymat(2) = 2-robxy(2);%
        robxymat(1) = 2+robxy(1);     
        robor = robotOrient(6);
        %% Is the robot inside the bounds of the next position? 
        if robxymat(1) < (posxy(1) + 0.25*accuracy) && robxymat(1) > (posxy(1) - 0.25*accuracy) && robxymat(2) < (posxy(2)+0.25*accuracy) && robxymat(2) > (posxy(2)-0.25*accuracy)
            inpos = 1;%If the robot is inside the bounds of the position that it moves towards   
        else%If the robot is out of bounds then get the wanted positions orientation with respect to the robot
            dx = posxy(1) - robxymat(1);
            dy = posxy(2) - robxymat(2);     
            if dy > 0
               if dx > 0%Southeast
                  angle = -atan(dy/dx);
               elseif dx == 0%
                  angle = -1.57;
               else%
                  angle =  -(3.14 + atan(dy/dx));
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
               elseif dx == 0%south
                  angle = -1.57;
               else%northwest quadrant 
                  angle = 3.14 - atan(dy/dx);
               end
            end%Get the orientation that the robot has to rotate to
            if angle < 0 || robor < 0
                if robor < 0
                    robor = 6.28 + robor;
                end
                if angle < 0
                    angle = 6.28 + angle;
                end
            end
            anglediff = robor - angle;
            if abs(anglediff) > 3.14
               if anglediff < 0 
                  anglediff = 6.28 + anglediff; 
               else
                  anglediff = anglediff - 6.28;
               end
            end 
            diffe(vrep, clientID,motor_fl, motor_fr, motor_rl, motor_rr,anglediff,myNet,FuzzySpeed);
        end
    end
    i  = floor((2-robxy(2))/0.5);
    j  = floor((2+robxy(1))/0.5);
end