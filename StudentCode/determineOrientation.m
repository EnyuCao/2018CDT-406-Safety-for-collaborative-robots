function [wanted] = determineOrientation(clientID,orientHandle,vrep)
    [~,~,~,robotOrient,~] = vrep.simxGetObjectGroupData(clientID,orientHandle,10,vrep.simx_opmode_blocking);%Gets the robot's information
    robor = robotOrient(6);%get the robots position 
    if robor < 0.785 && robor > 0
        diff1 = 0.785 - robor;
        diff2 = robor - 0;
        if diff1 > diff2
            wanted = 0;
        else
            wanted = 0.785;
        end
    elseif robor < 1.57 && robor > 0.785
        diff1 = 1.57 - robor;
        diff2 = robor - 0.785;
        if diff1 > diff2
            wanted = 0.785;
        else
            wanted = 1.57;
        end
    elseif robor < 2.355 && robor > 1.57
        diff1 = 2.355 - robor;
        diff2 = robor - 1.57;
        if diff1 > diff2
            wanted = 1.57;
        else
            wanted = 2.355;
        end
    elseif robor < 3.14 && robor > 2.355
        diff1 = 3.14 - robor;
        diff2 = robor - 2.355;
        if diff1 > diff2
            wanted = 2.355;
        else
            wanted = 3.14;
        end
    elseif robor > -3.14 && robor < -2.355
        diff1 = abs(-3.14 - robor);
        diff2 = abs(-2.355 - robor);
        if diff1 > diff2
            wanted = -2.355;
        else
            wanted = 3.14;
        end
    elseif robor > -2.335 && robor < -1.57
        diff1 = abs(-2.355 - robor);
        diff2 = abs(-1.57 - robor);
        if diff1 > diff2
            wanted = -1.57;
        else
            wanted = -2.355;
        end
    elseif robor > -1.57 && robor < -0.785
        diff1 = abs(-1.57 - robor);
        diff2 = abs(-0.785 - robor);
        if diff1 > diff2
            wanted = -0.785;
        else
            wanted = -1.57;
        end
    elseif robor > -0.785 && robor < 0
        diff1 = abs(-0.785 - robor);
        diff2 = abs(0 - robor);
        if diff1 > diff2
            wanted = -0.785;
        else
            wanted = 0;
        end
    end
end