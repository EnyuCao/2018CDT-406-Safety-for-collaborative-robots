function [matrixdir] = getrobotsurroundings(states, robor)
    if robor == 3.14%If robot is facing west
        matrixdir(1) = states(3);
        matrixdir(2) = states(4);
        matrixdir(3) = states(5);
        matrixdir(4) = states(6);
        matrixdir(5) = states(7);
        matrixdir(6) = states(8);
        matrixdir(7) = states(1);
        matrixdir(8) = states(2);
    elseif robor == 2.355%-||- north-west
        matrixdir(1) = states(2);
        matrixdir(2) = states(3);
        matrixdir(3) = states(4);
        matrixdir(4) = states(5);
        matrixdir(5) = states(6);
        matrixdir(6) = states(7);
        matrixdir(7) = states(8);
        matrixdir(8) = states(1);
    elseif robor == 1.57%-||- north
        matrixdir(1) = states(1);
        matrixdir(2) = states(2);
        matrixdir(3) = states(3);
        matrixdir(4) = states(4);
        matrixdir(5) = states(5);
        matrixdir(6) = states(6);
        matrixdir(7) = states(7);
        matrixdir(8) = states(8);
    elseif robor == 0.785%-||- north-east
        matrixdir(1) = states(8);
        matrixdir(2) = states(1);
        matrixdir(3) = states(2);
        matrixdir(4) = states(3);
        matrixdir(5) = states(4);
        matrixdir(6) = states(5);
        matrixdir(7) = states(6);
        matrixdir(8) = states(7);
    elseif robor == 0%-||- east
        matrixdir(1) = states(7);
        matrixdir(2) = states(8);
        matrixdir(3) = states(1);
        matrixdir(4) = states(2);
        matrixdir(5) = states(3);
        matrixdir(6) = states(4);
        matrixdir(7) = states(5);
        matrixdir(8) = states(6);
    elseif robor == -0.785%-||- south-east
        matrixdir(1) = states(6);
        matrixdir(2) = states(7);
        matrixdir(3) = states(8);
        matrixdir(4) = states(1);
        matrixdir(5) = states(2);
        matrixdir(6) = states(3);
        matrixdir(7) = states(4);
        matrixdir(8) = states(5);
    elseif robor == -1.57%-||- south
        matrixdir(1) = states(5);
        matrixdir(2) = states(6);
        matrixdir(3) = states(7);
        matrixdir(4) = states(8);
        matrixdir(5) = states(1);
        matrixdir(6) = states(2);
        matrixdir(7) = states(3);
        matrixdir(8) = states(4);
    elseif robor == -2.355%-||- south-west
        matrixdir(1) = states(4);
        matrixdir(2) = states(5);
        matrixdir(3) = states(6);
        matrixdir(4) = states(7);
        matrixdir(5) = states(8);
        matrixdir(6) = states(1);
        matrixdir(7) = states(2);
        matrixdir(8) = states(3);
    end
end%Define the objects positions with respect to the current orientation of the robot