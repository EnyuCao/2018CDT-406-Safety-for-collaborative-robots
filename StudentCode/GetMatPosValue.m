function [pos] = GetMatPosValue(i,j)
 % gets the center postion of the block we want to go to
    pos(1) = 0.5*j + 0.25;
    pos(2) = 0.5*i + 0.25;
end