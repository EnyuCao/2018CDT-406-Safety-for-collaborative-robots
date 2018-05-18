function [posxy] = Getposition2go2(clientID,orientHandle,vrep,wanted)
    [~,~,~,robotOrient,~]=vrep.simxGetObjectGroupData(clientID,orientHandle,10,vrep.simx_opmode_blocking);
    robxy = robotOrient(1:2);
    i  = floor((2-robxy(2))/0.5);
    j  = floor((2+robxy(1))/0.5);

    if abs(wanted) == 3.14
        posxy = GetMatPosValue(i,j-1);
    elseif wanted == 2.355
        posxy = GetMatPosValue(i-1,j-1);
    elseif wanted == 1.57
        posxy = GetMatPosValue(i-1,j);
    elseif wanted == 0.785
        posxy = GetMatPosValue(i-1,j+1);
    elseif wanted == 0
        posxy = GetMatPosValue(i,j+1);
    elseif wanted == -0.785
        posxy = GetMatPosValue(i+1,j+1);
    elseif wanted == -1.57
        posxy = GetMatPosValue(i+1,j);
    elseif wanted == -2.355
        posxy = GetMatPosValue(i+1,j-1);
    end
end