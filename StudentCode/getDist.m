function [map] = getDist(map, i,j, gi,gj)
    %% Define adjacent position indeces
        ix = zeros(8,2);
        ix(1,1) = i-1;
        ix(1,2) = j;
        ix(2,1) = i-1;
        ix(2,2) = j+1;
        ix(3,1) = i;
        ix(3,2) = j+1;
        ix(4,1) = i+1;
        ix(4,2) = j+1;
        ix(5,1) = i+1;
        ix(5,2) = j;
        ix(6,1) = i+1;
        ix(6,2) = j-1;
        ix(7,1) = i;
        ix(7,2) = j-1;
        ix(8,1) = i-1;
        ix(8,2) = j-1;
    %% Evaluate surrounding positions
    for h = 1:8 
        if map(ix(h,1),ix(h,2),1) ~= 2 && map(ix(h,1),ix(h,2),1) ~= 3
            if mod(h,2) == 1
               map(ix(h,1),ix(h,2),2) = (abs(gi-ix(h,1)) + abs(gj-ix(h,2)))*10 + 10; 
            else
               map(ix(h,1),ix(h,2),2) = (abs(gi-ix(h,1)) + abs(gj-ix(h,2)))*10 + 14;
            end
        end
    end
end