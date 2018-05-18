function [map] = objScene(position, maxXY)

map = zeros(10,10,1);
count = 0;
for i = 1:length(position)
    if(position(1,i) < 0 || position(1,i) > 5 || position(2,i) < 0 || position(2,i) > 5)
       continue;
    else
        count = count +1;
        position2(:,count) = position(1:2,i);
        maxX2(count) = maxXY(1,i);
        maxY2(count) = maxXY(2,i);
        
    end    
end

for i = 1:length(position2)
    URx=ceil((position2(1,i)+maxX2(i))/0.5); % x-position Upper Right
    URy=ceil((position2(2,i)+maxY2(i))/0.5); % y-position Upper Right
    
    LRx=ceil((position2(1,i)+maxX2(i))/0.5); % x-position Lower Right
    LRy=ceil((position2(2,i)-maxY2(i))/0.5); % y-position Lower Right
    
    ULx=ceil((position2(1,i)-maxX2(i))/0.5); % x-position Upper Left
    ULy=ceil((position2(2,i)+maxY2(i))/0.5); % y-position Upper Left
    
    LLx=ceil((position2(1,i)-maxX2(i))/0.5); % x-position Lower Left
    LLy=ceil((position2(2,i)-maxY2(i))/0.5); % y-position Lower Left
    
    map(URy, URx,1) = 3; % if static object found, map=3
    map(LRy,LRx, 1) = 3; % if static object found, map=3
    map(ULy, ULx, 1) = 3; % if static object found, map=3
    map(LLy, LLx, 1) = 3; % if static object found, map=3
    
    if((ULy-1) ~= LLy && ULy ~= LLy) % if all 4 coordinates,dont lie 
        map(ULy-1,ULx,1) = 3;        % in the same point,fill the vertices
    end                              % with 3, to get a rectangular structure.
    if((URy-1) ~= LRy && URy ~= LRy)
        map(URy-1,URx,1) = 3;
    end
    if((ULx+1) ~= URx && ULx ~= URx)
        map(ULy,ULx+1,1) = 3;        
    end
    if((LLx+1) ~= LRx && LLx ~= LRx)
        map(LLy,LLx+1,1) = 3;
    end
end

end

