function [travel2, map] = Where2go(matrixdir,i,j, map)
    indx = 0;
    %% Valid Path ?
    validpath = checkPath(map,i,j);%Check if the current path is valid
    if validpath == 1
        %% adjacent positions
        index = getIndex(i,j);
        %% Are there any objects in the robot's vicinity?
        if matrixdir(1) == 1
           map(index(1,1),index(1,2),1) = 3;
           index(1,3) = 1;
        end
        if matrixdir(2) == 1
           map(index(2,1),index(2,2),1) = 3;
           index(2,3) = 1;
        end
        if matrixdir(3) == 1
           map(index(3,1),index(3,2),1) = 3;
           index(3,3) = 1;
        end
        if matrixdir(4) == 1
           map(index(4,1),index(4,2),1) = 3;
           index(4,3) = 1;
        end
        if matrixdir(5) == 1
           map(index(5,1),index(5,2),1) = 3;
           index(5,3) = 1;
        end
        if matrixdir(6) == 1
           map(index(6,1),index(6,2),1) = 3;
           index(6,3) = 1;
        end
        if matrixdir(7) == 1
           map(index(7,1),index(7,2),1) = 3;
           index(7,3) = 1;
        end
        if matrixdir(8) == 1
           map(index(8,1),index(8,2),1) = 3;
           index(8,3) = 1;
        end
        %% Define what positions the robot can go to.
        for h = 1:8
           if mod(h,2) == 1%If the positions are horizontally or vertically placed with respect to the current position
               if index(h,3) == 0%If the position does not have a object placed in it, the robot can travel to it
                  indx = indx + 1;
                  travel2(indx) = h;
               end
           else%If the position is daigonally placed with respect to the current position
               if index(h,3) == 0%If the position doesn't have a object placed in it, the robot can travel to it 
                  if h == 8%If the position has rotary index 8, it cannot be generalized
                     if index(7,3) == 0 && index(1,3) == 0%If there are no objects in the common adjacent positions of the current and evaluated position, the robot can travel here
                         indx = indx + 1;
                         travel2(indx) = h;
                     end
                  else
                     if index(h-1,3) == 0 && index(h+1,3) == 0%If there are no objects in the common adjacent positions of the current and evaluated position, the robot can travel here
                         indx = indx + 1;
                         travel2(indx) = h;
                     end
                  end
               end
           end
        end
    else%Else there are no positions to go 
       travel2 = 0; 
    end
end

function validpath = checkPath(map,i,j)%Check the path so that it is not incorrect or blocked by a newly mapped object 
    validpath = 0;
    index = getIndex(i,j);

    for h = 1:8%For all adjacent positions 
       if map(index(h,1),index(h,2),1) == 2 && predecessor(h) == map(index(h,1),index(h,2),3)%find the adjacent position which has the current position as predecessor
          validpath = checkPath(map,index(h,1),index(h,2));%Check that adjacent position 
          break
       elseif map(index(h,1),index(h,2),1) == 0 && predecessor(h) == map(index(h,1),index(h,2),3)%If we at last are at the goal, the path is valid 
          validpath = 1;
          break
       end
    end
end

function [index] = getIndex(i,j)
    index = zeros(8,3);
    index(1,1) = i-1;
    index(1,2) = j;
    index(2,1) = i-1;
    index(2,2) = j+1;
    index(3,1) = i;
    index(3,2) = j+1;
    index(4,1) = i+1;
    index(4,2) = j+1;
    index(5,1) = i+1;
    index(5,2) = j;
    index(6,1) = i+1;
    index(6,2) = j-1;
    index(7,1) = i;
    index(7,2) = j-1;
    index(8,1) = i-1;
    index(8,2) = j-1;
end%Get the matrix index values for the adjacent positions of the current position
