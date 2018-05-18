function [travel2, map] = Where2go(matrixdir,i,j, map)
    indx = 0;
    %% adjacent positions
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
       if mod(h,2) == 1
           if index(h,3) == 0
              indx = indx + 1;
              travel2(indx) = h;
           end
       else
           if index(h,3) == 0
              if h == 8
                 if index(7,3) == 0 && index(1,3) == 0
                     indx = indx + 1;
                     travel2(indx) = h;
                 end
              else
                 if index(h-1,3) == 0 && index(h+1,3) == 0
                     indx = indx + 1;
                     travel2(indx) = h;
                 end
              end
           end
       end
    end
end