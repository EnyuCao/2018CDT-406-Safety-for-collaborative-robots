function [map] = PlanPath(map, robxy, goalxy)
    iterations = 0;
    goal = 0;%We are not in the goal position yet
    
    %Convert goal and robot position into matrix entries 
    i  = floor((2-robxy(2))/0.5);
    j  = floor((2+robxy(1))/0.5);
    gi  = floor((2-goalxy(2))/0.5);
    gj  = floor((2+goalxy(1))/0.5);
    
    for h = 1:length(map(:,1,1))
       for k = 1:length(map(1,:,1))
          if map(h,k,1) == 2 || map(h,k,1) == 1
             map(h,k,1) = 0;
             map(h,k,2) = 0;
             map(h,k,3) = 0;
          end
          if map(h,k,1) == 3
             map(h,k,2) = 0;
             map(h,k,3) = 0;
          end
       end
    end
    map(i,j,1) = 2;
    
    while goal == 0%While not in goal position
     %% Initialize     
        iterations = iterations + 1;
        redo = 0;
        travel2 = 0;
        indx = 1;%Define index
        map = getDist(map,i,j,gi,gj);%Get distance to goal-position 
        minvalue(1) = inf;%Set the minimum value to infinity to not risk having a initial value larger than the F-values of the adjacent positions
        minvalue(2) = 0;%Set index to 0 initially, since it does not exist
     %% Save start value 
        if iterations == 1
           startpos(1) = i;
           startpos(2) = j;
        end
     %% Define indeces of adjacent positions    
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
     %% Check for objects in adjacent positions 
        for h = 1:8
            if map(index(h,1),index(h,2),1) == 3 || map(index(h,1),index(h,2),1) == 2 || map(index(h,1),index(h,2),1) == 1%If the entries in the map have a state 3 there is a object there
               index(h,3) = 1;%add that the adjacent position is a wall
            end
        end
     %% Which positions can we travel to?       
         for h = 1:8
           if mod(h,2) == 1%If the position to evaluate is not diagonal with respect to the current position
               if index(h,3) == 0%If the position is not a 'Wall'
                  travel2(indx) = h;% Add that this possition can be moved to
                  indx = indx + 1;
               end
           else% else the position is diagonal with respect to the current position 
               if h ~= 8%To not exceed indeces in 'index'
                   if index(h,3) == 0
                      if index(h-1,3) == 0 && index(h+1,3) == 0%As long as the common ajdacent positions of the diagonal and current position are not walls, then the robot can move to the diagonal position
                         travel2(indx) = h;%Add the position to the set of possible position to travel to.
                         indx = indx + 1;
                      end
                   end
               else
                   if index(h,3) == 0
                      if index(7,3) == 0 && index(1,3) == 0%As long as the common ajdacent positions of the diagonal and current position are not walls, then the robot can move to the diagonal position 
                         travel2(indx) = h; 
                         indx = indx + 1;
                      end 
                   end
               end
           end
         end
         if travel2 == 0%If there is nowhere to go 
            i = firstmove(1);
            j = firstmove(2);
            map(i,j,1) = 1;
            i = startpos(1);
            j = startpos(2);
            redo = 1;
            iterations = 0;
         end
     %% Choose next position                    
         if redo == 0
            %CONTINUE HERE:
            %-For the current scene and position of the robot:
            %--If two adjacent positions have the same F value then compare
            % the two positions and choose the path with the smallest f
            % value sum
            %--If one path does not find the goal, choose another
             for h = 1:8
                if ismember(h,travel2)%If the current h value/adjacent position is allowed to travel to 
                   if map(index(h,1), index(h,2), 2) < minvalue(1)%If the Fvalue of the currently evaluated position is lesser than the current minimum value, set the currently evaluated position as the current minimum position 
                      minvalue(1) = map(index(h,1), index(h,2), 2);%Update minimum value
                      minvalue(2) = h;%Update which position has this value
                   end
                end
            end
         %% go to next position                     
            map(index(minvalue(2),1),index(minvalue(2),2),3) = predecessor(minvalue(2));%set the current position as the predecessor to the next position 
            map(index(minvalue(2),1),index(minvalue(2),2),1) = 2;%Set the next position as evaluated
            %go to next position and redo until in goal position
            i = index(minvalue(2),1); 
            j = index(minvalue(2),2);
            if iterations == 1%Save the first move made
               firstmove(1) = i;
               firstmove(2) = j;
            end
            if i == gi && j == gj%If in goal position, end the loop
                goal = 1;
            end

        end
    end   
end