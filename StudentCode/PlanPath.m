function [map, steps] = PlanPath(map,i,j,gi,gj,predecessor)
    steps = -1;
    if i == gi && j == gj % If we are in the goal position, return to parent function
        steps = 0; 
        map(i,j,3) = predecessor;%Update predecessor to the goal position
    else
        
        if predecessor ~= -1 %If we are not in the start position, update the predecessor to this position 
           map(i,j,3) = predecessor;
        else%If we are in the start position, clear the map from previous path planning data
           map = clearMap(map);
        end
        map(i,j,1) = 2;%Update our current position as "visited"
        map = getDist(map,i,j,gi,gj);%Get the Fvalues of the surrounding positions
        index = getIndex(i,j);%Get the matrix index values of the adjacent positions in rotary indeces
        index = updateSurroundings(map,index);%Check if there are any "already visited" positions or objects in the way
        invalidposition = rePlan(index,predecessor);%Determine if the current position will be used or not
        if invalidposition == 0%If we are in a valid position, proceed
            travel2 = possPos(index);%Check which positions we actually can travel to
            if travel2 == 0%If there is nowhere to travel to 
            else%If there is somewhere to travel to
                [que, tocheck] = orderContenders(travel2,index,map);%Evaluate the positions we can travel to and see which are the most suitable
                if tocheck > 1%If there are several adjacent positions that share the minimum F-value
                    failcount = 0;
                    for h = 1:tocheck%For the number of adj. positions that share the min. F-value, compare each of their paths
                       [tempmap(:,:,:,h), comparepath(h)] = PlanPath(map,index(que(h),1),index(que(h),2),gi,gj,getPredecessor(que(h)));%recursively go into the next position in the que and calculate the path, save the number of steps taken and the map that was created 
                       map = clearBranch(map,index(que(h),1),index(que(h),2));%Clear the changes made by the currently chosen position
                       if comparepath(h) == -1%If the position returns -1 as the number of steps then there was no solution/path to the goal found  
                           failcount = failcount + 1;%Add the failure to the failcounter
                       end
                    end
                    if failcount == tocheck%If, for all of the tested positions, every single one failed then the rest of the positions are checked 
                        for h=1:(length(que)-tocheck)%for the remaining positions, choose the next one in the que and check if it finds a path, if not then the loop continues 
                            [map,steps] = PlanPath(map,index(que(h+tocheck),1),index(que(h+tocheck),2),gi,gj,getPredecessor(que(h+tocheck)));
                            if steps ~= -1%If a path was found then break the loop and return the number of steps to the parent function
                               steps = steps + 1;
                               break
                            else%If no path was found then cleanse the map from the tested position's path finding process
                               map = clearBranch(map,index(que(h+tocheck),1),index(que(h+tocheck),2));
                            end
                        end
                    else%Else there is at least one of the adj. pos. that managed to create a path
                        minval = inf;
                        minindex = 0;
                        for h = 1:tocheck%for the number of positions that could find a path to the goal, compare the steps sizes and save the best one
                            if comparepath(h) ~= -1%If the path is invalid
                               if comparepath(h) < minval
                                   minval = comparepath(h);
                                   minindex = h;
                               end
                           end
                        end
                        if minval ~= inf
                            steps = minval;%update the number of steps 
                            map = tempmap(:,:,:,minindex);%Update the map 
                        end
                    end    
                end 
                if tocheck == 1%If there is only one position with the minimum F-value 
                   [map,steps] = PlanPath(map,index(que(1),1),index(que(1),2),gi,gj,getPredecessor(que(1)));%Get the path and number of steps from this position
                   if steps == -1%If the first position did not find a valid path, try the next position 
                      map = clearBranch(map,index(que(1),1),index(que(1),2));%Clear the map from the invalid path 
                      for h = 1:(length(que)-1)%Until there are no more positions, try to find a position with a valid path
                        [map,steps] = PlanPath(map,index(que(h+1),1),index(que(h+1),2),gi,gj,getPredecessor(que(h+1)));% Get the path and number of steps given the current position
                        if steps ~= -1%If the position yielded a valid path, break the loop and return the  number of steps
                           steps = steps + 1;
                           break
                        else%If the positions path was invalid, clear the path that the position created
                           map = clearBranch(map,index(que(h+1),1),index(que(h+1),2));
                        end
                      end
                   else
                       steps = steps + 1;%add 1 to the steps
                   end
                end
            end
        else%Else we are not in a valid position, return to parent function
           steps = -1; 
        end
    end
end

function [invalidposition] = rePlan(index,predecessor)
    riskzone = 0;
    invalidposition = 0;
    ix = 1;
    for m = 1:8%For all adjacent positions, determine if any of the positions have previously been visited, in that case, add the rotary index to rotix
        if index(m,3) == 2 && m ~= predecessor
           riskzone = 1;
           rotix(ix) = m;
           ix = ix + 1;
        end
    end
    if riskzone == 1%if the current position is in the riskzone of being invalid
        for m = 1:length(rotix)%for the number of visited adjacent positions to the current position
            if mod(rotix(m), 2) == 1%if the visited position is placed vertically or horizontally with respect to the current position, it is guaranteed that the current position can be traveled to by the visited position hence the current position is invalid to use due to a loop in the path
                invalidposition = 1;
                break
            else%else if the visited position is placed diagonally with respect to the current position, it is unsure if you can travel between the visited- and the current- position due to objects in the common adjacent positions of the two positions 
                if rotix(m) == 8%if the rotary index is 8 then a special condition must be created due to the rotary index max value is eight and then it start on 1 again 
                    if index(1,3) ~= 1 && index(7,3) ~= 1
                        invalidposition = 1;
                        break
                    end
                else%if the rotary index is not 8 then check the common adjacent positions of the visited and current position. if there are no objects in the way then the current position can be traveled to from the visited position thereby rendering the current position invalid due to it creating a path loop 
                    if index(rotix(m) - 1) ~= 1 && index(rotix(m) + 1) ~= 1 
                        invalidposition = 1;
                        break
                    end
                end
            end
        end
    end
end%Determine if current position creates path loops or not, if it does then do not use this position, if not then proceed with finding a path to the goal

function [index] = updateSurroundings(map,index)
    for m = 1:8
        if map(index(m,1),index(m,2),1) == 3 || map(index(m,1),index(m,2),1) == 1%If the entries in the map have a state 3 there is a object there
           index(m,3) = 1;%add that the adjacent position is a wall
        elseif map(index(m,1),index(m,2),1) == 2%If the position already has been visited and evaluated then update the index with this information
           index(m,3) = 2;
        end
    end
end%Get the surroundings of the robot 

function [travel2] = possPos(index)
    indx = 1;
    for m = 1:8
       if mod(m,2) == 1%If the position to evaluate is not diagonal with respect to the current position
           if index(m,3) == 0%If the position is not a 'Wall' or 'visited'
              travel2(indx) = m;% Add that this possition can be moved to
              indx = indx + 1;
           end
       else% else the position is diagonal with respect to the current position 
           if m ~= 8%To not exceed indeces in 'index'
               if index(m,3) == 0
                  if index(m-1,3) == 0 && index(m+1,3) == 0 || index(m-1,3) == 2 && index(m+1,3) == 0 || index(m-1,3) == 0 && index(m+1,3) == 2 || index(m-1,3) == 2 && index(m+1,3) == 2%As long as the common ajdacent positions of the diagonal and current position are not walls, then the robot can move to the diagonal position
                     travel2(indx) = m;%Add the position to the set of possible positions to travel to.
                     indx = indx + 1;
                  end
               end
           else%If the currently evaluated rotary index value is 8 then some exceptions have to be made
               if index(m,3) == 0
                  if index(7,3) == 0 && index(1,3) == 0 || index(7,3) == 2 && index(1,3) == 0 || index(7,3) == 0 && index(1,3) == 2 || index(7,3) == 2 && index(1,3) == 2%As long as the common ajdacent positions of the diagonal and current position are not walls, then the robot can move to the diagonal position 
                     travel2(indx) = m; 
                     indx = indx + 1;
                  end 
               end
           end
       end
    end
    if indx == 1%If there were no possible positions, return 0
       travel2 = 0; 
    end
end%Determine which positions the robot can travel to 

function [que, tocheck] = orderContenders(travel2,rotindex,map)
    %initialize 
    fvaluearray = zeros(1,length(travel2));
    que = zeros(1,length(travel2));
    tocheck = 0;
    
    for i = 1:length(travel2)%Get the F-values of all the adjacent positions 
        fvaluearray(i) = map(rotindex(travel2(i),1), rotindex(travel2(i),2), 2);
    end   
    [minval,~] = min(fvaluearray);%Get the minimum F-value of the adjacent postions
    for i = 1:length(fvaluearray)%Check how many positions share the minimum F-value 
        if fvaluearray(i) == minval
            tocheck = tocheck + 1;%Add the number of positions that need to be compared
        end
    end
    
    sortedarray = sort(fvaluearray);%Sort the F-value array 
    for i = 1:length(travel2)
        for j = 1:length(travel2)
            if fvaluearray(i) == sortedarray(j) && que(j) == 0%Order the index values according to the order of the f-value array
                que(j) = travel2(i);
                break
            end
        end
    end
end% Order the positions/contenders by their F-values left to right by smallest to largest f-value 

function [map] = clearMap(map)
    for i = 1:length(map(:,1,1))%for all the rows in the map 
       for j = 1:length(map(1,:,1))%for all the columns in the map 
          if map(i,j,1) == 2 || map(i,j,1) == 1%If the position is a temporary object or a visited position, clear the position
             map(i,j,1) = 0;
             map(i,j,2) = 0;
             map(i,j,3) = 0;
          end
          if map(i,j,1) == 3%If there is a object in the map then clear the position from predecessor values and f-values that might have emerged due to uncertain calculations
             map(i,j,2) = 0;
             map(i,j,3) = 0;
          end
          if map(i,j,2) > 0%If there are positions with f-values, clear these
             map(i,j,2) = 0;
          end
       end
    end
end%Clear the map from temporary objects, fvalues and visited positions

function [map] = clearBranch(map,faili,failj)%Clear the branch which has been specified by the indeces faili and failj
    index = getIndex(faili,failj);%Get the entries of the surrounding positions
    for m = 1:8
        if map(index(m,1),index(m,2),1) ~= 3 && map(index(m,1),index(m,2),3) == getPredecessor(m)%if the adjacent position that is being evaluated has the current position as predecessor, recursively check that position for further positions in the path 
            map = clearBranch(map, index(m,1), index(m,2));
            break
        end
    end
    %Clear all information about this position
    map(faili,failj,1) = 0;
    map(faili,failj,2) = 0;
    map(faili,failj,3) = 0;
end

function [predecessor] = getPredecessor(rotaryindex)
    switch rotaryindex
        
        case 1
            predecessor = 5;
        case 2
            predecessor = 6;
        case 3
            predecessor = 7;
        case 4
            predecessor = 8;
        case 5
            predecessor = 1;
        case 6 
            predecessor = 2;
        case 7 
            predecessor = 3;
        case 8
            predecessor = 4;
    end
end%Get the predecessor of the evaluated position with respect to the rotary index of the current position

function [map] = getDist(map,i,j,gi,gj)
index = getIndex(i,j);%get the indeces of all adjacent positions
    for m = 1:8%for all surrounding positions
        if map(index(m,1),index(m,2),1) ~= 2 && map(index(m,1),index(m,2),1) ~= 3%If the current position is no wall and has not been visited
            if mod(m,2) == 1%if the position is not diagonal with respect to the current position, update the position value
               map(index(m,1),index(m,2),2) = (abs(gi-index(m,1)) + abs(gj-index(m,2)))*10 + 10;
            else%if the the position is daigonal with respect to the current position, update the position value
               map(index(m,1),index(m,2),2) = (abs(gi-index(m,1)) + abs(gj-index(m,2)))*10 + 14;
            end
        end
    end
end%Evaluate the fvalues of all adjacent positions

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
