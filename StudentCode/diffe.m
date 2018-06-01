function [map,OiF] = diffe(vrep, clientID,orientHandle,motor_fl, motor_fr, motor_rl, motor_rr,theta,sensorhandles,map,myNet,FuzzySpeed)
ixlog = 1;
tic;
% This function make it possible for the robot to
% move smoothly by applying differnetial drive system. 

[~,robotHandle] = vrep.simxGetObjectHandle(clientID, 'youBot',vrep.simx_opmode_blocking); % gets the robot handle
vrep.simxGetObjectVelocity(clientID,robotHandle,vrep.simx_opmode_streaming);



%% Speed definition
 %%% different operation modes that is used in the funcion
mode1 = vrep.simx_opmode_blocking;
mode2 = vrep.simx_opmode_streaming;
mode3 = vrep.simx_opmode_buffer;
OiF = 0;

[v_des,output] = FuzzyEval2(vrep, clientID, myNet, FuzzySpeed, sensorhandles); % output from fuzzy, speed and detection value.
output = char(output); % converts output string into char
rot_des = v_des*theta*5; % desired rotation rate.
d = 0.3;                  % distace between wheels
r_w = 0.05*5;            % wheel radius
v_rr = v_des + d*rot_des;
v_rl = v_des - d*rot_des;
speed_right = v_rr/r_w;   % speed of the right side of the robot
speed_left  = v_rl/r_w;   % speed of the left side of the robot
speed = (speed_right + speed_left)/2; 

vrep.simxSetJointTargetVelocity(clientID, motor_fl, speed_left, mode2);
vrep.simxSetJointTargetVelocity(clientID, motor_rl, speed_left, mode2);
vrep.simxSetJointTargetVelocity(clientID, motor_fr, speed_right,mode2);
vrep.simxSetJointTargetVelocity(clientID, motor_rr, speed_right,mode2);

[~,linearVelocity,~] = vrep.simxGetObjectVelocity(clientID,robotHandle,vrep.simx_opmode_buffer);
Velocity(ixlog) = sqrt(linearVelocity(1)^2 + linearVelocity(2)^2);
CurrentTime(ixlog) = toc;
ixlog = ixlog + 1;
%% Read sensors                                       
[~,State_l,detected_left,~,~]=vrep.simxReadProximitySensor(clientID,sensorhandles(7),mode2);
[~,State_r,detected_right,~,~]=vrep.simxReadProximitySensor(clientID,sensorhandles(3),mode2);
[~,State_f,detected_front,~,~]=vrep.simxReadProximitySensor(clientID,sensorhandles(1),mode2);
[~,State_b,detected_back,~,~]=vrep.simxReadProximitySensor(clientID,sensorhandles(5),mode2);
[~,State_lb,detected_lb,~,~]=vrep.simxReadProximitySensor(clientID,sensorhandles(6),mode2);
[~,State_lf,detected_lf,~,~]=vrep.simxReadProximitySensor(clientID,sensorhandles(8),mode2);
[~,State_rb,detected_rb,~,~]=vrep.simxReadProximitySensor(clientID,sensorhandles(4),mode2);
[~,State_rf,detected_rf,~,~]=vrep.simxReadProximitySensor(clientID,sensorhandles(2),mode2);
%% check for and act on objects in the robots vicinity
distmin = 0.7;

    if strcmp(output,'NoDetection') == 1
       
        speed = 2; % if nothing detected, set speed =2;
    end
    if (State_f && (norm(detected_front) < distmin)|| State_b)
        flag = 1; % if front sensor is active and the distance is less than dist, set flag =1
    else
        flag = 0;
    end
    if State_f == 1 && (norm(detected_front) < distmin) || State_lf == 1 || State_rf == 1 % lf=leftfront sensor, rf=rightfront sensor
        state = zeros(1,3);
        OiF = 1;%There is an object in front of the robot, replan the robots path.
        robor = determineOrientation(clientID,orientHandle,vrep);
        [~,~,~,robotOrient,~]=vrep.simxGetObjectGroupData(clientID,orientHandle,10,vrep.simx_opmode_blocking); % retrieves the local object positions and orientations of the robot
        robxy = robotOrient(1:2);
        i  = floor((2-robxy(2))/0.5);
        j  = floor((2+robxy(1))/0.5);
        if State_f == 1 && (norm(detected_front) < distmin)
            state(2) = 1;
        end
        if State_lf == 1
            state(1) = 1;
        end
        if State_rf == 1
            state(3) = 1;
        end
        map = Updateobjects(i,j,robor,map,state); % updates the objects
    end
    while flag
        mindist = 0.8;
        if State_f && (norm(detected_front) < distmin)  
            if theta < 0 % if the angle is less than 0, go backwards and rotate towards right
                vrep.simxSetJointTargetVelocity(clientID, motor_fl, -speed, mode2);
                vrep.simxSetJointTargetVelocity(clientID, motor_rl, -speed, mode2);
                vrep.simxSetJointTargetVelocity(clientID, motor_fr, -speed*2,mode2);
                vrep.simxSetJointTargetVelocity(clientID, motor_rr, -speed*2,mode2);
            elseif theta > 0 % if the angle is greater than 0, go backwards and rotate towards right
                vrep.simxSetJointTargetVelocity(clientID, motor_fl, -speed*2, mode2);
                vrep.simxSetJointTargetVelocity(clientID, motor_rl, -speed*2, mode2);
                vrep.simxSetJointTargetVelocity(clientID, motor_fr, -speed,mode2);
                vrep.simxSetJointTargetVelocity(clientID, motor_rr, -speed,mode2);
            end
            [~,State_f,detected_front,~,~]=vrep.simxReadProximitySensor(clientID,sensorhandles(1),mode3);
            de_f = norm(detected_front);
            if (de_f > mindist || State_f == 0) % if distnace between an object and the front_sensor is greater than mindist
                flag = 0;                       % or it is not detecting, go out from the loop
            end
        elseif (State_b) && (norm(detected_back) < distmin)% If back sensor is activated
            forward(vrep, clientID, motor_fl, motor_fr, motor_rl, motor_rr, speed, mode1); % go forward...
            [~,State_b,detected_back,~,~]=vrep.simxReadProximitySensor(clientID,sensorhandles(5),mode3);
            de_b = norm(detected_back);
            if (de_b > mindist || State_b == 0) % until the distance is greater than mindist or the sensor is not active anymore
                flag = 0;
            end
        else
            flag = 0;
        end
        [~,linearVelocity,~] = vrep.simxGetObjectVelocity(clientID,robotHandle,vrep.simx_opmode_buffer);
        Velocity(ixlog) = sqrt(linearVelocity(1)^2 + linearVelocity(2)^2);
        CurrentTime(ixlog) = toc;
        ixlog = ixlog + 1;
    end
    if (State_lf || State_l || State_lb) || (State_rf||State_r||State_rb) % if one of the sensors on either side is active 
        
        shortest_left = inf;
        shortest_right = inf;
        sens_l = 0;
        sens_r = 0;
        if(State_lf) % if left front is active
            shortest_left = norm(detected_lf);
            sens_l = 1;
        end
        if State_rf % if right front is active
            shortest_right = norm(detected_rf);
            sens_r = 1;
        end
        if(shortest_left > norm(detected_left) && State_l ~= 0 && norm(detected_left) < distmin)% if left sensor i active, assign shrotest_left = dist. of left sensor
            shortest_left = norm(detected_left);
            sens_l = 2;
        end
        if(shortest_left > norm(detected_lb) && State_lb ~= 0) % if left_back sensor is active 
            shortest_left = norm(detected_lb);
            sens_l = 3;
        end
        if(shortest_right> norm(detected_right) && State_r ~= 0 && norm(detected_right) < distmin)% if right sensor is active, shortest_right = dist. of right sensor
            shortest_right = norm(detected_right);
            sens_r = 2;
        end
        if(shortest_right > norm(detected_rb) && State_rb ~=0 ) % if right_back sensor active
            shortest_right = norm(detected_rb);
            sens_r = 3;
        end
        if(State_lf || (State_l &&  norm(detected_left) < distmin) || State_lb) && (State_rf || (State_r &&  norm(detected_right) < distmin) || State_rb) % if one of the sensor is active
            if shortest_right > shortest_left % if it is one of the left side sensors, traverse right. this is if there is objects on both side of the robot
               
                traverse_right(vrep, clientID, motor_fl, motor_fr, motor_rl, motor_rr, ...
                    speed,vrep.simx_opmode_blocking);
                stop_move(vrep, clientID,[sens_l sens_r]);
            else  % traverse left
                traverse_left(vrep, clientID, motor_fl, motor_fr, motor_rl, motor_rr, ...
                    speed,vrep.simx_opmode_blocking);
                stop_move(vrep, clientID,[sens_l sens_r]);
            end
        elseif (State_lf || (State_l &&  norm(detected_left) < distmin) || State_lb) %if only object to the left then go right
           
            traverse_right(vrep, clientID, motor_fl, motor_fr, motor_rl, motor_rr, ...
                speed_right,vrep.simx_opmode_blocking);
            stop_move(vrep, clientID,[sens_l 0]);
        elseif (State_rf || (State_r &&  norm(detected_right) < distmin)|| State_rb) %if only object to the right then go left
            
            traverse_left(vrep, clientID, motor_fl, motor_fr, motor_rl, motor_rr, ...
                speed_left,vrep.simx_opmode_blocking);
            stop_move(vrep, clientID,[0 sens_r]);
        end
    end
    
    [~,linearVelocity,~] = vrep.simxGetObjectVelocity(clientID,robotHandle,vrep.simx_opmode_buffer);
    Velocity(ixlog) = sqrt(linearVelocity(1)^2 + linearVelocity(2)^2);
    CurrentTime(ixlog) = toc;
    ixlog = ixlog + 1;
    
    logTimeandVelocity(Velocity, CurrentTime);
end

function [map] = Updateobjects(i,j,robor,map,state) % update
    index = getIndex(i,j);
    if robor == 0
        if state(1) == 1
            map(index(2,1),index(2,2),1) = 3;
        end   
        if state(2) == 1
            map(index(3,1),index(3,2),1) = 3;
        end 
        if state(3) == 1
            map(index(4,1),index(4,2),1) = 3;
        end 
    elseif robor == 0.785
        if state(1) == 1
            map(index(1,1),index(1,2),1) = 3;
        end   
        if state(2) == 1
            map(index(2,1),index(2,2),1) = 3;
        end 
        if state(3) == 1
            map(index(3,1),index(3,2),1) = 3;
        end 
    elseif robor == 1.57
        if state(1) == 1
            map(index(8,1),index(8,2),1) = 3;
        end   
        if state(2) == 1
            map(index(1,1),index(1,2),1) = 3;
        end 
        if state(3) == 1
            map(index(2,1),index(2,2),1) = 3;
        end 
    elseif robor == 2.355
        if state(1) == 1
            map(index(7,1),index(7,2),1) = 3;
        end   
        if state(2) == 1
            map(index(8,1),index(8,2),1) = 3;
        end 
        if state(3) == 1
            map(index(1,1),index(1,2),1) = 3;
        end 
    elseif robor == 3.14
        if state(1) == 1
            map(index(6,1),index(6,2),1) = 3;
        end   
        if state(2) == 1
            map(index(7,1),index(7,2),1) = 3;
        end 
        if state(3) == 1
            map(index(8,1),index(8,2),1) = 3;
        end 
    elseif robor == -2.355
        if state(1) == 1
            map(index(5,1),index(5,2),1) = 3;
        end   
        if state(2) == 1
            map(index(6,1),index(6,2),1) = 3;
        end 
        if state(3) == 1
            map(index(7,1),index(7,2),1) = 3;
        end 
    elseif robor == -1.57
        if state(1) == 1
            map(index(4,1),index(4,2),1) = 3;
        end   
        if state(2) == 1
            map(index(5,1),index(5,2),1) = 3;
        end 
        if state(3) == 1
            map(index(6,1),index(6,2),1) = 3;
        end 
    elseif robor == -0.785
        if state(1) == 1
            map(index(3,1),index(3,2),1) = 3;
        end   
        if state(2) == 1
            map(index(4,1),index(4,2),1) = 3;
        end 
        if state(3) == 1
            map(index(5,1),index(5,2),1) = 3;
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

function [] = logTimeandVelocity(savedVelocities, savedTimes)
    load('VT.mat');
    V = [V savedVelocities];
    savedTimes = savedTimes + max(T);
    T = [T savedTimes];
    delete('VT.mat');
    save('VT', 'V', 'T');
end