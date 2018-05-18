function [map] = Astar(clientID,orientHandle,destinationHandle ,motor_fl, motor_fr, motor_rl, motor_rr,vrep,map)
    %% initialization               
    goalstate = 0;
    tic;
    load('FaceNoFaceNet.mat');
    FuzzySpeed = readfis('NewFuzzy2.fis');
    toc;
    %Get sensor handles
    [~,front_sensor]=vrep.simxGetObjectHandle(clientID,'front_sensor',vrep.simx_opmode_blocking);
    [~,back_sensor]=vrep.simxGetObjectHandle(clientID,'back_sensor',vrep.simx_opmode_blocking);
    [~,left_sensor]=vrep.simxGetObjectHandle(clientID,'left_sensor',vrep.simx_opmode_blocking);
    [~,right_sensor]=vrep.simxGetObjectHandle(clientID,'right_sensor',vrep.simx_opmode_blocking);
    [~,rightfront_sensor]=vrep.simxGetObjectHandle(clientID,'rightfront_sensor',vrep.simx_opmode_blocking);
    [~,leftfront_sensor]=vrep.simxGetObjectHandle(clientID,'leftfront_sensor',vrep.simx_opmode_blocking);
    [~,leftback_sensor]=vrep.simxGetObjectHandle(clientID,'leftback_sensor',vrep.simx_opmode_blocking);
    [~,rightback_sensor]=vrep.simxGetObjectHandle(clientID,'rightback_sensor',vrep.simx_opmode_blocking);
    
    %Get goal and robot position
    [~,~,~,goalPos,~] = vrep.simxGetObjectGroupData(clientID,destinationHandle,10,vrep.simx_opmode_blocking);
    goalxy = goalPos(1:2);
    [~,~,~,robotOrient,~] = vrep.simxGetObjectGroupData(clientID,orientHandle,10,vrep.simx_opmode_blocking);
    robxy = robotOrient(1:2);
    
    %Initially set 
    i  = floor((2-robxy(2))/0.5);
    j  = floor((2+robxy(1))/0.5);
    poscorr = GetMatPosValue(i,j);%Get the middle position of the robots current matrix entry
    move2position(vrep,clientID,orientHandle,motor_fl, motor_fr, motor_rl, motor_rr,poscorr,myNet,FuzzySpeed);%move the robot into the correct position initially 
    %Get the goal position entry
    gi  = floor((2-goalxy(2))/0.5);
    gj  = floor((2+goalxy(1))/0.5);    
    %% perform A* until at goalstate
    iteration = 0;
    while (goalstate ~= 1)
       iteration = iteration + 1;
       
       wanted = rot2nearest(clientID,orientHandle,motor_fl, motor_fr, motor_rl, motor_rr,vrep);%Rotate the robot such that it is facing the closest adjacent position
       [states, ~] = getsensordata(clientID, [front_sensor, rightfront_sensor, right_sensor, rightback_sensor, back_sensor, leftback_sensor, left_sensor, leftfront_sensor], vrep);%Get the states of the robots sensors
       matrixdir = getrobotsurroundings(states,wanted);%Get the states of the positions around the current position
       [~,~,~,robotOrient,~] = vrep.simxGetObjectGroupData(clientID,orientHandle,10,vrep.simx_opmode_blocking);%Gets the robot's information
       robxy = robotOrient(1:2);%get the robots position  
       
       %update robots position entry in matrix 
       i  = floor((2-robxy(2))/0.5);
       j  = floor((2+robxy(1))/0.5);
       [posspos,map] = Where2go(matrixdir,i,j,map);%Get possible positions to go to
       wanted = FindNextPos(i,j,posspos,map,gi,gj);%Get the next position to go to in the path
       if wanted ~= -4 %If the intended path is not blocked
          if wanted == -5
             goalstate = 1;
             posxy(2) = 0.5*gi+0.25;
             posxy(1) = 0.5*gj+0.25;
             move2position(vrep,clientID,orientHandle,motor_fl, motor_fr, motor_rl, motor_rr,posxy,myNet,FuzzySpeed);%moves the robot to the correct position
             forward(vrep, clientID, motor_fl, motor_fr, motor_rl, motor_rr, 0 ,vrep.simx_opmode_blocking);
          else
             %rot2wanted(vrep, clientID,orientHandle,motor_fl, motor_fr, motor_rl, motor_rr, wanted);%Rotates robot such that it faces the wanted adjacent position
             posxy = Getposition2go2(clientID,orientHandle,vrep,wanted);
             move2position(vrep,clientID,orientHandle,motor_fl, motor_fr, motor_rl, motor_rr,posxy,myNet,FuzzySpeed);%moves the robot to the correct position   
          end
       else%If the intended path is blocked
          [~,~,~,robotOrient,~] = vrep.simxGetObjectGroupData(clientID,orientHandle,10,vrep.simx_opmode_blocking);
          robxy = robotOrient(1:2);
          i  = floor((2-robxy(2))/0.5);
          j  = floor((2+robxy(1))/0.5);
          [posspos,map] = Where2go(matrixdir,i,j,map);%Get possible positions to go to
          map = PlanPath(map, robxy, goalxy);
          wanted = FindNextPos(i,j,posspos,map,gi,gj);%Get the next position to go to in the path
          rot2wanted(vrep, clientID,orientHandle,motor_fl, motor_fr, motor_rl, motor_rr, wanted);%Rotates robot such that it faces the wanted adjacent position
          posxy = Getposition2go2(clientID,orientHandle,vrep,wanted);
          move2position(vrep,clientID,orientHandle,motor_fl, motor_fr, motor_rl, motor_rr,posxy,myNet,FuzzySpeed);%moves the robot to the correct position;
       end
    end
end