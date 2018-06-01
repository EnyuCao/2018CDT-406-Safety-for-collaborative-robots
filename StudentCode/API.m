clc, clear, close all

disp('Program started');
vrep=remApi('remoteApi');
vrep.simxFinish(-1);

clientID = vrep.simxStart('127.0.0.1',19999,true,true,5000,5);%Get Client ID
mode = vrep.simx_opmode_blocking;

V = 0;
T = 0;
save('VT', 'V', 'T');
if(clientID > -1)
    disp('Connected');                      
    vrep.simxAddStatusbarMessage(clientID,'Russian_Hacker in control.',vrep.simx_opmode_oneshot);
    %% obtain Handles
    [~,motor_rr]=vrep.simxGetObjectHandle(clientID,'rollingJoint_rr',vrep.simx_opmode_blocking);
    [~,motor_rl]=vrep.simxGetObjectHandle(clientID,'rollingJoint_rl',vrep.simx_opmode_blocking);
    [~,motor_fr]=vrep.simxGetObjectHandle(clientID,'rollingJoint_fr',vrep.simx_opmode_blocking);
    [~,motor_fl]=vrep.simxGetObjectHandle(clientID,'rollingJoint_fl',vrep.simx_opmode_blocking);
    [~,rob] = vrep.simxGetObjectHandle(clientID,'youBot',vrep.simx_opmode_blocking);
    [~,destinationHandle]=vrep.simxGetCollectionHandle(clientID,'Destination',vrep.simx_opmode_blocking);
    [~,orientHandle]=vrep.simxGetCollectionHandle(clientID,'RobotOrient',vrep.simx_opmode_blocking); 
    %% create grid map and preliminary path                       
    %goal and robot position aswell as robot orientation
    [~,~,~,robotOrient,~]=vrep.simxGetObjectGroupData(clientID,orientHandle,10,vrep.simx_opmode_blocking);
    [~,~,~,goalPos,~]=vrep.simxGetObjectGroupData(clientID,destinationHandle,3,vrep.simx_opmode_blocking);
    robxy = robotOrient(1:2);
    goal = goalPos(1:2);
    %Define matrix entries for the robot and goal positions
    i  = floor((2-robxy(2))/0.5);
    j  = floor((2+robxy(1))/0.5);
    gi  = floor((2-goal(2))/0.5);
    gj  = floor((2+goal(1))/0.5);
    %Define grid map 
    map = DefineMap(robxy);% Define map
    map = PlanPath(map,i,j,gi,gj,-1);% Plan preliminary path from robot- to goal- position 
    disp('initial map path:')
    disp(map(:,:,1))
    %% Begin real-time process       
    map = Astar(clientID,orientHandle,destinationHandle,motor_fl, motor_fr, motor_rl, motor_rr,vrep,map);
    vrep.simxSetJointTargetVelocity(clientID,motor_rl,0,vrep.simx_opmode_blocking);
    vrep.simxSetJointTargetVelocity(clientID,motor_rr,0,vrep.simx_opmode_blocking);
    vrep.simxFinish(-1);
end
vrep.delete();
