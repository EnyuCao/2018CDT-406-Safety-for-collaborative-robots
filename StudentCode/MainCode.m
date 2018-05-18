clc, clear, close all

disp('Program started');
vrep=remApi('remoteApi');
vrep.simxFinish(-1);

clientID = vrep.simxStart('127.0.0.1',19999,true,true,5000,5);
mode = vrep.simx_opmode_blocking;

if(clientID > -1)
    disp('Connected');                   
    vrep.simxAddStatusbarMessage(clientID,'Russian_Hacker in control.',vrep.simx_opmode_oneshot);
%% Handles and robot position           
%Handles
[~,motor_rr]=vrep.simxGetObjectHandle(clientID,'rollingJoint_rr',vrep.simx_opmode_blocking);
[~,motor_rl]=vrep.simxGetObjectHandle(clientID,'rollingJoint_rl',vrep.simx_opmode_blocking);
[~,motor_fr]=vrep.simxGetObjectHandle(clientID,'rollingJoint_fr',vrep.simx_opmode_blocking);
[~,motor_fl]=vrep.simxGetObjectHandle(clientID,'rollingJoint_fl',vrep.simx_opmode_blocking);
[~,rob] = vrep.simxGetObjectHandle(clientID,'youBot',vrep.simx_opmode_blocking);
[~,destinationHandle]=vrep.simxGetCollectionHandle(clientID,'Destination',vrep.simx_opmode_blocking);
[~,orientHandle]=vrep.simxGetCollectionHandle(clientID,'RobotOrient',vrep.simx_opmode_blocking);
%% Sensor states       
%goal and robot position aswell as robot orientation
[~,~,~,robotOrient,~]=vrep.simxGetObjectGroupData(clientID,orientHandle,10,vrep.simx_opmode_blocking);
[~,~,~,goalPos,~]=vrep.simxGetObjectGroupData(clientID,destinationHandle,3,vrep.simx_opmode_blocking);
robxy = robotOrient(1:2);
robor = robotOrient(6);
goal = goalPos(1:2);
incorrpos = 0;
map = DefineMap(robxy);% Define map
map(2:11,2:11,1) = pointCloud(vrep, clientID);
map = PlanPath(map, robxy, goal);% Plan path from robot- to goal- position not taking objects into account 

%% Update positions and orientations    
[~,~,~,robotOrient,~]=vrep.simxGetObjectGroupData(clientID,orientHandle,10,vrep.simx_opmode_blocking);
[~,~,~,goalPos,~]=vrep.simxGetObjectGroupData(clientID,destinationHandle,3,vrep.simx_opmode_blocking);
robxy = robotOrient(1:2);%Get position
robor = robotOrient(6);%Get orientation
goal = goalPos(1:2);%Get goal position

%% prerequisites to A* algorithm        
map = Astar(clientID,orientHandle,destinationHandle,motor_fl, motor_fr, motor_rl, motor_rr,vrep,map);
vrep.simxSetJointTargetVelocity(clientID,motor_rl,0,vrep.simx_opmode_blocking);
vrep.simxSetJointTargetVelocity(clientID,motor_rr,0,vrep.simx_opmode_blocking);
vrep.simxFinish(-1);
end
vrep.delete();
