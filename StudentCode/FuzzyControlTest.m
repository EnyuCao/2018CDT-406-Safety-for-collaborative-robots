% This script is used to test the fuzzy controller, the scenario is: The
% robot drives straight towards Bill in the v-rep scene. The distance
% between Bill and the robot is monitored aswell as the speed of the robot.
% The speed and distance is plotted against each other, the script stops
% when the robot velocity is 0 (or close to 0). 

clc, clear, close all
load('FaceNoFaceNet.mat');
FuzzySpeed = readfis('FuzzySpeedControl.fis');
disp('Program started');
vrep=remApi('remoteApi');
vrep.simxFinish(-1);
clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);

if(clientID > -1)
    disp('Connected'); 
    
    %% Handles
    [~,robotHandle]=vrep.simxGetObjectHandle(clientID,'youBot',vrep.simx_opmode_blocking);
    [~,motor_rr]=vrep.simxGetObjectHandle(clientID,'rollingJoint_rr',vrep.simx_opmode_blocking);
    [~,motor_rl]=vrep.simxGetObjectHandle(clientID,'rollingJoint_rl',vrep.simx_opmode_blocking);
    [~,motor_fr]=vrep.simxGetObjectHandle(clientID,'rollingJoint_fr',vrep.simx_opmode_blocking);
    [~,motor_fl]=vrep.simxGetObjectHandle(clientID,'rollingJoint_fl',vrep.simx_opmode_blocking);
    [~,front_sensor]=vrep.simxGetObjectHandle(clientID,'front_sensor',vrep.simx_opmode_blocking);
    [~,back_sensor]=vrep.simxGetObjectHandle(clientID,'back_sensor',vrep.simx_opmode_blocking);
    [~,left_sensor]=vrep.simxGetObjectHandle(clientID,'left_sensor',vrep.simx_opmode_blocking);
    [~,right_sensor]=vrep.simxGetObjectHandle(clientID,'right_sensor',vrep.simx_opmode_blocking);
    [~,camera]=vrep.simxGetObjectHandle(clientID,'Vision_sensor',vrep.simx_opmode_blocking);
    
    %% Initiate Camera
    [~,resolution,image]=vrep.simxGetVisionSensorImage2(clientID,camera,0,vrep.simx_opmode_streaming);
    
    %% Initiate Sensors
    
    % Proximity Sensors
    [~,~,fRead,~,~]=vrep.simxReadProximitySensor(clientID,front_sensor,vrep.simx_opmode_streaming);
    [~,~,bRead,~,~]=vrep.simxReadProximitySensor(clientID,back_sensor,vrep.simx_opmode_streaming);
    [~,~,lRead,~,~]=vrep.simxReadProximitySensor(clientID,left_sensor,vrep.simx_opmode_streaming);
    [~,~,rRead,~,~]=vrep.simxReadProximitySensor(clientID,right_sensor,vrep.simx_opmode_streaming);
    
    % Speed sensor
    [~,linearVelocity,~] = vrep.simxGetObjectVelocity(clientID,robotHandle,vrep.simx_opmode_streaming);
     
    %% Give program time to connect
    pause(2);
    %% Main Loop
    for i = 1:400
       tic;
       %Camera feed
       [~,resolution,image]=vrep.simxGetVisionSensorImage2(clientID,camera,0,vrep.simx_opmode_buffer);
       image = imresize(image, [227 227]);
       [output,scores] = classify(myNet, image);
       
       % Uncomment/comment to enable/disable camera feed
       imshow(image) 
       text(80, 10, char(output));
       drawnow;
       
       %% Sensor read
       [~,~,fRead,~,~]=vrep.simxReadProximitySensor(clientID,front_sensor,vrep.simx_opmode_buffer);
       [~,~,bRead,~,~]=vrep.simxReadProximitySensor(clientID,back_sensor,vrep.simx_opmode_buffer);
       [~,~,lRead,~,~]=vrep.simxReadProximitySensor(clientID,left_sensor,vrep.simx_opmode_buffer);
       [~,~,rRead,~,~]=vrep.simxReadProximitySensor(clientID,right_sensor,vrep.simx_opmode_buffer);
       % The sensor values is normalized to get the distance
       fDist = norm(fRead);
       rDist = norm(rRead);
       lDist = norm(lRead);
       bDist = norm(bRead);
       
       % This is only used for data collection
       trueDist = fDist;
       if trueDist < 1e-4 || trueDist > 3 
           trueDist = 3;
       end
       
       % The sensors send very high or low values when not detecting
       % anything, so when not detecting, the value is set to the maximum
       % distance for each sensor
       if fDist < 1e-4 || fDist > 1.5
           fDist = 1.5;
       end
       if rDist < 1e-4 || rDist > 0.8
           rDist = 0.8;
       end
       if lDist < 1e-4 || lDist > 0.8
           lDist = 0.8;
       end
       if bDist < 1e-4 || bDist > 0.8
           bDist = 0.8;
       end
       %% Safety measures (Fuzzy)
       if max(scores) == scores(2)
           FaceDirection = 0.5+max(scores);
       else
           FaceDirection = 1 - max(scores);
       end
       
       % Accuire speed from Fuzzy function
       [Speed, ~] = evalfis([double(FaceDirection),double(fDist),double(lDist),double(rDist),double(bDist)],FuzzySpeed);
       
       % There is a possibility for negative speed, but we only want
       % positive speed for the current purpose
       if Speed < 0
           Speed = 0;
       end
       % Set speed for each joint
       vrep.simxSetJointTargetVelocity(clientID,motor_rl,Speed,vrep.simx_opmode_blocking);
       vrep.simxSetJointTargetVelocity(clientID,motor_rr,Speed,vrep.simx_opmode_blocking);
       vrep.simxSetJointTargetVelocity(clientID,motor_fl,Speed,vrep.simx_opmode_blocking);
       vrep.simxSetJointTargetVelocity(clientID,motor_fr,Speed,vrep.simx_opmode_blocking);
       
       %% Distane and speed between Bill and Robot
       [~,linearVelocity,~] = vrep.simxGetObjectVelocity(clientID,robotHandle,vrep.simx_opmode_buffer);
       Distance(i) = trueDist;
       Velocity(i) = sqrt(linearVelocity(1)^2 + linearVelocity(2)^2);
       
       disp(['Distane: ', num2str(Distance(i)), ' | Velocity: ', num2str(Velocity(i)),' | Facial Pose: ',char(output), ' | Joint Speed: ',num2str(Speed),' | Time: ', num2str(toc)])
       
        if Velocity(i) < 0.005
            break
        end
    end
    vrep.simxSetJointTargetVelocity(clientID,motor_rl,0,vrep.simx_opmode_blocking);
    vrep.simxSetJointTargetVelocity(clientID,motor_rr,0,vrep.simx_opmode_blocking);
    vrep.simxSetJointTargetVelocity(clientID,motor_fl,0,vrep.simx_opmode_blocking);
    vrep.simxSetJointTargetVelocity(clientID,motor_fr,0,vrep.simx_opmode_blocking);
    vrep.simxFinish(-1);
    
    plot(Distance, Velocity)
    title('Non-Human Object')
    ylabel('Velocity (m/s)')
    xlabel('Distance (m)')
end
vrep.delete();

