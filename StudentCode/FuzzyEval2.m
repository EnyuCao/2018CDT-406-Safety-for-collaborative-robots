function [Speed]= FuzzyEval2(vrep, clientID, myNet, FuzzySpeed, image)
mode = vrep.simx_opmode_blocking;
    
%% Get Handles from sensors of interest
    [~,front_sensor]=vrep.simxGetObjectHandle(clientID,'front_sensor',mode);
    [~,back_sensor]=vrep.simxGetObjectHandle(clientID,'back_sensor',mode);
    [~,left_sensor]=vrep.simxGetObjectHandle(clientID,'left_sensor',mode);
    [~,right_sensor]=vrep.simxGetObjectHandle(clientID,'right_sensor',mode);
    [~,camera]=vrep.simxGetObjectHandle(clientID,'Vision_sensor',mode);
   
%% Initiate Sensors
    [~,~,fRead,~,~]=vrep.simxReadProximitySensor(clientID,front_sensor,vrep.simx_opmode_streaming);
    [~,~,bRead,~,~]=vrep.simxReadProximitySensor(clientID,back_sensor,vrep.simx_opmode_streaming);
    [~,~,lRead,~,~]=vrep.simxReadProximitySensor(clientID,left_sensor,vrep.simx_opmode_streaming);
    [~,~,rRead,~,~]=vrep.simxReadProximitySensor(clientID,right_sensor,vrep.simx_opmode_streaming);
%% Identify If person
	[~,resolution,image]=vrep.simxGetVisionSensorImage2(clientID,camera,0,vrep.simx_opmode_buffer);
    image = imresize(image, [227 227]);
    [output,scores] = classify(myNet, image);
       
%% Sensor read
    [~,~,fRead,~,~]=vrep.simxReadProximitySensor(clientID,front_sensor,vrep.simx_opmode_buffer);
    [~,~,bRead,~,~]=vrep.simxReadProximitySensor(clientID,back_sensor,vrep.simx_opmode_buffer);
    [~,~,lRead,~,~]=vrep.simxReadProximitySensor(clientID,left_sensor,vrep.simx_opmode_buffer);
    [~,~,rRead,~,~]=vrep.simxReadProximitySensor(clientID,right_sensor,vrep.simx_opmode_buffer);
    fDist = norm(fRead);
    rDist = norm(rRead);
    lDist = norm(lRead);
    bDist = norm(bRead);
       
    % very high or low values when not detecting. If so, the value is set to the maximum possible distance
    if (fDist < 1e-3 || fDist > 1.5)
       fDist = 1.5;
    end
    if (rDist < 1e-3 || rDist > 0.8)
       rDist = 0.8;
    end
    if (lDist < 1e-3 || lDist > 0.8)
       lDist = 0.8;
    end
    if (bDist < 1e-3 || bDist > 0.8)
       bDist = 0.8;
    end

%% Safety measures (Fuzzy)
    if max(scores) == scores(2) %Front
       FaceDirection = 0.5+max(scores);
    else
       FaceDirection = 1 - max(scores);
    end

    %Accuire speed from Fuzzy function
    [Speed, ~] = evalfis([double(FaceDirection),double(fDist),double(lDist),double(rDist),double(bDist)],FuzzySpeed);

    %Possible negative speed, but we only wantv positive speed for the current purpose
    if Speed < 0
       Speed = 0;
    end
    Speed = Speed/15;
%     disp('Speed');disp(Speed);
end

