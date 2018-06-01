function [Speed,output]= FuzzyEval2(vrep, clientID, myNet, FuzzySpeed, sensorhandles)
% this function determins the speed of the robot and if a human is detected
% or not detected by the robot camera.
mode = vrep.simx_opmode_buffer;
%% Identify If person       
	[~,~,image]=vrep.simxGetVisionSensorImage2(clientID,sensorhandles(9),0,vrep.simx_opmode_buffer); % read image from the camera 
    image = imresize(image, [227 227]); % resize the image 
    [output,scores] = classify(myNet, image);  % output,  detected or not 
%% Sensor read              
    [~,~,fRead,~,~]=vrep.simxReadProximitySensor(clientID,sensorhandles(1),mode);
    [~,~,bRead,~,~]=vrep.simxReadProximitySensor(clientID,sensorhandles(5),mode);
    [~,~,lRead,~,~]=vrep.simxReadProximitySensor(clientID,sensorhandles(7),mode);
    [~,~,rRead,~,~]=vrep.simxReadProximitySensor(clientID,sensorhandles(3),mode);
    %%% distances of all sensors to an object or a person
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
    
    %Possible negative speed, but we only want positive speed for the current purpose
    if Speed < 0
       Speed = 0;
    end
    Speed = Speed/10; % return speed to diffe 


end

