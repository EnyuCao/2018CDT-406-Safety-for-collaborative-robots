function stop_move(vrep,clientID,sens)
%%Check what sensors were activated from left and right
    switch(sens(1))
        case 0 
              detected_l = 0;
        case 1
                  [~,left_handle]=vrep.simxGetObjectHandle(clientID,'leftfront_sensor',vrep.simx_opmode_blocking);
                  [~,~,detected_l,~,~]=vrep.simxReadProximitySensor(clientID,left_handle,vrep.simx_opmode_streaming);
                   [~,~,detected_l,~,~]=vrep.simxReadProximitySensor(clientID,left_handle,vrep.simx_opmode_buffer);
        case 2
                  [~,left_handle]=vrep.simxGetObjectHandle(clientID,'left_sensor',vrep.simx_opmode_blocking);
                  [~,~,detected_l,~,~]=vrep.simxReadProximitySensor(clientID,left_handle,vrep.simx_opmode_streaming);
                  [~,~,detected_l,~,~]=vrep.simxReadProximitySensor(clientID,left_handle,vrep.simx_opmode_buffer);

        case 3
                  [~,left_handle]=vrep.simxGetObjectHandle(clientID,'leftback_sesor',vrep.simx_opmode_blocking);
                  [~,~,detected_l,~,~]=vrep.simxReadProximitySensor(clientID,left_handle,vrep.simx_opmode_streaming);
                  [~,~,detected_l,~,~]=vrep.simxReadProximitySensor(clientID,left_handle,vrep.simx_opmode_buffer);
    end
    switch(sens(2))
        case 0
               detected_r = 0;
        case 1
                [~,right_handle]=vrep.simxGetObjectHandle(clientID,'rigthfront_sensor',vrep.simx_opmode_blocking);
                [~,~,detected_r,~,~]=vrep.simxReadProximitySensor(clientID,right_handle,vrep.simx_opmode_streaming);

        case 2
              [~,right_handle]=vrep.simxGetObjectHandle(clientID,'right_sensor',vrep.simx_opmode_blocking);
              [~,~,detected_r,~,~]=vrep.simxReadProximitySensor(clientID,right_handle,vrep.simx_opmode_streaming);
        case 3
               [~,right_handle]=vrep.simxGetObjectHandle(clientID,'rightback_sensor',vrep.simx_opmode_blocking);
               [~,~,detected_r,~,~]=vrep.simxReadProximitySensor(clientID,right_handle,vrep.simx_opmode_streaming);
    end
  %% Stay in loop until safe distance from objects
    thrhold = 0.01; % min distance allowed
    mindist = 0.2; % min distance allowed
    flag =1;
    while (flag)
        if(sens) %If objects on both sides
            [~,State_l,detected_l,~,~]=vrep.simxReadProximitySensor(clientID,left_handle,vrep.simx_opmode_buffer); %read from left
            [~,State_r,detected_r,~,~]=vrep.simxReadProximitySensor(clientID,right_handle,vrep.simx_opmode_buffer);%read from right
            de_l=norm(detected_l);%get distance to object from left
            de_r = norm(detected_r); %get distance to object from right
            detect_diff = abs(de_l - de_r); %same distance to right and left objects (middle)
            if(detect_diff <= thrhold) % if middle then go out from loop
                flag = 0;
            end
        else %If only one side
            if(sens(1)) %If object to the left
                [~,State_l,detected_l,~,~]=vrep.simxReadProximitySensor(clientID,left_handle,vrep.simx_opmode_buffer);
                   de_l = norm(detected_l); %Distance to object
                if (de_l > mindist || State_l == 0) %if safe distance stop going right
                    flag=0; 
                end
            else %If object to the right
                [~,State_r,detected_r,~,~]=vrep.simxReadProximitySensor(clientID,right_handle,vrep.simx_opmode_buffer);
                 de_r = norm(detected_r); %Distance to object
                if (de_r > mindist || State_r == 0) %if safe distance stop going left
                    flag=0; 
                end
            end
        end
    end
end

