function stop_move(vrep,clientID,sens)
%     display(sens);
    switch(sens(1))
        case 0 
              disp('left side clear');
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
               disp('right side clean');
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
    
    thrhold = 0.01; % change this value
    mindist = 0.2; % change this value 2
    flag =1;
    while (flag)
        if(sens)
            [~,State_l,detected_l,~,~]=vrep.simxReadProximitySensor(clientID,left_handle,vrep.simx_opmode_buffer);
            [~,State_r,detected_r,~,~]=vrep.simxReadProximitySensor(clientID,right_handle,vrep.simx_opmode_buffer);
            de_l=norm(detected_l);
            de_r = norm(detected_r);
            detect_diff = abs(de_l - de_r);
            if(detect_diff <= thrhold)%go middle
                flag = 0;
            end
        else
            if(sens(1))
                disp('Im here');
                [~,State_l,detected_l,~,~]=vrep.simxReadProximitySensor(clientID,left_handle,vrep.simx_opmode_buffer);
                   de_l = norm(detected_l);
                if (de_l > mindist || State_l == 0)
                    flag=0;%go rite
                end
            else
                [~,State_r,detected_r,~,~]=vrep.simxReadProximitySensor(clientID,right_handle,vrep.simx_opmode_buffer);
                 de_r = norm(detected_r);
                if (de_r > mindist || State_r == 0)
                    flag=0;%go left
                end
            end
        end

        %disp('Inside');
    end
end

