function state = getsensordata(clientID, sensorhandle, vrep)
    for i = 1:length(sensorhandle)% Check states of all eight sensors
        [~,state(i),detect,~,~] = vrep.simxReadProximitySensor(clientID,sensorhandle(i),vrep.simx_opmode_streaming);
        [~,state(i),detect,~,~] = vrep.simxReadProximitySensor(clientID,sensorhandle(i),vrep.simx_opmode_buffer);
        if i == 1%if the front sensor is being diagnosed, make sure that it is only set to active inside the correct bounds
            if state(1) == 1 && norm(detect) > 0.65
                state(1) = 0;
            end
        end
    end
end