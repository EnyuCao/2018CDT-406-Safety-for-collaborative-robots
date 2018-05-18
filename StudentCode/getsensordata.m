function [state, detection] = getsensordata(clientID, sensorhandle, vrep)
    detection = 1;
    for i = 1:length(sensorhandle)
        [~,state(i),~,~,~] = vrep.simxReadProximitySensor(clientID,sensorhandle(i),vrep.simx_opmode_streaming);
        [~,state(i),~,~,~] = vrep.simxReadProximitySensor(clientID,sensorhandle(i),vrep.simx_opmode_buffer);
    end
end