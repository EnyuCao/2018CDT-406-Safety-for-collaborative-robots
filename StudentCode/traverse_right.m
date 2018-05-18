function traverse_right(vrep, clientID, motor_fl, motor_fr, motor_rl, motor_rr, speed, mode)
    vrep.simxSetJointTargetVelocity(clientID, motor_fl, speed, mode);
    vrep.simxSetJointTargetVelocity(clientID, motor_fr, -speed, mode);
    vrep.simxSetJointTargetVelocity(clientID, motor_rl, -speed, mode);
    vrep.simxSetJointTargetVelocity(clientID, motor_rr, speed, mode);
end