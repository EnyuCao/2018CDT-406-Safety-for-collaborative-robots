function diffe(vrep, clientID,motor_fl, motor_fr, motor_rl, motor_rr,theta,myNet,FuzzySpeed)
[~,camera]=vrep.simxGetObjectHandle(clientID,'Vision_sensor',vrep.simx_opmode_blocking); %Get handle from camera
[~,~,image]=vrep.simxGetVisionSensorImage2(clientID,camera,0,vrep.simx_opmode_streaming); %Initialize Camera
mode1 = vrep.simx_opmode_blocking;
mode2 = vrep.simx_opmode_streaming;
mode3 = vrep.simx_opmode_buffer;
v_des = FuzzyEval2(vrep, clientID, myNet, FuzzySpeed, image); % Get speed from fuzzy controller
rot_des = v_des * (theta); % desired rotation rate.

d = 0.3;               % distance between two wheels
r_w = 0.05;            % wheel radius
v_rr = v_des + d*rot_des; 
v_rl = v_des - d*rot_des; 
speed_right = v_rr/r_w    % velocity right side
speed_left  = v_rl/r_w    % velocity left side
speed = (speed_right + speed_left)/2;  % forward/backward velocity

%% Set motors velocity
vrep.simxSetJointTargetVelocity(clientID, motor_fl, speed_left, mode1);
vrep.simxSetJointTargetVelocity(clientID, motor_fr, speed_right,mode1);
vrep.simxSetJointTargetVelocity(clientID, motor_rl, speed_left, mode1);
vrep.simxSetJointTargetVelocity(clientID, motor_rr, speed_right,mode1);
 
%% Get handles from sensors 
[~,front_sensor]=vrep.simxGetObjectHandle(clientID,'front_sensor',mode1);
[~,back_sensor]=vrep.simxGetObjectHandle(clientID,'back_sensor',mode1);
[~,left_sensor]=vrep.simxGetObjectHandle(clientID,'left_sensor',mode1);
[~,right_sensor]=vrep.simxGetObjectHandle(clientID,'right_sensor',mode1);
[~,leftback_sensor]=vrep.simxGetObjectHandle(clientID,'leftback_sensor',mode1);
[~,leftfront_sensor]=vrep.simxGetObjectHandle(clientID,'leftfront_sensor',mode1);
[~,rightback_sensor]=vrep.simxGetObjectHandle(clientID,'rightback_sensor',mode1);
[~,rightfront_sensor]=vrep.simxGetObjectHandle(clientID,'rightfront_sensor',mode1);

%% Start streaming from sensors
[~,State_l,detected_left,~,~]=vrep.simxReadProximitySensor(clientID,left_sensor,mode2);
[~,State_r,detected_right,~,~]=vrep.simxReadProximitySensor(clientID,right_sensor,mode2);
[~,State_f,detected_front,~,~]=vrep.simxReadProximitySensor(clientID,front_sensor,mode2);
[~,State_b,detected_back,~,~]=vrep.simxReadProximitySensor(clientID,back_sensor,mode2);
[~,State_lb,detected_lb,~,~]=vrep.simxReadProximitySensor(clientID,leftback_sensor,mode2);
[~,State_lf,detected_lf,~,~]=vrep.simxReadProximitySensor(clientID,leftfront_sensor,mode2);
[~,State_rb,detected_rb,~,~]=vrep.simxReadProximitySensor(clientID,rightback_sensor,mode2);
[~,State_rf,detected_rf,~,~]=vrep.simxReadProximitySensor(clientID,rightfront_sensor,mode2);
 
 %%  Read from sensors
[~,State_l,detected_left,~,~] = vrep.simxReadProximitySensor(clientID,left_sensor,mode3);
[~,State_r,detected_right,~,~]= vrep.simxReadProximitySensor(clientID,right_sensor,mode3);
[~,State_f,detected_front,~,~]= vrep.simxReadProximitySensor(clientID,front_sensor,mode3);
[~,State_b,detected_back,~,~] = vrep.simxReadProximitySensor(clientID,back_sensor,mode3);
[~,State_lb,detected_lb,~,~]  = vrep.simxReadProximitySensor(clientID,leftback_sensor,mode3);
[~,State_lf,detected_lf,~,~]  = vrep.simxReadProximitySensor(clientID,leftfront_sensor,mode3);
[~,State_rb,detected_rb,~,~]  = vrep.simxReadProximitySensor(clientID,rightback_sensor,mode3);
[~,State_rf,detected_rf,~,~]  = vrep.simxReadProximitySensor(clientID,rightfront_sensor,mode3);

%% Stay in loop until safe distance from objects
  flag = 1;
  distmin = 0.5;
  while flag
      if (State_f || State_b) % If front or back sensors are activated 
          mindist = 0.2; % min allowed distance to object
          if State_f % if front sensor is activated
              backward(vrep, clientID, motor_fl, motor_fr, motor_rl, motor_rr, speed, mode1); % Go backwards
              [~,State_f,detected_front,~,~]=vrep.simxReadProximitySensor(clientID,front_sensor,mode3); %Read sensor 
              de_f = norm(detected_front); %Get new distance to object
              if (de_f > mindist || State_f == 0) %if distance is safe or sensor not active then stop going backwards
                flag = 0;
              end
          elseif (State_b) && (norm(detected_back) < distmin)% If back sensor is activated
              forward(vrep, clientID, motor_fl, motor_fr, motor_rl, motor_rr, speed, mode1); % Go forward
              [~,State_b,detected_back,~,~]=vrep.simxReadProximitySensor(clientID,back_sensor,mode3); %Read sensor
              de_b = norm(detected_back); %Get new distance to object
              if (de_b > mindist || State_b == 0) %if distance is safe or sensor not active then stop going forward
                flag = 0;
              end        
          end
      else %if neither front or back are active, go out of loop
          flag = 0;
      end  
  end
  if (State_lf||State_l || State_lb) || (State_rf||State_r||State_rb) %If side sensors are active
     shortest_left = inf;
     shortest_right = inf;
     sens_l = 0;
     sens_r = 0;
     %%Choose sensor with shortest distance to object, if any.
     if(State_lf)
        shortest_left = norm(detected_lf);
        sens_l = 1;
     end
     if State_rf
        shortest_right = norm(detected_rf);
        sens_r = 1;
     end
      if(shortest_left > norm(detected_left) && State_l ~= 0 && norm(detected_left) < distmin)
        shortest_left = norm(detected_left);
        sens_l = 2;
      end
      if(shortest_left > norm(detected_lb) && State_lb ~= 0)
        shortest_left = norm(detected_lb); 
        sens_l = 3;
      end
      if(shortest_right> norm(detected_right) && State_r ~= 0 && norm(detected_right) < distmin)
        shortest_right = norm(detected_right);
        sens_r = 2;
      end
      if(shortest_right > norm(detected_rb) && State_rb ~=0 )
        shortest_right = norm(detected_rb); 
        sens_r = 3;
      end
    %%If objects on both sides, center the robot
     if(State_lf || (State_l &&  norm(detected_left) < distmin) || State_lb) && (State_rf || (State_r &&  norm(detected_right) < distmin) || State_rb)
        if shortest_right > shortest_left %if object closer to the left, then go right.
            traverse_right(vrep, clientID, motor_fl, motor_fr, motor_rl, motor_rr, ...
                speed_right,vrep.simx_opmode_blocking);
            stop_move(vrep, clientID,[sens_l sens_r]);   
        else %if object closer to the right, then go left.
            traverse_left(vrep, clientID, motor_fl, motor_fr, motor_rl, motor_rr, ...
                speed_left,vrep.simx_opmode_blocking);
            stop_move(vrep, clientID,[sens_l sens_r]);
        end

    elseif (State_lf || (State_l &&  norm(detected_left) < distmin) || State_lb) %if only object to the left then go right
        traverse_right(vrep, clientID, motor_fl, motor_fr, motor_rl, motor_rr, ...
            speed_right,vrep.simx_opmode_blocking);
        stop_move(vrep, clientID,[sens_l 0]);
     elseif (State_rf || (State_r &&  norm(detected_right) < distmin)|| State_rb) %if only object to the right then go left
        traverse_left(vrep, clientID, motor_fl, motor_fr, motor_rl, motor_rr, ...
            speed_left,vrep.simx_opmode_blocking);
        stop_move(vrep, clientID,[0 sens_r]);
    end
 end  
               
         
end
     
     
     
     
     
