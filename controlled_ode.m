function [xdot] = controlled_ode(t,x,payload,fig, writerObj)

    % variables
    kp = payload.kp;
    kd = payload.kd;
    ki = payload.ki;

    F = payload.Ft;
    I = payload.Izz;
    d = payload.l/2; % distance from bottom to CoM
    dt = payload.dt;

    theta_des = payload.theta_des;
    payload.bound = 20; % max magnitude of fin angle for small angle approx

    bounds = payload.bound;

    persistent count prev_error phi 
   
    theta = x(1);
    dtheta = x(2);

    % if the function is called for the first time initialize variables
    if isempty(prev_error)
        prev_error = 0;
        count = 1;
        phi = payload.phi0;
        prev_phi = 0;
        t_prev = 0;
        prev_error_i = 0;
    end

     % define the error in the system w/ euler approximations
    error = theta - theta_des; % error (theta - theta_des)
    error_d = x(2);
    %error_i = (error + prev_error) * (t-t_prev)/ 2;

    prev_error = error;
    
    % implement the PD controller
    phi= kp*error+ error_d*kd;% + ki*error_i;

    if phi<-bounds
        phi = -bounds;
    elseif phi>bounds
        phi = bounds;
    end
    
    % state vector
    xdot(1) = x(2);
    xdot(2) = (F*-sind(phi)*d)/I; % equation of motion for the system
    xdot = xdot';
    
   
    drawPayload(payload,theta,phi)
    title("Controlled Payload Animation")
    F = getframe(fig);
    writeVideo(writerObj,F);

    
    pause(0.00005)
    
    payload.phi_array(count) = phi;

    prev_phi = phi;

    count = count+1;