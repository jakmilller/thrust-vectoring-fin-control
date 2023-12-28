function drawPayload(payload, theta, phi)

    % variables
    l = payload.l;
    w = payload.w;
    fin_l = payload.fin_l;
    fin_w = payload.fin_w;
    plotbounds = payload.plotbounds;

    % x and y coordinates of payload body
    body_x = [-w/2 -w/2 w/2 w/2];
    body_y = [-l/2 l/2 l/2 -l/2];
    
    % x and y coordinates of fin
    fin_x = [-fin_w/2 -fin_w/2 fin_w/2 fin_w/2];
    fin_y = [-fin_l-(l/2) -l/2 -l/2 -fin_l-(l/2)];

    fin_point = [0;-(l/2)]; % point that the fin rotates about

    % rotation matrix for payload body
    rot_matrix = [cosd(theta) -sind(theta); sind(theta) cosd(theta)];
    rot_body = rot_matrix*[body_x; body_y];
    
    % rotated body coordinates
    rot_body_x = rot_body(1,:);
    rot_body_y = rot_body(2,:);
    rot_fin = rot_matrix*[fin_x;fin_y]; % fin rotated by payload heading
    rot_fin_point = rot_matrix*fin_point; % point after heading rotation
    
    
    % rotation matrix for fin
    rot_matrix2 = [cosd(phi) -sind(phi); sind(phi) cosd(phi)];
    
    % rotate fin around the connection point fin_point
    rot_fin = rot_fin - rot_fin_point;
    rot_fin = rot_matrix2*rot_fin;
    rot_fin = rot_fin + rot_fin_point;
    
    % final fin points
    rot_fin_x = rot_fin(1,:);
    rot_fin_y = rot_fin(2,:);

    
    % plot rotated coordinates
    cla
    fill(rot_body_x,rot_body_y,'b')
    hold on
    fill(rot_fin_x,rot_fin_y,'r')
    axis equal
    grid on
    axis(plotbounds)
    
    