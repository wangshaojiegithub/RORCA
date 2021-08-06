function [V_new] = RORCA_update(robot_positions, V_pref, V_current, robot_radius)

% Update the current velocity of all Agent A

tau=2;              
E=0.1;                          % Radius expansion
rob_rad = robot_radius + E;
V_new = V_current;                
for i=1:size(robot_positions,1)
    V_A = V_current(i,:);               % Current velocity of Agent A
    P_A = robot_positions(i,:);         % Current position of Agent A
    RORCA_BA_info = [];                   % RORCA information induced by all robots B

    %************************* consider all other robots *********************%
    for j=1:size(robot_positions,1)
        if i~=j
            V_B = V_current(j,:);                       % The current velocity of any robot B
            P_B = robot_positions(j,:);                 % The current position of any robot B
           
            vertex_VB_VA = V_B;           
            vec_vertex_VB_VA = V_A-V_B;    %  current relative velocity

            B_A = P_B-P_A;                              
            dist_BA = norm(B_A);
            rad_rad = 2*rob_rad;

            if rad_rad > dist_BA
                dist_BA = rad_rad;    
            end
            theta_BA_cone = asin(rad_rad/dist_BA);    % 1/2 *VO cone angle
    
            [VA_VB_In_between] = in_between(vec_vertex_VB_VA,B_A,theta_BA_cone);     % Determine whether the current relative speed is within the cone, and solve the angle with the center line of the cone
  
            proj_VA=dot(V_A,B_A)/norm(B_A)^2*B_A;       % Projection of V_A on the center line
            proj_VB=dot(V_B,B_A)/norm(B_A)^2*B_A;       % Projection of V_B on the center line
           
            alpha=norm(proj_VA)/(norm(proj_VA)+norm(proj_VB));      % The proportion of V_A in the velocity change
            
            V_A_B_tau = B_A/tau-rad_rad*B_A/norm(B_A);                          % The relative velocity at which A and B will collide at time ¦Ó
            proj_VA_VB_BA = dot(vec_vertex_VB_VA,B_A)/norm(B_A)^2*(B_A);        % The projection of the VA_VB vector on the center line!
            diff_VA_VB = -proj_VA_VB_BA + V_A_B_tau; 
            if dot(V_A,V_B)<0
                safe_VA = V_A+alpha*diff_VA_VB; 
            else
                safe_VA = V_A-proj_VA+alpha*V_A_B_tau;
            end
                
            if j > i
                RORCA_BA_info(j-1,:) = [safe_VA,-B_A,vertex_VB_VA,B_A,theta_BA_cone,rad_rad]; %###############       
            else
                RORCA_BA_info(j,:) = [safe_VA,-B_A,vertex_VB_VA,B_A,theta_BA_cone,rad_rad];    
            end 
            %##############################################################            
        end
    end
    

    V_post = compute_VA_new(V_A,V_pref(i,:), RORCA_BA_info);     
    V_new(i,:) = V_post;
    
end

end

