function [V_post] = compute_VA_new(V_A,V_pref_i,RORCA_BA_info)
V_pref_max = norm(V_pref_i);

suitable_V = [];
unsuitable_V = [];
suit_num=0;
unsuit_num=0;

if V_pref_max==0
    V_post = [0,0];     
else
    % *******************Determine whether the new sampling speed meets the conditions****************** %
    for theta = 0:0.1:2*pi
        for rad = 0.02:(V_pref_max/5):(V_pref_max+0.02)         % Add 0.02 to ensure that the starting value is 0.02 and will not be zero
            V_sample = [rad*cos(theta), rad*sin(theta)];        % Sample a new velocity
            suit = true;
            % *******The sampling speed is judged to be included with the speed obstacle cones of many Agent B and obstacle O********* %
            for n=1:size(RORCA_BA_info,1)
                safe_VA = [RORCA_BA_info(n,1),RORCA_BA_info(n,2)];        
                force_direct = [RORCA_BA_info(n,3),RORCA_BA_info(n,4)];
                
                V_B = [RORCA_BA_info(n,5),RORCA_BA_info(n,6)];        
                B_A = [RORCA_BA_info(n,7),RORCA_BA_info(n,8)]; 
                theta_BA_cone = RORCA_BA_info(n,9);

                vec_vertex_vnew =  V_sample - V_B;      
                [vnew_In_between]=in_between(vec_vertex_vnew,B_A,theta_BA_cone);
 
                if dot(V_sample-safe_VA,force_direct)<0 
                    suit = false;
                    break;
                end

            end

            if suit==true
                suit_num=suit_num+1;
                suitable_V(suit_num,:)=V_sample;            
            else
                unsuit_num=unsuit_num+1;
                unsuitable_V(unsuit_num,:)=V_sample;            
            end
        end      
    end

    %**************** Determine whether Vpref meets the conditions******************%
    V_sample = V_pref_i;
    suit = true;
    for m=1:size(RORCA_BA_info,1)
        safe_VA = [RORCA_BA_info(m,1),RORCA_BA_info(m,2)];       
        force_direct = [RORCA_BA_info(m,3),RORCA_BA_info(m,4)];
        
        V_B = [RORCA_BA_info(m,5),RORCA_BA_info(m,6)];      
        B_A = [RORCA_BA_info(m,7),RORCA_BA_info(m,8)]; 
        theta_BA_cone = RORCA_BA_info(m,9);
        dist_BA = norm(B_A);                       
        vec_vertex_vnew =  V_sample - V_B;     
        [vnew_In_between]=in_between(vec_vertex_vnew,B_A,theta_BA_cone);
        
        if dot(V_sample-safe_VA,force_direct)<0 
            suit = false;
            break;
        end
    end    

    if suit==true
        suit_num=suit_num+1;
        suitable_V(suit_num,:)=V_sample;
    else
        unsuit_num=unsuit_num+1;
        unsuitable_V(unsuit_num,:)=V_sample;
    end
    
    %************************************Find the optimal speed according to the situation************************************%
    %**********£¨1£©First consider the situation with feasible solutions, and find the optimal speed from the solution set (the closest distance to Vpref)
    suit_V_dist = zeros(suit_num,1);     % Used to store the distance between the feasible solution and Vpref, and then calculate its index number
    if suit_num~=0
        for s=1:suit_num

            suit_V_dist(s) = -0.7*dot(suitable_V(s,:),V_pref_i)/norm(V_pref_i)-0.3*dot(suitable_V(s,:),V_A)/norm(V_A);

        end
        [~,min_num] = min(suit_V_dist);             % min_num stores the index number with the shortest trade-off distance between feasible solutions and Vpref and VA   
        V_post = suitable_V(min_num,:);               % the optimal speed in this situation
    else

        tc_V = zeros(unsuit_num,1);
        penalty = zeros(unsuit_num,1);
        penalty_index = 0;
        for us=1:unsuit_num
            V_sample = unsuitable_V(us,:);
            collision_cone_num = 0;         
            for un=1:size(RORCA_BA_info,1)
                V_B = [RORCA_BA_info(n,5),RORCA_BA_info(n,6)];   
                 B_A = [RORCA_BA_info(un,7),RORCA_BA_info(un,8)]; 
                 dist_BA = norm(B_A);          
                theta_BA_cone = RORCA_BA_info(un,9);
                rad_rad = RORCA_BA_info(un,10);                    % The sum of the radius of a certain B and A

                vec_vertex_vnew =  V_sample - V_B;      
                dist_BA = norm(B_A);               
                
                [vnew_In_between]=in_between(vec_vertex_vnew,B_A,theta_BA_cone);
                if vnew_In_between==true
                    small_theta = acos(dot(vec_vertex_vnew,B_A)/(norm(vec_vertex_vnew)*dist_BA));    % The angle between the infeasible relative velocity and the center line of the VO cone
                    big_theta = asin(abs(dist_BA*sin(small_theta))/rad_rad);
                    dist_tg = abs(dist_BA*cos(small_theta))-abs(rad_rad*cos(big_theta)); % The distance between the infeasible relative velocity direction and the intersection of the VO cone expansion circle
                    tc_vab = dist_tg/norm(vec_vertex_vnew);             % The collision time between A and B at the sampling speed
                    collision_cone_num = collision_cone_num+1;
                    collision_t_rvo(collision_cone_num) = tc_vab;       % Store the time that the speed collides with all B             
                else
                    collision_cone_num = collision_cone_num+1;
                    collision_t_rvo(collision_cone_num) = 0;
                end
            end
           

            [tc_V(us),~] = min(collision_t_rvo);    % Store the shortest time that A collide with all Agent B
            tc_V(us) = tc_V(us)+0.001;              % Here is to avoid zero time       
        end
        %%%%%%%%****************** penalty function **********************
        weight = 0.2;
        for us=1:unsuit_num
            penalty(us) = weight/tc_V(us)+norm(unsuitable_V(us,:)-V_pref_i);
        end
        [~,penalty_index] = min(penalty);
        V_post = [0,0];
    end
end

end


