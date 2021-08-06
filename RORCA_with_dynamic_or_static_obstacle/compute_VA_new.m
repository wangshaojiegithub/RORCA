function [V_post] = compute_VA_new(V_A,V_pref_i,RORCA_BA_info,VO_BA_info)

V_pref_max = norm(V_pref_i);

suitable_V = [];
unsuitable_V = [];
suit_num=0;
unsuit_num=0;

        V_O = [VO_BA_info(5),VO_BA_info(6)];     
        O_A = [VO_BA_info(7),VO_BA_info(8)]; 
        
if norm(O_A)<=4
    
        V_angle_changed = V_O*[cos(pi/4),-sin(pi/4);sin(pi/4),cos(pi/4)];
        V_post=V_angle_changed/norm(V_angle_changed)*1;
        

elseif V_pref_max==0
    V_post = [0,0];    
else
    % *****************  Determine whether the new sampling speed meets the conditions  **************** %
    for theta = 0:0.1:2*pi
        for rad = 0.02:(V_pref_max/5):(V_pref_max+0.02)         % Add 0.02 to ensure that the starting value is 0.02 and will not be zero
            V_sample = [rad*cos(theta), rad*sin(theta)];        % Sample a new speed
            suit = true;
            % *******  Check the new speed of sampling and the speed obstacle cone of many Agent B and obstacle O  ********* %
            for n=1:size(RORCA_BA_info,1)
                safe_VA = [RORCA_BA_info(n,1),RORCA_BA_info(n,2)];    
                force_direct = [RORCA_BA_info(n,3),RORCA_BA_info(n,4)];
                
                V_B = [RORCA_BA_info(n,5),RORCA_BA_info(n,6)];      
                B_A = [RORCA_BA_info(n,7),RORCA_BA_info(n,8)]; 
                theta_BA_cone = RORCA_BA_info(n,9);
                vec_vertex_vnew =  V_sample - V_B; 
          
                if dot(V_sample-safe_VA,force_direct)<0 
                    suit = false;
                    break;
                end

            end
            % ********** (2)Check the  velocity outside of VO ********%
            if suit==true                
                for u=1:size(VO_BA_info,1)
                    safe_VA = [VO_BA_info(u,1),VO_BA_info(u,2)];        
                    force_direct = [VO_BA_info(u,3),VO_BA_info(u,4)];

                    V_O = [VO_BA_info(u,5),VO_BA_info(u,6)];    
                    O_A = [VO_BA_info(u,7),VO_BA_info(u,8)]; 
                    theta_OA_cone = VO_BA_info(u,9);
                    vec_vertex_vnew =  V_sample - V_O;      

                       
                    if dot(V_sample-safe_VA,force_direct)<0 
                        suit = false;
                        break;
                    end
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

    %**************** (3)judge whether Vpref meets the conditions******************%
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
        for u=1:size(VO_BA_info,1)
            safe_VA = [VO_BA_info(u,1),VO_BA_info(u,2)];       
            force_direct = [VO_BA_info(u,3),VO_BA_info(u,4)];

            V_O = [VO_BA_info(u,5),VO_BA_info(u,6)];   
            O_A = [VO_BA_info(u,7),VO_BA_info(u,8)]; 
            theta_OA_cone = VO_BA_info(u,9);
            vec_vertex_vnew =  V_sample - V_O;   
               
            if dot(V_sample-safe_VA,force_direct)<0 
                suit = false;
                break;
            end
        end   
    end

    if suit==true
        suit_num=suit_num+1;
        suitable_V(suit_num,:)=V_sample;
    else
        unsuit_num=unsuit_num+1;
        unsuitable_V(unsuit_num,:)=V_sample;
    end
    
    %*****************************  Find the optimal speed according to the situation  ************************************%
    %**********First consider the situation with feasible solutions, and find the optimal speed from the solution set (the closest distance to Vpref)*******%
    suit_V_dist = zeros(suit_num,1);     % Store the distance between the feasible solution and Vpref, and then calculate its index number
    if suit_num~=0
        for s=1:suit_num

            suit_V_dist(s) = -0.7*dot(suitable_V(s,:),V_pref_i)/norm(V_pref_i)-0.3*dot(suitable_V(s,:),V_A)/norm(V_A);

        end
            
        [~,min_num] = min(suit_V_dist);             % Store the index number with the shortest trade-off distance between feasible solutions and Vpref and VA
        V_post = suitable_V(min_num,:);               % optimal speed in this situation
        
    else
        %**************£¨2£©Consider the case where there is no feasible solution**************%
        V_post = [0,0];
    end
end

end


