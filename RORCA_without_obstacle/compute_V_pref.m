function [V_pref] = compute_V_pref(robot_positions, goal, V_max)


V_pref= zeros(size(robot_positions,1),2);               
for i=1:size(robot_positions,1)
    vec_pos_goal = goal(i,:) - robot_positions(i,:);   
    norm_pos_goal = norm(vec_pos_goal);                
    vec_pref = vec_pos_goal/norm_pos_goal*V_max(i);     
    V_pref(i,:) = vec_pref;                            
    [reach] = reach_goal(robot_positions(i,:),goal(i,:));
    if reach==true
        V_pref(i,:) = [0,0];
    end
end
    
end

