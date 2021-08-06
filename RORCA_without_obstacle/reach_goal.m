function [reach] = reach_goal(x,g)

if norm(g-x)<=0.8
    reach = true;
else
    reach = false;
end
    
end

