function [reach] = reach_goal(x,g)

if norm(g-x)<=1
    reach = true;
else
    reach = false;
end
    
end

