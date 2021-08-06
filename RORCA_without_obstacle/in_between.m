function [In_between]=in_between(vec_vertex_V,B_A,theta_BA_cone)

    theta_v = acos(dot(vec_vertex_V,B_A)/(norm(vec_vertex_V)*norm(B_A))); 

if theta_v<theta_BA_cone
    In_between=true;
else
    In_between=false;
end

