clc;clear;close all
robot_radius=0.5;               


obs_positions=[30,40;
               40,20];         % Dynamic random obstacle position
obs_radius = 1.5;           
V_obs_max = 1.4;

theta_obs1=rand(1,1)*6.14;         % The direction of the obstacle's movement
theta_obs2=rand(1,1)*3.14;

obs_velocity = [V_obs_max*cos(theta_obs1),V_obs_max*sin(theta_obs1);
                V_obs_max*cos(theta_obs2),V_obs_max*sin(theta_obs2)]; % velocity of dynamic obstacles

num=0;
for theta = 0:0.2*pi:2*pi-0.0001
    num=num+1;
    robot_positions(num,:) = [30*cos(theta),30*sin(theta)]+[30,30];
end
num=0;
for theta = -pi:0.2*pi:pi-0.0001
    num=num+1;
    goal(num,:) = [30*cos(theta),30*sin(theta)]+[30,30];
end

V_current= 0.01*zeros(size(robot_positions,1),2);    
V_max=     2*ones(size(robot_positions,1),1);        
step = 0.8;                                       
color = colormap(jet(size(robot_positions,1)));       
tau = 2;
t = 0;                                        

% % aviobj=VideoWriter('RORCA.avi');
% % open(aviobj);

while true
    theta_obs1=rand(1,1)*6.14;         % The direction of the obstacle's movement
    theta_obs2=rand(1,1)*6.14;

    obs_velocity = [V_obs_max*cos(theta_obs1),V_obs_max*sin(theta_obs1);
                    V_obs_max*cos(theta_obs2),V_obs_max*sin(theta_obs2)]; % velcity of dynamic obstacles
    
    V_pref    = compute_V_pref(robot_positions, goal, V_max);     % Preferred velocities of all agents
    V_current = RORCA_update(robot_positions, V_pref, V_current, robot_radius,obs_positions,obs_velocity,obs_radius);
    t=t+1; 
    
    if V_current == zeros(size(V_current))
        break;              % Judge whether the goal is reached
    end
    for i=1:size(robot_positions,1)
        robot_positions(i,:) = robot_positions(i,:)+ V_current(i,:)*step;
        eval(['rob_pos_',num2str(i) ,'(',num2str(t),',:) = robot_positions(',num2str(i),',:);']);   % Define and store the location of each robot
    end
    %%%%%%%%%%%%%  Dynamic obstacles
    for i=1:size(obs_positions,1)
        obs_positions(i,:) = obs_positions(i,:)+ obs_velocity(i,:)*step;
        if t>130
            obs_velocity(i,:)=[0,0];
        end
        eval(['obs_pos_',num2str(i) ,'(',num2str(t),',:) = obs_positions(',num2str(i),',:);']);   % Define and store the location of each obstacle
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % *******************  Display  *************%
    for i=1:size(robot_positions,1)
        p=plot(goal(i,1),goal(i,2),'v','color',color(i,:));
        hold on;  
        grid on;
    end
    
    for i=1:size(obs_positions,1)
        p=plot(obs_positions(i,1),obs_positions(i,2),'h','color',color(i,:));           
        axis([-10,70,-10,70]);
        quiver(obs_positions(i,1),obs_positions(i,2),obs_velocity(i,1),obs_velocity(i,2),'MaxHeadSize',2.5,'AutoScaleFactor',5.00,'AutoScale','off');

        p=eval(['plot(obs_pos_',num2str(i),'(:,1),obs_pos_',num2str(i),'(:,2),''-k'');']);  
    end

    for i=1:size(robot_positions,1)
        p=plot(robot_positions(i,1),robot_positions(i,2),'h','color',color(i,:));    
        axis([-10,70,-10,70]);
        quiver(robot_positions(i,1),robot_positions(i,2),V_current(i,1),V_current(i,2),'MaxHeadSize',2.5,'AutoScaleFactor',5.00,'AutoScale','off');

        p=eval(['plot(rob_pos_',num2str(i),'(:,1),rob_pos_',num2str(i),'(:,2),''-'',''color'',color( ',num2str(i),',:));']);
    end        
    pause(0.05);
    hold off; 
    
% %     currFrame = getframe;
% %     writeVideo(aviobj,currFrame);       
end

% % close(aviobj);

% Display the last output result
for i=1:size(robot_positions,1)
    p=plot(goal(i,1),goal(i,2),'v','color',color(i,:)); 
    axis([-10,70,-10,70]);
    hold on;
    grid on;
    p=plot(robot_positions(i,1),robot_positions(i,2),'h','color',color(i,:));  
    p=eval(['plot(rob_pos_',num2str(i),'(:,1),rob_pos_',num2str(i),'(:,2),''-'',''color'',color( ',num2str(i),',:));']);

end
