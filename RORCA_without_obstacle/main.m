clc;clear;close all
robot_radius=2;                       
  
num=0;

for theta = 0:0.2*pi:2*pi-0.0001
    num=num+1;
    robot_positions(num,:) = [30*cos(theta),30*sin(theta)]+[30,30];     % 10 robots are scattered on a circle
end
num=0;
for theta = -pi:0.2*pi:pi-0.0001
    num=num+1;
    goal(num,:) = [30*cos(theta),30*sin(theta)]+[30,30];                % 10 targets of the robots
end

V_current= 0.01*zeros(size(robot_positions,1),2);       % The initial velocity can be set to approximately zero
V_max=     2*ones(size(robot_positions,1),1);           
step = 0.4;                                             % Robot's planning cycle
color = colormap(jet(size(robot_positions,1)));        
tau = 2;
t = 0;                                                  % Starting moment

aviobj=VideoWriter('RORCA.avi');
open(aviobj);

while true
    V_pref    = compute_V_pref(robot_positions, goal, V_max);                       % Vpref is the preferred velocity of all robots
    V_current = RORCA_update(robot_positions, V_pref, V_current, robot_radius);     % Update the velocity of each robot
    t=t+1; 
    
    if V_current == zeros(size(V_current))
        break;              % all robots are reached their target
    end
    for i=1:size(robot_positions,1)
        robot_positions(i,:) = robot_positions(i,:)+ V_current(i,:)*step;
        eval(['rob_pos_',num2str(i) ,'(',num2str(t),',:) = robot_positions(',num2str(i),',:);']);   % Define and store the position coordinates of each robot
    end
    
    % ********************Display *************%
    for i=1:size(robot_positions,1)
        p=plot(goal(i,1),goal(i,2),'v','color',color(i,:));         % Display all targets
        hold on;           
        grid on;
    end
    for i=1:size(robot_positions,1)
        p=plot(robot_positions(i,1),robot_positions(i,2),'h','color',color(i,:));           % Display the current robot coordinates (circle)
        axis([-10,70,-10,70]);
        quiver(robot_positions(i,1),robot_positions(i,2),V_current(i,1),V_current(i,2),'MaxHeadSize',2.5,'AutoScaleFactor',5.00,'AutoScale','off');
        %%%%%%%%%%%  Draw the motion trajectory. Format£º p=plot(rob_pos_i(:,1),rob_pos_i(:,2),'-','color',color(i,:));    %%%%%%%%
        p=eval(['plot(rob_pos_',num2str(i),'(:,1),rob_pos_',num2str(i),'(:,2),''-'',''color'',color( ',num2str(i),',:));']);
    end        
    pause(0.05);
    hold off; 
    
    currFrame = getframe;
    writeVideo(aviobj,currFrame);       
end

close(aviobj);

% Output the last results
for i=1:size(robot_positions,1)
    p=plot(goal(i,1),goal(i,2),'v','color',color(i,:)); % Display targets
    axis([-10,70,-10,70]);
    hold on;
    grid on;
    p=plot(robot_positions(i,1),robot_positions(i,2),'h','color',color(i,:));                   % Display the current position of each robot
    p=eval(['plot(rob_pos_',num2str(i),'(:,1),rob_pos_',num2str(i),'(:,2),''-'',''color'',color( ',num2str(i),',:));']);    % The historical trajectory of each robot   
end
