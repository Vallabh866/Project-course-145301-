clearvars
close all

x_max = 200;
y_max = 160;
xy = [65 92; 70 85; 75 75; 85 100; 87 90; 90 80; 110 100; 107 90; 105 80; 120 95; 120 75];

%Define road edges as obstacles
obstacle = [50,15,20,20];
circr = @(radius,rad_ang)  [radius*cos(rad_ang);  radius*sin(rad_ang)];         % Circle Function For Angles In Radians
%circd = @(radius,deg_ang)  [radius*cosd(deg_ang);  radius*sind(deg_ang)];       % Circle Function For Angles In Degrees
N = 250;                                                         % Number Of Points In Complete Circle
r_angl = linspace(pi/4, 3*pi/4, N);                             % Angle Defining Arc Segment (radians)
radius1 = 105;                                                   % Arc Radius
center1 = [100 105];
close all;
clc;

radius2 = 75;
center2 = [100 75];
xy_r1 = circr(radius1,r_angl);                                    % Matrix (2xN) Of (x,y) Coordinates
xy_r2 = circr(radius2,r_angl);
xy_r1 = transpose(xy_r1);
xy_r2 = transpose(xy_r2);
plot(xy_r1(:,1)+100, xy_r1(:,2), 'b')
grid on
hold on

plot(xy_r2(:,1)+100, xy_r2(:,2), 'b')
hold on


EPS = 45;
numNodes = 100;        

q_start.coord = [65 82];
q_start.cost = 0;
q_start.parent = 0;
q_goal.coord = [120 75];
q_goal.cost = 0;

nodes(1) = q_start;
%figure(1)
%axis([0 x_max 0 y_max])
%rectangle('Position',obstacle,'FaceColor',[0 .5 .5])
%hold on

for i = 1:1:numNodes
    q_rand = xy(((round(rand*10)/10)*10) + 1, :);
    plot(q_rand(1), q_rand(2), 'x', 'Color',  [0 0.4470 0.7410])
    
    % Break if goal node is already reached
    for j = 1:1:length(nodes)
        if nodes(j).coord == q_goal.coord
            %fprintf("Goal reached");
            break
        end
    end
    
    % Pick the closest node from existing list to branch out from
    ndist = [];
    for j = 1:1:length(nodes)
        n = nodes(j);
        tmp = dist(n.coord, q_rand);
        ndist = [ndist tmp];
    end
    [val, idx] = min(ndist);
    q_near = nodes(idx);
    
    q_new.coord = steer(q_rand, q_near.coord, val, EPS);
    line([q_near.coord(1), q_new.coord(1)], [q_near.coord(2), q_new.coord(2)], 'Color', 'k', 'LineWidth', 2);
    drawnow
    hold on
    q_new.cost = dist(q_new.coord, q_near.coord) + q_near.cost;
        
    % Within a radius of r, find all existing nodes
    q_nearest = [];
    r = 10;
    neighbor_count = 1;
    for j = 1:1:length(nodes)
        if dist(nodes(j).coord, q_new.coord) <= r
            q_nearest(neighbor_count).coord = nodes(j).coord;
            q_nearest(neighbor_count).cost = nodes(j).cost;
            neighbor_count = neighbor_count+1;
        end
    end
        
    % Initialize cost to currently known value
    q_min = q_near;
    C_min = q_new.cost;
        
    % Iterate through all nearest neighbors to find alternate lower
    % cost paths
        
    for k = 1:1:length(q_nearest)
        if dist(q_nearest(k).coord, q_new.coord) < C_min
            q_min = q_nearest(k);
            C_min = q_nearest(k).cost + dist(q_nearest(k).coord, q_new.coord);
            end
    end
        
    % Update parent to least cost-from node
    for j = 1:1:length(nodes)
        if nodes(j).coord == q_min.coord
            q_new.parent = j;
        end
    end
        
    % Append to nodes
    nodes = [nodes q_new];
end

D = [];
for j = 1:1:length(nodes)
    tmpdist = dist(nodes(j).coord, q_goal.coord);
    %fprintf("Nodes: %d \n", nodes(j).coord);
    D = [D tmpdist];
end


% Search backwards from goal to start to find the optimal least cost path
[val, idx] = min(D);    
q_final = nodes(idx);
q_goal.parent = idx;
q_end = q_goal;
nodes = [nodes q_goal];
final_nodes = [];
while q_end.parent ~= 0
    start = q_end.parent;
    line([q_end.coord(1), nodes(start).coord(1)], [q_end.coord(2), nodes(start).coord(2)], 'Color', 'r', 'LineWidth', 2);
    hold on
    %fprintf("Line nodes: %d %d\n",nodes(start).coord(1), nodes(start).coord(2) );
    final_nodes = [final_nodes ; nodes(start).coord(1), nodes(start).coord(2)];
    final_nodes = unique(final_nodes, 'rows', 'stable');
    q_end = nodes(start);
end

for k = 1:1:length(final_nodes)
         fprintf("Nodes: %d %d\n",final_nodes(k,1),final_nodes(k,2));
end

