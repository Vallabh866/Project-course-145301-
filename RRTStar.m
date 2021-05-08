clearvars
close all
clc

% Create track and retrieve clothoid list of the track
Center_line = ClothoidList;
Center_line = create_RoadFile();

% CL2 = ClothoidCurve();
% CL2.build( -25, 100, 38, 0.000001, -0.00004, 420 );
% CL2.plot();
% Center_line = CL2;

% return the parameters defining the clothoid curve
%[x_Center_line,y_Center_line,theta_Center_line,k_Center_line,dk_Center_line,L_Center_line] = Center_line.getPars();

%Function to define way point positions based on given road edges
[waypoints, headings, curvatures] = define_waypoints(Center_line);
[waypoints_per_wayline,waylines_count] = size(waypoints);
fprintf('%d %d\n',waypoints_per_wayline ,waylines_count);
for k = 1:waylines_count
  for m = 1:waypoints_per_wayline
    plot(waypoints{m,k}{1}(1), waypoints{m,k}{1}(2), 'x');
  end
end

EPS = 10;
q_start.coord = [waypoints{2,1}{1}(1), waypoints{2,1}{1}(2)];
q_start.cost = 0;
q_start.parent = 0;
%q_goal.coord = [120 75];
q_goal.coord = [waypoints{2,ceil(0.3*waylines_count)}{1}(1), waypoints{2,ceil(0.3*waylines_count)}{1}(2)];
q_goal.cost = 0;

nodes(1) = q_start;
CL = ClothoidCurve(q_start.coord(1), q_start.coord(2), Center_line.theta(0), Center_line.kappa(0), 0, 1 );
npts = 100;
v_ini_clothoid = 2.3614;  % [m/s]

%Road edges
% CL2 = ClothoidCurve();
% CL2.build( -25, 100, 38, 0.000001, -0.00004, 420 );
% CL2.plot();
% CL3 = ClothoidCurve();
% CL3.build( -25, 70, 38, 0.000001, -0.00007, 300 );
% CL3.plot();
%

% return the parameters defining the clothoid curve
% [x_CL2,y_CL2,theta_CL2,k_CL2,dk_CL2,L_CL2] = CL2.getPars();
% [x_CL3,y_CL3,theta_CL3,k_CL3,dk_CL3,L_CL3] = CL3.getPars();

for i = 1:1:(waypoints_per_wayline*waylines_count)
%for i = 1:1:500
    rand_waypoint = min(max(floor(rand*10/3),1),3);
    rand_wayline = min(max(floor(rand*2000),1),waylines_count);
    
    q_rand = [waypoints{rand_waypoint,rand_wayline}{1}(1), waypoints{rand_waypoint,rand_wayline}{1}(2)];
    
    %q_rand = xy(((round(rand*10)/10)*10) + 1, :);
    plot(q_rand(1), q_rand(2), 'x', 'Color',  [0 0.4470 0.7410]);
    
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
    
    if val >= EPS
       q_new = q_near;
    else
       q_new.coord(1) = q_rand(1);
       q_new.coord(2) = q_rand(2);
    end
    
    line([q_near.coord(1), q_new.coord(1)], [q_near.coord(2), q_new.coord(2)], 'Color', 'k', 'LineWidth', 2);
    drawnow
    hold on
    
    for k = 1:waylines_count
        for m = 1:waypoints_per_wayline
            %plot(waypoints{m,k}{1}(1), waypoints{m,k}{1}(2), 'x');
            if ((q_near.coord(1) == waypoints{m,k}{1}(1)) && (q_near.coord(2) == waypoints{m,k}{1}(2)))
                q_near_theta = headings(1,k);
                q_near_kappa = curvatures(1,k);
            end
            if ((q_new.coord(1) == waypoints{m,k}{1}(1)) && (q_new.coord(2) == waypoints{m,k}{1}(2)))
                q_new_theta = headings(1,k);
                q_new_kappa = curvatures(1,k);
            end
        end
    end
    
    if(q_new.coord(1)>q_near.coord(1))
        CL.build_G1(q_near.coord(1), q_near.coord(2), q_near_theta, q_new.coord(1), q_new.coord(2), q_new_theta);
        CL.plot(npts,'Color','red');
    end
    
    CL_s = CL.length;
    CL_k = q_new_kappa;
    
    [cost, v_ini_clothoid] = cost_FWBW(CL_s, CL_k,v_ini_clothoid);
        
    q_new.cost = cost;
    
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
    %instantaneous_theta = atan()
    
    q_end = nodes(start);
end