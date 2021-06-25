clearvars
close all
clc

addpath(genpath('E:\Project-course-145301-'));
addpath('E:\Project-course-145301-\Project-course-145301-\Racetracks\Racetracks\PistaAzzurra_Track');
run('E:\Project-course-145301-\Project-course-145301-\Racetracks\Racetracks\PistaAzzurra_Track\create_RoadFile.m');

close all

%Create track and retrieve clothoid list of the track
Center_line = ClothoidList;
Center_line = create_RoadFile();

%Function to define way point positions based on given road edges
[waypoints, headings, curvatures] = define_waypoints(Center_line);
[waypoints_per_wayline,waylines_count] = size(waypoints);
fprintf('%d %d\n',waypoints_per_wayline ,waylines_count);
for k = 1:1:waylines_count
  for m = 1:1:waypoints_per_wayline
    %plot(waypoints{m,k}{1,1}(1,1), waypoints{m,k}{1,1}(1,2), 'x');
  end
end

%Percentage of track considered as the goal
Goal_position_portion_track = 70;%Percentage
Goal_position_portion_track = Goal_position_portion_track/100;

EPS = 3000;
Horizon_limit = 3;
q_start.coord = [waypoints{2,1}{1,1}(1,1), waypoints{2,1}{1,1}(1,2)];
q_start.cost = 0;
q_start.parent = 0;
q_goal.coord = [waypoints{2,ceil(Goal_position_portion_track*waylines_count)}{1,1}(1,1), waypoints{2,ceil(Goal_position_portion_track*waylines_count)}{1,1}(1,2)];
q_goal.cost = 0;
v_ini_clothoid = 2.3614;  % [m/s]
q_start.v_ini_clothoid = v_ini_clothoid;  % [m/s]

nodes(1) = q_start;
CL = ClothoidCurve(q_start.coord(1), q_start.coord(2), Center_line.theta(0), Center_line.kappa(0), 0, 1 );
CL_parent = ClothoidCurve();
CL_rewiring = ClothoidCurve();
npts = 100;

%Initializing q_new to starting point in case newly explored node doesnt
%find a parent node nearby
q_new = q_start;
q_new.cost = 0;

%Percentage of track to perform the whole logic upon
test_portion_track = 50;%Percentage
test_portion_track = test_portion_track/100;

 for i = 1:1:(waypoints_per_wayline*waylines_count)*test_portion_track
     
    [wayline_num_last_node, last_node_theta, last_node_curvature] = Calc_wayline_num(nodes(end).coord,waypoints, headings, curvatures);
     
    
    rand_waypoint = min(max(floor(rand*10/3),1),3);
    %rand_wayline = min((i+floor(rand*Horizon_limit)),waylines_count);
    
    rand_number = floor(rand*2*Horizon_limit)+1;
    
    %rand_wayline = floor((rand*waylines_count)*test_portion_track);
    if(wayline_num_last_node>Horizon_limit)
        rand_wayline = (wayline_num_last_node - Horizon_limit) + rand_number;
    else
        rand_wayline = (wayline_num_last_node) + rand_number;
    end
    
    while(1)
        if(rand_wayline>1)
        break;
        else
            rand_waypoint = min(max(floor(rand*10/3),1),3);
            rand_wayline = floor((rand*waylines_count)*test_portion_track);
        end    
    end    
    
    q_rand = [waypoints{rand_waypoint,rand_wayline}{1,1}(1,1), waypoints{rand_waypoint,rand_wayline}{1,1}(1,2)];
    
    plot(q_rand(1), q_rand(2), 'x', 'Color',  [0 0.4470 0.7410]);
    
    % Break if goal node is already reached
    for j = 1:1:length(nodes)
        if nodes(j).coord == q_goal.coord
            %fprintf("Goal reached");
            break
        end
    end
    
    % Pick the closest node from existing list to branch out from
    costs = [];
    [wayline_num_rand, qrand_theta, qrand_curvature] = Calc_wayline_num(q_rand,waypoints, headings, curvatures);

    tmp = 10000;
    for j = 1:1:length(nodes)
        
        node_xy = [nodes(j).coord(1), nodes(j).coord(2)];
        [wayline_num_node, node_theta, node_curvature] = Calc_wayline_num(node_xy,waypoints, headings, curvatures);
        
        if((wayline_num_node < wayline_num_rand)&&(wayline_num_rand<=wayline_num_node+Horizon_limit))
            CL_parent.build_G1(nodes(j).coord(1), nodes(j).coord(2), node_theta, q_rand(1), q_rand(2), qrand_theta);
            CL_s = CL_parent.length;
            
            nodes_j_xy = [nodes(j).coord(1), nodes(j).coord(2)];
            [wayline_num_nodes_j, nodes_j_theta, nodes_j_kappa] = Calc_wayline_num(nodes_j_xy,waypoints, headings, curvatures);
            
            q_rand_xy = [q_rand(1), q_rand(2)];
            [wayline_num_q_rand, q_rand_theta, q_rand_kappa] = Calc_wayline_num(q_rand_xy,waypoints, headings, curvatures);
     
            [cost, v_ini_clothoid] = cost_FWBW(CL_s, nodes_j_kappa,q_rand_kappa,v_ini_clothoid);
%     
%            q_rand.v_ini_clothoid = v_ini_clothoid;
%         
            %q_new.cost = cost;
            tmp=cost;
            %tmp = CL_parent.length;
        end
        %n = nodes(j);
        %tmp = dist(n.coord, q_rand);
        costs = [costs tmp];
    end
    [min_cost, idx] = min(costs);
    q_near = nodes(idx);
    
    q_new.coord(1) = q_rand(1);
    q_new.coord(2) = q_rand(2);

    q_near_xy = [q_near.coord(1), q_near.coord(2)];
    [wayline_num_q_near, q_near_theta, q_near_kappa] = Calc_wayline_num(q_near_xy,waypoints, headings, curvatures);
    
    q_new_xy = [q_new.coord(1), q_new.coord(2)];
    [wayline_num_q_new, q_new_theta, q_new_kappa] = Calc_wayline_num(q_new_xy,waypoints, headings, curvatures);
    
    CL.build_G1(q_near.coord(1), q_near.coord(2), q_near_theta, q_new.coord(1), q_new.coord(2), q_new_theta);
    CL.plot(npts,'Color','red');

    CL_s = CL.length;
     
    [cost, v_ini_clothoid] = cost_FWBW(CL_s, q_near_kappa,q_new_kappa,v_ini_clothoid);     
     
    q_new.v_ini_clothoid = v_ini_clothoid;         
    q_new.cost = q_near.cost + cost;
     
%   Rewiring begins

     q_nearest = [];

     neighbor_count = 1;
     for j = 1:1:length(nodes)
        node_xy = [nodes(j).coord(1), nodes(j).coord(2)];
        [wayline_num_node, node_theta, node_curvature] = Calc_wayline_num(node_xy,waypoints, headings, curvatures);
        if(wayline_num_node > wayline_num_q_new)&&(wayline_num_node<=wayline_num_q_new+Horizon_limit)
             q_nearest(neighbor_count).coord = nodes(j).coord;
             q_nearest(neighbor_count).cost = nodes(j).cost;
             q_nearest(neighbor_count).v_ini_clothoid = nodes(j).v_ini_clothoid;
             neighbor_count = neighbor_count+1;
        end
     end

   q_min = q_near; 
   C_min = [];
   cost_rewiring = 0;
   if(length(q_nearest)>=1)
     for k = 1:1:length(q_nearest)
        q_nearest_xy = [q_nearest(k).coord(1), q_nearest(k).coord(2)];
        [wayline_num_q_nearest, q_nearest_theta, q_nearest_kappa] = Calc_wayline_num(q_nearest_xy,waypoints, headings, curvatures);
        
        CL_rewiring.build_G1(q_new.coord(1), q_new.coord(2), q_new_theta, q_nearest(k).coord(1), q_nearest(k).coord(2), q_nearest_theta);

        CL_s_rewiring = CL_rewiring.length;
              
        %q_new_xy = [q_new.coord(1), q_new.coord(2)];
        %[wayline_num_q_new, q_new_theta, q_new_kappa] = Calc_wayline_num(q_new_xy,waypoints, headings, curvatures);
              
        [cost_rewiring, v_ini] = cost_FWBW(CL_s_rewiring, q_new_kappa,q_nearest_kappa,q_nearest(k).v_ini_clothoid);
        cost_rewiring = cost_rewiring + q_new.cost;

        if cost_rewiring < q_nearest(k).cost
            q_nearest(k).parent = q_new;
            q_nearest(k).cost = cost_rewiring;
        end
     end
   end
         
%     % Append to nodes
     nodes = [nodes q_new];
 end
% 
% D = [];
% for j = 1:1:length(nodes)
%     tmpdist = dist(nodes(j).coord, q_goal.coord);
%     %fprintf("Nodes: %d \n", nodes(j).coord);
%     D = [D tmpdist];
% end
% 
% 
% % Search backwards from goal to start to find the optimal least cost path
% [val, idx] = min(D);    
% q_final = nodes(idx);
% q_goal.parent = idx;
% q_end = q_goal;
% nodes = [nodes q_goal];
% final_nodes = [];
% while q_end.parent ~= 0
%     start = q_end.parent;
%     line([q_end.coord(1), nodes(start).coord(1)], [q_end.coord(2), nodes(start).coord(2)], 'Color', 'r', 'LineWidth', 2);
%     hold on
%     %fprintf("Line nodes: %d %d\n",nodes(start).coord(1), nodes(start).coord(2) );
%     final_nodes = [final_nodes ; nodes(start).coord(1), nodes(start).coord(2)];
%     final_nodes = unique(final_nodes, 'rows', 'stable');
%     %instantaneous_theta = atan()
%     
%     q_end = nodes(start);
% end