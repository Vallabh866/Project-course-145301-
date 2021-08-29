clearvars
close all
clc

addpath(genpath('C:\Project-course-145301--main\Project-course-145301--main'));
addpath('C:\Project-course-145301--main\Project-course-145301--main\Racetracks\Racetracks\PistaAzzurra_Track');
run('C:\Project-course-145301--main\Project-course-145301--main\Racetracks\Racetracks\PistaAzzurra_Track\create_RoadFile.m');

close all

npts = 100;

%Create track and retrieve clothoid list of the track
Center_line = ClothoidList;
Center_line = create_RoadFile();

%Function to define "way point positions" based on given road center line
[waypoints, headings, curvatures] = define_waypoints(Center_line);
[waypoints_per_wayline,waylines_count] = size(waypoints);
fprintf('\nNumber of generated waylines for given road track are: %d \n with %d waypoints per wayline\n',waylines_count, waypoints_per_wayline);
for k = 1:1:waylines_count
  for m = 1:1:waypoints_per_wayline
    %plot(waypoints{m,k}{1,1}(1,1), waypoints{m,k}{1,1}(1,2), 'x');
  end
end

%Function to define "road edges" based on given road center line
edge_left = ClothoidList();
edge_right = ClothoidList();
[edge_left, edge_right]=define_road_edges(Center_line);

edge_left.plot;
edge_right.plot;

%Percentage of track considered as the goal
Goal_position_portion_track = 100;%Percentage
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
CL = ClothoidCurve();
CL_min_cost_parent = ClothoidCurve();
CL_parent = ClothoidCurve();
CL_rewiring = ClothoidCurve();
CL_rewiring_final = ClothoidCurve();
CL_final_result = ClothoidCurve();

%Initializing q_new to starting point in case newly explored node doesnt
%find a parent node nearby
q_new = q_start;
q_new.cost = 0;

%Percentage of track to perform the whole logic upon
test_portion_track = 150;%Percentage
test_portion_track = test_portion_track/100;

exploration_matrix = zeros(size(waypoints));
%The starting point is initialized as "explored"
exploration_matrix(2,1) = 1;
exploration_saturation_percentage = 90;
exploration_saturation_percentage = exploration_saturation_percentage/100;

cost_matrix = 10000*ones(size(waypoints));
%Cost to begin from the 1st wayline is 0
cost_matrix(1,1)=0;
cost_matrix(2,1)=0;
cost_matrix(3,1)=0;

v_ini_clothoid_matrix = v_ini_clothoid*ones(size(waypoints));
parent_matrix = waypoints;

%Initialize all contents of the parent matrix to 0
for k = 1:1:waylines_count
  for m = 1:1:waypoints_per_wayline
    %plot(waypoints{m,k}{1,1}(1,1), waypoints{m,k}{1,1}(1,2), 'x');
    parent_matrix{m,k}{1,1}(1,1) = 0;
    parent_matrix{m,k}{1,1}(1,2) = 0;
  end
end

%Matrices containing all the thetas and curvatures "at every wayline"
%theta_matrix = zeros(1, waylines_count);
%for i = 1:waylines_count
%    wayline_centre_xy = [waypoints{2,i}{1,1}(1,1), waypoints{2,i}{1,1}(1,2)];
%    [~, wayline_theta, wayline_curvature] = Calc_wayline_num(wayline_centre_xy,waypoints, headings, curvatures);
%    theta_matrix(1,i) = 
%end

curvature_matrix = zeros(1, waylines_count);

wayline_num_last_node = 1;
wayline_num_last_node_max = 1;
loop_iteration_counter = 1;

finish_line_closeness_check_1 = 0;
finish_line_closeness_check_2 = 0;

Horizon_limit_tmp = Horizon_limit;

goal_reached_counter = 0;

 %for i = 1:1:(waypoints_per_wayline*waylines_count)*(test_portion_track/waypoints_per_wayline)
 %while (wayline_num_last_node ~= (waylines_count))
 while (1)
    
    if (wayline_num_last_node == (waylines_count))
        goal_reached_counter = goal_reached_counter + 1;
    end
    
    
    %%% These are the 2 goal checking conditions 
    
    %%% Condition 1 : Check if the all the waypoints on the last wayline
    %%% are explored
    final_saturation_condition_counter = 0;
    for tmp = 1:waypoints_per_wayline
        if(exploration_matrix(tmp, waylines_count) == 1)
            final_saturation_condition_counter = final_saturation_condition_counter + 1;
        end
    end
    
    %%% Condition 2 : Check if a given percentage of the whole track is
    %%% explored
    if (final_saturation_condition_counter >= 1)
       if sum(exploration_matrix,'all') >= (exploration_saturation_percentage)*(sum((ones(size(waypoints))),'all'))
            break;
       end
    end
    
    %%% If both the above conditions are fulfilled then we can stop
    %%% exploring more and start plotting the final path of the vehicle
    %%%
    
     
    %[wayline_num_last_node, last_node_theta, last_node_curvature] = Calc_wayline_num(nodes(end).coord,waypoints, headings, curvatures);
    if(wayline_num_last_node>wayline_num_last_node_max)
        wayline_num_last_node_max = wayline_num_last_node;
    end
    
    if(wayline_num_last_node_max > (waylines_count - 2*Horizon_limit_tmp))
        Horizon_limit = Horizon_limit_tmp+1;
    elseif(wayline_num_last_node_max > (waylines_count - Horizon_limit_tmp + 1))
        Horizon_limit = Horizon_limit_tmp+2;
    %elseif(wayline_num_last_node_max > (waylines_count - 2*Horizon_limit_tmp + 2))
    %    Horizon_limit = 4*Horizon_limit_tmp;
    end
    
    rand_waypoint = min(max(floor(rand*10/3),1),3);
    %rand_wayline = min((i+floor(rand*Horizon_limit)),waylines_count);
    
    rand_number = floor(rand*2*Horizon_limit)+1;
    
    %rand_wayline = floor((rand*waylines_count)*test_portion_track);
    %if(wayline_num_last_node>Horizon_limit)
    %    rand_wayline = min(waylines_count, max(2,((wayline_num_last_node - Horizon_limit) + rand_number)));
    %else
    %    rand_wayline = (wayline_num_last_node) + rand_number;
    %end
    
    rand_wayline = max(2,((wayline_num_last_node - Horizon_limit) + rand_number));
    
    if (rand_wayline > waylines_count)
        rand_wayline = max(2,(rand_wayline - waylines_count));
    end
    
    exploration_saturation_counter = 0;
    while(1)
        %Check exploration saturation in case the next Horizon is explored
        exploration_saturation_counter = exploration_saturation_counter + 1;
        
        %if (rand_wayline > waylines_count)
        %    rand_wayline = max(2,(rand_wayline - waylines_count));
        %end
        
        if(rand_wayline>waylines_count)
            disp('Check');
        end
        
        if((rand_wayline>1) && (exploration_matrix(rand_waypoint,rand_wayline) ~= 1))
        break;
        else
            if(exploration_saturation_counter <= 2*Horizon_limit)
                rand_number = floor(rand*2*Horizon_limit)+1;
            else
                %Horizon_limit_2 = min(6,(Horizon_limit*2)); % Increase the Horizon limit if all the nodes around are explored
                Horizon_limit_2 = Horizon_limit*2; % Increase the Horizon limit if all the nodes around are explored
                rand_number = rand_number + floor(rand*2*Horizon_limit_2)+1;
                
                if(rand_number > waylines_count)
                    rand_number = rem(rand_number,waylines_count);
                end                
            end
            rand_waypoint = min(max(floor(rand*10/3),1),3);
            %rand_wayline = floor((rand*waylines_count)*test_portion_track);
            %if(exploration_saturation_counter < 2*Horizon_limit)
                %rand_wayline = min(waylines_count, max(2,((wayline_num_last_node - Horizon_limit) + rand_number)));
            %else
            %    rand_wayline = (wayline_num_last_node) + rand_number;
            %end
            
            rand_wayline = max(2,((wayline_num_last_node - Horizon_limit) + rand_number));
            
            if (rand_wayline > waylines_count)
                %rand_wayline = rand_wayline - waylines_count;
                rand_wayline = max(2,(rand_wayline - waylines_count));
            end
        end    
    end
    
    if(rand_wayline > waylines_count)
        rand_wayline = max(2,(rand_wayline - waylines_count));
        loop_iteration_counter = loop_iteration_counter + 1;
    end
    
    if((wayline_num_last_node_max > (waylines_count - 2*Horizon_limit_tmp)) && finish_line_closeness_check_1 == 0)
        rand_wayline = waylines_count - 2*Horizon_limit_tmp + 2;
        finish_line_closeness_check_1 = 1;
    elseif((wayline_num_last_node_max > (waylines_count - Horizon_limit_tmp)) && finish_line_closeness_check_2 == 0)
        rand_wayline = waylines_count - Horizon_limit_tmp + 2;
        finish_line_closeness_check_2 = 1;
    %elseif(wayline_num_last_node_max > (waylines_count - 2*Horizon_limit + 2))
    %    Horizon_limit = 4*Horizon_limit_tmp;
    end
    
    q_rand = [waypoints{rand_waypoint,rand_wayline}{1,1}(1,1), waypoints{rand_waypoint,rand_wayline}{1,1}(1,2)];
    
    plot(q_rand(1), q_rand(2), 'x', 'Color',  [0 0.4470 0.7410]);
    
    %exploration_matrix(rand_waypoint,rand_wayline)=1;
    
    % Break if goal node is already reached
    %for j = 1:1:length(nodes)
    %    if nodes(j).coord == q_goal.coord
            %fprintf("Goal reached");
    %        break
    %    end
    %end
    
    % Pick the closest node from existing list to branch out from
    costs = [];
    [~ , qrand_theta, qrand_curvature] = Calc_wayline_num(q_rand,waypoints, headings, curvatures);
    
    %q_rand_xy = [q_rand(1), q_rand(2)];
    %[wayline_num_q_rand, q_rand_theta, q_rand_kappa] = Calc_wayline_num(q_rand_xy,waypoints, headings, curvatures);

    tmp = 10000;
    max_cost = tmp;
    min_cost = tmp;
    %for j = 1:1:length(nodes)
    for parent_node_wayline =  max(1,(rand_wayline-Horizon_limit)):1:max(1,(rand_wayline-1))
        
         %if ((i == 1) && (parent_node_wayline ~= 1))
         %     continue;
         %end
        
        for parent_node_waypoint = 1:1:waypoints_per_wayline
            
            %if ((parent_node_wayline == 1) && (parent_node_waypoint ~= 2))
            %  continue;
            %end
            
            if(exploration_matrix(parent_node_waypoint,parent_node_wayline) == 1) % Dont re-explore previously explored nodes
                
                intersection_detected = 1;

                parent_node_xy = [waypoints{parent_node_waypoint, parent_node_wayline}{1,1}(1,1), waypoints{parent_node_waypoint, parent_node_wayline}{1,1}(1,2)];
                [wayline_num_parent_node, parent_node_theta, parent_node_curvature] = Calc_wayline_num(parent_node_xy,waypoints, headings, curvatures);

                %if((wayline_num_node < rand_wayline)&& (rand_wayline<=wayline_num_node+Horizon_limit))
                    CL_parent.build_G1(parent_node_xy(1), parent_node_xy(2), parent_node_theta, q_rand(1), q_rand(2), qrand_theta);
                    CL_s = CL_parent.length;

                    %nodes_j_xy = [nodes(j).coord(1), nodes(j).coord(2)];
                    %[wayline_num_nodes_j, nodes_j_theta, nodes_j_kappa] = Calc_wayline_num(nodes_j_xy,waypoints, headings, curvatures);

                    %q_rand_xy = [q_rand(1), q_rand(2)];
                    %[wayline_num_q_rand, q_rand_theta, q_rand_kappa] = Calc_wayline_num(q_rand_xy,waypoints, headings, curvatures);

                    [cost, v_ini_clothoid] = cost_FWBW(CL_s, parent_node_curvature,qrand_curvature,v_ini_clothoid_matrix(parent_node_waypoint, parent_node_wayline));

                    % Check the intersection
                    s_inters_left = CL_parent.intersect(edge_left);
                    s_inters_right = CL_parent.intersect(edge_right);
                    if (isempty(s_inters_left) && isempty(s_inters_right))
                        % this means that no intersection exists
                        intersection_detected = 0;
                        
                        if(cost < min_cost) % Find minimum cost parent
                            min_cost = cost;
                            %Update the minimum cost in the cost matrix
                            if (parent_node_wayline == 1)
                                cost_matrix(rand_waypoint,rand_wayline) = min_cost;% + 0 //because cost of being on the 1st wayline is 0
                            else
                                cost_matrix(rand_waypoint,rand_wayline) = min_cost + cost_matrix(parent_node_waypoint,parent_node_wayline);
                            end
                            
                            %Update the parent
                            %parent_matrix(q_rand(1), q_rand(2)) = 
                            parent_matrix{rand_waypoint,rand_wayline}{1,1}(1,1) = parent_node_waypoint;
                            parent_matrix{rand_waypoint,rand_wayline}{1,1}(1,2) = parent_node_wayline;
                            
                            %Store the position of the parent
                            min_cost_parent_node_waypoint = parent_node_waypoint;
                            min_cost_parent_node_wayline = parent_node_wayline;
                            
                            %Store the spline from the parent to newly
                            %explored node, to be used to plot later
                            CL_min_cost_parent = CL_parent;
                            
                            %Store the spline length of the min cost parent
                            CL_s_min_cost_parent = CL_s;
                            
                            %Store the v_ini_clothoid at the newly explored
                            %node
                            v_ini_clothoid_matrix(rand_waypoint,rand_wayline) = v_ini_clothoid;
                            
                            %Update exploration matrix with the explored node position
                            exploration_matrix(rand_waypoint,rand_wayline)=1;
                            
                            %Store the wayline of the latest explored node as the last explored wayline
                            wayline_num_last_node =  rand_wayline;
                        end

                    end
        %     
        %            q_rand.v_ini_clothoid = v_ini_clothoid;
        %         
                    %q_new.cost = cost;
                    %tmp=cost;
                    %tmp = CL_parent.length;
                %end
                %n = nodes(j);
                %tmp = dist(n.coord, q_rand);
                %if intersection_detected == 0
                %    costs = [costs tmp];
                %    cost_matrix(parent_node_waypoint, parent_node_wayline) = tmp;
                %end
            end
        end
    end
    
    %if(isempty(costs))
        %disp('Costs array empty');
    %end
    
    %[min_cost, idx] = min(costs);
    
    %if ((min_cost == max_cost) || (isempty(costs)))
        %q_near = nodes(last);
        %continue;
    %else
        %q_near = nodes(last);
        %q_near = nodes(idx);
    %end
    
    %q_new.coord(1) = q_rand(1);
    %q_new.coord(2) = q_rand(2);

    %q_near_xy = [q_near.coord(1), q_near.coord(2)];
    %q_near_xy = [min_cost_parent_node_waypoint, min_cost_parent_node_wayline];
    %[wayline_num_q_near, q_near_theta, q_near_kappa] = Calc_wayline_num(q_near_xy,waypoints, headings, curvatures);
    
    %q_new_xy = [q_new.coord(1), q_new.coord(2)];
    %q_new_xy = [q_rand(1), q_rand(2)];
    %[wayline_num_q_new, q_new_theta, q_new_kappa] = Calc_wayline_num(q_new_xy,waypoints, headings, curvatures);
    
    %CL.build_G1(q_near_xy(1), q_near_xy(2), q_near_theta, q_new_xy(1), q_new_xy(2), q_new_theta);
    CL_min_cost_parent.plot(npts,'Color','red');
    
    %Store the wayline of the latest explored node as the last explored wayline
    % wayline_num_last_node =  rand_wayline;
   

    %CL_s = CL.length;
     
    %[cost, v_ini_clothoid] = cost_FWBW(CL_s_min_cost_parent, q_near_kappa,qrand_curvature,v_ini_clothoid);     
     
    %q_new.v_ini_clothoid = v_ini_clothoid;         
    %q_new.cost = q_near.cost + cost;
     
%   Rewiring begins

     q_nearest = [];

     neighbor_count = 1;
     rewiring_check = 0;
     cost_rewiring = 0;
     %for j = 1:1:length(nodes) 
     for rewiring_node_wayline =  (rand_wayline+1):1:min(waylines_count,(rand_wayline+Horizon_limit))
        for rewiring_node_waypoint = 1:1:waypoints_per_wayline
            if(exploration_matrix(rewiring_node_waypoint,rewiring_node_wayline) == 1) % Consider only explored nodes from the  waylines ahead for rewiring
                
                rewiring_node_xy = [waypoints{rewiring_node_waypoint,rewiring_node_wayline}{1,1}(1,1), waypoints{rewiring_node_waypoint,rewiring_node_wayline}{1,1}(1,2)];
                [wayline_num_rewiring_node, rewiring_node_theta, rewiring_node_curvature] = Calc_wayline_num(rewiring_node_xy,waypoints, headings, curvatures);
                
                CL_rewiring.build_G1(q_rand(1), q_rand(2), qrand_theta, rewiring_node_xy(1), rewiring_node_xy(2), rewiring_node_theta);
                CL_s_rewiring = CL_rewiring.length;
                
                [cost_rewiring, v_ini_clothoid_rewiring] = cost_FWBW(CL_s_rewiring, qrand_curvature,rewiring_node_curvature,v_ini_clothoid_matrix(rand_waypoint,rand_wayline));
                
                cost_rewiring = cost_rewiring + cost_matrix(rand_waypoint,rand_wayline);
                
                % Check the intersection
                s_rewiring_inters_left = CL_rewiring.intersect(edge_left);
                s_rewiring_inters_right = CL_rewiring.intersect(edge_right);
                if (isempty(s_rewiring_inters_left) && isempty(s_rewiring_inters_right))
                    if (cost_rewiring <= cost_matrix(rewiring_node_waypoint, rewiring_node_wayline))
                        rewiring_check = rewiring_check + 1;

                        %Update the rewired node with the new parent
                        parent_matrix{rewiring_node_waypoint, rewiring_node_wayline}{1,1}(1,1) = rand_waypoint;
                        parent_matrix{rewiring_node_waypoint, rewiring_node_wayline}{1,1}(1,2) = rand_wayline;

                        %Update the cost of the rewired node
                        cost_matrix(rewiring_node_waypoint, rewiring_node_wayline) = cost_rewiring;

                        %Update the v_ini of the rewired node
                        v_ini_clothoid_matrix(rewiring_node_waypoint, rewiring_node_wayline) = v_ini_clothoid_rewiring;

                        %Store the rewired node position and heading
                        rewired_node_waypoint = rewiring_node_waypoint;
                        rewired_node_wayline = rewiring_node_wayline;
                        rewired_node_theta = rewiring_node_theta;
                    end
                end
                %node_xy = [nodes(j).coord(1), nodes(j).coord(2)];
                %[wayline_num_node, node_theta, node_curvature] = Calc_wayline_num(node_xy,waypoints, headings, curvatures);
                %if(wayline_num_node > wayline_num_q_new)&&(wayline_num_node<=wayline_num_q_new+Horizon_limit)
                %     q_nearest(neighbor_count).coord = nodes(j).coord;
                %     q_nearest(neighbor_count).cost = nodes(j).cost;
                %     q_nearest(neighbor_count).v_ini_clothoid = nodes(j).v_ini_clothoid;
                %    neighbor_count = neighbor_count+1;
                %end
            end
        end
     end
    

%    q_min = q_near; 
%    C_min = [];
%    cost_rewiring = 0;
%    rewire_idx = 0;
%    if(length(q_nearest)>=1)
%      for k = 1:1:length(q_nearest)
%         q_nearest_xy = [q_nearest(k).coord(1), q_nearest(k).coord(2)];
%         [wayline_num_q_nearest, q_nearest_theta, q_nearest_kappa] = Calc_wayline_num(q_nearest_xy,waypoints, headings, curvatures);
%         
%         CL_rewiring.build_G1(q_new.coord(1), q_new.coord(2), q_new_theta, q_nearest(k).coord(1), q_nearest(k).coord(2), q_nearest_theta);
%         CL_s_rewiring = CL_rewiring.length;
%               
%         %q_new_xy = [q_new.coord(1), q_new.coord(2)];
%         %[wayline_num_q_new, q_new_theta, q_new_kappa] = Calc_wayline_num(q_new_xy,waypoints, headings, curvatures);
%               
%         [cost_rewiring, v_ini] = cost_FWBW(CL_s_rewiring, q_new_kappa,q_nearest_kappa,q_nearest(k).v_ini_clothoid);
%         cost_rewiring = cost_rewiring + q_new.cost;
% 
%         if cost_rewiring < q_nearest(k).cost
%             q_nearest(k).parent = q_new;
%             q_nearest(k).cost = cost_rewiring;
%             rewire_idx = k;
%         end
%      end
%    end
   
   %if rewire_idx ~= 0
   
   if rewiring_check ~= 0
       rewired_node_xy = [waypoints{rewired_node_waypoint, rewired_node_wayline}{1,1}(1,1), waypoints{rewired_node_waypoint, rewired_node_wayline}{1,1}(1,2)];
       %q_nearest_xy = [q_nearest(rewire_idx).coord(1), q_nearest(rewire_idx).coord(2)];
       %[wayline_num_q_nearest, q_nearest_theta, q_nearest_kappa] = Calc_wayline_num(q_nearest_xy,waypoints, headings, curvatures);
       CL_rewiring_final.build_G1(q_rand(1), q_rand(2), qrand_theta, rewired_node_xy(1), rewired_node_xy(2), rewired_node_theta);
       CL_rewiring_final.plot(npts,'Color','blue');
       
       %Store the wayline of the latest explored node as the last explored wayline
        wayline_num_last_node =  rewired_node_wayline;
   end
   
   
   
   %for k = 1:1:waylines_count
   % for l = 1:1:waypoints_per_wayline
   %     if exploration_matrix(l,k)
   %     wayline_num_last_node
       
   % end
   %end
   
   
         
%     % Append to nodes
     %nodes = [nodes q_new];
 end
 
 
figure('Name','Optimum path','NumberTitle','off'), clf
hold on
axis equal

edge_left.plot;
edge_right.plot;

%Move backwards from Goal to start based on the parent matrix
goal_coord = [waypoints{2,ceil(Goal_position_portion_track*waylines_count)}{1,1}(1,1), waypoints{2,ceil(Goal_position_portion_track*waylines_count)}{1,1}(1,2)];
%[wayline_num_goal, goal_theta, goal_curvature] = Calc_wayline_num(goal_coord,waypoints, headings, curvatures);
child_temp = goal_coord;
child_wayline = waylines_count;

parent_wayline = parent_matrix{2, child_wayline}{1,1}(1,2);
parent_temp = [waypoints{2, parent_wayline}{1,1}(1,1), waypoints{2, parent_wayline}{1,1}(1,2)];

while(1)
    CL_final_result.build_G1(parent_temp(1), parent_temp(2), headings(1, parent_wayline),child_temp(1), child_temp(2), headings(1, child_wayline));
    CL_final_result.plot;
    
    child_temp = parent_temp;
    child_wayline = parent_wayline;
     
    parent_wayline = parent_matrix{2, child_wayline}{1,1}(1,2); 
    parent_temp = [waypoints{2, parent_wayline}{1,1}(1,1), waypoints{2, parent_wayline}{1,1}(1,2)];
    
    if(parent_wayline == 1)
        CL_final_result.build_G1(parent_temp(1), parent_temp(2), headings(1, parent_wayline),child_temp(1), child_temp(2), headings(1, child_wayline));
        CL_final_result.plot;
        break;
    end
end

grid on
xlabel('x [m]')
ylabel('y [m]')
title('Circuit-Final')
 
 
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
disp('End');