clearvars
close all
clc

addpath(genpath('C:\Project-course-145301--main\Project-course-145301--main'));
addpath('C:\Project-course-145301--main\Project-course-145301--main\Racetracks\Racetracks\PistaAzzurra_Track');
run('C:\Project-course-145301--main\Project-course-145301--main\Racetracks\Racetracks\PistaAzzurra_Track\create_RoadFile.m');

close all

fprintf("\n-----------------------------\n");
Selection = '\nPlease select one option:\n1. Full track exploration\n2. Single corner exploration\n';
Exploration_option = input(Selection);
%fprintf("\n1. Full track exploration\n");
%fprintf("\n2. Single corner exploration\n");
fprintf("\n-----------------------------\n");


% Arrays for storing computation time values
loop_iteration_time_check_end_array = [];
initial_conditions_time_end_array = [];
rand_wayline_generation_time_end_array = [];
intermediate_logic_1_time_end_array = [];
parent_spline_generation_time_end_array = [];
rewiring_spline_generatioin_time_end_array = [];
total_time_vect=[];

% Array to store the velocity profile of the final path
total_velocity_profile=[];

%Arrays to store the time vect and velocity profile of individual splines
time_vect = [];
speed_profile = [];

%Array to store the length of the splines contained in the final path
CL_final_path_length=[];

%Array to store the curvature of the splines contained in the final path
CL_final_path_curv=[];

t_track_creation_start = tic;

%Create track and retrieve clothoid list of the track
Center_line = ClothoidList;
Center_line = create_RoadFile();

if(Exploration_option==2)
    %Center_line = Center_line*0.1;
    % This condition is specifically for Pista azzura, Change the min and
    % max value of trim based on the track and the location on the track you wish to
    % explore
    s_min = 0;
    s_max = 200;    
    
    %Trim the track to limit the exploration
    Center_line.trim( s_min, s_max );
end

%Function to define "way point positions" based on given road center line
[waypoints, headings, curvatures] = define_waypoints(Center_line);
[waypoints_per_wayline,waylines_count] = size(waypoints);

fprintf('\nNumber of generated waylines for given road track are: %d \n with: %d waypoints per wayline\n',waylines_count, waypoints_per_wayline);

%%% If one wants to plot all the generated waypoints by the
%%% "define_waypoints" function
for k = 1:1:waylines_count
  for m = 1:1:waypoints_per_wayline
    plot(waypoints{m,k}{1,1}(1,1), waypoints{m,k}{1,1}(1,2), 'x');
  end
end

%Function to define "road edges" based on given road center line
edge_left = ClothoidList();
edge_right = ClothoidList();
[edge_left, edge_right]=define_road_edges(Center_line);

%Plot the left and right edges of the road
edge_left.plot;
edge_right.plot;

Horizon_limit = 3; %[number of waylines] Horizon value upto which the RRT algorithm can explore at 1 time step
v_ini_clothoid = 2.3614;  %[m/s] Initial speed value for the FWBW velocity planner

npts = 100;

% Exploration splines
CL_min_cost_parent = ClothoidCurve();
CL_parent = ClothoidCurve();

%Rewiring splines
CL_rewiring = ClothoidCurve();
CL_rewiring_final = ClothoidCurve();

% Final path splines
CL_final_result = ClothoidCurve();

% An "exploration matrix" indicating status of explored and unexplored
% nodes
exploration_matrix = zeros(size(waypoints));

%The starting point is initialized as "explored"
exploration_matrix(2,1) = 1;

% Percentage of the waypoints to be explored before finalizing the
% optimal path
exploration_saturation_percentage = 94;
exploration_saturation_percentage = exploration_saturation_percentage/100;

% A "cost matrix" indicating the minimum cost required to reach a waypoint
% from the start of the track
cost_matrix = 10000*ones(size(waypoints)); % inf instead of 10000

% Cost to begin from the 1st wayline is 0
cost_matrix(1,1)=0;
cost_matrix(2,1)=0;
cost_matrix(3,1)=0;

% A "v_ini_clothoid" matrix to indicate the velocity at each point by the
% velocity planner
v_ini_clothoid_matrix = v_ini_clothoid*ones(size(waypoints));
parent_matrix = waypoints;

%Initialize all contents of the parent matrix to 0
for k = 1:1:waylines_count
  for m = 1:1:waypoints_per_wayline
    parent_matrix{m,k}{1,1}(1,1) = 0;
    parent_matrix{m,k}{1,1}(1,2) = 0;
  end
end

% A "curvature" matrix indicating the curvature at each wayline
curvature_matrix = zeros(1, waylines_count);

% Initializing the "wayline number explored at last iteration"
wayline_num_last_node = 1;

% Initializing the "farthest wayline" that the RRT exploration has reached
wayline_num_last_node_max = 1;

% Binary variables used to indicate need for expanding the horizon if the explorer is stuck near
% the end of the track but is not close enough to the goal
finish_line_closeness_check_1 = 0;
finish_line_closeness_check_2 = 0;

Horizon_limit_tmp = Horizon_limit;

t_track_creation_end = toc(t_track_creation_start);
fprintf("\nIt took %d seconds for the Track to be created along with the waylines\n", t_track_creation_end);


t_RRT_star_exploration_start = tic;

 while (1)    
    
    loop_iteration_time_check = tic;
    
    initial_conditions_time_start = tic;
     
    %%% Check for the 2 goal reached conditions 
    
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
                break; % Stop the RRT* exploration
           end
        end
        
    %%% If both the above conditions are fulfilled then we can stop
    %%% exploring more and start plotting the final path of the vehicle
    %%%    
     
    % Storing the farthest wayline that the algorithm has reached
    if(wayline_num_last_node>wayline_num_last_node_max)
        wayline_num_last_node_max = wayline_num_last_node;
    end
    
    rand_waypoint = min(max(floor(rand*10/3),1),3);
    
    rand_number = floor(rand*2*Horizon_limit)+1;
    
    rand_wayline = max(2,((wayline_num_last_node - Horizon_limit) + rand_number));
    
    if (rand_wayline > waylines_count)
        rand_wayline = max(2,(rand_wayline - waylines_count));
    end
    
    exploration_saturation_counter = 0;
    
    initial_conditions_time_end = toc(initial_conditions_time_start);
    
    rand_wayline_generation_time_start = tic;
    
    while(1)
        %Check exploration saturation in case the next Horizon is explored
        exploration_saturation_counter = exploration_saturation_counter + 1;
        
        if((rand_wayline>1) && (exploration_matrix(rand_waypoint,rand_wayline) ~= 1))
        break;
        else
            if(exploration_saturation_counter <= 2*Horizon_limit)
                rand_number = floor(rand*2*Horizon_limit)+1;
            else
                Horizon_limit_2 = Horizon_limit*2; % Increase the Horizon limit if all the nodes around are explored
                rand_number = rand_number + floor(rand*2*Horizon_limit_2)+1;
                
                if(rand_number > waylines_count)
                    rand_number = rem(rand_number,waylines_count);
                end                
            end
            rand_waypoint = min(max(floor(rand*10/3),1),3);
            
            rand_wayline = max(2,((wayline_num_last_node - Horizon_limit) + rand_number));
            
            if (rand_wayline > waylines_count)
                rand_wayline = max(2,(rand_wayline - waylines_count));
            end
        end    
    end
    
    rand_wayline_generation_time_end = toc(rand_wayline_generation_time_start);
    
    intermediate_logic_1_time_start = tic;
    
    if(rand_wayline > waylines_count)
        rand_wayline = max(2,(rand_wayline - waylines_count));
    end
    
    if((wayline_num_last_node_max > (waylines_count - 2*Horizon_limit_tmp)) && finish_line_closeness_check_1 == 0)
        rand_wayline = waylines_count - 2*Horizon_limit_tmp + 2;
        finish_line_closeness_check_1 = 1;
    elseif((wayline_num_last_node_max > (waylines_count - Horizon_limit_tmp)) && finish_line_closeness_check_2 == 0)
        rand_wayline = waylines_count - Horizon_limit_tmp + 2;
        finish_line_closeness_check_2 = 1;
    end
    
    q_rand = [waypoints{rand_waypoint,rand_wayline}{1,1}(1,1), waypoints{rand_waypoint,rand_wayline}{1,1}(1,2)];
    
    plot(q_rand(1), q_rand(2), 'x', 'Color',  [0 0.4470 0.7410]);
    
    % Pick the closest node from existing list to branch out from
    costs = [];
    %[~ , qrand_theta, qrand_curvature] = Calc_wayline_num(q_rand,waypoints, headings, curvatures);
    qrand_theta = headings(1, rand_wayline);
    qrand_curvature = curvatures(1, rand_wayline);

    tmp = 10000;
    max_cost = tmp;
    min_cost = tmp;
    
    intermediate_logic_1_time_end = toc(intermediate_logic_1_time_start);
    
    parent_spline_generation_time_start = tic;
    
    for parent_node_wayline =  max(1,(rand_wayline-Horizon_limit)):1:max(1,(rand_wayline-1))
        
        for parent_node_waypoint = 1:1:waypoints_per_wayline
            
            if(exploration_matrix(parent_node_waypoint,parent_node_wayline) == 1) % Dont re-explore previously explored nodes

                parent_node_xy = [waypoints{parent_node_waypoint, parent_node_wayline}{1,1}(1,1), waypoints{parent_node_waypoint, parent_node_wayline}{1,1}(1,2)];
                %[wayline_num_parent_node, parent_node_theta, parent_node_curvature] = Calc_wayline_num(parent_node_xy,waypoints, headings, curvatures);
                
                parent_node_theta = headings(1, parent_node_wayline);
                parent_node_curvature = curvatures(1, parent_node_wayline);

                 CL_parent.build_G1(parent_node_xy(1), parent_node_xy(2), parent_node_theta, q_rand(1), q_rand(2), qrand_theta);
                 CL_s = CL_parent.length;

                 [cost, v_ini_clothoid, time_vect, speed_profile] = cost_FWBW(CL_s, parent_node_curvature,qrand_curvature,v_ini_clothoid_matrix(parent_node_waypoint, parent_node_wayline));

                 % Check the intersection
                 s_inters_left = CL_parent.intersect(edge_left);
                 s_inters_right = CL_parent.intersect(edge_right);
                 if (isempty(s_inters_left) && isempty(s_inters_right))
                    % this means that no intersection exists
                        
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
                        
                        %Store the speed profile of the minimum cost spline
                        time_vect_temp = time_vect;
                        speed_profile_vect_temp = speed_profile;
                        
                    end
                end
            end
        end
    end
    
    
    CL_min_cost_parent.plot(npts,'Color','red');
    
    parent_spline_generation_time_end = toc(parent_spline_generation_time_start);
     
    rewiring_spline_generatioin_time_start = tic;
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
                %[wayline_num_rewiring_node, rewiring_node_theta, rewiring_node_curvature] = Calc_wayline_num(rewiring_node_xy,waypoints, headings, curvatures);
                rewiring_node_theta = headings(1,rewiring_node_wayline);
                rewiring_node_curvature = curvatures(1,rewiring_node_wayline);
                
                CL_rewiring.build_G1(q_rand(1), q_rand(2), qrand_theta, rewiring_node_xy(1), rewiring_node_xy(2), rewiring_node_theta);
                CL_s_rewiring = CL_rewiring.length;
                
                [cost_rewiring, v_ini_clothoid_rewiring, time_vect, speed_profile] = cost_FWBW(CL_s_rewiring, qrand_curvature,rewiring_node_curvature,v_ini_clothoid_matrix(rand_waypoint,rand_wayline));
                
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
                        
                        %Store the speed profile of the minimum cost spline
                        time_vect_temp = time_vect;
                        speed_profile_vect_temp = speed_profile;
                    end
                end
            end
        end
     end
     
   total_time_vect = [total_time_vect time_vect_temp];
   total_velocity_profile = [total_velocity_profile speed_profile_vect_temp];
   
   if rewiring_check ~= 0
       rewired_node_xy = [waypoints{rewired_node_waypoint, rewired_node_wayline}{1,1}(1,1), waypoints{rewired_node_waypoint, rewired_node_wayline}{1,1}(1,2)];
       CL_rewiring_final.build_G1(q_rand(1), q_rand(2), qrand_theta, rewired_node_xy(1), rewired_node_xy(2), rewired_node_theta);
       CL_rewiring_final.plot(npts,'Color','blue');
       
       %Store the wayline of the latest explored node as the last explored wayline
        wayline_num_last_node =  rewired_node_wayline;
   end
   
   rewiring_spline_generatioin_time_end = toc(rewiring_spline_generatioin_time_start);
   
   loop_iteration_time_check_end = toc(loop_iteration_time_check);
   
   % Create computation time arrays for plotting later
   loop_iteration_time_check_end_array = [loop_iteration_time_check_end_array 10*loop_iteration_time_check_end];
   initial_conditions_time_end_array = [initial_conditions_time_end_array 10*initial_conditions_time_end];
   rand_wayline_generation_time_end_array = [rand_wayline_generation_time_end_array 10*rand_wayline_generation_time_end];
   intermediate_logic_1_time_end_array = [intermediate_logic_1_time_end_array 10*intermediate_logic_1_time_end];
   parent_spline_generation_time_end_array = [parent_spline_generation_time_end_array 10*parent_spline_generation_time_end];
   rewiring_spline_generatioin_time_end_array = [rewiring_spline_generatioin_time_end_array 10*rewiring_spline_generatioin_time_end];
   
   
   %fprintf("\nLoop iteration time is :%d =>\n %d + %d + %d + %d + %d\n", %loop_iteration_time_check_end, initial_conditions_time_end, ...
   %rand_wayline_generation_time_end, intermediate_logic_1_time_end, parent_spline_generation_time_end, rewiring_spline_generatioin_time_end);
   
 end
 
%  figure('Name', 'Computation Time consumption checker'), clf
%    hold on
%    axis equal   
%    plot(loop_iteration_time_check_end_array, 'o');
%    %hold on
%    plot(initial_conditions_time_end_array, '--');
%    %hold on
%    plot(rand_wayline_generation_time_end_array, '*');
%    %hold on
%    plot(intermediate_logic_1_time_end_array, 's');
%    %hold on
%    plot(parent_spline_generation_time_end_array, 'd');
%    %hold on
%    plot(rewiring_spline_generatioin_time_end_array, '>');
%    
%    grid on
%    xlabel('iteration number [-]')
%    ylabel('duration [sec]')
%    title('Computation time')
   
%   grid on
%   xlabel('iteration number [-]')
%   ylabel('duration [sec]')
%   title('Computation time')
 
 t_RRT_star_exploration_end = toc(t_RRT_star_exploration_start);
 
 fprintf("\nIt took %d seconds for the RRT* exploration\n", t_RRT_star_exploration_end);
 
t_final_path_plot_start = tic; 
figure('Name','Optimum path','NumberTitle','off'), clf
hold on
axis equal

edge_left.plot;
edge_right.plot;

%Move backwards from Goal to start based on the parent matrix
goal_coord = [waypoints{2,waylines_count}{1,1}(1,1), waypoints{2,waylines_count}{1,1}(1,2)];
%goal_coord = [waypoints{2,wayline_num_last_node_max}{1,1}(1,1), waypoints{2,wayline_num_last_node_max}{1,1}(1,2)];
child_temp = goal_coord;
child_wayline = waylines_count;
%child_wayline = wayline_num_last_node_max;

parent_wayline = parent_matrix{2, child_wayline}{1,1}(1,2);
parent_temp = [waypoints{2, parent_wayline}{1,1}(1,1), waypoints{2, parent_wayline}{1,1}(1,2)];

CL_final_path_curv = 0;

while(1)
    CL_final_result.build_G1(parent_temp(1), parent_temp(2), headings(1, parent_wayline),child_temp(1), child_temp(2), headings(1, child_wayline));
    CL_final_result.plot;
    
    CL_final_path_length=[CL_final_path_length CL_final_result.length];
    CL_final_path_curv = [CL_final_path_curv CL_final_result.kappa(CL_final_result.length)];
    
    child_temp = parent_temp;
    child_wayline = parent_wayline;
     
    parent_wayline = parent_matrix{2, child_wayline}{1,1}(1,2);
    
    if (parent_wayline > 0)
        parent_temp = [waypoints{2, parent_wayline}{1,1}(1,1), waypoints{2, parent_wayline}{1,1}(1,2)];
    else
        disp('discontinuity detected in final path');
        break;
    end
    
    if(parent_wayline == 1)
        CL_final_result.build_G1(parent_temp(1), parent_temp(2), headings(1, parent_wayline),child_temp(1), child_temp(2), headings(1, child_wayline));
        CL_final_result.plot;
        break;
    end
end

%%% If one wants to plot all the generated waypoints by the
%%% "define_waypoints" function
for k = 1:1:waylines_count
%for k = 1:1:wayline_num_last_node_max
  for m = 1:1:waypoints_per_wayline
    plot(waypoints{m,k}{1,1}(1,1), waypoints{m,k}{1,1}(1,2), 'x');
  end
end

grid on
xlabel('x [m]')
ylabel('y [m]')
title('Circuit-Final')

t_final_path_plot_end = toc(t_final_path_plot_start);

fprintf("\nIt took %d seconds for plotting the final plot\n", t_final_path_plot_end);

[cost_final_path, v_ini_clothoid_final_path, time_vect_final_path, speed_profile_final_path] = cost_FWBW(CL_final_path_length, CL_final_path_curv(1, 1:(end-1)),CL_final_path_curv(1, end),v_ini_clothoid_matrix(2,1));

figure('Name','Speed Profile','NumberTitle','off'), clf
hold on
axis equal
plot(time_vect_final_path,  speed_profile_final_path, 'x');
grid on
xlabel('t[0.01s]')
ylabel('v(m/s)')

fprintf('\n------------------End------------------\n');