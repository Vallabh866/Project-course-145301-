clearvars
close all
clc

project_folder = pwd;
addpath(genpath(project_folder));
addpath(strcat(project_folder,'\Racetracks\Racetracks\PistaAzzurra_Track'));
run(strcat(project_folder,'\Racetracks\Racetracks\PistaAzzurra_Track\create_RoadFile.m'));
close all

t_track_creation_start = tic;

% Perform all Parameter initializations along with track paramaters definition
run(strcat(project_folder,'\param_init.m'));

t_track_creation_end = toc(t_track_creation_start);
fprintf("\nIt took %d seconds for the Track to be created along with the waylines\n", t_track_creation_end);

t_RRT_star_exploration_start = tic;

 while (1)    
    % Check for the 2 goal reached conditions 
    
    % Condition 1 : Check if the all the waypoints on the last wayline are explored
    final_saturation_condition_counter = 0;
    for tmp = 1:waypoints_per_wayline
        if(exploration_matrix(tmp, waylines_count) == 1)
            final_saturation_condition_counter = final_saturation_condition_counter + 1;
        end
    end 
    % Condition 2 : Check if a given percentage of the whole track is explored
        if (final_saturation_condition_counter >= 1)
           if sum(exploration_matrix,'all') >= (exploration_saturation_percentage)*(sum((ones(size(waypoints))),'all'))
                break; % Stop the RRT* exploration
           end
        end   
     
    % Store the farthest wayline that the algorithm has reached
    if(wayline_num_last_node>wayline_num_last_node_max)
        wayline_num_last_node_max = wayline_num_last_node;
    end
    
    % Generate a random waypoint q_rand which RRT* will explore next
    run(strcat(project_folder,'\rand_waypoint_generator.m'));
    plot(q_rand(1), q_rand(2), 'x', 'Color',  [0 0.4470 0.7410]);
    
    % Pick the closest node from existing list to branch out from
    costs = [];
    qrand_theta = headings(1, rand_wayline);
    qrand_curvature = curvatures(1, rand_wayline);
    min_cost = 10000;
    
    % Perform forward exploration to find the most suitable parent based on the FW-BW cost
    run(strcat(project_folder,'\Forward_exploration.m'));
    CL_min_cost_parent.plot(npts,'Color','red');

    %   Perform Rewiring to refine the planned path
   run(strcat(project_folder,'\Rewiring.m'));
   
 end
 
 % Plot the time consumption graphs for different sections of the algorithm
 % run(strcat(project_folder,'\Time_cosumption.m'));
 
 t_RRT_star_exploration_end = toc(t_RRT_star_exploration_start);
 
 fprintf("\nIt took %d seconds for the RRT* exploration\n", t_RRT_star_exploration_end);
 
t_final_path_plot_start = tic; 
figure('Name','Optimum path','NumberTitle','off'), clf
hold on
axis equal

edge_left.plot;
edge_right.plot;

%Plot the final optimal path found after the exploration and rewiring
run(strcat(project_folder,'\Final_path.m'));

t_final_path_plot_end = toc(t_final_path_plot_start);

fprintf("\nIt took %d seconds for plotting the final plot\n", t_final_path_plot_end);

[cost_final_path, v_ini_clothoid_final_path, time_vect_final_path, speed_profile_final_path] = ...
    cost_FWBW(CL_final_path_length, CL_final_path_curv(1, 1:(end-1)),CL_final_path_curv(1, end),v_ini_clothoid_matrix(2,1));

figure('Name','Speed Profile','NumberTitle','off'), clf
hold on
axis equal
plot(time_vect_final_path,  speed_profile_final_path, 'x');
grid on
xlabel('t[0.01s]')
ylabel('v(m/s)')

fprintf('\n------------------End------------------\n');