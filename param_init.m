fprintf("\n-----------------------------\n");
Selection = '\nPlease select one option:\n1. Full track exploration\n2. Single corner exploration\n';
Exploration_option = input(Selection);
fprintf("\n-----------------------------\n");

% Array to store the velocity profile of the final path
total_velocity_profile=[];

%Arrays to store the time vect and velocity profile of individual splines
total_time_vect = [];
time_vect = [];
speed_profile = [];

%Array to store the length of the splines contained in the final path
CL_final_path_length=[];

%Array to store the curvature of the splines contained in the final path
CL_final_path_curv=[];

%Create track and retrieve clothoid list of the track
Center_line = ClothoidList;
Center_line = create_RoadFile();

if(Exploration_option==2)
    %Center_line = Center_line*0.1;
    % This condition is specifically for Pista Azzura, Change the min and
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
CL_final_result_combined = ClothoidList();

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

%end