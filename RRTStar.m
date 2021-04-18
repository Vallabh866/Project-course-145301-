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
%r_angl = linspace(pi/4, 3*pi/4, N);                             % Angle Defining Arc Segment (radians)
r_angl = linspace(0, pi, N);                             % Angle Defining Arc Segment (radians)
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
slope_q_start = (q_start.coord(2))/(q_start.coord(1)-100);
if(rad2deg(atan(slope_q_start))>0)
        instant_theta_q_start = -(pi/2 - atan(slope_q_start));
    else
        instant_theta_q_start = pi/2 + atan(slope_q_start);
    end
CL = ClothoidCurve(q_start.coord(1), q_start.coord(2), instant_theta_q_start, 1/((radius1+radius2)/2), 0, 1 );
npts = 100;
v_ini_clothoid = 2.3614;  % [m/s]
CL_s = zeros(numNodes);
CL_k = zeros(numNodes);

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
    
    slope_q_near = (q_near.coord(2))/(q_near.coord(1)-100);
    slope_q_new = (q_new.coord(2))/(q_new.coord(1)-100);
    
    if(rad2deg(atan(slope_q_near))>0)
        instant_theta_q_near = -(pi/2 - atan(slope_q_near));
    else
        instant_theta_q_near = pi/2 + atan(slope_q_near);
    end
    
    if(rad2deg(atan(slope_q_new))>0)
        instant_theta_q_new = -(pi/2 - atan(slope_q_new));
    else
        instant_theta_q_new = pi/2 + atan(slope_q_new);
    end
    
    if(q_new.coord(1)>q_near.coord(1))
        CL.build_G1(q_near.coord(1), q_near.coord(2), instant_theta_q_near, q_new.coord(1), q_new.coord(2), instant_theta_q_new);
        CL.plot(npts,'Color','red');
    end
    
    %fprintf("Length: %f\n",CL.length);
    
    CL_s(i) = CL.length;
    CL_k(i) = 1/((radius1+radius2)/2);
    %fprintf("Clothoids : %d\n", CL_s);
    %fprintf("------------------------------\n\n");
    
    if(i>=2)
        cost = cost_FWBW(CL_s(i), CL_s(i-1), 1/((radius1+radius2)/2), 1/((radius1+radius2)/2),v_ini_clothoid);
    end
    
    %fprintf("cost: %f\n",cost);
    %fprintf("Initial velocity: %f\n",v_ini_clothoid);
    %fprintf("------------------------------\n\n");
    
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
    %instantaneous_theta = atan()
    
    q_end = nodes(start);
end

for k = 1:1:length(final_nodes)
    slope = (final_nodes(k,2))/(final_nodes(k,1)-100);
    %fprintf("Slope: %d\n", slope);
    if(rad2deg(atan(slope))>0)
        instantaneous_theta(k) = -(pi/2 - atan(slope));
    else
        instantaneous_theta(k) = pi/2 + atan(slope);
    end
    %final_nodes = [final_nodes instantaneous_theta];
end
% 
% dist = 0;
% for k = 1:1:length(final_nodes)-1 
%     dist1 = sqrt((final_nodes(k,2)-final_nodes(k+1,2))*(final_nodes(k,2)-final_nodes(k+1,2))+(final_nodes(k,1)-final_nodes(k+1,1))*(final_nodes(k,1)-final_nodes(k+1,1)));
%     dist = dist + dist1;
%     %fprintf("%d\n", dist);
% end
% 
% 
% 
% instantaneous_theta = instantaneous_theta';
% interp_sampling = 30;
% interp_vector_fewPoints = 0:interp_sampling:dist;
% refPath_poses_fewPoints = [final_nodes instantaneous_theta];
% 
% % ----------------------
% % Fit the interpolated path with clothoids
% % ----------------------
% path_fewPoints = ClothoidList();
% numOfClothoids_fewPoints = size(refPath_poses_fewPoints,1);
% for jj = 1:numOfClothoids_fewPoints-1
%     path_fewPoints.push_back_G1(refPath_poses_fewPoints(jj,1),refPath_poses_fewPoints(jj,2),deg2rad(refPath_poses_fewPoints(jj,3)), refPath_poses_fewPoints(jj+1,1),refPath_poses_fewPoints(jj+1,2),deg2rad(refPath_poses_fewPoints(jj+1,3))); 
%     %fprintf("%d, %d, %d \n",refPath_poses_fewPoints(jj+1,1),refPath_poses_fewPoints(jj+1,2),refPath_poses_fewPoints(jj+1,3));
% end
% 
% % Compute the local curvature for the reference route
% interp_vector_fewPoints = [interp_vector_fewPoints, path_fewPoints.length];    % add also the final route point
% [x_cloth_refPath_fewPoints,y_cloth_refPath_fewPoints,theta_cloth_refPath_fewPoints,curv_refPath_fewPoints] = path_fewPoints.evaluate(interp_vector_fewPoints);
% refRoute_fewPoints = [x_cloth_refPath_fewPoints',y_cloth_refPath_fewPoints',theta_cloth_refPath_fewPoints',curv_refPath_fewPoints'];
% 
% %scenario = load('./Scenario/scenario');
% save('scenario.mat','refRoute_fewPoints');
% plot(refRoute_fewPoints(:,1),refRoute_fewPoints(:,2),'go','MarkerFaceColor','g','MarkerSize',4)
% scatter(refPath_poses_fewPoints(:,1),refPath_poses_fewPoints(:,2),'DisplayName','Interpolated Points')

%CL = ClothoidCurve(q_start.coord(1), q_start.coord(2), instantaneous_theta(1), 1/((radius1+radius2)/2), 0, 0.5 );
%CL.build_G1(refPath_poses_fewPoints(1,1), refPath_poses_fewPoints(1,2), refPath_poses_fewPoints(1,3), refPath_poses_fewPoints(2,1), refPath_poses_fewPoints(2,2), refPath_poses_fewPoints(2,3));

%npts = 100;
%CL.plot(npts,'Color','red');

% for jj = 2:size(refPath_poses_fewPoints,1)
%    CL.build_G1(refPath_poses_fewPoints(jj,1), refPath_poses_fewPoints(jj,2), refPath_poses_fewPoints(jj,3), refPath_poses_fewPoints(jj-1,1), refPath_poses_fewPoints(jj-1,2), refPath_poses_fewPoints(jj-1,3));
%    CL.plot(npts,'Color','red');
% end