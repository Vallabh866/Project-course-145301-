function [waypoints, headings, curvatures] = define_waypoints(Center_line)

slope = -1.5;
c = 18;
const = 100;
for k = 1:Center_line.length()
   curvature = Center_line.kappa(k);
   if curvature>=0
        sw(k) = max(abs(c+slope*(const*curvature)),8);
    else
        sw(k) = max(abs(c-slope*(const*curvature)),8);
    end
    %plot(const*curvature, sw(k));
    %hold on;
end

waypoints = cell(3,floor(Center_line.length()));
headings = zeros(1,floor(Center_line.length()));
curvatures = zeros(1,floor(Center_line.length()));

side_waypoint_gap = 2;
for smin = 1:Center_line.length()
    [x_Center_line,y_Center_line] = Center_line.evaluate(smin+sw(smin));
    
    waypoint_right = zeros(1,2);
    waypoint_left = zeros(1,2);
    waypoint_centre = zeros(1,2);
    waypoints_temp = cell(3,1);
    
    %waypoints = zeros(3, floor(Center_line.length()));
    
    Translation_matrix = [cos(Center_line.theta(smin)), -1*sin(Center_line.theta(smin)); sin(Center_line.theta(smin)), cos(Center_line.theta(smin))];
    gap_matrix = [0;side_waypoint_gap];
    
    waypoint_right_temp = [x_Center_line ; y_Center_line] + Translation_matrix*gap_matrix;
    waypoint_left_temp = [x_Center_line ; y_Center_line] - Translation_matrix*gap_matrix;
    
    %Plot the waylines
    wayline_right = LineSegment();
    %wayline_right.build( x_Center_line,y_Center_line, Center_line.theta(smin)+pi/2, side_waypoint_gap );
    wayline_right.build( [x_Center_line,y_Center_line], [waypoint_right_temp(1,1), waypoint_right_temp(2,1)] );
    wayline_right.plot();
    wayline_left = LineSegment();
    wayline_left.build( [x_Center_line,y_Center_line], [waypoint_left_temp(1,1), waypoint_left_temp(2,1)] );
    %wayline_left.build( x_Center_line,y_Center_line, Center_line.theta(smin)+pi/2, -1*side_waypoint_gap );
    wayline_left.plot();
    %    
    
    %Plot the waypoint on each wayline
    waypoint_centre = {[x_Center_line,y_Center_line]};
    waypoint_right = {[waypoint_right_temp(1,1), waypoint_right_temp(2,1)]};
    waypoint_left = {[waypoint_left_temp(1,1), waypoint_left_temp(2,1)]};
    
    %plot(waypoint_centre(1,1), waypoint_centre(2,1), 'x');
    %plot(waypoint_right(1,1), waypoint_right(2,1), 'x');
    %plot(waypoint_left(1,1), waypoint_left(2,1), 'x');
    %
    
    waypoints_temp = {waypoint_right; waypoint_centre; waypoint_left};
    %fprintf('%d %d\n%d %d\n%d %d\n-------------\n',waypoints_temp(1,1),waypoints_temp(1,2),waypoints_temp(2,1),waypoints_temp(2,2),waypoints_temp(3,1),waypoints_temp(3,2));
    %if smin>0
    waypoints(1,smin) = waypoints_temp(1,1);
    waypoints(2,smin) = waypoints_temp(2,1);
    waypoints(3,smin) = waypoints_temp(3,1);
    
    headings(1,smin) = Center_line.theta(smin+sw(smin));
    curvatures(1,smin) = Center_line.kappa(smin+sw(smin));
end

end