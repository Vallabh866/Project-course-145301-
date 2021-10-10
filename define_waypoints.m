function [waypoints, headings, curvatures] = define_waypoints(Center_line)

    slope = -1.7;
    c = 20;
    const = 100;
    wayline_min_gap = 3;

    % for k = 1:wayline_min_gap:Center_line.length()% Make sure this is an integer(not lentgh but no. of points)
    %    curvature = Center_line.kappa(k);
    %    if curvature>=0
    %         sw(k) = max(abs(c+slope*(const*curvature)),5);
    %     else
    %         sw(k) = max(abs(c-slope*(const*curvature)),5);
    %     end
    %     %plot(const*curvature, sw(k));
    %     %hold on;
    % end


    wayline_count = 0;

    for smin = 1:wayline_min_gap:Center_line.length()
        wayline_count=wayline_count+1;
    end

    % waypoints = cell(3,wayline_count);
    % headings = zeros(1,wayline_count);
    % curvatures = zeros(1,wayline_count);

    waypoints = cell(3,1);
    headings = zeros(1,1);
    curvatures = zeros(1,1);

    % waypoints = cell(3,floor(Center_line.length()));
    % headings = zeros(1,floor(Center_line.length()));
    % curvatures = zeros(1,floor(Center_line.length()));

    next_wayline=1;

    side_waypoint_gap = 2;
    for smin = 1:wayline_min_gap:Center_line.length()
         %for smin = 1:Center_line.length()
            %[x_Center_line,y_Center_line] = Center_line.evaluate(smin+sw(smin));
           if(next_wayline<Center_line.length()) 
                [x_Center_line,y_Center_line] = Center_line.evaluate(next_wayline);


                waypoint_right = zeros(1,2);
                waypoint_left = zeros(1,2);
                waypoint_centre = zeros(1,2);
                waypoints_temp = cell(3,1);

                %fprintf('s: %d  Gap : %d\n',smin,sw(smin));
                %waypoints = zeros(3, floor(Center_line.length()));

                %Translation_matrix = [cos(Center_line.theta(smin+sw(smin))), -1*sin(Center_line.theta(smin+sw(smin))); sin(Center_line.theta(smin+sw(smin))), cos(Center_line.theta(smin+sw(smin)))];
                Translation_matrix = [cos(Center_line.theta(next_wayline)), -1*sin(Center_line.theta(next_wayline)); sin(Center_line.theta(next_wayline)), cos(Center_line.theta(next_wayline))];
                %next_wayline = next_wayline + sw(smin);

                gap_matrix = [0;side_waypoint_gap];

                waypoint_right_temp = [x_Center_line ; y_Center_line] + Translation_matrix*gap_matrix;
                waypoint_left_temp = [x_Center_line ; y_Center_line] - Translation_matrix*gap_matrix;

                %Plot the waylines
                wayline_right = LineSegment();
                %wayline_right.build( x_Center_line,y_Center_line, Center_line.theta(smin)+pi/2, side_waypoint_gap );
                wayline_right.build( [x_Center_line,y_Center_line], [waypoint_right_temp(1,1), waypoint_right_temp(2,1)] );
                %wayline_right.plot();
                wayline_left = LineSegment();
                wayline_left.build( [x_Center_line,y_Center_line], [waypoint_left_temp(1,1), waypoint_left_temp(2,1)] );
                %wayline_left.build( x_Center_line,y_Center_line, Center_line.theta(smin)+pi/2, -1*side_waypoint_gap );
                %wayline_left.plot();
                %    

                %Plot the waypoint on each wayline
                waypoint_centre = {[x_Center_line,y_Center_line]};
                waypoint_right = {[waypoint_right_temp(1,1), waypoint_right_temp(2,1)]};
                waypoint_left = {[waypoint_left_temp(1,1), waypoint_left_temp(2,1)]};

            %     plot(waypoint_centre(1,1), waypoint_centre(2,1), 'x');
            %     plot(waypoint_right(1,1), waypoint_right(2,1), 'x');
            %     plot(waypoint_left(1,1), waypoint_left(2,1), 'x');
                %

                waypoints_temp = {waypoint_right; waypoint_centre; waypoint_left};
                %fprintf('%d %d\n%d %d\n%d %d\n-------------\n',waypoints_temp(1,1),waypoints_temp(1,2),waypoints_temp(2,1),waypoints_temp(2,2),waypoints_temp(3,1),waypoints_temp(3,2));
                %if smin>1
            %         waypoints(1,((smin-1)/wayline_min_gap)+1) = waypoints_temp(1,1);
            %         waypoints(2,((smin-1)/wayline_min_gap)+1) = waypoints_temp(2,1);
            %         waypoints(3,((smin-1)/wayline_min_gap)+1) = waypoints_temp(3,1);
                                        
                    waypoints = [waypoints waypoints_temp];
                    
                    if(smin == 1)
                        waypoints = waypoints(:,2:end);
                    end

                    %headings(1,smin) = Center_line.theta(smin+sw(smin));
                    %curvatures(1,smin) = Center_line.kappa(smin+sw(smin));
                    headings = [headings Center_line.theta(next_wayline)];
                    %headings = [headings (-deg2rad(90)-Center_line.theta(next_wayline))];
                    curvatures = [curvatures Center_line.kappa(next_wayline)];
            %     else
            %         waypoints(1,smin) = waypoints_temp(1,1);
            %         waypoints(2,smin) = waypoints_temp(2,1);
            %         waypoints(3,smin) = waypoints_temp(3,1);
            %     
            %         %headings(1,smin) = Center_line.theta(smin+sw(smin));
            %         %curvatures(1,smin) = Center_line.kappa(smin+sw(smin));
            %         headings(1,smin) = Center_line.theta(next_wayline);
            %         curvatures(1,smin) = Center_line.kappa(next_wayline);
                
                curvature = Center_line.kappa(next_wayline);
                %slope = -1.7;
                %c = 20;
                %const = 100;

                if curvature>=0
                    sw = max(abs(c+slope*(const*curvature)),5);
                else
                    sw = max(abs(c-slope*(const*curvature)),5);
                end
                next_wayline = next_wayline + sw;

           end
    end
    
    headings = headings(1,2:end);
    curvatures = curvatures(1,2:end);

end