function [wayline_number, theta, curvature] = Calc_wayline_num(node,waypoints, headings, curvatures)
    %[waypoints, headings, curvatures] = define_waypoints(Center_line);
    [waypoints_per_wayline,waylines_count] = size(waypoints);
    for wayline_num = 1:waylines_count
        for waypoint_num = 1:waypoints_per_wayline
            if ((node(1) == waypoints{waypoint_num,wayline_num}{1,1}(1,1)) && (node(2) == waypoints{waypoint_num,wayline_num}{1,1}(1,2)))
               wayline_number = wayline_num;
               theta = headings(1,wayline_num);
               curvature = curvatures(1,wayline_num);
               break;
            end            
        end
    end

end