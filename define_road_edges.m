function [edge_left, edge_right] = define_road_edges(Center_line)

[waypoints, headings, curvatures] = points_for_edge_definition(Center_line);

edge_left = ClothoidList();
edge_right = ClothoidList();

%spline_temp = ClothoidList();

[waypoints_per_wayline, number_of_waylines] = size(waypoints);

    for wayline_number = 1:number_of_waylines-1
        for waypoint_number = 1:waypoints_per_wayline            
            edge_spline_init_point_x = waypoints{waypoint_number,wayline_number}{1,1}(1,1);
            edge_spline_init_point_y = waypoints{waypoint_number,wayline_number}{1,1}(1,2);
            edge_spline_final_point_x = waypoints{waypoint_number,wayline_number+1}{1,1}(1,1);
            edge_spline_final_point_y = waypoints{waypoint_number,wayline_number+1}{1,1}(1,2);
            spline_init_point_theta = headings(1,wayline_number);
            spline_final_point_theta = headings(1,wayline_number+1);
            %spline_temp.build_G1(edge_spline_init_point_x, edge_spline_init_point_y, spline_init_point_theta, edge_spline_final_point_x, edge_spline_final_point_y, spline_final_point_theta);        
            
            if waypoint_number == 1
                edge_left.push_back_G1(edge_spline_init_point_x, edge_spline_init_point_y, spline_init_point_theta, edge_spline_final_point_x, edge_spline_final_point_y, spline_final_point_theta);
            else
                edge_right.push_back_G1(edge_spline_init_point_x, edge_spline_init_point_y, spline_init_point_theta, edge_spline_final_point_x, edge_spline_final_point_y, spline_final_point_theta);
            end 
        end
    end
    
end