%Move backwards from Goal to start based on the parent matrix
goal_coord = [waypoints{2,waylines_count}{1,1}(1,1), waypoints{2,waylines_count}{1,1}(1,2)];
%goal_coord = [waypoints{2,wayline_num_last_node_max}{1,1}(1,1), waypoints{2,wayline_num_last_node_max}{1,1}(1,2)];
child_temp = goal_coord;
child_wayline = waylines_count;
child_waypoint = 2;
%child_wayline = wayline_num_last_node_max;

parent_wayline = parent_matrix{child_waypoint, child_wayline}{1,1}(1,2);
celldisp(parent_matrix);
parent_waypoint = parent_matrix{child_waypoint, child_wayline}{1,1}(1,1);
parent_temp = [waypoints{parent_waypoint, parent_wayline}{1,1}(1,1), waypoints{parent_waypoint, parent_wayline}{1,1}(1,2)];

CL_final_path_curv = 0;

while(1)
    %Single spline out of all the list of splines constituting the final
    %path
    CL_final_result.build_G1(parent_temp(1), parent_temp(2), headings(1, parent_wayline),child_temp(1), child_temp(2), headings(1, child_wayline));
    
    %Single spline made out of all the splines constituting the final
    %path
    CL_final_result_combined.push_back_G1(parent_temp(1), parent_temp(2), headings(1, parent_wayline),child_temp(1), child_temp(2), headings(1, child_wayline));
    CL_final_result.plot;
    
    CL_final_path_length=[CL_final_path_length CL_final_result.length];
    CL_final_path_curv = [CL_final_path_curv CL_final_result.kappa(CL_final_result.length)];
    
    child_temp = parent_temp;
    child_wayline = parent_wayline;
    child_waypoint = parent_waypoint;
     
    parent_wayline = parent_matrix{child_waypoint, child_wayline}{1,1}(1,2);
    parent_waypoint = parent_matrix{child_waypoint, child_wayline}{1,1}(1,1);
    
    if (parent_wayline > 0)
        %parent_temp = [waypoints{2, parent_wayline}{1,1}(1,1), waypoints{2, parent_wayline}{1,1}(1,2)];
        parent_temp = [waypoints{parent_waypoint, parent_wayline}{1,1}(1,1), waypoints{parent_waypoint, parent_wayline}{1,1}(1,2)];
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
  for m = 1:1:waypoints_per_wayline
    plot(waypoints{m,k}{1,1}(1,1), waypoints{m,k}{1,1}(1,2), 'x');
  end
end

grid on
xlabel('x [m]')
ylabel('y [m]')
title('Circuit-Final')