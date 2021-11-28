% This function performs Rewiring of the waylines present in the 
% horizon limit in front of the the newly explored rand node to search for
% better, more cost-optimal child node

q_nearest = [];
neighbor_count = 1;
rewiring_check = 0;
cost_rewiring = 0;

for rewiring_node_wayline =  (rand_wayline+1):1:min(waylines_count,(rand_wayline+Horizon_limit))% Consider only explored nodes from the  waylines ahead for rewiring
    for rewiring_node_waypoint = 1:1:waypoints_per_wayline
        if(exploration_matrix(rewiring_node_waypoint,rewiring_node_wayline) == 1)
                
                rewiring_node_xy = [waypoints{rewiring_node_waypoint,rewiring_node_wayline}{1,1}(1,1), waypoints{rewiring_node_waypoint,rewiring_node_wayline}{1,1}(1,2)];
                rewiring_node_theta = headings(1,rewiring_node_wayline);
                rewiring_node_curvature = curvatures(1,rewiring_node_wayline);
                
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
        end
    end
end
   
   if rewiring_check ~= 0
       rewired_node_xy = [waypoints{rewired_node_waypoint, rewired_node_wayline}{1,1}(1,1), waypoints{rewired_node_waypoint, rewired_node_wayline}{1,1}(1,2)];
       CL_rewiring_final.build_G1(q_rand(1), q_rand(2), qrand_theta, rewired_node_xy(1), rewired_node_xy(2), rewired_node_theta);
       CL_rewiring_final.plot(npts,'Color','blue');
       
       %Store the wayline of the latest explored node as the last explored wayline
        wayline_num_last_node =  rewired_node_wayline;
   end