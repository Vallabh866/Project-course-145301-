%This function performs the forward exploration of the track 

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
                    % Ensure that no intersection with the borders exists
                        
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