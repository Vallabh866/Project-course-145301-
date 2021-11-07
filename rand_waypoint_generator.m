%function [q_rand] = rand_waypoint_generator()
    
    %rand_waypoint = min(max(floor(rand*10/3),1),3);
    rand_waypoint = randi([1,waypoints_per_wayline],1,1);
    
    rand_number = floor(rand*2*Horizon_limit)+1;
    
    rand_wayline = max(2,((wayline_num_last_node - Horizon_limit) + rand_number));
    
    if (rand_wayline > waylines_count)
        rand_wayline = max(2,(rand_wayline - waylines_count));
    end
    
    exploration_saturation_counter = 0;
    
    while(1)
        %Check exploration saturation in case the next Horizon is explored
        exploration_saturation_counter = exploration_saturation_counter + 1;
        
        if((rand_wayline>1) && (exploration_matrix(rand_waypoint,rand_wayline) ~= 1))
        break;
        else
            if(exploration_saturation_counter <= 2*Horizon_limit)
                rand_number = floor(rand*2*Horizon_limit)+1;
                rand_wayline = max(2,((wayline_num_last_node - Horizon_limit) + rand_number));
            else
                Horizon_limit_2 = Horizon_limit*2; % Increase the Horizon limit if all the nodes around are explored
                rand_number = rand_number + floor(rand*2*Horizon_limit_2)+1;
                
                if(rand_number > waylines_count)
                    rand_number = rem(rand_number,waylines_count);
                end
                rand_wayline = max(2,((wayline_num_last_node - Horizon_limit_2) + rand_number));
            end
            rand_waypoint = min(max(floor(rand*10/3),1),3);
            
            %rand_wayline = max(2,((wayline_num_last_node - Horizon_limit) + rand_number));
            
            if (rand_wayline > waylines_count)
                rand_wayline = max(2,(rand_wayline - waylines_count));
            end
        end    
    end
    
    if(rand_wayline > waylines_count)
        rand_wayline = max(2,(rand_wayline - waylines_count));
    end
    
    if((wayline_num_last_node_max > (waylines_count - 2*Horizon_limit_tmp)) && finish_line_closeness_check_1 == 0)
        rand_wayline = waylines_count - 2*Horizon_limit_tmp + 2;
        finish_line_closeness_check_1 = 1;
    elseif((wayline_num_last_node_max > (waylines_count - Horizon_limit_tmp)) && finish_line_closeness_check_2 == 0)
        rand_wayline = waylines_count - Horizon_limit_tmp + 2;
        finish_line_closeness_check_2 = 1;
    end
    
    q_rand = [waypoints{rand_waypoint,rand_wayline}{1,1}(1,1), waypoints{rand_waypoint,rand_wayline}{1,1}(1,2)];
%end