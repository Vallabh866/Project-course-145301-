
% Select a random waypooint on a random wayline but only within the Horizon
% limit
rand_waypoint = randi([1,waypoints_per_wayline],1,1);   
rand_number = floor(rand*2*Horizon_limit)+1;
rand_wayline = max(2,((wayline_num_last_node - Horizon_limit) + rand_number));

% In case explorer crosses the finish line or the exploration limit wayline
% during the exploration cycles, exploration resumes from the beginning
if (rand_wayline > waylines_count)
    rand_wayline = max(2,(rand_wayline - waylines_count));
end

% Counter to check if all waypoints inside current horizon is explored
exploration_saturation_counter = 0;

while(1)
    %Check 'also' the next Horizon for a potential parent if the current
    %horizon is fully explored
    exploration_saturation_counter = exploration_saturation_counter + 1;
    
    %Break if a potential parent is found
    if((rand_wayline>1) && (exploration_matrix(rand_waypoint,rand_wayline) ~= 1))
        break;
    else
        if(exploration_saturation_counter <= 2*Horizon_limit)
            rand_number = floor(rand*2*Horizon_limit)+1;
            rand_wayline = max(2,((wayline_num_last_node - Horizon_limit) + rand_number));
        else
            Horizon_limit_2 = Horizon_limit*2; % Keep increasing the horizon till a potential parent is found
            rand_number = rand_number + floor(rand*2*Horizon_limit_2)+1;
                
            if(rand_number > waylines_count)
                rand_number = rem(rand_number,waylines_count);
            end
                rand_wayline = max(2,((wayline_num_last_node - Horizon_limit_2) + rand_number));
        end
        rand_waypoint = min(max(floor(rand*10/3),1),3);
            
        if (rand_wayline > waylines_count)
                rand_wayline = max(2,(rand_wayline - waylines_count));
        end
    end    
end

if(rand_wayline > waylines_count)
    rand_wayline = max(2,(rand_wayline - waylines_count));
end
    
% if((wayline_num_last_node_max > (waylines_count - 2*Horizon_limit_tmp)) && finish_line_closeness_check_1 == 0)
%     rand_wayline = waylines_count - 2*Horizon_limit_tmp + 2;
%     finish_line_closeness_check_1 = 1;
% elseif((wayline_num_last_node_max > (waylines_count - Horizon_limit_tmp)) && finish_line_closeness_check_2 == 0)
%     rand_wayline = waylines_count - Horizon_limit_tmp + 2;
%     finish_line_closeness_check_2 = 1;
% end
q_rand = [waypoints{rand_waypoint,rand_wayline}{1,1}(1,1), waypoints{rand_waypoint,rand_wayline}{1,1}(1,2)];
   