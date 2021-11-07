 figure('Name', 'Computation Time consumption checker'), clf
   hold on
   axis equal   
   plot(loop_iteration_time_check_end_array, 'o');
   %hold on
   plot(initial_conditions_time_end_array, '--');
   %hold on
   plot(rand_wayline_generation_time_end_array, '*');
   %hold on
   plot(intermediate_logic_1_time_end_array, 's');
   %hold on
   plot(parent_spline_generation_time_end_array, 'd');
   %hold on
   plot(rewiring_spline_generatioin_time_end_array, '>');
   
   grid on
   xlabel('iteration number [-]')
   ylabel('duration [sec]')
   title('Computation time')
   
  grid on
  xlabel('iteration number [-]')
  ylabel('duration [sec]')
  title('Computation time')