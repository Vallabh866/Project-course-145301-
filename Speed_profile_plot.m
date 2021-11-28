%% Calculate the speed profile of the combined Clothoid list

Ax_brake_max = 10;      % [m/s^2] max braking deceleration (absolute value)
Ax_accel_max = 10;      % [m/s^2] max acceleration
Ay_max = 15;            % [m/s^2] max lateral acceleration
V_max  = 30;            % [m/s] max vehicle longitudinal speed
c0 = 0;                 % 'laminar' drag coefficient (a_drag = -c0*v)
c1 = 0.004940850;       % aerodynamic drag coefficient, normalized by the vehicle mass (a_drag = -c1*v^2)

% Store data in a struct to be used in the 'forward-backward' clothoid optimization
dataFB.Amax   = Ay_max;
dataFB.Vmax   = V_max;
dataFB.abrake = Ax_brake_max;
dataFB.apush  = Ax_accel_max;
dataFB.c0     = c0;
dataFB.c1     = c1;

% s, theta and curvature values defining the extrema of each clothoid in a list of clothoids
[s_list,theta_list,curv_list] = CL_final_result_combined.getSTK;

% final speed of the vehicle (at the end of the path)
v_fin = 30;  % [m/s]

FB = ForwardBackwardList(dataFB);
FB.maxSpeed(v_ini_clothoid,v_fin,s_list,curv_list);
final_time = FB.totalT();  % minimum travelling time (optimal)

% Plot the speed profile
figure('Name','Speed profile(wrt s[m])','NumberTitle','off')
FB.plot();
grid on
xlabel('curvilinear abscissa s');
ylabel('v(s)');
title('Speed profile');

% Plot the curvature profile of the path
figure('Name','Curvature profile','NumberTitle','off')
FB.plotk();
grid on
xlabel('s') ;
ylabel('curv(s)') ;
title('Curvature profile');

% Extract info about the solution
%FB.info();

% Time vector
time_sampling = 0.01;  % [s]
tim_vect = 0:time_sampling:final_time;
curv_absc_vect = FB.s(tim_vect);        % [m] curvilinear abscissa of the solution
speed_profile = FB.vs(curv_absc_vect);  % [m/s] speed profile of the solution

figure('Name','Speed profile(wrt t[sec])','NumberTitle','off')
plot(tim_vect,speed_profile,'.')
grid on
xlabel('time [s]');
ylabel('speed [m/s]');

%Plot the velocity values along the track
x_final_path_vect=[];
y_final_path_vect=[];
[~,final_path_speed_profile_vect_size]=size(speed_profile);
path_discretizn_step_size = CL_final_result_combined.length()/final_path_speed_profile_vect_size;

for i = 1:path_discretizn_step_size:CL_final_result_combined.length()
    if(i <=final_path_speed_profile_vect_size)
        [x_final_path, y_final_path] = CL_final_result_combined.evaluate(i);
        x_final_path_vect = [x_final_path_vect x_final_path];
        y_final_path_vect = [y_final_path_vect y_final_path];
    end
end

figure('Name','Speed Profile over the track','NumberTitle','off'), clf
[~,x_final_path_vect_size]=size(x_final_path_vect);
for i=1:1:x_final_path_vect_size
    plot3(x_final_path_vect(i),y_final_path_vect(i),speed_profile(i),'x')
hold on
end
grid on
xlabel('x [m]')
ylabel('y [m]')
zlabel('z [m/s]')
