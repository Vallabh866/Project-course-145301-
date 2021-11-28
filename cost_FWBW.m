function [cost, v_ini_clothoid] = cost_FWBW(CL_s, CL_k_ini,CL_k_fin, v_ini_clothoid)

set(0,'DefaultFigureWindowStyle','docked');
set(0,'defaultAxesFontSize',20)
set(0,'DefaultLegendFontSize',20)

% Set LaTeX as default interpreter for axis labels, ticks and legends
set(0,'defaulttextinterpreter','latex')
set(groot,'defaultAxesTickLabelInterpreter','latex');
set(groot,'defaultLegendInterpreter','latex');

addpath('C:\Project-course-145301--main\Project-course-145301--main\forward-backward_new2\forward-backward_new2\matlab');

%% Limits on longitudinal and lateral acceleration and speed
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

% curvilinear abscissa values defining the extrema of each clothoid in a list of clothoids
%s_list = [0,55.7216,95.9378,136.1294,176.5633,216.7047];  % [m]
%s_list = [0,55.7216];
s_list = [0, CL_s]; % [m]
%fprintf("Clothoids : %d\n", s_list);
%fprintf("------------------------------\n\n");

% curvature values defining the extrema of each clothoid in a list of clothoids
%curv_list = [0.0105,-0.0084,0.0104,-0.0180,-0.0073,0.0034];  % [m^-1]
%curv_list = [0.0105,-0.0084];
curv_list = [CL_k_ini, CL_k_fin]; % [m^-1]

% initial speed of the vehicle
%v_ini_clothoid = 2.3614;  % [m/s]

% final speed of the vehicle (at the end of the path)
v_fin = sqrt(Ay_max/CL_k_fin);  % [m/s]

FB = ForwardBackwardList(dataFB);
FB.maxSpeed(v_ini_clothoid,v_fin,s_list,curv_list);
final_time = FB.totalT();  % minimum travelling time (optimal)

cost = final_time;
%fprintf("cost: %f\n",cost);

% % Plot the speed profile
% figure(1)
% FB.plot();
% grid on
% xlabel('curvilinear abscissa s');
% ylabel('v(s)');
% 
% % Plot the curvature profile of the path
% figure(2)
% FB.plotk();
% grid on
% xlabel('s') ;
% ylabel('curv(s)') ;
% 
% % Extract info about the solution
% FB.info();

% Time vector
time_sampling = 0.01;  % [s]
tim_vect = 0:time_sampling:final_time;
curv_absc_vect = FB.s(tim_vect);        % [m] curvilinear abscissa of the solution
speed_profile = FB.vs(curv_absc_vect);  % [m/s] speed profile of the solution

v_ini_clothoid = speed_profile(end);
%fprintf("Initial velocity inside function: %f\n",v_ini_clothoid);

% figure(3)
% plot(tim_vect,speed_profile,'.')
% grid on
% xlabel('time [s]');
% ylabel('speed [m/s]');

end