% ------------------------------------------------------------------------
%% Evaluate intersections of a clothoid with other clothoids and segments
% ------------------------------------------------------------------------

clc; clearvars;

set(0,'DefaultFigureWindowStyle','docked');
set(0,'defaultAxesFontSize',20)
set(0,'DefaultLegendFontSize',20)

set(0,'defaulttextinterpreter','latex')
set(groot, 'defaultAxesTickLabelInterpreter','latex'); 
set(groot, 'defaultLegendInterpreter','latex');

% Updated documentation of the clothoid toolbox: https://ebertolazzi.github.io/Clothoids/Matlab_manual.html#clothoidcurve

% -----------------------------------------------
%% Case 1: check intersection between 2 clothoids
% -----------------------------------------------

% -----------------
% Initialize and build two clothoid curves
% -----------------
cloth_curve_1 = ClothoidCurve();
cloth_curve_2 = ClothoidCurve();

% Build the 1st clothoid
x0 = 1;       x1 = 3;
y0 = 0.5;     y1 = 2;
theta0 = 0;   theta1 = pi/6;
cloth_curve_1.build_G1(x0, y0, theta0, x1, y1, theta1);
s_vect_1 = 0:1e-2:cloth_curve_1.length();   % curvilinear abscissae at which the (x,y) coords are evaluated
[x_vect_1,y_vect_1] = cloth_curve_1.evaluate(s_vect_1);

% Build the 2nd clothoid 
x2 = 0;       x3 = 3;
y2 = 1.5;     y3 = 1.4;
theta2 = 0;   theta3 = -pi/6;
cloth_curve_2.build_G1(x2, y2, theta2, x3, y3, theta3);
s_vect_2 = 0:1e-2:cloth_curve_2.length();   % curvilinear abscissae at which the (x,y) coords are evaluated
[x_vect_2,y_vect_2] = cloth_curve_2.evaluate(s_vect_2);

% Check the intersection
tic;
s_inters_1 = cloth_curve_1.intersect(cloth_curve_2);
elapsed_tim_1 = toc;
fprintf('cpu time to find the intersection between 2 clothoids: %.3e ms\n',elapsed_tim_1*1000)
if (isempty(s_inters_1))
    % this means that no intersection exists
else
    [x_inters_1,y_inters_1] = cloth_curve_1.evaluate(s_inters_1);
end

% -----------------
% Plot the 2 clothoids
% -----------------
figure('Name','2 clothoids','NumberTitle','off'), clf   
hold on
plot(x_vect_1,y_vect_1,'LineWidth',2)
plot(x_vect_2,y_vect_2,'LineWidth',2)
if (~isempty(s_inters_1))
    plot(x_inters_1,y_inters_1,'go','MarkerFaceColor','g','MarkerSize',18)
end
grid on
legend('clothod 1','clothoid 2','location','best')

% -----------------------------------------------
%% Case 2: check intersection between a clothoid list and a clothoid
%  USE CASE: check intersection of a generated clothoid segment with the 
%            road boundaries, which are defined as a clothoid list
% -----------------------------------------------

clc; clearvars;

% -----------------
% Initialize and build a clothoid curve and a clothoid list
% -----------------
cloth_curve = ClothoidCurve();
cloth_list  = ClothoidList();

% Build the clothoid curve
x0 = 1;       x1 = 3;
y0 = 0.5;     y1 = 2;
theta0 = 0;   theta1 = pi/6;
cloth_curve.build_G1(x0, y0, theta0, x1, y1, theta1);
s_vect_1 = 0:1e-2:cloth_curve.length();   % curvilinear abscissae at which the (x,y) coords are evaluated
[x_vect_1,y_vect_1] = cloth_curve.evaluate(s_vect_1);

% Build the clothoid list
x2 = 0;       x3 = 3;           x4 = 4; 
y2 = 1.5;     y3 = 1.4;         y4 = 2;
theta2 = 0;   theta3 = -pi/6;   theta4 = pi/3;
cloth_list.push_back_G1(x2,y2,theta2, x3,y3,theta3);
cloth_list.push_back_G1(x4,y4,theta4);
s_vect_2 = 0:1e-2:cloth_list.length();   % curvilinear abscissae at which the (x,y) coords are evaluated
[x_vect_2,y_vect_2] = cloth_list.evaluate(s_vect_2);

% Check the intersection
s_inters_1 = cloth_curve.intersect(cloth_list);
if (isempty(s_inters_1))
    % this means that no intersection exists
else
    [x_inters_1,y_inters_1] = cloth_curve.evaluate(s_inters_1);
end

% -----------------
% Plot the 2 clothoids
% -----------------
figure('Name','clothoid curve + clothoid list','NumberTitle','off'), clf   
hold on
plot(x_vect_1,y_vect_1,'LineWidth',2)
plot(x_vect_2,y_vect_2,'LineWidth',2)
if (~isempty(s_inters_1))
    plot(x_inters_1,y_inters_1,'go','MarkerFaceColor','g','MarkerSize',18)
end
grid on
legend('clothod curve','clothoid list','location','best')

% -----------------------------------------------
%% Case 3: check intersection between a clothoid (or clothoid list) and a line segment
%  USE CASE: check intersection of a generated clothoid segment with another 
%            vehicle (represented as a rectangle)
% -----------------------------------------------

clc; clearvars;

% -----------------
% Initialize and build a clothoid curve and line segments
% -----------------
cloth_curve = ClothoidCurve();
line_segm_1 = LineSegment();
line_segm_2 = LineSegment();
line_segm_3 = LineSegment();

% Build the clothoid curve
x0 = 1;       x1 = 3;
y0 = 0.5;     y1 = 2;
theta0 = 0;   theta1 = pi/6;
cloth_curve.build_G1(x0, y0, theta0, x1, y1, theta1);
s_vect_1 = 0:1e-2:cloth_curve.length();   % curvilinear abscissae at which the (x,y) coords are evaluated
[x_vect_1,y_vect_1] = cloth_curve.evaluate(s_vect_1);

% Build the line segments
line_segm_1.build([2.2,1.5],[2.8,1.5]);
line_segm_2.build([2.2,1.0],[2.2,1.5]);
line_segm_3.build([2.2,1.0],[2.8,1.0]);
s_vect_line_1 = 0:1e-2:line_segm_1.length();   % curvilinear abscissae at which the (x,y) coords are evaluated
s_vect_line_2 = 0:1e-2:line_segm_2.length();
s_vect_line_3 = 0:1e-2:line_segm_3.length();
[x_vect_line_1,y_vect_line_1] = line_segm_1.evaluate(s_vect_line_1);
[x_vect_line_2,y_vect_line_2] = line_segm_2.evaluate(s_vect_line_2);
[x_vect_line_3,y_vect_line_3] = line_segm_3.evaluate(s_vect_line_3);

% Check the intersection
s_inters_line_1 = cloth_curve.intersect(line_segm_1);
s_inters_line_2 = cloth_curve.intersect(line_segm_2);
s_inters_line_3 = cloth_curve.intersect(line_segm_3);
if (~isempty(s_inters_line_1))
    [x_inters_line_1,y_inters_line_1] = cloth_curve.evaluate(s_inters_line_1);
end
if (~isempty(s_inters_line_2))
    [x_inters_line_2,y_inters_line_2] = cloth_curve.evaluate(s_inters_line_2);
end
if (~isempty(s_inters_line_3))
    [x_inters_line_3,y_inters_line_3] = cloth_curve.evaluate(s_inters_line_3);
end

% -----------------
% Plot the clothoid and the line segments
% -----------------
figure('Name','clothoid + line segments','NumberTitle','off'), clf   
hold on
plot(x_vect_1,y_vect_1,'LineWidth',2)
plot(x_vect_line_1,y_vect_line_1,'-r','LineWidth',2)
plot(x_vect_line_2,y_vect_line_2,'-r','LineWidth',2)
plot(x_vect_line_3,y_vect_line_3,'-r','LineWidth',2)
if (~isempty(s_inters_line_1))
    plot(x_inters_line_1,y_inters_line_1,'go','MarkerFaceColor','g','MarkerSize',18)
end
if (~isempty(s_inters_line_2))
    plot(x_inters_line_2,y_inters_line_2,'go','MarkerFaceColor','g','MarkerSize',18)
end
if (~isempty(s_inters_line_3))
    plot(x_inters_line_3,y_inters_line_3,'go','MarkerFaceColor','g','MarkerSize',18)
end
grid on
legend('clothoid','obstacle vehicle','location','best')

% IMPORTANT: an obstacle vehicle can be represented using 3 line segments, for the
% rear edge of the car, the right edge and the left edge (the front edge can be
% neglected to save cpu time). If the obstacle is moving, initialize the 3
% line segments only once, and then update (i.e. redefine) their coordinates with the
% build method, without reinitializing them

