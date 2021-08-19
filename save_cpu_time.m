% --------------------------
%% Saving computational time
% --------------------------

clc; clearvars;

% Updated documentation of the clothoid toolbox: https://ebertolazzi.github.io/Clothoids/Matlab_manual.html#clothoidcurve

tic;
cloth_list = ClothoidList; 
elapsed_tim_1 = toc;
fprintf('cpu time to initialize a clothoid list: %.3e ms\n',elapsed_tim_1*1000)

tic;
cloth_curve = ClothoidCurve();
elapsed_tim_2 = toc;
fprintf('cpu time to initialize a clothoid curve: %.3e ms\n',elapsed_tim_2*1000)

x0 = 1;       x1 = 3;
y0 = 0.5;     y1 = 2;
theta0 = 0;   theta1 = pi/6;
tic;
cloth_curve.build_G1(x0, y0, theta0, x1, y1, theta1);
elapsed_tim_3 = toc;
fprintf('cpu time to build a clothoid curve: %.3e ms\n',elapsed_tim_3*1000)

% IMPORTANT: initialize a clothoid curve / list only ONCE, at the beginning of the simulation, and then use 
%            the build_G1 command to redefine the clothoid, without re-initializing it. This is useful when 
%            doing the operations to choose the parent node and rewire the graph
tic;
cloth_curve.build_G1(x0+2, y0, theta0, x1, y1, theta1);  % redefine the clothoid without reinitializing it 
elapsed_tim_3 = toc;
fprintf('cpu time to build a clothoid curve: %.3e ms\n\n\n',elapsed_tim_3*1000)

% The same idea applies to LineSegment objects:
% Initialize it only ONCE
tic;
LinSegm = LineSegment();
elapsed_tim_4 = toc;
fprintf('cpu time to initialize a line segment: %.3e ms\n',elapsed_tim_4*1000)

% And then redefine it without re-initializing it
tic;
LinSegm.build([x0, y0], [x1, y1]);
elapsed_tim_5 = toc;
fprintf('cpu time to build a line segment: %.3e ms\n',elapsed_tim_5*1000)
