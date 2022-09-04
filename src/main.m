close all; clear; clc;

obstacleList = [3, 3, 1.5;
    12, 2, 3;
    3, 9, 2;
    9, 11, 2;
    7, 6, 2;
    14, 8, 2];
randArea = [-2, 18];
expandDis = 2.0;
goalSampleRate = 10;
maxIter = 200;
start = [0, 0];
goal = [15, 12];

figure(1);
[rows_obstacleList, ~] = size(obstacleList);
for i = 1:rows_obstacleList
    viscircles(obstacleList(i, [1,2]), obstacleList(i, 3));
    hold on;
end
scatter(start(1), start(2), 'black');
text(start(1), start(2)+0.5, 'start');
hold on;
scatter(goal(1), goal(2), 'b');
text(goal(1), goal(2)-0.5, 'goal');
hold on;
axis equal

rrt = RRT(obstacleList, randArea, expandDis, goalSampleRate, maxIter);

path = rrt.rrt_planning(start, goal);

figure(2);
[rows_obstacleList, ~] = size(obstacleList);
for i = 1:rows_obstacleList
    viscircles(obstacleList(i, [1,2]), obstacleList(i, 3));
    hold on;
end
scatter(start(1), start(2), 'black');
hold on;
scatter(goal(1), goal(2), 'b');
hold on;
plot(path(:, 1), path(:, 2), 'g')
axis equal


[rows_path, ~] = size(path);
path_real = [];
for i = 0 : rows_path-1
    path_real = [path_real; path(end - i, :)];
end
path_Pt1 = [];
path_Pt2 = [];
path_C = [];
path_theta = [];
path_r = [];
path_d1 = [];
path_d2 = [];
for i = 2 : rows_path - 1
    P0 = [path_real(i - 1, :), 0];
    P1 = [path_real(i, :), 0];
    P2 = [path_real(i + 1, :), 0];
    r = 0.5;
    t = 0;
    [Pt1, Pt2, d1, d2, C, theta, r] = PathSegmentTrans(P0, P1, P2, r, t);
    path_Pt1 = [path_Pt1; Pt1];
    path_Pt2 = [path_Pt2; Pt2];
    path_C = [path_C; C];
    path_theta = [path_theta; theta];
    path_r = [path_r; r];
    path_d1 = [path_d1; d1];
    path_d2 = [path_d2; d2];
end

point_X = [];
point_Y = [];
for s = 0:0.001:1
    [point_x, point_y] = PathSynthesis(path_real, path_Pt1, path_Pt2, path_C, path_theta, path_r, s);
    point_X = [point_X; point_x];
    point_Y = [point_Y; point_y];
end
figure(3);
for i = 1:rows_obstacleList
    viscircles(obstacleList(i, [1,2]), obstacleList(i, 3));
    hold on;
end
scatter(start(1), start(2), 'black');
hold on;
scatter(goal(1), goal(2), 'b');
hold on;
plot(point_X, point_Y, 'g')
axis equal

p0 = 0;
p1 = 1;
vmax = 10;
amax = 10;
deltaT = 0.05;
[point_x, point_y, path_length] = PathSynthesis(path_real, path_Pt1, path_Pt2, path_C, path_theta, path_r, 0);
vhatmax = vmax / path_length;
ahatmax = amax / path_length;
[~, tf] = T_curve(p0, p1, vhatmax, ahatmax, 0);
t = 0:deltaT:ceil(tf);
point_X = [];
point_Y = [];
for i = 1:length(t)
    p = T_curve(p0, p1, vhatmax, ahatmax, t(i));
    [point_x, point_y] = PathSynthesis(path_real, path_Pt1, path_Pt2, path_C, path_theta, path_r, p);
    point_X = [point_X; point_x];
    point_Y = [point_Y; point_y];
end

RobotPlot(point_X, point_Y, obstacleList);