function RobotPlot(point_X, point_Y, obstacleList)
a2 = 0; a3 = 10; a4 = 10;
d1 = 0;
ToolX = 5;
Alpha = [0, pi/2, 0, 0];
A = [0, a2, a3, a4];
D = [d1, 0, 0, 0];
Theta = [0, 0, 0, 0];
Tool = [ToolX, 0, 0];
DH_para.Alpha = Alpha;
DH_para.A = A;
DH_para.D = D;
DH_para.Theta = Theta;
DH_para.Tool = Tool;

L(1) = Link('alpha',Alpha(1),'a', A(1),'d',D(1));L(1).offset = Theta(1); L(1).qlim=[-pi, pi];L(1).mdh = 1;
L(2) = Link('alpha',Alpha(2),'a', A(2),'d',D(2));L(2).offset = Theta(2); L(2).qlim=[-pi, pi];L(2).mdh = 1;
L(3) = Link('alpha',Alpha(3),'a', A(3),'d',D(3));L(3).offset = Theta(3); L(3).qlim=[-pi, pi];L(3).mdh = 1;
L(4) = Link('alpha',Alpha(4),'a', A(4),'d',D(4));L(4).offset = Theta(4); L(4).qlim=[-pi, pi];L(4).mdh = 1;

robot = SerialLink(L, 'name', 'robot');
robot.tool = SE3(Tool);

qe = [];
[rows_point_X, ~] = size(point_X);
for i = 1:rows_point_X
    qee = IKine(point_X(i), point_Y(i), 0);
    qe = [qe; qee];
end
figure(4);

[rows_qe, ~] = size(qe);
[rows_obstacleList, ~] = size(obstacleList);
for i = 1:rows_qe
    robot.plot(qe(i, :));
    plot(point_X, point_Y,  'g');
    hold on;
    for j = 1:rows_obstacleList
        plot_circle(obstacleList(j, 1:2), obstacleList(j, 3), 'r')
        hold on;
    end
    view([55, 35])
    zlim([-30, 30]);   
end
