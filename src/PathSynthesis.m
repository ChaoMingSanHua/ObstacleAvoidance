function [point_x, point_y, path_length] = PathSynthesis(path, path_Pt1, path_Pt2, path_C, path_theta, path_r, s)
path_length = 0;
[rows_Pt1, ~] = size(path_Pt1);
path_length_I = [];
path_length = path_length + norm(path(1, :) - path_Pt1(1, 1:2));
path_length_I = [path_length_I; norm(path(1, :) - path_Pt1(1, 1:2))];
for i = 1:rows_Pt1 - 1
    path_length = path_length + path_theta(i) * path_r(i);
    path_length_I = [path_length_I; path_theta(i) * path_r(i)];
    path_length = path_length + norm(path_Pt2(i, :) - path_Pt1(i+1, :));
    path_length_I = [path_length_I; norm(path_Pt2(i, :) - path_Pt1(i+1, :))];
end
path_length = path_length + path_theta(end) * path_r(end);
path_length_I = [path_length_I; path_theta(end) * path_r(end)];
path_length = path_length + norm(path(end, :) - path_Pt2(end, 1:2)); 
path_length_I = [path_length_I; norm(path(end, :) - path_Pt2(end, 1:2))];

s_length = s * path_length;

[rows_path_length_I, ~] = size(path_length_I);

P_Line = [path(1, :), 0];
[rows_path_Pt1, ~] = size(path_Pt1);
for i = 1 : rows_path_Pt1
    P_Line = [P_Line; path_Pt1(i, :)];
    P_Line = [P_Line; path_Pt2(i, :)];
end
P_Line = [P_Line; path(end, :), 0];

px = nan;
py = nan;

for i = 1 : rows_path_length_I
    if s_length <= sum(path_length_I(1:i))
        sI = (s_length - (sum(path_length_I(1:i)) - path_length_I(i) )) / path_length_I(i);
        if mod(i,2) ~= 0
            [px, py] = LineInterpolation(P_Line(i, :), P_Line(i + 1, :), sI);
            break
        else
            [px, py] = ArcInterpolation(path_C(i / 2, :), P_Line(i, :), P_Line(i + 1, :), sI);
            break
        end
    end
end

point_x = px;
point_y = py;