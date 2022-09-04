function [px, py] = LineInterpolation(p0, p1, s)
p = p0 + (p1 - p0) * s;
px = p(1);
py = p(2);