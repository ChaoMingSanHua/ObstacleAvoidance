function [Pt1, Pt2, d1, d2, C, theta_m, r] = PathSegmentTrans(P0, P1, P2, r, t)
P1P0 = sqrt(power(P0(1) - P1(1), 2) + power(P0(2) - P1(2), 2) + power(P0(3) - P1(3), 2));
P1P2 = sqrt(power(P2(1) - P1(1), 2) + power(P2(2) - P1(2), 2) + power(P2(3) - P1(3), 2));
vec_P1P0 = [P0(1) - P1(1), P0(2) - P1(2), P0(3) - P1(3)];
vec_P1P2 = [P2(1) - P1(1), P2(2) - P1(2), P2(3) - P1(3)];
theta = acos((dot(vec_P1P0, vec_P1P2)) / (P1P0*P1P2));
if abs(abs(theta) - pi) < 1e-6
    Pt1 = P1;
    Pt2 = P1;
    d1 = sqrt(power(Pt1(1) - P0(1), 2) + power(Pt1(2) - P0(2), 2) + power(Pt1(3) - P0(3), 2));
    d2 = 0;
    theta_m = 0;
    C = P1;
    r = 0;
    return 
end
vec_P1Pt1 = (r/tan(theta/2)/P1P0) * vec_P1P0;
vec_P1Pt2 = (r/tan(theta/2)/P1P2) * vec_P1P2;
Pt1 = P1 + vec_P1Pt1;
Pt2 = P1 + vec_P1Pt2;
d1 = sqrt(power(Pt1(1) - P0(1), 2) + power(Pt1(2) - P0(2), 2) + power(Pt1(3) - P0(3), 2));
d2 = (pi - theta) * r;
theta_m = pi - theta;

vec_Pt1M = (1/2) * (Pt2 - Pt1);
M = Pt1 + vec_Pt1M;
vec_P1M = M - P1;
P1M = sqrt(power(M(1) - P1(1), 2) + power(M(2) - P1(2), 2) + power(M(3) - P1(3), 2));
P1C = r / sin(theta/2);
vec_P1C = (P1C / P1M) * vec_P1M;
C = P1 + vec_P1C;
end