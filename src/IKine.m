function q = IKine(px, py, pz)

a2 = 10; a3 = 10; a4 = 5;
pz = pz + a4;

a0 = sqrt(px^2 + py^2 + pz^2);
q0 = atan2(pz/a0, sqrt(px^2 + py^2)/a0);

q2 = acos((px^2 + py^2 + pz^2 + a2^2 - a3^2) / (2 * a0 * a2)) + q0;
q3 = -acos((px^2 + py^2 + pz^2 + a3^2 - a2^2) / (2 * a0 * a3)) + q0 - q2;

q1 = atan2(py / (a3*cos(q2 + q3) + a4*cos(q2 + q3) + a2*cos(q2)), px / (a3*cos(q2 + q3) + a4*cos(q2 + q3) + a2*cos(q2)));
q4 =  - q2 - q3 - pi/2;

q = [q1, q2, q3, q4];