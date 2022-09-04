function [p, tf] = T_curve(p0, p1, vmax, amax, t)

ta = vmax / amax;
Pa = 0.5 * amax * ta^2;
tm = (p1 - 2 * Pa) / vmax;
tf = tm + 2 * ta;

if tf - 2 * ta > 0
    if t <= ta
        p = p0 + 0.5 * amax * t^2;
    elseif t <= tf - ta
        p = p0 + 0.5 * amax * ta^2 + amax * ta * (t - ta);
    elseif t <= tf
        p = p1 - 0.5 * amax * (tf - t)^2;
    else
        p = p1;
    end
else
    ta = sqrt((p1 - p0) / amax);
    tf = 2 * ta;
    if t <= ta
        p = p0 + 0.5 * amax * t^2;
    elseif t <= tf
        p = p1 - 0.5 * amax * (tf - t)^2;
    else
        p = p1;
    end
end