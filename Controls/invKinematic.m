function [l_v, r_v] = invKinematic(angle, velocity, ROBOT_T, ROBOT_L, WHEEL_R)
% invKinematic Calculate Ackerman Drive Inverse Kinematic
    s2 = sin(angle/2);
    l_v = -velocity .* (1 - s2*ROBOT_T/ROBOT_L) / (2*pi()*WHEEL_R);
    r_v = velocity .* (1 + s2*ROBOT_T/ROBOT_L) / (2*pi()*WHEEL_R);
end

