global ROBOT_T
global ROBOT_L
global WHEEL_R

ROBOT_T = 0.16;
ROBOT_L = 0.14;
WHEEL_R = 0.0325;

angle = 0;
velocity = 1;

[left_vel, right_vel] = ik(angle, velocity);
display(left_vel);
display(right_vel);

function [l_v, r_v] = ik(angle, velocity)
    global ROBOT_T
    global ROBOT_L
    global WHEEL_R
    s2 = sin(angle/2);
    l_v = velocity * (1 - s2*ROBOT_T/ROBOT_L) / (2*pi()*WHEEL_R);
    r_v = velocity * (1 + s2*ROBOT_T/ROBOT_L) / (2*pi()*WHEEL_R);
end