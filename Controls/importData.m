file_name = "OpenLoop1/encData_A0_V0_2_R0.csv";

ROBOT_T = 0.16;
ROBOT_L = 0.14;
WHEEL_R = 0.0325;

table_data = readtable(file_name);
[l_cmd, r_cmd] = invKinematic(table_data.a, table_data.v, ROBOT_T, ROBOT_L, WHEEL_R);
table_data.l_cmd = l_cmd;
table_data.r_cmd = r_cmd;