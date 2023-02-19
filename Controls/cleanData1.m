file_name = "encData_A0_V1_R1.csv";

table_data = readtable(file_name);

left_diff = diff(table_data.l);
left_filter_index = find(left_diff, 1);
table_data.l(1:left_filter_index) = 0.0;
table_data.l = abs(table_data.l);

table_data.l_cmd(1:left_filter_index-1) = 0.0;
table_data.l_cmd(left_filter_index-1:end) = left_vel;

right_diff = diff(table_data.r);
right_filter_index = find(right_diff, 1);
table_data.r(1:right_filter_index) = 0.0;
table_data.r = abs(table_data.r);

table_data.r_cmd(1:right_filter_index-1) = 0.0;
table_data.r_cmd(right_filter_index-1:end) = right_vel;

ts_left_vel = timeseries(table_data.l, table_data.t_s_);
ts_left_cmd = timeseries(table_data.l_cmd, table_data.t_s_);
ts_right_vel = timeseries(table_data.r, table_data.t_s_);
ts_right_cmd = timeseries(table_data.r_cmd, table_data.t_s_);