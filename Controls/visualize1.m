importData

figure
hold on
title("Response")
plot(table_data.t_s_, table_data.l)
plot(table_data.t_s_, table_data.l_cmd)

plot(table_data.t_s_, table_data.r)
plot(table_data.t_s_, table_data.r_cmd)

legend("Left Response", "Left Command", "Right Response", "Right Command")