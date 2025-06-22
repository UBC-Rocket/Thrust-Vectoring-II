data_file = "rocket_sim_data.csv"; % path to file
data = readtable(data_file);

time = data.x_Time_s_;
thrust = timeseries(data.Thrust_N_, time);
drag_force = timeseries(data.DragForce_N_, time);