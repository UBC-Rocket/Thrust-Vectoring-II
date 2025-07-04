data_file = "rocket_sim_data.csv"; % path to file
data = readtable(data_file);

time = data.x_Time_s_;
thrust = timeseries(data.Thrust_N_, time);

mass = 1.38; % kg
g = 9.81; % m/s^2
cog = 0.4025; % m
cog_to_mount = 0.20; % m
str_len = 2.1; % m
osc_period = mean([50/15 51.91/15 33.56/10]);
mmoi = mass*g*(cog_to_mount)^2*(osc_period)^2/(4*pi^2*str_len);