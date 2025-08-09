clear all;
clc;
data_file = "rocket_sim_data.csv"; % path to file
data = readtable(data_file);

time = data.x_Time_s_;
thrust = timeseries(data.Thrust_N_, time);
mass = 1.17; % kg
g = 9.81; % m/s^2
cog = convlength(16, 'in', 'm'); % m
cog_to_mount = convlength(12, 'in', 'm'); % m
str_len = convlength(83+7/16, 'in', 'm'); % m
osc_period = 30/8.8;
mmoi = mass*g*(cog_to_mount)^2*(osc_period)^2/(4*pi^2*str_len);

flight_data_file = "flight_data_03-08-2025.csv";
flight_data = readtable(flight_data_file);

outputX = flight_data.InputX; % "InputX" is the Kalmann filtered angle
inputX = flight_data.OutputX; % "OutputX" is the PID control factor 

outputY = flight_data.InputY;
inputY = flight_data.OutputY;
time = flight_data.Time_ms_;
time_step = (time(end) - time(2))/length(time);
Ts = 50/1000;
for ind = 1:length(time)
    time(ind) = ind*Ts;
end

sysX = ssest(inputX, outputX, 8, 'Ts', Ts);
sysY = ssest(inputY, outputY, 8, 'Ts', Ts);

tferX = idtf(sysX);
tferY = idtf(sysY);

input_x_vals = timeseries(inputX, time);
output_x_vals = timeseries(outputX, time);
input_y_vals = timeseries(inputY, time);
output_y_vals = timeseries(inputY, time);


Kpx=4;
Kix=0.03;
Kdx=0.7;

Kpy=4;
Kiy=0.03;
Kdy=0.7;

pid_ctrlx = pid(Kpx, Kix, Kdx, 'Ts', Ts);
pid_ctrly = pid(Kpy, Kiy, Kdy, 'Ts', Ts);

opentfX = series(pid_ctrlx, tferX);
opentfY = series(pid_ctrly, tferY);
figure; 
bode(opentfX)
margin(opentfX)
figure;
bode(opentfY)
margin(opentfY)
grid on;


%{
Ts = 0.05;
tferX = tf([-0.05353, 0.009934], [1, -0.1191, -0.7466], Ts);
opts = pidtuneOptions('PhaseMargin', 45, 'CrossoverFrequency', 2);
[C, info] = pidtune(tferX, 'pid', opts);
disp(C);
disp(info);

%}


% 0.02, 0.002, 0.0055 very stable but no move