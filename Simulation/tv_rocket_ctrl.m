

%note you must open it first in excel and save it
rocket_file = 'funnys.csv';
rocket_data = readtable(rocket_file);

data.time_raw = rocket_data.x_Time_s_;

NaN_idx = find(isnan(data.time_raw));
launch_idx = NaN_idx(4)+1; % First time index of when the rocket got launched
apogee_idx = NaN_idx(5)-1;

% Pre-Processed Launch Data
data.time                               = rocket_data.x_Time_s_(launch_idx:apogee_idx);
data.air_pressure                       = rocket_data.AirPressure_mbar_(launch_idx:apogee_idx)*100; % mbar = 100 Pa
data.air_temperature                    = rocket_data.AirTemperature__C_(launch_idx:apogee_idx)+273.15; % Celcius to Kelvin
data.total_velocity                     = rocket_data.TotalVelocity_m_s_(launch_idx:apogee_idx);
data.wind_velocity                      = rocket_data.WindVelocity_m_s_(launch_idx:apogee_idx);
data.thrust                             = rocket_data.Thrust_N_(launch_idx:apogee_idx);
data.CG_location                        = rocket_data.CGLocation_cm_(launch_idx:apogee_idx)/100; %  cm to m
data.reference_length                   = rocket_data.ReferenceLength_cm_(launch_idx:apogee_idx)/100; % cm to m
data.reference_area                     = rocket_data.ReferenceArea_cm__(launch_idx:apogee_idx)/10000; % cm^2 to m^2
data.pitch_moment_coefficient           = rocket_data.PitchMomentCoefficient___(launch_idx:apogee_idx);
data.yaw_moment_coefficient             = rocket_data.YawMomentCoefficient___(launch_idx:apogee_idx);
data.AoA                                = deg2rad(rocket_data.AngleOfAttack___(launch_idx:apogee_idx)); % degrees to radians
data.longitudinal_moment_of_inertia     = rocket_data.LongitudinalMomentOfInertia_kg_m__(launch_idx:apogee_idx);
data.rotational_moment_of_inertia       = rocket_data.RotationalMomentOfInertia_kg_m__(launch_idx:apogee_idx);
data.zenith                             = deg2rad(rocket_data.VerticalOrientation_zenith____(launch_idx:apogee_idx)); % degrees to radians
data.azimuth                            = deg2rad(rocket_data.LateralOrientation_azimuth____(launch_idx:apogee_idx)); % degrees to radians

% Convert into Time Series to Map Data 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if any(isnan(data.air_pressure))
    data.air_pressure                   = fillmissing(data.air_pressure, 'linear', 'EndValues', 'nearest');
end
air_press                               = timeseries(data.air_pressure, data.time); 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if any(isnan(data.air_temperature))
    data.air_temperature                = fillmissing(data.air_temperature, 'linear', 'EndValues', 'nearest');
end
air_temp                                = timeseries(data.air_temperature, data.time); 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if any(isnan(data.total_velocity))
    data.total_velocity                 = fillmissing(data.total_velocity, 'linear', 'EndValues', 'nearest');
end
total_velocity                          = timeseries(data.total_velocity, data.time); 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if any(isnan(data.wind_velocity))
    data.wind_velocity                  = fillmissing(data.wind_velocity, 'linear', 'EndValues', 'nearest');
end
wind_velocity                           = timeseries(data.wind_velocity, data.time);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if any(isnan(data.thrust))
    data.thrust                         = fillmissing(data.thrust, 'linear', 'EndValues', 'nearest');
end
thrust                                  = timeseries(data.thrust, data.time); 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if any(isnan(data.CG_location))
    data.CG_location                    = fillmissing(data.CG_location, 'linear', 'EndValues', 'nearest');
end
CG_location                             = timeseries(data.CG_location, data.time); 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if any(isnan(data.reference_length))
    data.reference_length               = fillmissing(data.reference_length, 'linear', 'EndValues', 'nearest');
end
ref_length                              = timeseries(data.reference_length, data.time); 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if any(isnan(data.reference_area))
    data.reference_area                            = fillmissing(data.reference_area, 'linear', 'EndValues', 'nearest');
end
ref_area                                = timeseries(data.reference_area, data.time); 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if any(isnan(data.pitch_moment_coefficient))
    data.pitch_moment_coefficient       = fillmissing(data.pitch_moment_coefficient, 'linear', 'EndValues', 'nearest');
end
pitch_moment_coeff                      = timeseries(data.pitch_moment_coefficient, data.time);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if any(isnan(data.yaw_moment_coefficient))
    data.yaw_moment_coefficient         = fillmissing(data.yaw_moment_coefficient, 'linear', 'EndValues', 'nearest');
end
yaw_moment_coeff                        = timeseries(data.yaw_moment_coefficient, data.time); 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if any(isnan(data.AoA))
    data.AoA                            = fillmissing(data.AoA, 'linear', 'EndValues', 'nearest');
end
AoA                                     = timeseries(data.AoA, data.time);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if any(isnan(data.longitudinal_moment_of_inertia))
    data.longitudinal_moment_of_inertia = fillmissing(data.longitudinal_moment_of_inertia, 'linear', 'EndValues', 'nearest');
end
long_moment_inert                       = timeseries(data.longitudinal_moment_of_inertia, data.time); 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if any(isnan(data.rotational_moment_of_inertia  ))
    data.rotational_moment_of_inertia   = fillmissing(data.rotational_moment_of_inertia, 'linear', 'EndValues', 'nearest');
end
rot_moment_inert                        = timeseries(data.rotational_moment_of_inertia, data.time);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if any(isnan(data.zenith))
    data.zenith                         = fillmissing(data.zenith, 'linear', 'EndValues', 'nearest');
end
theta_zenith                            = timeseries(data.zenith, data.time); 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if any(isnan(data.azimuth))
    data.azimuth                        = fillmissing(data.azimuth, 'linear', 'EndValues', 'nearest');
end
theta_azimuth                           = timeseries(data.azimuth, data.time); 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
init_pitch_rate                         = timeseries(deg2rad(rocket_data.PitchRate___s_(launch_idx+1)), data.time(launch_idx));
init_yaw_rate                           = timeseries(deg2rad(rocket_data.YawRate___s_(launch_idx+1)), data.time(launch_idx));

init_pitch_angle                        = timeseries(deg2rad(rocket_data.VerticalOrientation_zenith____(launch_idx+1)), data.time(launch_idx));
init_yaw_angle                          = timeseries(deg2rad(rocket_data.LateralOrientation_azimuth____(launch_idx+1)), data.time(launch_idx));

M_aero = ref_area.Data(1)*ref_length.Data(1)*pitch_moment_coeff.Data(1)*sin(AoA.Data(1))*total_velocity.Data(1)^2/2*1.255;
hold on
grid on

plot(data.time, air_temp.Data)
%}

