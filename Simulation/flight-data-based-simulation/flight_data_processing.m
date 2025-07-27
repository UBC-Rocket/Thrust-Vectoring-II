
csv_file = "flight_data_processed.csv"; % open csv file
data = readtable(csv_file); % inserts the file into table in matlab

time = data.Time;
accel_x = data.accelX; accel_y = data.accelY; accel_z = data.accelZ;
gyro_x = data.gyroX; gyro_y = data.gyroY; gyro_z = data.gyroZ;
data_size = size(time);

KalmanAngleRoll = zeros(data_size); KalmanUncertaintyAngleRoll = zeros(data_size); KalmanUncertaintyAngleRoll(1) = 4;
KalmanAnglePitch = zeros(data_size); KalmanUncertaintyAnglePitch = zeros(data_size); KalmanUncertaintyAnglePitch(1) = 4;
dt = 0.004;

% Kalman Angle Filtering
function [KalmanAngle, KalmanAngleUncertainty] = kalman1d(KalmanState, KalmanUncertainty, KalmanInput, KalmanMeasurement)
  dt = 0.004;
  KalmanState = KalmanState + dt * KalmanInput;
  KalmanUncertainty = KalmanUncertainty + dt * dt * 16; 
  
  KalmanGain = KalmanUncertainty / (KalmanUncertainty + 9);
  KalmanState = KalmanState + KalmanGain * (KalmanMeasurement - KalmanState);
  KalmanUncertainty = (1 - KalmanGain) * KalmanUncertainty;

  KalmanAngle = KalmanState;
  KalmanAngleUncertainty = KalmanUncertainty;

end
%{
% gimbal angle to servo position mapping (microseconds for servo PWM)
function YPos = servoY_PWM(gimbalY) 
  YPos = 1453 - 39.3 * gimbalY + 0.464 * gimbalY^2;
end

%}

function XPos = servoX_PWM(gimbalX) 
  XPos = 1373 + 48.6 * gimbalX - 0.288 * pow(gimbalX, 2);
end

AccAngleRoll = ones(data_size);
AccAnglePitch = ones(data_size);

for idx = 1:length(time)
    AccAngleRoll(idx) = atan(accel_y(idx)/ sqrt(accel_z(idx) * accel_z(idx) + accel_x(idx) * accel_x(idx))) * 180.0 / pi;
    AccAnglePitch(idx) = -atan(accel_z(idx) / sqrt(accel_y(idx) * accel_y(idx) + accel_x(idx) * accel_x(idx))) * 180.0 / pi;
    [KalmanAngleRoll(idx), KalmanUncertaintyAngleRoll(idx)]=kalman1d(KalmanAngleRoll(idx), KalmanUncertaintyAngleRoll(idx), gyro_z(idx), AccAngleRoll(idx));
    [KalmanAnglePitch(idx), KalmanUncertaintyAnglePitch(idx)]=kalman1d(KalmanAnglePitch(idx), KalmanUncertaintyAnglePitch(idx), gyro_y(idx), AccAnglePitch(idx));

end



