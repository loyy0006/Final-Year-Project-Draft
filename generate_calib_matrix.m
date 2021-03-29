% This script uses Multiple linear regression, least squares method to
% obtain the calibration matrix and beta constants for a 6-axis loadcell.
% Please refer to report for more details. 
%
% calib_struct is a struct array that holds the following elements:
% calib_struct.desc (char array) - short description of point
% calib_struct.expt (1x7 matrix) - [1 SG0 SG1 ... SG5] measured voltages
% calib_struct.calc (1x6 matrix) - [Fx Fy Fz Tx Ty Tz] expected F/T reading
%   from calculation using Solidworks model data.
clear X Y beta

% check if calib_struct exists
if ~exist('calib_struct', 'var')
    disp('Error: variable calib_struct is missing. Please collect data first using save_point.m .');
    return
end

% initialise matrices
X = zeros(7, length(calib_struct));
X = X';
Y = zeros(6, length(calib_struct));
Y = Y'

% populate
for i = 1:length(calib_struct)
    X(i,:) = calib_struct(i).expt;
    Y(i,:) = calib_struct(i).calc;
end

%solve
beta = inv(X' * X) * X' * Y;
calib_matrix = beta(2:7, :)';
intercepts = beta(1, :)';
%% completed.
% The formula to convert the loadcell's voltage measurement to F/T is
% FT = intercepts + calib_matrix * [voltage]
