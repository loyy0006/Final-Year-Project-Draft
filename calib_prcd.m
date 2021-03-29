% This function generates a vector [Fx Fy Fz Tx Ty Tz] according to user
% input. Using predetermined values defined by lookup table. The procedure
% is detailed in report.
%
% lookupOrient: Struct holding the Cardan angle rotation values required to
%   rotate the ATI's coordinate system to world (Z-positive vertically
%   upward).
% lookupMass: Struct holding the Center-of-mass magnitude and position
%   relative to ATI's coordinate system.
%
function [ft, desc] = calib_prcd()
    %% lookup tables
    lookupOrient = struct('name', {'xp','xm','yp','ym','zp','zm'}, ...
        ...               cardan angle rotation in degrees [Rx Ry Rz] (Ati CS)
                          'rotation', {[  0   90	0], ... vertical axis X-positive
                                       [  0  -90	0], ... vertical axis X-negative
                                       [-90    0	0], ... vertical axis Y-positive
                                       [ 90    0	0], ... vertical axis Y-negative
                                       [  0    0	0], ... vertical axis Z-positive
                                       [180    0	0]}); % vertical axis Z-negative

    lookupMass = struct('name', {'none','plate','yp50','ym50','xp50','xm50','yp120','ym120','xp120','xm120'}, ...
                        'mass', {0, 108.2516876, ... % Total mass w/o plate and w plate. (g)
                            164.9995609, 164.9995609, 164.9995609, 164.9995609, ... % w 50gm brass wgt
                            238.1662863, 238.1662863, 238.1662863, 238.1662863}, ... % w 120gm brass wgt
                        'pos', {[0               0               0], ...            Center of mass position (mm)
                                [0               0               2.50198806], ...   plate
                                [0.00053426      13.54914618     4.83694467], ...   yp50
                                [0.00053426      -13.96504745	4.83694467], ...    ym50
                                [13.75763107     -0.20795064     4.83694467], ...   xp50
                                [-13.75656256	-0.20795064     4.83694467], ...    xm50
                                [0.00087552      21.47217866     9.7147455], ...    yp120
                                [0.00087552      -22.16610474	9.7147455], ...     ym120
                                [21.82001722     -0.34696304     9.7147455], ...    xp120
                                [-21.81826618	-0.34696304     9.7147455]});%      xm120

    %% ask for orientation
    listofnames = '';
    for i = 1:length(lookupOrient)
        listofnames = strcat(listofnames, lookupOrient(i).name, ', ');
    end
    fprintf('ATI orientation list: %s\n', listofnames);
    str = input('Enter orientation: ', 's');
    desc = [str '_'];
    % try find orientation from lookup table; error if not found.
    flag = 0;
    for i = 1:length(lookupOrient)
        if strcmpi(str, lookupOrient(i).name)
            flag = 1;
            break
        end
    end
    if flag == 0
        disp('Error: could not find specified orientation from list.');
        return
    else
        rotation = lookupOrient(i).rotation / 180 * pi; % rotation in radians 
    end

    %% ask for mass setting
    listofnames = '';
    for i = 1:length(lookupMass)
        listofnames = strcat(listofnames, lookupMass(i).name, ', ');
    end
    fprintf('Mass setting list: %s\n', listofnames);
    str = input('Enter setting: ', 's');
    desc = [desc str];
    % try find orientation from lookup table; error if not found.
    flag = 0;
    for i = 1:length(lookupMass)
        if strcmpi(str, lookupMass(i).name)
            flag = 1;
            break
        end
    end
    if flag == 0
        disp('Error: could not find specified mass setting from list.');
        return
    else
        mass = lookupMass(i).mass * 0.001; % mass in kg
        pos = lookupMass(i).pos   * 0.001; % position in m
    end

    %% calculate expected F/T
    % Force
    F = [0 0 mass*-9.80665]'; % gravitational force in world coordinate
    R = eul2rotm(rotation,'XYZ'); % generate rotation matrix from Rx Ry Rz
    F = R*F; % gravitational force in ATI coordinate.
    F = F'; % transpose to 1x3 row vector
    
    % Torque
    T = cross(pos, F);
    
    ft = [F T];
end