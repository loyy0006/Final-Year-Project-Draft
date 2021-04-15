%% Generation of calibration matrix for ATI F/T sensor

% _________________________________________________________________________
% This functin generates the calibration matrix for ATI sensors based on
% the calibration matrix from the CD that comes with the sensor.
% Input: None.
% Output: 6x6 calibration matrix for converting the voltage data to
% force-torque data. 
% Generate the calibration matrix using ATI_CalMat.m file.
% Adapted from: Rakhi Agarwal
% Created on: 11 April 2021
% Usage note: Select the appropriate calibration matrix and scales CD data 
% for ATI Nao17, Mini40 or Nano43 while commenting the rest.
% _________________________________________________________________________


function CalMatrix = ATI_CalMat()

%% Calibration data and scales from CD - Nano17

% <?xml version="1.0" encoding="utf-8"?>
% <!-- NOTE: To ensure compatibility between your software and future F/T calibrations -->
% <!-- (such as recalibrations of your transducer or future purchases),                -->
% <!-- ATI does not support parsing of this file.  The only supported methods for      -->
% <!-- loading calibration data are the ATIDAQFT ActiveX component, the ATI DAQ F/T C  -->
% <!-- Library, and the ATICombinedDAQFT .NET Assembly.                                -->
% <FTSensor  Serial="FT11564" BodyStyle="Nano17" Family="DAQ" NumGages="6" CalFileVersion="1.0">
% 	<Calibration  PartNumber="SI-50-0.5" CalDate="10/21/2011" ForceUnits="N" TorqueUnits="N-mm" DistUnits="mm" OutputMode="Ground Referenced Differential" OutputRange="20" HWTempComp="True" GainMultiplier="1" CableLossDetection="False" OutputBipolar="True">
% 		<Axis Name="Fx" values=" -0.10881   0.43873  -0.47052 -38.68508  -0.22449  38.39309 " max="50" scale="5.655108226808"/>
% 		<Axis Name="Fy" values="  1.97590  44.10168  -0.07271 -22.10090  -0.07648 -22.00380 " max="50" scale="5.655108226808"/>
% 		<Axis Name="Fz" values=" 22.35086  -0.41711  22.03651  -0.27066  22.21187  -0.10971 " max="70" scale="2.93250301081869"/>
% 		<Axis Name="Tx" values=" -0.63206   0.19812  38.34624  -0.33018 -38.00888   0.04101 " max="500" scale="0.907070482578282"/>
% 		<Axis Name="Ty" values="-44.30528   0.66569  21.89791   0.22289  21.71848  -0.52191 " max="500" scale="0.907070482578282"/>
% 		<Axis Name="Tz" values="  0.87368  22.05490   0.17324  21.46599  -0.03097  22.41739 " max="500" scale="0.783306456805324"/>
% 		<BasicTransform Dx="0" Dy="0" Dz="6.096" Rx="0" Ry="0" Rz="0"/>
% 	</Calibration>
% </FTSensor>


% C = [-0.10881	0.43873	-0.47052	-38.68508	-0.22449	38.39309;
%     1.9759	44.10168	-0.07271	-22.1009	-0.07648	-22.0038;
%     22.35086	-0.41711	22.03651	-0.27066	22.21187	-0.10971;
%     -0.63206	0.19812	38.34624	-0.33018	-38.00888	0.04101;
%     -44.30528	0.66569	21.89791	0.22289	21.71848	-0.52191;
%     0.87368	22.0549	0.17324	21.46599	-0.03097	22.41739];
% Scales = [5.655108226808 5.655108226808 2.93250301081869 0.907070482578282 0.907070482578282 0.783306456805324];
% Dx = 0; Dy = 0; Dz = 6.096; Rx = 0; Ry = 0; Rz = 0;

%% Calibration data and scales from CD - Mini40
% 
% % <?xml version="1.0" encoding="utf-8"?>
% % <!-- NOTE: To ensure compatibility between your software and future F/T calibrations -->
% % <!-- (such as recalibrations of your transducer or future purchases),                -->
% % <!-- ATI does not support parsing of this file.  The only supported methods for      -->
% % <!-- loading calibration data are the ATIDAQFT ActiveX component, the ATI DAQ F/T C  -->
% % <!-- Library, and the ATICombinedDAQFT .NET Assembly.                                -->
% % <FTSensor  Serial="FT12288" BodyStyle="Mini40" Family="DAQ" NumGages="6" CalFileVersion="1.0">
% % 	<Calibration  PartNumber="SI-80-4" CalDate="4/5/2012" ForceUnits="N" TorqueUnits="N-m" DistUnits="m" OutputMode="Ground Referenced Differential" OutputRange="20" HWTempComp="True" GainMultiplier="1" CableLossDetection="False" OutputBipolar="True">
% % 		<Axis Name="Fx" values="  0.49669  -0.30752  -0.67526  31.58401  -0.52567 -32.13403 " max="80" scale="2.60913047446393"/>
% % 		<Axis Name="Fy" values="  0.91512 -37.57532  -0.01988  17.91305   0.26027  18.99891 " max="80" scale="2.60913047446393"/>
% % 		<Axis Name="Fz" values=" 18.30062  -0.22759  18.41379  -0.40968  18.58899  -0.15117 " max="240" scale="0.88615981830186"/>
% % 		<Axis Name="Tx" values="  0.26910  -0.13463  32.49482  -0.24474 -32.11057  -0.15796 " max="4" scale="108.759012758181"/>
% % 		<Axis Name="Ty" values="-36.63785   0.16587  18.24145  -0.39863  19.10634   0.16747 " max="4" scale="108.759012758181"/>
% % 		<Axis Name="Tz" values="  0.78551 -18.83761   0.00110 -18.39501  -0.10621 -18.71622 " max="4" scale="108.834783900343"/>
% % 		<BasicTransform Dx="0" Dy="0" Dz="0.0053594" Rx="0" Ry="0" Rz="0"/>
% % 	</Calibration>
% % </FTSensor>



% C = [0.49669  -0.30752  -0.67526  31.58401  -0.52567 -32.13403;
%     0.91512 -37.57532  -0.01988  17.91305   0.26027  18.99891;
%     18.30062  -0.22759  18.41379  -0.40968  18.58899  -0.15117;
%     0.26910  -0.13463  32.49482  -0.24474 -32.11057  -0.15796;
%     -36.63785   0.16587  18.24145  -0.39863  19.10634   0.16747;
%     0.78551 -18.83761   0.00110 -18.39501  -0.10621 -18.71622];
% 
% Scales = [2.60913047446393 2.60913047446393 0.88615981830186 108.759012758181 108.759012758181 108.834783900343];
% Dx = 0; Dy = 0; Dz = 0.0053594; Rx = 0; Ry = 0; Rz = 0;

%% Calibration data and scales from CD - Nano43

% <?xml version="1.0" encoding="utf-8"?>
% <!-- NOTE: To parse this file for your own software, use the "UserAxis" elements     -->
% <!-- to construct the calibration matrix.  The "Axis" elements contain a scaled and  -->
% <!-- transformed version of the matrix used for ATI's internal purposes only.  When  -->
% <!-- you send your sensor back for recalibration, or replace your sensor with a new  -->
% <!-- one, you will receive a new calibration file with a different matrix.  Make     -->
% <!-- sure any custom software you write stays up to date with the latest version of  -->
% <!-- the calibration matrix.                                                         -->
% <FTSensor  Serial="FT32831" BodyStyle="Nano43" Family="DAQ" NumGages="6" CalFileVersion="1.1">
% 	<Calibration  PartNumber="SI-36-0.5" CalDate="12/14/2020" ForceUnits="N" TorqueUnits="N-mm" DistUnits="mm" OutputMode="Ground Referenced Differential" OutputRange="20" HWTempComp="True" GainMultiplier="1" CableLossDetection="False" OutputBipolar="True">
% 		<Axis Name="Fx" values="  0.17124  -0.23480  -0.42753  36.86148  -0.44469 -36.43208 " max="36" scale="8.1634377013728"/>
% 		<Axis Name="Fy" values=" -1.11998 -41.82688  -0.34426  21.11841   0.32722  20.58933 " max="36" scale="8.1634377013728"/>
% 		<Axis Name="Fz" values=" 20.64039  -0.11474  20.88593   0.15774  20.81387   0.08546 " max="36" scale="6.93470571615175"/>
% 		<Axis Name="Tx" values=" -0.17249  -0.69307  36.14830   0.63283 -35.99689   0.26524 " max="500" scale="0.825836392458168"/>
% 		<Axis Name="Ty" values="-41.68215   0.33486  20.86280  -0.56441  20.56406   0.66095 " max="500" scale="0.825836392458168"/>
% 		<Axis Name="Tz" values=" -0.72205 -22.00406   0.30563 -21.35581  -0.24436 -21.47014 " max="500" scale="0.574757166607079"/>
% 		<BasicTransform Dx="0" Dy="0" Dz="4.3434" Rx="0" Ry="0" Rz="0"/>
% 	</Calibration>
% </FTSensor>


C = [0.17124  -0.23480  -0.42753  36.86148  -0.44469 -36.43208;
    -1.11998 -41.82688  -0.34426  21.11841   0.32722  20.58933;
    20.64039  -0.11474  20.88593   0.15774  20.81387   0.08546;
    -0.17249  -0.69307  36.14830   0.63283 -35.99689   0.26524;
    -41.68215   0.33486  20.86280  -0.56441  20.56406   0.66095;
    -0.72205 -22.00406   0.30563 -21.35581  -0.24436 -21.47014];
Scales = [8.1634377013728 8.1634377013728 6.93470571615175 0.825836392458168 0.825836392458168 0.574757166607079];
Dx = 0; Dy = 0; Dz = 4.3434; Rx = 0; Ry = 0; Rz = 0;


%% Calibration matric calculation
WorkingMatrix = zeros(6,6);
for i = 1:6
    for j = 1:6
        WorkingMatrix(i, j) = C(i, j) / Scales(i); %diff sensors have diff sensitivity therefore need to scale to fit the actual 
    end
end
RotMatrix(0+1, 0+1) = cos(Ry) * cos(Rz);
RotMatrix(0+1, 1+1) = sin(Rx) * sin(Ry) * cos(Rz) + cos(Rx) * sin(Rz);
RotMatrix(0+1, 2+1) = sin(Rx) * sin(Rz) - cos(Rx) * sin(Ry) * cos(Rz);
RotMatrix(0+1, 3+1) = 0;
RotMatrix(0+1, 4+1) = 0;
RotMatrix(0+1, 5+1) = 0;
RotMatrix(1+1, 0+1) = (-1) * cos(Ry) * sin(Rz);
RotMatrix(1+1, 1+1) = (-1) * sin(Rx) * sin(Ry) * sin(Rz) + cos(Rx) * cos(Rz);
RotMatrix(1+1, 2+1) = sin(Rx) * cos(Rz) + cos(Rx) * sin(Ry) * sin(Rz);
RotMatrix(1+1, 3+1) = 0;
RotMatrix(1+1, 4+1) = 0;
RotMatrix(1+1, 5+1) = 0;
RotMatrix(2+1, 0+1) = sin(Ry);
RotMatrix(2+1, 1+1) = -sin(Rx) * cos(Ry);
RotMatrix(2+1, 2+1) = cos(Rx) * cos(Ry);
RotMatrix(2+1, 3+1) = 0;
RotMatrix(2+1, 4+1) = 0;
RotMatrix(2+1, 5+1) = 0;
RotMatrix(3+1, 0+1) = 0;
RotMatrix(3+1, 1+1) = 0;
RotMatrix(3+1, 2+1) = 0;
RotMatrix(3+1, 3+1) = cos(Ry) * cos(Rz);
RotMatrix(3+1, 4+1) = sin(Rx) * sin(Ry) * cos(Rz) + cos(Rx) * sin(Rz);
RotMatrix(3+1, 5+1) = sin(Rx) * sin(Rz) - cos(Rx) * sin(Ry) * cos(Rz);
RotMatrix(4+1, 0+1) = 0;
RotMatrix(4+1, 1+1) = 0;
RotMatrix(4+1, 2+1) = 0;
RotMatrix(4+1, 3+1) = (-1) * cos(Ry) * sin(Rz);
RotMatrix(4+1, 4+1) = (-1) * sin(Rx) * sin(Ry) * sin(Rz) + cos(Rx) * cos(Rz);
RotMatrix(4+1, 5+1) = sin(Rx) * cos(Rz) + cos(Rx) * sin(Ry) * sin(Rz);
RotMatrix(5+1, 0+1) = 0;
RotMatrix(5+1, 1+1) = 0;
RotMatrix(5+1, 2+1) = 0;
RotMatrix(5+1, 3+1) = sin(Ry);
RotMatrix(5+1, 4+1) = -sin(Rx) * cos(Ry);
RotMatrix(5+1, 5+1) = cos(Rx) * cos(Ry);

DispMatrix(0+1, 0+1) = 1;
DispMatrix(0+1, 1+1) = 0;
DispMatrix(0+1, 2+1) = 0;
DispMatrix(0+1, 3+1) = 0;
DispMatrix(0+1, 4+1) = 0;
DispMatrix(0+1, 5+1) = 0;
DispMatrix(1+1, 0+1) = 0;
DispMatrix(1+1, 1+1) = 1;
DispMatrix(1+1, 2+1) = 0;
DispMatrix(1+1, 3+1) = 0;
DispMatrix(1+1, 4+1) = 0;
DispMatrix(1+1, 5+1) = 0;
DispMatrix(2+1, 0+1) = 0;
DispMatrix(2+1, 1+1) = 0;
DispMatrix(2+1, 2+1) = 1;
DispMatrix(2+1, 3+1) = 0;
DispMatrix(2+1, 4+1) = 0;
DispMatrix(2+1, 5+1) = 0;
DispMatrix(3+1, 0+1) = 0;
DispMatrix(3+1, 1+1) = Dz;
DispMatrix(3+1, 2+1) = (-1) * Dy;
DispMatrix(3+1, 3+1) = 1;
DispMatrix(3+1, 4+1) = 0;
DispMatrix(3+1, 5+1) = 0;
DispMatrix(4+1, 0+1) = (-1) * Dz;
DispMatrix(4+1, 1+1) = 0;
DispMatrix(4+1, 2+1) = Dx;
DispMatrix(4+1, 3+1) = 0;
DispMatrix(4+1, 4+1) = 1;
DispMatrix(4+1, 5+1) = 0;
DispMatrix(5+1, 0+1) = Dy;
DispMatrix(5+1, 1+1) = (-1) * Dx;
DispMatrix(5+1, 2+1) = 0;
DispMatrix(5+1, 3+1) = 0;
DispMatrix(5+1, 4+1) = 0;
DispMatrix(5+1, 5+1) = 1;

Matrix = RotMatrix*DispMatrix;
Matrix = Matrix*WorkingMatrix;

CalMatrix = Matrix;
end