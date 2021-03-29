clc; clear; close all;

mm=1/1000;
cm=1/100;

%has format : 12x4 - off which first 4x4 is the SE3 matrix and the next 8x3 are the LED's in order 1:8
load('trial_settings.mat');
load('data_settings.mat');
load('SE3filter_params.mat')
%% MoCap data structures
LEDnames = VzGetNam;
%% Recording data
tic;

j = 1;
while toc<exp_dur

    % Receive ext clock via UDP
    curr_time = recv_timer;
    
    % Collect data from MoCap
    [data] = VzGetDat; % format : no_markers x 6 - TCM#, LED#, X, Y, Z, t
    
    % Record time-indexed data
    mocap_record(j,:) = reshape(data',1,[]);
    time_mocap(j,:) = curr_time;
   
    j = j +1;
end

%Handle plots
figure('Name','Mocap Elliptical Buccal Surface (Handle)','NumberTitle','off');
handle_idx = toothbrush_handle_frame_idx(1,:); % first LED on toothbrush_handle
subplot(3,2,1);
plot(mocap_record(:,handle_idx))
title('LED23','fontweight','bold');


handle_idx = toothbrush_handle_frame_idx(2,:); 
subplot(3,2,2);
plot(mocap_record(:,handle_idx))
title('LED24','fontweight','bold');

handle_idx = toothbrush_handle_frame_idx(3,:); 
subplot(3,2,3);
plot(mocap_record(:,handle_idx))
title('LED25','fontweight','bold');

handle_idx = toothbrush_handle_frame_idx(4,:); 
subplot(3,2,4);
plot(mocap_record(:,handle_idx))
title('LED26','fontweight','bold');

handle_idx = toothbrush_handle_frame_idx(5,:); 
subplot(3,2,5);
plot(mocap_record(:,handle_idx))
title('LED27','fontweight','bold');

handle_idx = toothbrush_handle_frame_idx(6,:); 
subplot(3,2,6);
plot(mocap_record(:,handle_idx))
title('LED28','fontweight','bold');

%Platform plots
figure('Name','Mocap Elliptical Buccal Surface (Platform)','NumberTitle','off');
platform_idx = platform_frame_idx(1,:);
subplot(3,1,1);
plot(mocap_record(:,platform_idx))
title('LED17','fontweight','bold');

platform_idx = platform_frame_idx(2,:);
subplot(3,1,2);
plot(mocap_record(:,platform_idx))
title('LED18','fontweight','bold');

platform_idx = platform_frame_idx(3,:);
subplot(3,1,3);
plot(mocap_record(:,platform_idx))
title('LED19','fontweight','bold');

[x_range,y_range] = ginput(2);

full_range = [1:length(mocap_record)]';
range_world_frame_calibration = and(full_range>x_range(1),full_range<x_range(2));
mocap_record_filtered = mocap_record(range_world_frame_calibration,:);

figure('Name','Mocap Elliptical Buccal Surface (Platform_Calibrated)','NumberTitle','off');
platform_idx = platform_frame_idx(1,:);
subplot(3,1,1);
plot(mocap_record_filtered(:,platform_idx))
title('LED17','fontweight','bold');

platform_idx = platform_frame_idx(2,:);
subplot(3,1,2);
plot(mocap_record_filtered(:,platform_idx))
title('LED18','fontweight','bold');

platform_idx = platform_frame_idx(3,:);
subplot(3,1,3);
plot(mocap_record_filtered(:,platform_idx))
title('LED19','fontweight','bold');



%% Save data
save(mocap_filename,'time_mocap','mocap_record');