clc; clear; close all;

mm=1/1000;
cm=1/100;

%has format : 12x4 - off which first 4x4 is the SE3 matrix and the next 8x3 are the LED's in order 1:8
load('trial_settings.mat');
load('data_settings.mat');
load('SE3filter_params.mat')

%% Recording data
tic;

j = 1;
while toc<exp_dur

    % Receive ext clock via UDP
    loadcell_record_current = recv_loadcell_packet
    
    % Record time-indexed data
    loadcell_record(j,:) = loadcell_record_current();
    time_loadcell(j,:) = loadcell_record_current(1); %% ASSUMING that the incoming data is a 1x7 matrix with the format [t fx fy fz tx ty tz]
   
    j = j +1;
end

%% Save data
save(loadcell_filename,'time_loadcell','loadcell_record');